/**
 * PSG Synthesizer — 4-Channel Chip Music Engine Implementation
 *
 * Generates 8-bit/16-bit-style chip music audio using:
 *   - CH1: Pulse wave (duty cycle: 12.5%, 25%, 50%, 75%)
 *   - CH2: Pulse wave (same)
 *   - CH3: Triangle wave
 *   - CH4: LFSR noise (short/long period)
 *
 * All channels have ADSR envelope, mixed to 16-bit mono output.
 */

#include "audio_synth.h"
#include "audio_driver.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "synth";

/* ========================================================================
 * ADSR Envelope State Machine
 * ======================================================================== */

typedef enum {
    ADSR_IDLE,
    ADSR_ATTACK,
    ADSR_DECAY,
    ADSR_SUSTAIN,
    ADSR_RELEASE,
} adsr_phase_t;

typedef struct {
    adsr_phase_t phase;
    float level;           /* current envelope level 0.0-1.0 */
    float attack_rate;     /* per sample increment */
    float decay_rate;      /* per sample decrement */
    float sustain_level;   /* 0.0-1.0 */
    float release_rate;    /* per sample decrement */
} adsr_t;

/* ========================================================================
 * Per-Channel State
 * ======================================================================== */

typedef struct {
    bool    active;           /* channel enabled */
    int     midi_note;        /* 0=silence, 1-127=active note */
    float   frequency;        /* Hz */
    float   phase;            /* 0.0-1.0 */
    float   phase_inc;        /* per sample increment */
    int     duty;             /* 12/25/50/75 for pulse */
    int     volume;           /* 0-15 base volume (pre-envelope) */
    adsr_t  adsr;

    /* Noise channel specific */
    uint16_t lfsr;            /* 15-bit LFSR state */
    bool     noise_short;     /* true=short period (93 samples), false=long */
} synth_channel_t;

/* ========================================================================
 * SFX State
 * ======================================================================== */

typedef struct {
    bool    active;
    sfx_type_t type;
    uint32_t start_sample;  /* sample counter when SFX started */
    uint32_t duration_samples;
    uint8_t saved_vol[SYNTH_CH_COUNT]; /* BGM volumes saved during SFX */
} sfx_state_t;

/* ========================================================================
 * BGM State
 * ======================================================================== */

typedef enum {
    BGM_STOPPED,
    BGM_PLAYING,
    BGM_PAUSED,
} bgm_state_t;

/* BGM data imported from bgm_data.h */
#include "bgm_data.h"

typedef struct {
    bgm_state_t state;
    int         step;            /* current step (16th note) index */
    uint32_t    samples_per_step; /* samples per 16th note at current tempo */
    uint32_t    sample_counter;   /* samples elapsed in current step */
    int         tempo;           /* BPM */
} bgm_player_t;

/* ========================================================================
 * Global State
 * ======================================================================== */

static synth_channel_t g_ch[SYNTH_CH_COUNT];
static sfx_state_t     g_sfx;
static bgm_player_t    g_bgm;
static uint32_t        g_sample_count = 0;
static int             g_master_vol = 60;  /* 0-100 */

/* MIDI note → frequency (A4=440Hz) */
static float midi_to_freq(int note)
{
    if (note <= 0 || note > 127) return 0.0f;
    return 440.0f * powf(2.0f, (note - 69) / 12.0f);
}

/* ========================================================================
 * ADSR
 * ======================================================================== */

static void adsr_reset(adsr_t *a)
{
    a->phase  = ADSR_IDLE;
    a->level  = 0.0f;
}

static void adsr_note_on(adsr_t *a)
{
    a->phase = ADSR_ATTACK;
    /* Start from current level (important for legato / re-trigger) */
    if (a->level < 0.01f) a->level = 0.0f;
}

static void adsr_note_off(adsr_t *a)
{
    if (a->phase != ADSR_IDLE && a->phase != ADSR_RELEASE) {
        a->phase = ADSR_RELEASE;
    }
}

static float adsr_tick(adsr_t *a)
{
    switch (a->phase) {
    case ADSR_IDLE:
        a->level = 0.0f;
        break;
    case ADSR_ATTACK:
        a->level += a->attack_rate;
        if (a->level >= 1.0f) {
            a->level = 1.0f;
            a->phase = ADSR_DECAY;
        }
        break;
    case ADSR_DECAY:
        a->level -= a->decay_rate;
        if (a->level <= a->sustain_level) {
            a->level = a->sustain_level;
            a->phase = ADSR_SUSTAIN;
        }
        break;
    case ADSR_SUSTAIN:
        /* Hold at sustain level */
        break;
    case ADSR_RELEASE:
        a->level -= a->release_rate;
        if (a->level <= 0.0f) {
            a->level = 0.0f;
            a->phase = ADSR_IDLE;
        }
        break;
    }
    return a->level;
}

static void adsr_set_params(adsr_t *a, int attack_ms, int decay_ms,
                             int sustain, int release_ms)
{
    float sr = (float)AUDIO_SAMPLE_RATE;

    /* sustain: 0-15 → 0.0-1.0 */
    a->sustain_level = (float)sustain / 15.0f;
    if (a->sustain_level > 1.0f) a->sustain_level = 1.0f;

    /* Rate = 1.0 / (time_ms / 1000 * sample_rate) */
    a->attack_rate  = (attack_ms > 0)  ? (1.0f / (attack_ms * 0.001f * sr)) : 1.0f;
    a->decay_rate   = (decay_ms > 0)   ? ((1.0f - a->sustain_level) / (decay_ms * 0.001f * sr)) : 1.0f;
    a->release_rate = (release_ms > 0) ? (a->sustain_level / (release_ms * 0.001f * sr)) : 0.5f;
}

/* ========================================================================
 * Waveform generators
 * ======================================================================== */

/* Pulse wave with duty cycle (12, 25, 50, 75) */
static float gen_pulse(float phase, int duty)
{
    float threshold;
    switch (duty) {
    case 12: threshold = 0.125f; break;
    case 25: threshold = 0.25f;  break;
    case 75: threshold = 0.75f;  break;
    default: threshold = 0.50f;  break; /* 50% = square */
    }
    return (phase < threshold) ? 1.0f : -1.0f;
}

/* Triangle wave: linear ramp up then down */
static float gen_triangle(float phase)
{
    if (phase < 0.5f) {
        return 4.0f * phase - 1.0f;          /* 0→0.5 → -1→+1 */
    } else {
        return 3.0f - 4.0f * phase;          /* 0.5→1.0 → +1→-1 */
    }
}

/*
 * LFSR noise: 15-bit shift register, tap bits 14 and ~13 (NES-APU style).
 *
 * Unlike tone channels, the noise LFSR is NOT clocked every sample.
 * Instead, a frequency divider derived from the channel's midi_note
 * determines how often the LFSR shifts.  The phase accumulator is reused
 * as the divider counter: each time it wraps (>= 1.0), the LFSR advances
 * one step.  This keeps the noise output in the audible range instead of
 * producing 11 kHz white-noise hiss.
 *
 * Short mode (metallic / snare / hihat):   divider ≈ note_freq * 4
 * Long mode  (rumbling / kick):            divider ≈ note_freq
 */
static int gen_noise(synth_channel_t *ch)
{
    /* Accumulate phase — wrap at 1.0; each wrap advances the LFSR once */
    float step = ch->phase_inc;
    if (step <= 0.0f) {
        return (ch->lfsr & 1) ? 1 : -1;   /* silent — hold last bit */
    }

    ch->phase += step;
    if (ch->phase < 1.0f) {
        /* No wrap yet — output holds previous bit (sample-and-hold emulation) */
        return (ch->lfsr & 1) ? 1 : -1;
    }
    ch->phase -= 1.0f;   /* wrap, keeping fractional remainder */

    /* Short mode: clock the LFSR more often (higher pitch) */
    if (ch->noise_short) {
        ch->phase += step * 3.0f;   /* effective divider = freq * 4 */
    }
    while (ch->phase >= 1.0f) {
        ch->phase -= 1.0f;
        /* Advance LFSR: bit 14 XOR bit 13 (NES-style feedback) */
        int bit14 = (ch->lfsr >> 14) & 1;
        int bit13 = (ch->lfsr >> 13) & 1;
        int feedback = bit14 ^ bit13;
        ch->lfsr = (uint16_t)(((ch->lfsr << 1) | feedback) & 0x7FFF);
    }

    /* Short mode: output bit 6 for metallic periodic noise */
    if (ch->noise_short) {
        return ((ch->lfsr >> 6) & 1) ? 1 : -1;
    }

    return (ch->lfsr & 1) ? 1 : -1;
}

/* ========================================================================
 * Channel render — returns sample in range [-1.0, 1.0]
 * ======================================================================== */

static float channel_render(synth_channel_t *ch)
{
    if (!ch->active || ch->midi_note == 0 || ch->phase_inc == 0.0f) {
        /* Tick ADSR even when silent (release may be in progress) */
        adsr_tick(&ch->adsr);
        return 0.0f;
    }

    float env = adsr_tick(&ch->adsr);
    if (env <= 0.0f) return 0.0f;

    float sample;
    int idx = (int)(ch - g_ch);

    if (idx == SYNTH_CH_NOISE) {
        sample = (float)gen_noise(ch);
    } else if (idx == SYNTH_CH_TRI) {
        sample = gen_triangle(ch->phase);
    } else {
        sample = gen_pulse(ch->phase, ch->duty);
    }

    /* Advance phase (noise channel manages its own phase inside gen_noise) */
    if (idx != SYNTH_CH_NOISE) {
        ch->phase += ch->phase_inc;
        if (ch->phase >= 1.0f) ch->phase -= (float)((int)ch->phase + 1);
    }

    /* Volume: base vol (0-15) × envelope (0-1) × master vol scale */
    return sample * ((float)ch->volume / 15.0f) * env;
}

/* ========================================================================
 * BGM Player — note data from bgm_data.h
 * ======================================================================== */

static void bgm_advance_step(void)
{
    if (g_bgm.state != BGM_PLAYING || g_bgm.step >= BGM_PATTERN_LENGTH) {
        return;
    }

    int step = g_bgm.step;

    /* Each channel is stored as a flat array of [note, duration] pairs.
     * The step position maps to the next note in each pattern. */
    int idx = step * 2;

    /* Pulse 1 */
    {
        uint8_t note = bgm_pulse1_data[idx];
        uint8_t dur  = bgm_pulse1_data[idx + 1];
        if (note > 0) {
            audio_synth_note_on(SYNTH_CH_PULSE1, note, 50, 10);
        }
        /* Duration handled by BGM tick timing */
        (void)dur;
    }

    /* Pulse 2 */
    {
        uint8_t note = bgm_pulse2_data[idx];
        uint8_t dur  = bgm_pulse2_data[idx + 1];
        if (note > 0) {
            audio_synth_note_on(SYNTH_CH_PULSE2, note, 25, 8);
        }
        (void)dur;
    }

    /* Triangle */
    {
        uint8_t note = bgm_tri_data[idx];
        uint8_t dur  = bgm_tri_data[idx + 1];
        if (note > 0) {
            audio_synth_note_on(SYNTH_CH_TRI, note, 50, 12);
        }
        (void)dur;
    }

    /* Noise — drum trigger (0x01=kick, 0x02=snare, 0x04=hihat) */
    {
        uint8_t drum = bgm_noise_data[idx];
        if (drum & 0x01) {
            g_ch[SYNTH_CH_NOISE].noise_short = false;
            audio_synth_note_on(SYNTH_CH_NOISE, 60, 50, 14);
        } else if (drum & 0x02) {
            g_ch[SYNTH_CH_NOISE].noise_short = true;
            audio_synth_note_on(SYNTH_CH_NOISE, 84, 50, 12);
        } else if (drum & 0x04) {
            g_ch[SYNTH_CH_NOISE].noise_short = true;
            audio_synth_note_on(SYNTH_CH_NOISE, 108, 50, 5);
        }
    }
}

static void bgm_tick(void)
{
    if (g_bgm.state != BGM_PLAYING) return;
    if (BGM_PATTERN_LENGTH <= 0) return;

    g_bgm.sample_counter++;
    if (g_bgm.sample_counter >= g_bgm.samples_per_step) {
        g_bgm.sample_counter = 0;

        /* Note-off previous step's notes */
        audio_synth_note_off(SYNTH_CH_PULSE1);
        audio_synth_note_off(SYNTH_CH_PULSE2);
        audio_synth_note_off(SYNTH_CH_TRI);
        audio_synth_note_off(SYNTH_CH_NOISE);

        g_bgm.step++;
        if (g_bgm.step >= BGM_PATTERN_LENGTH) {
            g_bgm.step = 0; /* Loop */
        }
        bgm_advance_step();
    }
}

/* ========================================================================
 * SFX Engine
 * ======================================================================== */

/* SFX parameter table */
static const struct {
    uint16_t duration_ms;
    struct {
        uint8_t start_note;
        int8_t  note_sweep;     /* semitones change per 50ms */
        int     duty;
        int     vol;            /* 0-15 */
    } ch[SYNTH_CH_COUNT];
} g_sfx_defs[SFX_COUNT] = {
    /* SFX_WALL_BOUNCE — short noise burst + low thud */
    [SFX_WALL_BOUNCE] = {
        .duration_ms = 60,
        .ch = {
            [SYNTH_CH_PULSE1] = { .start_note = 48, .note_sweep = -8, .duty = 50, .vol = 8 },
            [SYNTH_CH_NOISE]  = { .start_note = 1,  .note_sweep = 0,  .duty = 0,  .vol = 12 },
        },
    },
    /* SFX_FRUIT_PICKUP — ascending arpeggio C-E-G-C */
    [SFX_FRUIT_PICKUP] = {
        .duration_ms = 200,
        .ch = {
            [SYNTH_CH_PULSE1] = { .start_note = NOTE_C5, .note_sweep = 0, .duty = 50, .vol = 12 },
            [SYNTH_CH_PULSE2] = { .start_note = NOTE_E5, .note_sweep = 0, .duty = 25, .vol = 8 },
        },
    },
    /* SFX_PORTAL — frequency sweep up + wide noise */
    [SFX_PORTAL] = {
        .duration_ms = 250,
        .ch = {
            [SYNTH_CH_PULSE1] = { .start_note = 36, .note_sweep = 12, .duty = 50, .vol = 10 },
            [SYNTH_CH_NOISE]  = { .start_note = 1,  .note_sweep = 0,  .duty = 0,  .vol = 8 },
        },
    },
    /* SFX_DEATH — descending slide */
    [SFX_DEATH] = {
        .duration_ms = 400,
        .ch = {
            [SYNTH_CH_PULSE1] = { .start_note = 72, .note_sweep = -6, .duty = 50, .vol = 12 },
            [SYNTH_CH_NOISE]  = { .start_note = 1,  .note_sweep = 0,  .duty = 0,  .vol = 6 },
        },
    },
    /* SFX_WIN — ascending major triad arpeggio */
    [SFX_WIN] = {
        .duration_ms = 600,
        .ch = {
            [SYNTH_CH_PULSE1] = { .start_note = NOTE_C5, .note_sweep = 0, .duty = 50, .vol = 14 },
            [SYNTH_CH_PULSE2] = { .start_note = NOTE_E5, .note_sweep = 0, .duty = 25, .vol = 10 },
            [SYNTH_CH_TRI]    = { .start_note = NOTE_C4, .note_sweep = 0, .duty = 0,  .vol = 12 },
        },
    },
    /* SFX_LOSE — descending minor triad */
    [SFX_LOSE] = {
        .duration_ms = 500,
        .ch = {
            [SYNTH_CH_PULSE1] = { .start_note = NOTE_A4, .note_sweep = -3, .duty = 50, .vol = 12 },
            [SYNTH_CH_PULSE2] = { .start_note = NOTE_E4, .note_sweep = -3, .duty = 25, .vol = 8 },
        },
    },
};

static void sfx_tick(uint32_t elapsed_ms)
{
    if (!g_sfx.active) return;

    const sfx_type_t type = g_sfx.type;
    if (type >= SFX_COUNT) return;

    if (elapsed_ms >= g_sfx.duration_samples * 1000 / AUDIO_SAMPLE_RATE) {
        /* SFX done — restore channels to pre-SFX state */
        g_sfx.active = false;
        for (int i = 0; i < SYNTH_CH_COUNT; i++) {
            g_ch[i].volume = g_sfx.saved_vol[i];
            if (g_sfx_defs[type].ch[i].vol > 0) {
                /* This channel was used by the SFX — silence it so
                 * stale midi_note values don't leak into BGM output */
                g_ch[i].midi_note = 0;
                g_ch[i].phase_inc = 0.0f;
                g_ch[i].phase     = 0.0f;
                adsr_note_off(&g_ch[i].adsr);
            }
            /* Channels NOT used by this SFX keep their current state
             * (BGM may be playing on them) — no touch */
        }
        /* Restore noise to long-period for BGM */
        g_ch[SYNTH_CH_NOISE].noise_short = false;
        return;
    }

    /* Update per-channel SFX parameters */
    for (int i = 0; i < SYNTH_CH_COUNT; i++) {
        if (g_sfx_defs[type].ch[i].vol == 0) continue;

        synth_channel_t *ch = &g_ch[i];
        const int base_note = g_sfx_defs[type].ch[i].start_note;
        const int sweep = g_sfx_defs[type].ch[i].note_sweep;

        if (base_note > 0) {
            /* Calculate note with sweep applied */
            int sweeps = (int)(elapsed_ms / 50);
            int note = base_note + sweep * sweeps;
            if (note < 1)  note = 1;
            if (note > 127) note = 127;
            ch->midi_note = note;
            ch->frequency  = midi_to_freq(note);
            ch->phase_inc  = ch->frequency / (float)AUDIO_SAMPLE_RATE;
            ch->duty       = g_sfx_defs[type].ch[i].duty;
        }
    }
}

/* ========================================================================
 * Public API
 * ======================================================================== */

void audio_synth_init(void)
{
    memset(g_ch, 0, sizeof(g_ch));
    memset(&g_sfx, 0, sizeof(g_sfx));
    memset(&g_bgm, 0, sizeof(g_bgm));

    /* Default ADSR: slow attack, medium decay, high sustain, medium release */
    for (int i = 0; i < SYNTH_CH_COUNT; i++) {
        audio_synth_set_adsr(i, 20, 50, 10, 100);
        g_ch[i].volume = 10;
        g_ch[i].active = true;

        /* Init noise LFSR to non-zero */
        g_ch[i].lfsr = 0x5555;
        g_ch[i].noise_short = false;
    }

    /* Special ADSR for noise (percussive) */
    audio_synth_set_adsr(SYNTH_CH_NOISE, 2, 30, 0, 20);

    /* BGM tempo: 120 BPM, 16th note = 31.25ms */
    g_bgm.tempo = 120;
    g_bgm.samples_per_step = AUDIO_SAMPLE_RATE / (g_bgm.tempo / 60 * 4);
    g_bgm.state = BGM_STOPPED;
    g_bgm.step = 0;
    g_bgm.sample_counter = 0;

    g_sample_count = 0;
    g_master_vol = 60;

    ESP_LOGI(TAG, "PSG Synth initialized: 4ch, %dHz", AUDIO_SAMPLE_RATE);
}

void audio_synth_set_note(int channel, int midi_note, int duty)
{
    if (channel < 0 || channel >= SYNTH_CH_COUNT) return;

    synth_channel_t *ch = &g_ch[channel];
    ch->midi_note = midi_note;
    ch->frequency  = midi_to_freq(midi_note);
    ch->phase_inc  = ch->frequency / (float)AUDIO_SAMPLE_RATE;
    ch->duty       = duty;

    /* Reset phase on new note for cleaner attack */
    if (midi_note > 0) {
        ch->phase = 0.0f;
    }
}

void audio_synth_note_on(int channel, int midi_note, int duty, uint8_t velocity)
{
    if (channel < 0 || channel >= SYNTH_CH_COUNT) return;

    synth_channel_t *ch = &g_ch[channel];
    ch->active = true;

    /* velocity 0-15 → volume */
    ch->volume = (velocity > 15) ? 15 : velocity;

    audio_synth_set_note(channel, midi_note, duty);
    adsr_note_on(&ch->adsr);
}

void audio_synth_note_off(int channel)
{
    if (channel < 0 || channel >= SYNTH_CH_COUNT) return;
    adsr_note_off(&g_ch[channel].adsr);
}

void audio_synth_set_adsr(int channel, int attack_ms, int decay_ms,
                           int sustain, int release_ms)
{
    if (channel < 0 || channel >= SYNTH_CH_COUNT) return;
    adsr_set_params(&g_ch[channel].adsr, attack_ms, decay_ms, sustain, release_ms);
}

void audio_synth_set_noise_mode(bool short_period)
{
    g_ch[SYNTH_CH_NOISE].noise_short = short_period;
}

void audio_synth_trigger_sfx(sfx_type_t sfx)
{
    if (sfx >= SFX_COUNT) return;

    /* Save current channel volumes (for BGM restore) */
    for (int i = 0; i < SYNTH_CH_COUNT; i++) {
        g_sfx.saved_vol[i] = g_ch[i].volume;
    }

    /* Silence BGM channels that SFX will use */
    for (int i = 0; i < SYNTH_CH_COUNT; i++) {
        if (g_sfx_defs[sfx].ch[i].vol > 0) {
            g_ch[i].active = true;
            adsr_reset(&g_ch[i].adsr);
            /* Set quick ADSR for SFX */
            audio_synth_set_adsr(i, 2, 30, 10, 50);
        }
    }

    /* Fire initial notes */
    for (int i = 0; i < SYNTH_CH_COUNT; i++) {
        if (g_sfx_defs[sfx].ch[i].vol > 0) {
            int note = g_sfx_defs[sfx].ch[i].start_note;
            if (note > 0) {
                int duty = g_sfx_defs[sfx].ch[i].duty;
                int vol  = g_sfx_defs[sfx].ch[i].vol;
                audio_synth_note_on(i, note, duty, (uint8_t)vol);
            }
        }
    }

    /* Set SFX noise mode to short (metallic) */
    if (g_sfx_defs[sfx].ch[SYNTH_CH_NOISE].vol > 0) {
        g_ch[SYNTH_CH_NOISE].noise_short = true;
    }

    g_sfx.active = true;
    g_sfx.type = sfx;
    g_sfx.start_sample = g_sample_count;
    g_sfx.duration_samples = (uint32_t)g_sfx_defs[sfx].duration_ms
                            * AUDIO_SAMPLE_RATE / 1000;

    ESP_LOGD(TAG, "SFX triggered: %d, dur=%dms", sfx,
             g_sfx_defs[sfx].duration_ms);
}

bool audio_synth_sfx_active(void)
{
    return g_sfx.active;
}

void audio_synth_render(int16_t *buf, int num_samples)
{
    for (int n = 0; n < num_samples; n++) {
        /* Update SFX if active */
        if (g_sfx.active) {
            uint32_t elapsed = g_sample_count - g_sfx.start_sample;
            uint32_t elapsed_ms = elapsed * 1000 / AUDIO_SAMPLE_RATE;
            sfx_tick(elapsed_ms);
        }

        /* Update BGM */
        bgm_tick();

        /* Render and mix channels */
        float mixed = 0.0f;
        for (int c = 0; c < SYNTH_CH_COUNT; c++) {
            mixed += channel_render(&g_ch[c]);
        }

        /* Clamp and scale to 16-bit */
        if (mixed > 1.0f)  mixed = 1.0f;
        if (mixed < -1.0f) mixed = -1.0f;

        /* Master volume */
        mixed *= (float)g_master_vol / 100.0f;

        /* Scale to int16 range (leave headroom to avoid clipping with 4 channels) */
        int16_t s = (int16_t)(mixed * 16384.0f);
        buf[n] = s;

        g_sample_count++;
    }
}

/* ---- BGM Control ---- */

void audio_synth_bgm_start(void)
{
    g_bgm.state = BGM_PLAYING;
    g_bgm.step  = 0;
    g_bgm.sample_counter = g_bgm.samples_per_step - 1; /* Trigger immediately */
    ESP_LOGI(TAG, "BGM started (tempo=%d)", g_bgm.tempo);
}

void audio_synth_bgm_stop(void)
{
    g_bgm.state = BGM_STOPPED;
    /* Note-off and silence all channels */
    for (int i = 0; i < SYNTH_CH_COUNT; i++) {
        audio_synth_note_off(i);
        g_ch[i].midi_note = 0;
        g_ch[i].phase_inc = 0.0f;
        g_ch[i].phase     = 0.0f;
    }
    ESP_LOGI(TAG, "BGM stopped");
}

void audio_synth_bgm_pause(void)
{
    g_bgm.state = BGM_PAUSED;
    /* Note-off and silence all channels */
    for (int i = 0; i < SYNTH_CH_COUNT; i++) {
        audio_synth_note_off(i);
        g_ch[i].midi_note = 0;
        g_ch[i].phase_inc = 0.0f;
        g_ch[i].phase     = 0.0f;
    }
    ESP_LOGI(TAG, "BGM paused");
}

bool audio_synth_bgm_is_playing(void)
{
    return g_bgm.state == BGM_PLAYING;
}

void audio_synth_set_master_vol(int vol)
{
    if (vol < 0)   vol = 0;
    if (vol > 100) vol = 100;
    g_master_vol = vol;
}
