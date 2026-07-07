/**
 * PSG Synthesizer — 4-Channel Chip Music Engine
 *
 * Channels:
 *   CH1: Pulse wave (duty cycle adjustable)
 *   CH2: Pulse wave (duty cycle adjustable)
 *   CH3: Triangle wave
 *   CH4: LFSR noise (short/long period)
 *
 * Each channel has independent ADSR envelope and frequency.
 * Output: 22050 Hz / 16-bit signed mono.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "ipc/ipc.h"  /* for sfx_type_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Channel IDs ---- */
#define SYNTH_CH_PULSE1  0
#define SYNTH_CH_PULSE2  1
#define SYNTH_CH_TRI     2
#define SYNTH_CH_NOISE   3
#define SYNTH_CH_COUNT   4

/* ---- MIDI note helpers ---- */
#define NOTE_C2  36
#define NOTE_D2  38
#define NOTE_E2  40
#define NOTE_F2  41
#define NOTE_G2  43
#define NOTE_A2  45
#define NOTE_B2  47
#define NOTE_C3  48
#define NOTE_D3  50
#define NOTE_E3  52
#define NOTE_F3  53
#define NOTE_G3  55
#define NOTE_A3  57
#define NOTE_B3  59
#define NOTE_C4  60
#define NOTE_D4  62
#define NOTE_E4  64
#define NOTE_F4  65
#define NOTE_G4  67
#define NOTE_A4  69
#define NOTE_B4  71
#define NOTE_C5  72
#define NOTE_D5  74
#define NOTE_E5  76
#define NOTE_F5  77
#define NOTE_G5  79
#define NOTE_A5  81
#define NOTE_B5  83
#define NOTE_C6  84
#define NOTE_SILENCE  0

/**
 * @brief Initialize the PSG synthesizer
 */
void audio_synth_init(void);

/**
 * @brief Set a channel's waveform parameters immediately
 *
 * @param channel   SYNTH_CH_PULSE1 / PULSE2 / TRI / NOISE
 * @param midi_note MIDI note (0=silence, 1-127=note)
 * @param duty      0-100 (pulse duty cycle, ignored for TRI/NOISE)
 */
void audio_synth_set_note(int channel, int midi_note, int duty);

/**
 * @brief Trigger note-on for a channel (enters Attack phase)
 */
void audio_synth_note_on(int channel, int midi_note, int duty, uint8_t velocity);

/**
 * @brief Trigger note-off for a channel (enters Release phase)
 */
void audio_synth_note_off(int channel);

/**
 * @brief Set ADSR parameters for a channel
 *
 * @param channel    Channel ID
 * @param attack_ms  Attack time (ms)
 * @param decay_ms   Decay time (ms)
 * @param sustain    Sustain level (0-15)
 * @param release_ms Release time (ms)
 */
void audio_synth_set_adsr(int channel, int attack_ms, int decay_ms,
                          int sustain, int release_ms);

/**
 * @brief Set noise channel mode
 * @param short_period true=short (metallic), false=long (rumbling)
 */
void audio_synth_set_noise_mode(bool short_period);

/**
 * @brief Trigger a sound effect
 *
 * Overrides channel usage for the duration of the effect.
 * BGM channels are temporarily muted during SFX playback.
 *
 * @param sfx  Sound effect type
 */
void audio_synth_trigger_sfx(sfx_type_t sfx);

/**
 * @brief Check if any SFX is currently playing
 */
bool audio_synth_sfx_active(void);

/**
 * @brief Render N samples of mixed audio (16-bit signed mono)
 * @param buf          Destination buffer
 * @param num_samples  Number of samples to generate
 */
void audio_synth_render(int16_t *buf, int num_samples);

/**
 * @brief Control BGM playback
 */
void audio_synth_bgm_start(void);
void audio_synth_bgm_stop(void);
void audio_synth_bgm_pause(void);
bool audio_synth_bgm_is_playing(void);

/**
 * @brief Set master volume (0-100)
 */
void audio_synth_set_master_vol(int vol);

#ifdef __cplusplus
}
#endif
