/**
 * BGM Data — 8-bit Chip Music Pattern
 *
 * 4-track pattern data for the PSG synthesizer.
 * Format: flat array of [note, duration] pairs.
 *   - note: MIDI note (0=silence)
 *   - duration: 16th note units (typically 1)
 *
 * Tempo: 120 BPM (16th note = 31.25ms)
 * Step 0-31 = 2 measures of 4/4
 */

#pragma once

#include <stdint.h>
#include "audio_synth.h"  /* for NOTE_* macros */

#ifdef __cplusplus
extern "C" {
#endif

/* 32-step pattern (2 bars, ~8s at 120 BPM), loops.
 *
 * Melody: 8-bit style pentatonic + diatonic phrase in C major.
 * Harmony: lower chord tones, 25% duty (softer).
 * Bass: triangle wave, half-note rhythm.
 * Drums: noise channel, kick on beat 1, snare on beat 3. */

static const uint8_t bgm_pulse1_data[] = {
    /* Step  0 */  NOTE_C5, 1,
    /* Step  1 */  NOTE_SILENCE, 1,
    /* Step  2 */  NOTE_E5, 1,
    /* Step  3 */  NOTE_SILENCE, 1,
    /* Step  4 */  NOTE_G5, 1,
    /* Step  5 */  NOTE_SILENCE, 1,
    /* Step  6 */  NOTE_A5, 1,
    /* Step  7 */  NOTE_G5, 1,

    /* Step  8 */  NOTE_E5, 1,
    /* Step  9 */  NOTE_SILENCE, 1,
    /* Step 10 */  NOTE_D5, 1,
    /* Step 11 */  NOTE_SILENCE, 1,
    /* Step 12 */  NOTE_C5, 1,
    /* Step 13 */  NOTE_D5, 1,
    /* Step 14 */  NOTE_E5, 1,
    /* Step 15 */  NOTE_SILENCE, 1,

    /* Step 16 */  NOTE_A4, 1,
    /* Step 17 */  NOTE_SILENCE, 1,
    /* Step 18 */  NOTE_C5, 1,
    /* Step 19 */  NOTE_SILENCE, 1,
    /* Step 20 */  NOTE_E5, 1,
    /* Step 21 */  NOTE_SILENCE, 1,
    /* Step 22 */  NOTE_G5, 1,
    /* Step 23 */  NOTE_A5, 1,

    /* Step 24 */  NOTE_G5, 1,
    /* Step 25 */  NOTE_E5, 1,
    /* Step 26 */  NOTE_D5, 1,
    /* Step 27 */  NOTE_SILENCE, 1,
    /* Step 28 */  NOTE_C5, 1,
    /* Step 29 */  NOTE_D5, 1,
    /* Step 30 */  NOTE_E5, 1,
    /* Step 31 */  NOTE_SILENCE, 1,
};

static const uint8_t bgm_pulse2_data[] = {
    /* Step  0 */  NOTE_C4, 1,
    /* Step  1 */  NOTE_SILENCE, 1,
    /* Step  2 */  NOTE_SILENCE, 1,
    /* Step  3 */  NOTE_SILENCE, 1,
    /* Step  4 */  NOTE_G3, 1,
    /* Step  5 */  NOTE_SILENCE, 1,
    /* Step  6 */  NOTE_SILENCE, 1,
    /* Step  7 */  NOTE_SILENCE, 1,

    /* Step  8 */  NOTE_C4, 1,
    /* Step  9 */  NOTE_SILENCE, 1,
    /* Step 10 */  NOTE_SILENCE, 1,
    /* Step 11 */  NOTE_SILENCE, 1,
    /* Step 12 */  NOTE_G3, 1,
    /* Step 13 */  NOTE_SILENCE, 1,
    /* Step 14 */  NOTE_SILENCE, 1,
    /* Step 15 */  NOTE_SILENCE, 1,

    /* Step 16 */  NOTE_F3, 1,
    /* Step 17 */  NOTE_SILENCE, 1,
    /* Step 18 */  NOTE_SILENCE, 1,
    /* Step 19 */  NOTE_SILENCE, 1,
    /* Step 20 */  NOTE_C4, 1,
    /* Step 21 */  NOTE_SILENCE, 1,
    /* Step 22 */  NOTE_SILENCE, 1,
    /* Step 23 */  NOTE_SILENCE, 1,

    /* Step 24 */  NOTE_G3, 1,
    /* Step 25 */  NOTE_SILENCE, 1,
    /* Step 26 */  NOTE_SILENCE, 1,
    /* Step 27 */  NOTE_SILENCE, 1,
    /* Step 28 */  NOTE_C4, 1,
    /* Step 29 */  NOTE_SILENCE, 1,
    /* Step 30 */  NOTE_G3, 1,
    /* Step 31 */  NOTE_SILENCE, 1,
};

static const uint8_t bgm_tri_data[] = {
    /* Step  0 */  NOTE_C3, 1,
    /* Step  1 */  NOTE_SILENCE, 1,
    /* Step  2 */  NOTE_SILENCE, 1,
    /* Step  3 */  NOTE_SILENCE, 1,
    /* Step  4 */  NOTE_G2, 1,
    /* Step  5 */  NOTE_SILENCE, 1,
    /* Step  6 */  NOTE_SILENCE, 1,
    /* Step  7 */  NOTE_SILENCE, 1,

    /* Step  8 */  NOTE_C3, 1,
    /* Step  9 */  NOTE_SILENCE, 1,
    /* Step 10 */  NOTE_SILENCE, 1,
    /* Step 11 */  NOTE_SILENCE, 1,
    /* Step 12 */  NOTE_G2, 1,
    /* Step 13 */  NOTE_SILENCE, 1,
    /* Step 14 */  NOTE_SILENCE, 1,
    /* Step 15 */  NOTE_SILENCE, 1,

    /* Step 16 */  NOTE_F2, 1,
    /* Step 17 */  NOTE_SILENCE, 1,
    /* Step 18 */  NOTE_SILENCE, 1,
    /* Step 19 */  NOTE_SILENCE, 1,
    /* Step 20 */  NOTE_C3, 1,
    /* Step 21 */  NOTE_SILENCE, 1,
    /* Step 22 */  NOTE_SILENCE, 1,
    /* Step 23 */  NOTE_SILENCE, 1,

    /* Step 24 */  NOTE_G2, 1,
    /* Step 25 */  NOTE_SILENCE, 1,
    /* Step 26 */  NOTE_SILENCE, 1,
    /* Step 27 */  NOTE_SILENCE, 1,
    /* Step 28 */  NOTE_C3, 1,
    /* Step 29 */  NOTE_SILENCE, 1,
    /* Step 30 */  NOTE_G2, 1,
    /* Step 31 */  NOTE_SILENCE, 1,
};

/* Drum pattern: bits represent drum type
 *   0x01 = kick (noise long period)
 *   0x02 = snare (noise short period)
 *   0x04 = hihat (noise short, low vol)
 */
static const uint8_t bgm_noise_data[] = {
    /* Step  0 */  0x01, 1,  /* Kick */
    /* Step  1 */  0x04, 1,  /* Hihat */
    /* Step  2 */  0x00, 1,
    /* Step  3 */  0x04, 1,  /* Hihat */
    /* Step  4 */  0x02, 1,  /* Snare */
    /* Step  5 */  0x04, 1,  /* Hihat */
    /* Step  6 */  0x00, 1,
    /* Step  7 */  0x04, 1,  /* Hihat */

    /* Step  8 */  0x01, 1,  /* Kick */
    /* Step  9 */  0x04, 1,  /* Hihat */
    /* Step 10 */  0x00, 1,
    /* Step 11 */  0x04, 1,  /* Hihat */
    /* Step 12 */  0x02, 1,  /* Snare */
    /* Step 13 */  0x04, 1,  /* Hihat */
    /* Step 14 */  0x04, 1,  /* Hihat */
    /* Step 15 */  0x04, 1,  /* Hihat */

    /* Step 16 */  0x01, 1,  /* Kick */
    /* Step 17 */  0x04, 1,  /* Hihat */
    /* Step 18 */  0x00, 1,
    /* Step 19 */  0x04, 1,  /* Hihat */
    /* Step 20 */  0x02, 1,  /* Snare */
    /* Step 21 */  0x04, 1,  /* Hihat */
    /* Step 22 */  0x00, 1,
    /* Step 23 */  0x00, 1,

    /* Step 24 */  0x01, 1,  /* Kick */
    /* Step 25 */  0x04, 1,  /* Hihat */
    /* Step 26 */  0x02, 1,  /* Snare */
    /* Step 27 */  0x04, 1,  /* Hihat */
    /* Step 28 */  0x00, 1,
    /* Step 29 */  0x04, 1,  /* Hihat */
    /* Step 30 */  0x00, 1,
    /* Step 31 */  0x04, 1,  /* Hihat */
};

#define BGM_PATTERN_LENGTH  32
#define BGM_DATA_SIZE       (BGM_PATTERN_LENGTH * 2)  /* [note, dur] pairs */

#ifdef __cplusplus
}
#endif
