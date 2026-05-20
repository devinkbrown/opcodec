/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef OPCODEC_PLC_H
#define OPCODEC_PLC_H

#include <stdint.h>
#include <stdbool.h>

#define PLC_MAX_FRAME     960
#define PLC_HISTORY_FRAMES 3    /* keep 3 frames of history */
#define PLC_MAX_LOSSES     10   /* max consecutive losses before mute */
#define PLC_FADE_START     3    /* start fading after N losses */

/**
 * Packet Loss Concealment context
 *
 * Maintains state for pitch-based interpolation and graceful degradation
 * during packet loss events in audio streams.
 */
typedef struct {
    /* Time-domain history for pitch-based synthesis */
    float history[PLC_MAX_FRAME * PLC_HISTORY_FRAMES];
    int   history_len;

    /* Pitch from last good frame */
    int   last_pitch;         /* period in samples */
    float last_pitch_gain;    /* correlation strength */

    /* Fade state */
    uint8_t consecutive_losses;
    float   fade_gain;        /* current fade multiplier */

    /* Phase alignment for overlap-add */
    float overlap_buf[PLC_MAX_FRAME];
    int   overlap_len;

    /* RNG for noise injection */
    uint32_t rng;

    uint16_t frame_size;
    bool     initialized;
} plc_ctx_t;

/**
 * Initialize PLC context
 *
 * @param ctx        PLC context to initialize
 * @param frame_size Audio frame size in samples
 * @return 0 on success, negative on error
 */
int plc_init(plc_ctx_t *ctx, uint16_t frame_size);

/**
 * Update PLC state with a successfully decoded frame
 *
 * Call this after every successful decode to update history and pitch info.
 * Resets loss counters and updates the history buffer.
 *
 * @param ctx         PLC context
 * @param pcm_float   Time-domain audio samples
 * @param frame_size  Number of samples in frame
 * @param pitch_period Pitch period in samples (0 if unvoiced)
 * @param pitch_gain  Pitch correlation strength [0.0, 1.0]
 */
void plc_update_good(plc_ctx_t *ctx, const float *pcm_float, int frame_size,
                     int pitch_period, float pitch_gain);

/**
 * Generate concealment frame for packet loss
 *
 * Creates a synthetic audio frame using pitch-based interpolation for voiced
 * segments or comfort noise for unvoiced segments. Applies progressive fading
 * for extended loss periods.
 *
 * @param ctx        PLC context
 * @param pcm_float  Output buffer for concealed audio
 * @param frame_size Number of samples to generate
 */
void plc_conceal(plc_ctx_t *ctx, float *pcm_float, int frame_size);

/**
 * Smooth transition when good frame arrives after concealment
 *
 * Applies cross-fade between concealment tail and new good frame to avoid
 * audible artifacts at loss recovery boundaries.
 *
 * @param ctx        PLC context
 * @param pcm_float  Good frame to blend (modified in-place)
 * @param frame_size Number of samples in frame
 */
void plc_overlap_add(plc_ctx_t *ctx, float *pcm_float, int frame_size);

#endif /* OPCODEC_PLC_H */