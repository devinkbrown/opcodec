/*
 * Voice/Music Detection for OpCodec
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 *
 * Lightweight content classification to adapt codec behavior:
 * - Voice: more pitch filtering, lower bitrate
 * - Music: wider bandwidth, less pitch processing
 */

#ifndef OPCODEC_VMD_H
#define OPCODEC_VMD_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Content type classification */
typedef enum {
    VMD_VOICE   = 0,  /* Speech/vocals - enable pitch filtering */
    VMD_MUSIC   = 1,  /* Music - wider bandwidth, less pitch processing */
    VMD_MIXED   = 2,  /* Mixed content - blend settings */
    VMD_SILENCE = 3   /* Silence/low energy - comfort noise */
} vmd_content_t;

/* Classifier state */
typedef struct {
    /* Feature history for smoothing (exponential moving average) */
    float    avg_zcr;           /* average zero-crossing rate */
    float    avg_flatness;      /* average spectral flatness */
    float    avg_centroid;      /* average spectral centroid (normalized 0-1) */
    float    avg_pitch_corr;    /* average pitch correlation strength */
    float    avg_energy_var;    /* energy variance across recent frames */

    /* Energy history for variance computation */
    float    energy_history[16]; /* last 16 frame energies */
    uint8_t  history_idx;
    uint8_t  history_count;

    /* Smoothed classification probability */
    float    voice_prob;        /* probability of voice [0,1] */

    /* Configuration */
    uint32_t sample_rate;

    bool     initialized;
} vmd_ctx_t;

/* Initialize the classifier */
void vmd_init(vmd_ctx_t *ctx, uint32_t sample_rate);

/* Classify a frame of audio.
 *
 * Parameters:
 *   ctx          — classifier state
 *   pcm_float    — pre-emphasized float samples for this frame
 *   frame_size   — number of samples
 *   mdct         — MDCT coefficients (if available, NULL otherwise)
 *   num_coeffs   — number of MDCT coefficients
 *   pitch_corr   — pitch correlation from pitch detector (0-1), or -1 if unavailable
 *   frame_energy — energy of this frame
 *
 * Returns the content type classification.
 */
vmd_content_t vmd_classify(vmd_ctx_t *ctx,
                           const float *pcm_float, int frame_size,
                           const float *mdct, int num_coeffs,
                           float pitch_corr, float frame_energy);

/* Get the current voice probability [0,1].
 * Useful for soft blending between voice and music codec parameters. */
float vmd_voice_probability(const vmd_ctx_t *ctx);

/* Reset classifier state (e.g., on channel change) */
void vmd_reset(vmd_ctx_t *ctx);

#endif /* OPCODEC_VMD_H */