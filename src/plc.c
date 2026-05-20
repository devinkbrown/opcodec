/* SPDX-License-Identifier: LGPL-2.1-or-later */

#include "opcodec/plc.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Simple LCG for noise generation */
#define PLC_RNG_A 1664525
#define PLC_RNG_C 1013904223

/**
 * Update internal RNG state and return normalized float [-1.0, 1.0]
 */
static float plc_rand(plc_ctx_t *ctx)
{
    ctx->rng = ctx->rng * PLC_RNG_A + PLC_RNG_C;
    return ((float)(int32_t)ctx->rng) / 2147483648.0f;
}

/**
 * Apply triangular window for overlap-add operations
 *
 * @param samples Array of audio samples
 * @param len     Number of samples
 * @param fade_in If true, fade from 0->1, else fade from 1->0
 */
static void apply_triangular_window(float *samples, int len, bool fade_in)
{
    for (int i = 0; i < len; i++) {
        float weight = fade_in ? ((float)i / len) : (1.0f - (float)i / len);
        samples[i] *= weight;
    }
}

/**
 * Extract pitch period with slight jitter to avoid metallic artifacts
 */
static void extract_pitch_period(plc_ctx_t *ctx, float *output, int output_len,
                                 int base_pitch)
{
    /* Apply small random jitter to pitch period */
    int jitter = (int)(plc_rand(ctx) * 2.0f); /* -1, 0, or +1 samples */
    int actual_pitch = base_pitch + jitter;

    /* Clamp to reasonable bounds */
    if (actual_pitch < 20) actual_pitch = 20;
    if (actual_pitch > ctx->history_len / 2) actual_pitch = ctx->history_len / 2;

    /* Extract one pitch period from end of history */
    int start_pos = ctx->history_len - actual_pitch;
    if (start_pos < 0) start_pos = 0;

    /* Repeat the pitch period to fill output */
    int src_idx = start_pos;
    for (int i = 0; i < output_len; i++) {
        output[i] = ctx->history[src_idx];
        src_idx++;
        if (src_idx >= ctx->history_len) {
            src_idx = start_pos; /* Wrap around */
        }
    }
}

/**
 * Generate comfort noise shaped by spectral characteristics
 */
static void generate_comfort_noise(plc_ctx_t *ctx, float *output, int output_len)
{
    /* Simple approach: white noise with envelope matching */
    float energy = 0.0f;

    /* Calculate energy from last quarter of history */
    int energy_samples = ctx->history_len / 4;
    int start_pos = ctx->history_len - energy_samples;

    for (int i = start_pos; i < ctx->history_len; i++) {
        energy += ctx->history[i] * ctx->history[i];
    }
    energy = sqrtf(energy / energy_samples) * 0.3f; /* Scale down for comfort */

    /* Generate shaped noise */
    for (int i = 0; i < output_len; i++) {
        output[i] = plc_rand(ctx) * energy;
    }
}

int plc_init(plc_ctx_t *ctx, uint16_t frame_size)
{
    if (!ctx || frame_size == 0 || frame_size > PLC_MAX_FRAME) {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));

    ctx->frame_size = frame_size;
    ctx->fade_gain = 1.0f;
    ctx->rng = 12345; /* Seed with fixed value for deterministic testing */
    ctx->initialized = true;

    return 0;
}

void plc_update_good(plc_ctx_t *ctx, const float *pcm_float, int frame_size,
                     int pitch_period, float pitch_gain)
{
    if (!ctx || !ctx->initialized || !pcm_float) {
        return;
    }

    /* Shift history buffer to make room for new frame */
    int max_history = PLC_MAX_FRAME * PLC_HISTORY_FRAMES;
    if (ctx->history_len + frame_size > max_history) {
        int shift_amount = ctx->history_len + frame_size - max_history;
        memmove(ctx->history, ctx->history + shift_amount,
                (ctx->history_len - shift_amount) * sizeof(float));
        ctx->history_len -= shift_amount;
    }

    /* Append new frame to history */
    memcpy(ctx->history + ctx->history_len, pcm_float, frame_size * sizeof(float));
    ctx->history_len += frame_size;

    /* Update pitch information */
    if (pitch_period > 0 && pitch_gain > 0.5f) {
        ctx->last_pitch = pitch_period;
        ctx->last_pitch_gain = pitch_gain;
    }

    /* Reset loss tracking */
    ctx->consecutive_losses = 0;
    ctx->fade_gain = 1.0f;
    ctx->overlap_len = 0;
}

void plc_conceal(plc_ctx_t *ctx, float *pcm_float, int frame_size)
{
    if (!ctx || !ctx->initialized || !pcm_float) {
        return;
    }

    ctx->consecutive_losses++;

    /* After maximum losses, output silence */
    if (ctx->consecutive_losses > PLC_MAX_LOSSES) {
        memset(pcm_float, 0, frame_size * sizeof(float));
        return;
    }

    /* Generate concealment based on voicing */
    if (ctx->last_pitch > 0 && ctx->last_pitch_gain > 0.5f &&
        ctx->history_len >= ctx->last_pitch) {

        /* Pitch-based synthesis for voiced segments */
        extract_pitch_period(ctx, pcm_float, frame_size, ctx->last_pitch);

        /* Add low-level noise for naturalness */
        float noise_level = (1.0f - ctx->last_pitch_gain) * 0.1f;
        for (int i = 0; i < frame_size; i++) {
            pcm_float[i] += plc_rand(ctx) * noise_level;
        }

    } else {
        /* Comfort noise for unvoiced segments */
        if (ctx->history_len > 0) {
            generate_comfort_noise(ctx, pcm_float, frame_size);
        } else {
            /* No history available, use minimal noise */
            for (int i = 0; i < frame_size; i++) {
                pcm_float[i] = plc_rand(ctx) * 0.001f;
            }
        }
    }

    /* Apply fade after initial losses */
    if (ctx->consecutive_losses >= PLC_FADE_START) {
        ctx->fade_gain *= 0.85f; /* Exponential fade */
        for (int i = 0; i < frame_size; i++) {
            pcm_float[i] *= ctx->fade_gain;
        }
    }

    /* Store for potential overlap-add with next good frame */
    ctx->overlap_len = frame_size / 4; /* Quarter frame overlap */
    if (ctx->overlap_len > PLC_MAX_FRAME) {
        ctx->overlap_len = PLC_MAX_FRAME;
    }

    /* Store tail samples for overlap */
    int start_idx = frame_size - ctx->overlap_len;
    if (start_idx < 0) start_idx = 0;

    for (int i = 0; i < ctx->overlap_len; i++) {
        ctx->overlap_buf[i] = pcm_float[start_idx + i];
    }
}

void plc_overlap_add(plc_ctx_t *ctx, float *pcm_float, int frame_size)
{
    if (!ctx || !ctx->initialized || !pcm_float || ctx->overlap_len == 0) {
        return;
    }

    /* Cross-fade between concealment tail and new good frame */
    int blend_len = ctx->overlap_len;
    if (blend_len > frame_size) {
        blend_len = frame_size;
    }

    /* Create fade windows */
    float *temp_old = ctx->overlap_buf;
    float *temp_new = malloc(blend_len * sizeof(float));

    if (!temp_new) {
        /* Fallback: simple replacement without smooth transition */
        ctx->overlap_len = 0;
        return;
    }

    /* Copy samples to fade */
    memcpy(temp_new, pcm_float, blend_len * sizeof(float));

    /* Apply cross-fade windows */
    apply_triangular_window(temp_old, blend_len, false); /* Fade out old */
    apply_triangular_window(temp_new, blend_len, true);  /* Fade in new */

    /* Blend the samples */
    for (int i = 0; i < blend_len; i++) {
        pcm_float[i] = temp_old[i] + temp_new[i];
    }

    free(temp_new);
    ctx->overlap_len = 0;
}