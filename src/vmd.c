/*
 * Voice/Music Detection for OpCodec
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 *
 * Lightweight content classification using simple signal features:
 * - Zero-crossing rate (ZCR)
 * - Spectral flatness (from MDCT coefficients)
 * - Spectral centroid
 * - Pitch correlation strength
 * - Energy variance over time
 */

#include "opcodec/vmd.h"
#include <math.h>
#include <string.h>

/* Constants for classification */
#define VMD_SILENCE_THRESHOLD   0.001f  /* Below this energy = silence */
#define VMD_ALPHA              0.1f     /* EMA smoothing factor */
#define VMD_VOICE_THRESHOLD    0.6f     /* Above this prob = voice */
#define VMD_MUSIC_THRESHOLD    0.3f     /* Below this prob = music */

/* Feature limits for normalization */
#define VMD_MAX_ZCR            0.5f     /* Maximum expected ZCR */
#define VMD_MIN_ENERGY         1e-10f   /* Minimum energy to prevent log(0) */

void vmd_init(vmd_ctx_t *ctx, uint32_t sample_rate)
{
    memset(ctx, 0, sizeof(*ctx));

    ctx->sample_rate = sample_rate;
    ctx->voice_prob = 0.5f;  /* Start neutral */
    ctx->initialized = true;

    /* Initialize moving averages to neutral values */
    ctx->avg_zcr = 0.1f;
    ctx->avg_flatness = 0.5f;
    ctx->avg_centroid = 0.5f;
    ctx->avg_pitch_corr = 0.0f;
    ctx->avg_energy_var = 1.0f;
}

void vmd_reset(vmd_ctx_t *ctx)
{
    if (!ctx->initialized) return;

    uint32_t sr = ctx->sample_rate;
    vmd_init(ctx, sr);
}

/* Compute zero-crossing rate */
static float compute_zcr(const float *pcm, int frame_size)
{
    int zcr_count = 0;

    for (int i = 1; i < frame_size; i++) {
        if ((pcm[i] >= 0.0f) != (pcm[i-1] >= 0.0f)) {
            zcr_count++;
        }
    }

    return (float)zcr_count / frame_size;
}

/* Compute spectral flatness from MDCT coefficients */
static float compute_spectral_flatness(const float *mdct, int num_coeffs)
{
    if (!mdct || num_coeffs <= 0) return 0.5f;  /* Neutral if no MDCT */

    float log_sum = 0.0f;
    float lin_sum = 0.0f;

    for (int i = 0; i < num_coeffs; i++) {
        float mag = fabsf(mdct[i]) + VMD_MIN_ENERGY;
        log_sum += log2f(mag);
        lin_sum += mag;
    }

    float geo_mean = exp2f(log_sum / num_coeffs);
    float arith_mean = lin_sum / num_coeffs;

    /* Spectral flatness: 0 = tonal/voice, 1 = flat/noise */
    return geo_mean / (arith_mean + VMD_MIN_ENERGY);
}

/* Compute spectral centroid (normalized 0-1) */
static float compute_spectral_centroid(const float *mdct, int num_coeffs)
{
    if (!mdct || num_coeffs <= 1) return 0.5f;  /* Neutral if no MDCT */

    float weighted_sum = 0.0f;
    float total_energy = 0.0f;

    for (int i = 0; i < num_coeffs; i++) {
        float mag = fabsf(mdct[i]);
        weighted_sum += (float)i * mag;
        total_energy += mag;
    }

    if (total_energy <= VMD_MIN_ENERGY) return 0.5f;

    /* Normalize to [0,1] where 0 = low frequencies, 1 = high frequencies */
    return weighted_sum / (total_energy * (num_coeffs - 1));
}

/* Update energy variance tracking */
static void update_energy_variance(vmd_ctx_t *ctx, float frame_energy)
{
    /* Store log energy in circular buffer */
    ctx->energy_history[ctx->history_idx] = log2f(frame_energy + VMD_MIN_ENERGY);
    ctx->history_idx = (ctx->history_idx + 1) % 16;

    if (ctx->history_count < 16) {
        ctx->history_count++;
    }

    /* Compute variance of log energies */
    float mean_log_e = 0.0f;
    for (int i = 0; i < ctx->history_count; i++) {
        mean_log_e += ctx->energy_history[i];
    }
    mean_log_e /= ctx->history_count;

    float variance = 0.0f;
    for (int i = 0; i < ctx->history_count; i++) {
        float diff = ctx->energy_history[i] - mean_log_e;
        variance += diff * diff;
    }
    variance /= ctx->history_count;

    /* Update moving average */
    ctx->avg_energy_var = (1.0f - VMD_ALPHA) * ctx->avg_energy_var +
                          VMD_ALPHA * variance;
}

/* Compute voice probability from features */
static float compute_voice_score(const vmd_ctx_t *ctx)
{
    float score = 0.0f;

    /* Strong pitch correlation indicates voice (+0.3) */
    if (ctx->avg_pitch_corr > 0.5f) {
        score += 0.3f;
    } else if (ctx->avg_pitch_corr > 0.3f) {
        score += 0.15f;
    }

    /* Low spectral flatness indicates tonal content = voice (+0.25) */
    if (ctx->avg_flatness < 0.3f) {
        score += 0.25f;
    } else if (ctx->avg_flatness < 0.5f) {
        score += 0.1f;
    }

    /* Low spectral centroid indicates energy below 4kHz = voice (+0.2) */
    if (ctx->avg_centroid < 0.3f) {
        score += 0.2f;
    } else if (ctx->avg_centroid < 0.5f) {
        score += 0.1f;
    }

    /* Moderate ZCR is typical for voice (+0.15) */
    if (ctx->avg_zcr > 0.05f && ctx->avg_zcr < 0.2f) {
        score += 0.15f;
    }

    /* High energy variance indicates talk/silence patterns = voice (+0.1) */
    if (ctx->avg_energy_var > 2.0f) {
        score += 0.1f;
    }

    return score;
}

vmd_content_t vmd_classify(vmd_ctx_t *ctx,
                           const float *pcm_float, int frame_size,
                           const float *mdct, int num_coeffs,
                           float pitch_corr, float frame_energy)
{
    if (!ctx || !ctx->initialized || !pcm_float || frame_size <= 0) {
        return VMD_SILENCE;
    }

    /* Check for silence first */
    if (frame_energy < VMD_SILENCE_THRESHOLD) {
        return VMD_SILENCE;
    }

    /* Compute features for this frame */
    float zcr = compute_zcr(pcm_float, frame_size);
    float flatness = compute_spectral_flatness(mdct, num_coeffs);
    float centroid = compute_spectral_centroid(mdct, num_coeffs);

    /* Normalize ZCR */
    zcr = fminf(zcr / VMD_MAX_ZCR, 1.0f);

    /* Update moving averages with exponential smoothing */
    ctx->avg_zcr = (1.0f - VMD_ALPHA) * ctx->avg_zcr + VMD_ALPHA * zcr;
    ctx->avg_flatness = (1.0f - VMD_ALPHA) * ctx->avg_flatness + VMD_ALPHA * flatness;
    ctx->avg_centroid = (1.0f - VMD_ALPHA) * ctx->avg_centroid + VMD_ALPHA * centroid;

    /* Update pitch correlation if available */
    if (pitch_corr >= 0.0f) {
        ctx->avg_pitch_corr = (1.0f - VMD_ALPHA) * ctx->avg_pitch_corr +
                              VMD_ALPHA * pitch_corr;
    }

    /* Update energy variance tracking */
    update_energy_variance(ctx, frame_energy);

    /* Compute voice probability */
    float voice_score = compute_voice_score(ctx);

    /* Smooth the voice probability with strong momentum to prevent flickering */
    ctx->voice_prob = 0.9f * ctx->voice_prob + 0.1f * voice_score;

    /* Classify based on smoothed probability */
    if (ctx->voice_prob > VMD_VOICE_THRESHOLD) {
        return VMD_VOICE;
    } else if (ctx->voice_prob < VMD_MUSIC_THRESHOLD) {
        return VMD_MUSIC;
    } else {
        return VMD_MIXED;
    }
}

float vmd_voice_probability(const vmd_ctx_t *ctx)
{
    if (!ctx || !ctx->initialized) {
        return 0.5f;  /* Neutral if uninitialized */
    }

    return ctx->voice_prob;
}