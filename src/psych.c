/*
 * opcodec/psych.c — Psychoacoustic Masking Model (Johnston spreading function)
 *
 * Copyright (c) 2026 Ophion Development Team.  GPL v2.
 */

#include "opcodec/psych.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Bark-scale conversion ─────────────────────────────────────────────────
 *
 * Bark(f) = 13·atan(0.00076f) + 3.5·atan((f/7500)²)
 *
 * This is Zwicker's formula (1961), which is more accurate than linear
 * approximations for the full audible frequency range.
 */
static float hz_to_bark(float hz)
{
    float f_kHz = hz / 1000.0f;
    return 13.0f * atanf(0.76f * f_kHz) +
           3.5f  * atanf((f_kHz / 7.5f) * (f_kHz / 7.5f));
}

/* ── Johnston spreading function ───────────────────────────────────────────
 *
 * SF(dz) = 15.811389 + 7.5*(dz + 0.474) - 17.5*sqrt(1 + (dz + 0.474)^2)
 *
 * where dz = z_maskee - z_masker  (positive = maskee is above masker in freq).
 *
 * Returns the spreading gain in dB.  Positive values mean the masker raises
 * the threshold at the maskee frequency.  Negative values (far from masker)
 * mean negligible contribution.
 * Clamped to [-40, 0] dB to prevent negative thresholds from dominating.
 */
static float spreading_function(float dz)
{
    float shifted = dz + 0.474f;
    float sf = 15.811389f + 7.5f * shifted -
               17.5f * sqrtf(1.0f + shifted * shifted);
    if (sf > 0.0f) sf = 0.0f;
    if (sf < -40.0f) sf = -40.0f;
    return sf;
}

/* ── Spectral Flatness Measure (SFM) ────────────────────────────────────────
 *
 * SFM_dB = 10·log10(geometric_mean / arithmetic_mean)
 *
 * Range: 0 dB (pure tone) to -∞ dB (white noise).
 * We clamp to [-60, 0] dB and derive tonality coefficient α:
 *   α = min(1.0, -SFM_dB / 60.0)    0 = noise, 1 = tonal
 */
static float band_tonality(const float *band_energy_dB, int b_start, int b_end)
{
    /* band_energy_dB is per-band (already integrated), so use relative
     * variation between adjacent bands as proxy for tonality.
     * A tonal band typically has energy significantly above its neighbors.
     * A noise-like band has similar energy to its neighbors. */
    (void)b_start; (void)b_end;

    /* Simple tonality proxy: compare band to its immediate neighbors.
     * Delta > 10 dB above both neighbors → tonal.
     * Delta < 3 dB → noise-like. */
    float center = band_energy_dB[0];
    float left   = (b_start > 0) ? band_energy_dB[-1] : center;
    float right  = (b_end   > 0) ? band_energy_dB[ 1] : center;

    float above_left  = center - left;
    float above_right = center - right;
    float peak_excess = (above_left < above_right) ? above_left : above_right;

    /* Map: peak_excess >= 10 dB → tonal (α=1), peak_excess <= 0 dB → noise (α=0) */
    float alpha = peak_excess / 10.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    return alpha;
}

/* ── psych_init ─────────────────────────────────────────────────────────── */

void psych_init(psych_ctx_t *ctx,
                const uint16_t *band_starts, const uint16_t *band_ends,
                int n_bands, uint32_t sample_rate)
{
    if (!ctx || !band_starts || !band_ends || n_bands <= 0) return;
    memset(ctx, 0, sizeof(*ctx));

    float bin_hz = (float)sample_rate / (2.0f * (float)
        /* total MDCT bins = frame_samples; use band_ends[n_bands-1] as proxy */
        (band_ends[n_bands - 1]));

    /* Compute Bark center frequency for each band */
    for (int b = 0; b < n_bands && b < PSYCH_MAX_BANDS; b++) {
        float center_bin = 0.5f * (float)(band_starts[b] + band_ends[b]);
        float center_hz  = center_bin * bin_hz;
        ctx->bark[b] = hz_to_bark(center_hz);
    }

    ctx->n_bands     = (n_bands < PSYCH_MAX_BANDS) ? n_bands : PSYCH_MAX_BANDS;
    ctx->initialized = true;
}

/* ── psych_analyze ──────────────────────────────────────────────────────── */

void psych_analyze(const psych_ctx_t *ctx,
                   const float *band_energy_dB,
                   const float *ath_dB,
                   int n_bands,
                   psych_result_t *result)
{
    if (!ctx || !ctx->initialized || !band_energy_dB || !result) return;
    if (n_bands <= 0 || n_bands > ctx->n_bands) n_bands = ctx->n_bands;

    /* Step 1: compute tonality per band and masker excitation level */
    float masker_dB[PSYCH_MAX_BANDS];
    for (int m = 0; m < n_bands; m++) {
        float alpha = band_tonality(band_energy_dB + m,
                                    m, n_bands - 1 - m);
        result->tonality[m] = alpha;

        /* Masker excitation level: signal minus tonality-weighted offset.
         * Tonal masker: masked N dB below masker (high masking power → more offset).
         * Noise masker: masked only 6 dB below masker. */
        float offset = PSYCH_NOISE_OFFSET +
                       alpha * (PSYCH_TONAL_OFFSET - PSYCH_NOISE_OFFSET);
        masker_dB[m] = band_energy_dB[m] - offset;
    }

    /* Step 2: compute masking threshold = max over spreading from all maskers.
     * For each maskee band n, accumulate contributions from all masker bands m. */
    for (int n = 0; n < n_bands; n++) {
        float threshold = (ath_dB != NULL) ? ath_dB[n] : -60.0f;

        for (int m = 0; m < n_bands; m++) {
            float dz = ctx->bark[n] - ctx->bark[m];  /* positive if n > m */
            float sf = spreading_function(dz);
            float contribution = masker_dB[m] + sf;
            if (contribution > threshold) threshold = contribution;
        }

        result->mask_dB[n] = threshold;
    }

    /* Step 3: SMR = signal energy - masking threshold */
    for (int n = 0; n < n_bands; n++) {
        result->smr_dB[n] = band_energy_dB[n] - result->mask_dB[n];
    }
}
