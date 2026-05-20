/* opcodec/pns.c — Perceptual Noise Substitution implementation
 *
 * Implements PNS (Perceptual Noise Substitution) for audio coding.
 * Identifies noise-like bands through spectral flatness analysis
 * and substitutes them with parametric noise to save bitrate.
 *
 * Key algorithms:
 *   - Spectral flatness measurement (geometric/arithmetic mean ratio)
 *   - Linear congruential generator for noise synthesis
 *   - Energy-scaled noise reconstruction
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/pns.h"
#include <math.h>
#include <string.h>

/* Math helpers */
__attribute__((unused))
static inline float fabsf_safe(float x)
{
    return x < 0.0f ? -x : x;
}

/* Linear Congruential Generator constants (same as Numerical Recipes) */
#define LCG_A 1664525U
#define LCG_C 1013904223U

/* Generate next pseudorandom value using LCG */
static inline uint32_t lcg_next(uint32_t *rng)
{
    *rng = (*rng) * LCG_A + LCG_C;
    return *rng;
}

/* Convert uint32 to float in range [-1, 1] */
static inline float uint32_to_float(uint32_t x)
{
    /* Convert to signed range and normalize */
    int32_t signed_x = (int32_t)x;
    return (float)signed_x / (float)(1U << 31);
}

void pns_init(pns_ctx_t *ctx)
{
    if (!ctx) return;

    /* Clear all state */
    memset(ctx, 0, sizeof(*ctx));

    /* Seed RNG with a fixed value for reproducibility
     * In a real codec, this might be frame-dependent */
    ctx->rng = 12345;
}

void pns_analyze(pns_ctx_t *ctx,
                 const float *mdct, int num_coeffs,
                 const band_range_t *bands, int num_bands)
{
    if (!ctx || !mdct || !bands || num_bands <= 0 || num_bands > PNS_MAX_BANDS) {
        return;
    }

    ctx->num_bands = (uint8_t)num_bands;

    /* Pre-pass: find peak band energy for threshold gating.
     * Bands more than 40 dB below the peak are sub-masking MDCT sidelobes —
     * their uniform amplitude makes them look noise-like to GM/AM, but they
     * are deterministic artefacts of the window, not real noise. */
    float max_band_energy = 1e-20f;
    for (int band = 0; band < num_bands; band++) {
        const band_range_t *b = &bands[band];
        if (b->start >= b->end || b->end > num_coeffs) continue;
        float e = 0.0f;
        for (int i = b->start; i < b->end; i++) e += mdct[i] * mdct[i];
        if (e > max_band_energy) max_band_energy = e;
    }
    /* 40 dB gate: 10^(-40/10) = 1e-4 */
    float energy_gate = max_band_energy * 1e-4f;

    for (int band = 0; band < num_bands; band++) {
        const band_range_t *b = &bands[band];

        /* Validate band range */
        if (b->start >= b->end || b->end > num_coeffs) {
            ctx->is_noise[band] = false;
            ctx->noise_energy[band] = 0.0f;
            continue;
        }

        /* Compute band energy first — used for both gating and noise_energy. */
        float band_energy = 0.0f;
        for (int i = b->start; i < b->end; i++)
            band_energy += mdct[i] * mdct[i];

        /* Gate: sub-threshold bands are silence or window sidelobes, not noise. */
        if (band_energy < energy_gate) {
            ctx->is_noise[band] = false;
            ctx->noise_energy[band] = 0.0f;
            continue;
        }

        /* Compute GM and AM of coefficient magnitudes in log space.
         *
         * Accumulating the product directly (GM = (∏x)^(1/n)) overflows
         * or underflows for band sizes ≥ 16, making all bands look tonal.
         * Log-domain computation avoids that entirely.
         *
         * For uniform noise:  GM/AM ≈ 2/e ≈ 0.74 (above threshold → NOISE)
         * For tonal content:  few large + many near-zero → GM ≈ 0 → TONAL
         */
        float am_sum = 0.0f;
        float log_sum = 0.0f;
        float max_coeff_sq = 0.0f;
        int valid_coeffs = 0;

        for (int i = b->start; i < b->end; i++) {
            float mag = mdct[i] < 0.0f ? -mdct[i] : mdct[i];
            float sq = mag * mag;
            if (sq > max_coeff_sq) max_coeff_sq = sq;
            if (mag > 1e-6f) {
                am_sum += mag;
                log_sum += logf(mag);
                valid_coeffs++;
            }
        }

        if (valid_coeffs <= 1) {
            /* Zero or one above-threshold coefficient → tonal or silence. */
            ctx->is_noise[band] = false;
            ctx->noise_energy[band] = 0.0f;
            continue;
        }

        float am = am_sum / valid_coeffs;
        float gm = expf(log_sum / valid_coeffs);

        /* Spectral flatness: GM/AM in (0,1]; 1 = perfectly flat (noise) */
        float spectral_flatness = (am > 1e-12f) ? (gm / am) : 0.0f;

        /* Concentration check: a tone at a non-integer MDCT bin spreads energy
         * across 2-3 adjacent bins, producing deceptively high flatness.
         * If a single coefficient holds > 50% of the band energy, it is tonal. */
        bool tonal = (max_coeff_sq > 0.5f * band_energy);

        /* Mark as noise only if flat AND no dominant peak */
        ctx->is_noise[band] = !tonal && (spectral_flatness > PNS_TONALITY_THRESHOLD);

        ctx->noise_energy[band] = ctx->is_noise[band] ? sqrtf(band_energy) : 0.0f;
    }
}

void pns_zero_noise_bands(const pns_ctx_t *ctx,
                          float *mdct, int num_coeffs,
                          const band_range_t *bands, int num_bands)
{
    if (!ctx || !mdct || !bands || num_bands != ctx->num_bands) {
        return;
    }

    for (int band = 0; band < num_bands; band++) {
        if (!ctx->is_noise[band]) continue;

        const band_range_t *b = &bands[band];

        /* Validate band range */
        if (b->start >= b->end || b->end > num_coeffs) {
            continue;
        }

        /* Zero out coefficients in noise band */
        for (int i = b->start; i < b->end; i++) {
            mdct[i] = 0.0f;
        }
    }
}

void pns_fill_noise(pns_ctx_t *ctx,
                    float *mdct, int num_coeffs,
                    const band_range_t *bands, int num_bands)
{
    if (!ctx || !mdct || !bands || num_bands != ctx->num_bands) {
        return;
    }

    for (int band = 0; band < num_bands; band++) {
        if (!ctx->is_noise[band]) continue;

        const band_range_t *b = &bands[band];

        /* Validate band range */
        if (b->start >= b->end || b->end > num_coeffs) {
            continue;
        }

        float target_energy = ctx->noise_energy[band];

        if (target_energy < 1e-12f) {
            /* No energy to reproduce, keep zeros */
            continue;
        }

        /* Generate random coefficients */
        float noise_sum_sq = 0.0f;
        for (int i = b->start; i < b->end; i++) {
            uint32_t rand_val = lcg_next(&ctx->rng);
            float noise_val = uint32_to_float(rand_val);
            mdct[i] = noise_val;
            noise_sum_sq += noise_val * noise_val;
        }

        /* Scale to match target energy */
        if (noise_sum_sq > 1e-12f) {
            float scale = target_energy / sqrtf(noise_sum_sq);
            for (int i = b->start; i < b->end; i++) {
                mdct[i] *= scale;
            }
        }
    }
}

int pns_encode(const pns_ctx_t *ctx,
               uint8_t *out, size_t out_cap)
{
    if (!ctx || !out || out_cap == 0) {
        return -1;
    }

    int num_bands = ctx->num_bands;
    if (num_bands > PNS_MAX_BANDS) {
        return -1;
    }

    /* Calculate required space:
     * - 1 byte for num_bands
     * - ceil(num_bands/8) bytes for noise flags (1 bit per band)
     * - 8 bits (1 byte) per noise band for energy quantization
     */
    int flag_bytes = (num_bands + 7) / 8;
    int noise_bands = 0;

    /* Count noise bands */
    for (int i = 0; i < num_bands; i++) {
        if (ctx->is_noise[i]) {
            noise_bands++;
        }
    }

    int required_bytes = 1 + flag_bytes + noise_bands;
    if (out_cap < (size_t)required_bytes) {
        return -1;
    }

    uint8_t *ptr = out;

    /* Write number of bands */
    *ptr++ = (uint8_t)num_bands;

    /* Pack noise flags (1 bit per band) */
    memset(ptr, 0, flag_bytes);
    for (int i = 0; i < num_bands; i++) {
        if (ctx->is_noise[i]) {
            int byte_idx = i / 8;
            int bit_idx = i % 8;
            ptr[byte_idx] |= (1 << bit_idx);
        }
    }
    ptr += flag_bytes;

    /* Quantize and write energies for noise bands */
    for (int i = 0; i < num_bands; i++) {
        if (ctx->is_noise[i]) {
            /* Quantize energy to 8 bits (simple linear quantization)
             * Range: 0.0 to 16.0 (assuming typical energy range) */
            float energy = ctx->noise_energy[i];
            if (energy < 0.0f) energy = 0.0f;
            if (energy > 16.0f) energy = 16.0f;

            uint8_t quantized = (uint8_t)(energy * 255.0f / 16.0f + 0.5f);
            *ptr++ = quantized;
        }
    }

    return (int)(ptr - out);
}

int pns_decode(pns_ctx_t *ctx,
               const uint8_t *in, size_t in_len)
{
    if (!ctx || !in || in_len == 0) {
        return -1;
    }

    const uint8_t *ptr = in;
    const uint8_t *end = in + in_len;

    /* Read number of bands */
    if (ptr >= end) return -1;
    int num_bands = *ptr++;

    if (num_bands > PNS_MAX_BANDS) {
        return -1;
    }

    ctx->num_bands = (uint8_t)num_bands;

    /* Read noise flags */
    int flag_bytes = (num_bands + 7) / 8;
    if (ptr + flag_bytes > end) return -1;

    /* Unpack noise flags */
    for (int i = 0; i < num_bands; i++) {
        int byte_idx = i / 8;
        int bit_idx = i % 8;
        ctx->is_noise[i] = (ptr[byte_idx] & (1 << bit_idx)) != 0;
    }
    ptr += flag_bytes;

    /* Read energies for noise bands */
    for (int i = 0; i < num_bands; i++) {
        if (ctx->is_noise[i]) {
            if (ptr >= end) return -1;

            /* Dequantize energy from 8 bits */
            uint8_t quantized = *ptr++;
            float energy = ((float)quantized / 255.0f) * 16.0f;
            ctx->noise_energy[i] = energy;
        } else {
            ctx->noise_energy[i] = 0.0f;
        }
    }

    return (int)(ptr - in);
}