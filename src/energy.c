/*
 * opcodec/energy.c — Band energy quantization for OPVOX audio codec
 *
 * Implements Opus-style two-pass (coarse/fine) energy quantization system
 * that provides 2-4 dB coding gain over simple approaches.
 *
 * The key insight: band energies are the most important side information
 * in a transform codec, so getting them right matters more than individual
 * coefficient accuracy. The two-pass approach allows optimal bit allocation.
 *
 * Coarse pass: Fixed 6 dB resolution with inter-band/inter-frame prediction
 * Fine pass: Variable resolution refinement using leftover bits
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/energy.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ENERGY_EPSILON 1e-10f    /* small value to avoid log(0) */
#define MIN_DB -90.0f            /* minimum dB value */
#define MAX_DB 90.0f             /* maximum dB value */
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* Initialize energy quantization context */
void
energy_init(energy_ctx_t *ctx, uint16_t num_bands)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->num_bands = num_bands;
    ctx->has_prev = false;

    /* Initialize previous energies to silence level */
    for (uint16_t b = 0; b < num_bands; b++) {
        ctx->prev_energy_dB[b] = MIN_DB;
        ctx->coarse_dB[b] = MIN_DB;
    }
}

/* Convert linear energy to dB, clamped to reasonable range */
static float
energy_to_dB(float energy)
{
    if (energy <= ENERGY_EPSILON)
        return MIN_DB;

    float dB = 10.0f * log10f(energy);
    return CLAMP(dB, MIN_DB, MAX_DB);
}

/* Convert dB back to linear energy */
static float
dB_to_energy(float dB)
{
    if (dB <= MIN_DB)
        return ENERGY_EPSILON;

    return powf(10.0f, dB / 10.0f);
}

/* Coarse energy quantization (encoder side) */
int
energy_encode_coarse(energy_ctx_t *ctx,
                     const float *band_energy, uint16_t num_bands,
                     int8_t *coarse_codes)
{
    int total_bits = 0;

    for (uint16_t b = 0; b < num_bands; b++) {
        /* Convert to dB */
        float e_dB = energy_to_dB(band_energy[b]);

        /* Prediction.
         *
         * First frame (no prior): use inter-band spectral smoothness.
         *   - Band 0: predict 0 (encode full energy from scratch)
         *   - Band b>0: predict from band b-1's reconstructed energy
         *
         * Subsequent frames: use pure temporal prediction.
         *   - For stationary signals: residual ≈ 0 (code=0) → exact reconstruction
         *   - For narrow-band signals (e.g. pure tone): temporal beats inter-band
         *     because the active band may have a silent neighbor band
         *   - prev_energy_dB is set via energy_commit() AFTER fine refinement,
         *     so subsequent frames benefit from fine-corrected accuracy */
        float pred;
        if (!ctx->has_prev) {
            pred = (b == 0) ? 0.0f : ctx->coarse_dB[b - 1];
        } else {
            pred = ctx->prev_energy_dB[b];
        }

        /* Compute residual and quantize to 6 dB steps */
        float residual = e_dB - pred;
        int8_t code = (int8_t)roundf(residual / ENERGY_COARSE_RES);

        /* Clamp to representable range for 7-bit signed */
        if (code > 63) code = 63;
        if (code < -64) code = -64;

        coarse_codes[b] = code;

        /* Reconstruct for prediction chain */
        ctx->coarse_dB[b] = pred + code * ENERGY_COARSE_RES;

        /* Each coarse code takes 7 bits (range -64 to +63) */
        total_bits += 7;
    }

    /* prev_energy_dB is updated via energy_commit() after fine refinement
     * so that subsequent frames predict from fine-corrected values. */

    return total_bits;
}

/* Fine energy refinement (encoder side) */
int
energy_encode_fine(energy_ctx_t *ctx,
                   const float *band_energy, uint16_t num_bands,
                   const uint8_t *fine_bits,
                   uint8_t *fine_codes)
{
    int total_bits = 0;

    for (uint16_t b = 0; b < num_bands; b++) {
        if (fine_bits[b] == 0) {
            fine_codes[b] = 0;
            continue;
        }

        float e_dB = energy_to_dB(band_energy[b]);
        float coarse = ctx->coarse_dB[b];
        float residual = e_dB - coarse;

        /* Quantize residual with fine resolution: 6 / 2^B dB steps */
        float fine_step = ENERGY_COARSE_RES / (1 << fine_bits[b]);
        int code = (int)roundf((residual / fine_step) + (1 << (fine_bits[b] - 1)));

        /* Clamp to [0, 2^B - 1] */
        int max_code = (1 << fine_bits[b]) - 1;
        if (code < 0) code = 0;
        if (code > max_code) code = max_code;

        fine_codes[b] = (uint8_t)code;

        /* Update coarse with refinement for future prediction */
        ctx->coarse_dB[b] = coarse + (code - (1 << (fine_bits[b] - 1))) * fine_step;

        total_bits += fine_bits[b];
    }

    return total_bits;
}

/* Coarse energy decoding (decoder side) */
void
energy_decode_coarse(energy_ctx_t *ctx,
                     const int8_t *coarse_codes, uint16_t num_bands,
                     float *band_energy)
{
    for (uint16_t b = 0; b < num_bands; b++) {
        float pred;
        if (!ctx->has_prev) {
            pred = (b == 0) ? 0.0f : ctx->coarse_dB[b - 1];
        } else {
            pred = ctx->prev_energy_dB[b];
        }

        /* Reconstruct dB value */
        ctx->coarse_dB[b] = pred + coarse_codes[b] * ENERGY_COARSE_RES;

        /* Convert back to linear energy */
        band_energy[b] = dB_to_energy(ctx->coarse_dB[b]);
    }

    /* prev_energy_dB is updated via energy_commit() after fine refinement. */
}

/* Fine energy decoding (decoder side) */
void
energy_decode_fine(energy_ctx_t *ctx,
                   const uint8_t *fine_codes, const uint8_t *fine_bits,
                   uint16_t num_bands,
                   float *band_energy)
{
    for (uint16_t b = 0; b < num_bands; b++) {
        if (fine_bits[b] == 0)
            continue;

        float coarse = ctx->coarse_dB[b];

        /* Decode fine refinement */
        float fine_step = ENERGY_COARSE_RES / (1 << fine_bits[b]);
        float refinement = (fine_codes[b] - (1 << (fine_bits[b] - 1))) * fine_step;

        /* Apply refinement */
        ctx->coarse_dB[b] = coarse + refinement;

        /* Convert back to linear energy */
        band_energy[b] = dB_to_energy(ctx->coarse_dB[b]);
    }
}

/* Allocate fine bits across bands based on perceptual importance.
 *
 * Uses reconstructed coarse_dB values: higher-energy bands matter more
 * perceptually and benefit most from precise energy representation. */
void
energy_allocate_fine_bits(uint16_t total_fine_bits,
                          const float *coarse_dB,
                          uint16_t num_bands,
                          uint8_t *fine_bits)
{
    memset(fine_bits, 0, num_bands);

    uint16_t remaining = total_fine_bits;

    while (remaining > 0) {
        int best_b = -1;
        float best_score = -1.0f;

        for (uint16_t b = 0; b < num_bands; b++) {
            /* Cap at 5 bits: 6dB / 32 = 0.1875 dB precision, beyond this
             * the coefficient quantization noise dominates */
            if (fine_bits[b] >= 5)
                continue;

            /* Base score: absolute band energy (dB above floor).
             * High-energy bands contain more signal and errors are more audible. */
            float score = coarse_dB[b] - MIN_DB;
            if (score < 0.0f) score = 0.0f;

            /* Low-frequency bands are perceptually most sensitive */
            if (b < num_bands / 4)
                score *= 1.25f;
            else if (b < num_bands / 2)
                score *= 1.1f;

            /* Diminishing returns: each additional bit halves the residual error,
             * but also halves the marginal gain — penalize exponentially */
            if (fine_bits[b] > 0)
                score *= powf(0.65f, (float)fine_bits[b]);

            if (score > best_score) {
                best_score = score;
                best_b = b;
            }
        }

        if (best_b < 0)
            break;

        fine_bits[best_b]++;
        remaining--;
    }
}

/* Commit current frame's (fine-refined) coarse_dB as the temporal prediction
 * reference for the next frame.  Call this after energy_encode/decode_fine. */
void
energy_commit(energy_ctx_t *ctx)
{
    for (uint16_t b = 0; b < ctx->num_bands; b++) {
        ctx->prev_energy_dB[b] = ctx->coarse_dB[b];
    }
    ctx->has_prev = true;
}

/* Reset context (e.g., on channel change or codec reset) */
void
energy_reset(energy_ctx_t *ctx)
{
    ctx->has_prev = false;

    /* Reset to silence level */
    for (uint16_t b = 0; b < ctx->num_bands; b++) {
        ctx->prev_energy_dB[b] = MIN_DB;
        ctx->coarse_dB[b] = MIN_DB;
    }
}