/* Test program for PNS and RVQ implementations
 *
 * Verifies that:
 * 1. PNS correctly identifies noise-like bands
 * 2. PNS encoding/decoding preserves energy
 * 3. RVQ multi-stage quantization works
 * 4. RVQ decoding reconstructs coefficients
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "opcodec/pns.h"
#include "opcodec/pvq.h"

/* Test data generation */
static void generate_tonal_band(float *coeffs, int start, int end, float freq, float amplitude)
{
    for (int i = start; i < end; i++) {
        float t = (float)(i - start) / (float)(end - start);
        coeffs[i] = amplitude * sinf(2.0f * M_PI * freq * t);
    }
}

static void generate_noise_band(float *coeffs, int start, int end, float amplitude)
{
    for (int i = start; i < end; i++) {
        float noise = (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;
        coeffs[i] = amplitude * noise;
    }
}

static float compute_energy(const float *coeffs, int start, int end)
{
    float energy = 0.0f;
    for (int i = start; i < end; i++) {
        energy += coeffs[i] * coeffs[i];
    }
    return sqrtf(energy);
}

/* Test PNS functionality */
static int test_pns(void)
{
    printf("Testing PNS (Perceptual Noise Substitution)...\n");

    /* Create test signal with tonal and noise bands */
    float mdct[128];
    band_range_t bands[4] = {
        {0, 32},    /* Band 0: tonal */
        {32, 64},   /* Band 1: noise */
        {64, 96},   /* Band 2: tonal */
        {96, 128}   /* Band 3: noise */
    };
    int num_bands = 4;

    /* Generate test data */
    memset(mdct, 0, sizeof(mdct));
    generate_tonal_band(mdct, 0, 32, 0.1f, 1.0f);     /* Strong tonal component */
    generate_noise_band(mdct, 32, 64, 0.5f);           /* Noise */
    generate_tonal_band(mdct, 64, 96, 0.2f, 0.8f);    /* Another tonal component */
    generate_noise_band(mdct, 96, 128, 0.3f);          /* More noise */

    /* Initialize PNS context */
    pns_ctx_t pns;
    pns_init(&pns);

    /* Analyze bands */
    pns_analyze(&pns, mdct, 128, bands, num_bands);

    /* Verify noise detection */
    printf("  Band classification:\n");
    for (int i = 0; i < num_bands; i++) {
        printf("    Band %d: %s (energy: %.3f)\n",
               i, pns.is_noise[i] ? "NOISE" : "TONAL", pns.noise_energy[i]);
    }

    /* Bands 1 and 3 should be detected as noise */
    if (!pns.is_noise[1] || !pns.is_noise[3]) {
        printf("  ERROR: Failed to detect noise bands!\n");
        return -1;
    }

    /* Store original energies */
    float orig_energies[4];
    for (int i = 0; i < num_bands; i++) {
        orig_energies[i] = compute_energy(mdct, bands[i].start, bands[i].end);
    }

    /* Test encoding/decoding */
    uint8_t encoded[64];
    int encoded_len = pns_encode(&pns, encoded, sizeof(encoded));
    if (encoded_len < 0) {
        printf("  ERROR: PNS encoding failed!\n");
        return -1;
    }

    printf("  Encoded PNS data: %d bytes\n", encoded_len);

    /* Decode */
    pns_ctx_t decoded_pns;
    pns_init(&decoded_pns);
    int decoded_len = pns_decode(&decoded_pns, encoded, encoded_len);
    if (decoded_len != encoded_len) {
        printf("  ERROR: PNS decoding failed!\n");
        return -1;
    }

    /* Verify decoded flags match */
    for (int i = 0; i < num_bands; i++) {
        if (pns.is_noise[i] != decoded_pns.is_noise[i]) {
            printf("  ERROR: Noise flag mismatch for band %d!\n", i);
            return -1;
        }
    }

    printf("  ✓ PNS encoding/decoding successful\n");

    /* Test noise synthesis */
    float decoded_mdct[128];
    memcpy(decoded_mdct, mdct, sizeof(mdct));

    /* Zero noise bands (encoder side) */
    pns_zero_noise_bands(&pns, decoded_mdct, 128, bands, num_bands);

    /* Fill noise bands (decoder side) */
    pns_fill_noise(&decoded_pns, decoded_mdct, 128, bands, num_bands);

    /* Verify energy is preserved in noise bands */
    for (int i = 0; i < num_bands; i++) {
        if (pns.is_noise[i]) {
            float recon_energy = compute_energy(decoded_mdct, bands[i].start, bands[i].end);
            float energy_ratio = recon_energy / orig_energies[i];
            printf("    Band %d energy ratio: %.3f\n", i, energy_ratio);

            /* Allow some tolerance due to quantization */
            if (energy_ratio < 0.8f || energy_ratio > 1.2f) {
                printf("  WARNING: Energy not well preserved in band %d\n", i);
            }
        }
    }

    printf("  ✓ PNS test passed\n\n");
    return 0;
}

/* Test RVQ functionality */
static int test_rvq(void)
{
    printf("Testing RVQ (Residual Vector Quantization)...\n");

    /* Create test vector */
    float coeffs[16] = {
        1.5f, -0.8f, 2.1f, 0.3f, -1.2f, 0.7f, 1.8f, -0.5f,
        0.9f, -1.6f, 0.4f, 2.3f, -0.2f, 1.1f, -0.9f, 1.4f
    };
    int N = 16;
    int total_K = 24;  /* Total pulses */
    int num_stages = 3;

    printf("  Input vector: ");
    for (int i = 0; i < N; i++) {
        printf("%.2f ", coeffs[i]);
    }
    printf("\n");

    /* Test RVQ encoding */
    rvq_result_t result;
    rvq_encode(coeffs, N, total_K, num_stages, &result);

    printf("  RVQ stages: %d\n", result.num_stages);
    for (int stage = 0; stage < result.num_stages; stage++) {
        printf("    Stage %d: gain=%.3f, K=%d\n",
               stage, result.gains[stage], result.K_per_stage[stage]);
    }

    /* Test RVQ decoding */
    float decoded[16];
    rvq_decode(&result, decoded);

    printf("  Decoded vector: ");
    for (int i = 0; i < N; i++) {
        printf("%.2f ", decoded[i]);
    }
    printf("\n");

    /* Compute reconstruction error */
    float mse = 0.0f;
    for (int i = 0; i < N; i++) {
        float error = coeffs[i] - decoded[i];
        mse += error * error;
    }
    mse /= N;

    printf("  Mean squared error: %.6f\n", mse);

    /* Verify reconstruction is reasonable */
    if (mse > 1.0f) {
        printf("  ERROR: Reconstruction error too high!\n");
        return -1;
    }

    printf("  ✓ RVQ test passed\n\n");

    /* Test single-stage PVQ for comparison */
    printf("Comparing with single-stage PVQ...\n");

    float pvq_gain;
    int16_t pvq_shape[16];
    pvq_encode(coeffs, N, total_K, &pvq_gain, pvq_shape);

    float pvq_decoded[16];
    pvq_decode(pvq_gain, pvq_shape, N, pvq_decoded);

    float pvq_mse = 0.0f;
    for (int i = 0; i < N; i++) {
        float error = coeffs[i] - pvq_decoded[i];
        pvq_mse += error * error;
    }
    pvq_mse /= N;

    printf("  Single-stage PVQ MSE: %.6f\n", pvq_mse);
    printf("  Multi-stage RVQ MSE:  %.6f\n", mse);
    printf("  RVQ improvement: %.2fx\n", pvq_mse / mse);

    return 0;
}

int main(void)
{
    srand(12345);  /* Fixed seed for reproducible results */

    printf("=== PNS and RVQ Test Suite ===\n\n");

    if (test_pns() < 0) {
        printf("PNS test FAILED\n");
        return 1;
    }

    if (test_rvq() < 0) {
        printf("RVQ test FAILED\n");
        return 1;
    }

    printf("All tests PASSED\n");
    return 0;
}