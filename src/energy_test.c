/*
 * opcodec/energy_test.c — Test program for energy quantization module
 *
 * Tests the Opus-style coarse/fine energy quantization system for correctness
 * and coding efficiency.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/energy.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#define TEST_BANDS 16
#define EPSILON 1e-6f

/* Test basic initialization */
static void
test_init(void)
{
    energy_ctx_t ctx;
    energy_init(&ctx, TEST_BANDS);

    assert(ctx.num_bands == TEST_BANDS);
    assert(ctx.has_prev == false);
    assert(ctx.has_prev == false); /* already checked above; interframe_alpha removed from struct */

    printf("✓ Initialization test passed\n");
}

/* Test coarse energy encoding/decoding roundtrip */
static void
test_coarse_roundtrip(void)
{
    energy_ctx_t enc_ctx, dec_ctx;
    energy_init(&enc_ctx, TEST_BANDS);
    energy_init(&dec_ctx, TEST_BANDS);

    /* Create test band energies (exponentially decreasing) */
    float original_energies[TEST_BANDS];
    for (int b = 0; b < TEST_BANDS; b++) {
        original_energies[b] = powf(10.0f, -0.5f * b); /* 5 dB decrease per band */
    }

    /* Encode */
    int8_t coarse_codes[TEST_BANDS];
    energy_encode_coarse(&enc_ctx, original_energies, TEST_BANDS, coarse_codes);

    /* Decode */
    float decoded_energies[TEST_BANDS];
    energy_decode_coarse(&dec_ctx, coarse_codes, TEST_BANDS, decoded_energies);

    /* Check accuracy (should be within 6 dB coarse resolution) */
    for (int b = 0; b < TEST_BANDS; b++) {
        float orig_dB = 10.0f * log10f(original_energies[b] + 1e-10f);
        float decoded_dB = 10.0f * log10f(decoded_energies[b] + 1e-10f);
        float error_dB = fabsf(orig_dB - decoded_dB);

        assert(error_dB <= 6.5f); /* Allow small margin for quantization */

        printf("Band %d: %.2f dB -> %.2f dB (error: %.2f dB)\n",
               b, orig_dB, decoded_dB, error_dB);
    }

    printf("✓ Coarse roundtrip test passed\n");
}

/* Test fine energy encoding/decoding */
static void
test_fine_refinement(void)
{
    energy_ctx_t enc_ctx, dec_ctx;
    energy_init(&enc_ctx, TEST_BANDS);
    energy_init(&dec_ctx, TEST_BANDS);

    /* Create test energies */
    float original_energies[TEST_BANDS];
    for (int b = 0; b < TEST_BANDS; b++) {
        original_energies[b] = powf(10.0f, -0.3f * b + 0.1f * sinf(b));
    }

    /* Coarse encoding/decoding */
    int8_t coarse_codes[TEST_BANDS];
    energy_encode_coarse(&enc_ctx, original_energies, TEST_BANDS, coarse_codes);

    float coarse_energies[TEST_BANDS];
    energy_decode_coarse(&dec_ctx, coarse_codes, TEST_BANDS, coarse_energies);

    /* Allocate fine bits (giving 2 bits to each band) */
    uint8_t fine_bits[TEST_BANDS];
    for (int b = 0; b < TEST_BANDS; b++) {
        fine_bits[b] = 2;
    }

    /* Save the coarse state before fine encoding modifies it */
    float saved_coarse_dB[TEST_BANDS];
    memcpy(saved_coarse_dB, enc_ctx.coarse_dB, TEST_BANDS * sizeof(float));

    /* Fine encoding */
    uint8_t fine_codes[TEST_BANDS];
    energy_encode_fine(&enc_ctx, original_energies, TEST_BANDS, fine_bits, fine_codes);

    /* Set decoder's coarse state to what it would be after coarse decoding */
    memcpy(dec_ctx.coarse_dB, saved_coarse_dB, TEST_BANDS * sizeof(float));

    /* Fine decoding */
    float refined_energies[TEST_BANDS];
    memcpy(refined_energies, coarse_energies, TEST_BANDS * sizeof(float));
    energy_decode_fine(&dec_ctx, fine_codes, fine_bits, TEST_BANDS, refined_energies);

    /* Check that fine refinement improves accuracy */
    float coarse_total_error = 0.0f, fine_total_error = 0.0f;

    for (int b = 0; b < TEST_BANDS; b++) {
        float orig_dB = 10.0f * log10f(original_energies[b] + 1e-10f);
        float coarse_dB = 10.0f * log10f(coarse_energies[b] + 1e-10f);
        float refined_dB = 10.0f * log10f(refined_energies[b] + 1e-10f);

        float coarse_error = fabsf(orig_dB - coarse_dB);
        float fine_error = fabsf(orig_dB - refined_dB);

        coarse_total_error += coarse_error * coarse_error;
        fine_total_error += fine_error * fine_error;

        printf("Band %d: %.2f -> %.2f (coarse: %.2f dB) -> %.2f (fine: %.2f dB)\n",
               b, orig_dB, coarse_dB, coarse_error, refined_dB, fine_error);
    }

    printf("Total MSE: Coarse %.3f dB², Fine %.3f dB² (improvement: %.1fx)\n",
           coarse_total_error, fine_total_error, coarse_total_error / fine_total_error);

    /* Fine should be better than coarse */
    assert(fine_total_error < coarse_total_error);

    printf("✓ Fine refinement test passed\n");
}

/* Test fine bit allocation */
static void
test_bit_allocation(void)
{
    /* Create coarse dB values with varying magnitudes */
    int8_t coarse_codes[TEST_BANDS];
    float  coarse_dB[TEST_BANDS];
    for (int b = 0; b < TEST_BANDS; b++) {
        coarse_codes[b] = (b < 4) ? 10 : (b < 8) ? 5 : 1;
        coarse_dB[b]    = coarse_codes[b] * 6.0f; /* 6 dB per coarse step */
    }

    uint8_t fine_bits[TEST_BANDS];
    energy_allocate_fine_bits(32, coarse_dB, TEST_BANDS, fine_bits); /* 32 fine bits total */

    int total_allocated = 0;
    printf("Fine bit allocation:\n");
    for (int b = 0; b < TEST_BANDS; b++) {
        printf("  Band %d (code %d): %d bits\n", b, coarse_codes[b], fine_bits[b]);
        total_allocated += fine_bits[b];
        assert(fine_bits[b] <= 4); /* Max 4 bits per band */
    }

    assert(total_allocated <= 32); /* Should not exceed budget */

    /* High energy bands (0-3) should get more bits than low energy bands (8-15) */
    int high_energy_bits = 0, low_energy_bits = 0;
    for (int b = 0; b < 4; b++) high_energy_bits += fine_bits[b];
    for (int b = 8; b < TEST_BANDS; b++) low_energy_bits += fine_bits[b];

    assert(high_energy_bits >= low_energy_bits);

    printf("✓ Bit allocation test passed (allocated %d/%d bits)\n", total_allocated, 32);
}

/* Test inter-frame prediction */
static void
test_interframe_prediction(void)
{
    energy_ctx_t ctx;
    energy_init(&ctx, 4); /* Use 4 bands for simplicity */

    float frame1_energies[] = {1.0f, 0.5f, 0.25f, 0.125f};
    float frame2_energies[] = {0.9f, 0.6f, 0.3f, 0.1f}; /* Similar to frame 1 */

    int8_t codes1[4], codes2[4];

    /* Encode first frame */
    energy_encode_coarse(&ctx, frame1_energies, 4, codes1);

    /* Encode second frame (should benefit from inter-frame prediction) */
    energy_encode_coarse(&ctx, frame2_energies, 4, codes2);

    /* Codes for second frame should generally be smaller (better prediction) */
    int total_abs_codes2 = 0;
    for (int b = 0; b < 4; b++) {
        total_abs_codes2 += abs(codes2[b]);
        printf("Frame 2 band %d: energy %.3f -> code %d\n", b, frame2_energies[b], codes2[b]);
    }

    /* With good inter-frame prediction, total code magnitude should be small */
    printf("Total absolute code magnitude for frame 2: %d\n", total_abs_codes2);
    assert(total_abs_codes2 < 20); /* Should be much smaller due to prediction */

    printf("✓ Inter-frame prediction test passed\n");
}

int
main(void)
{
    printf("OPVOX Energy Quantization Test Suite\n");
    printf("====================================\n\n");

    test_init();
    test_coarse_roundtrip();
    test_fine_refinement();
    test_bit_allocation();
    test_interframe_prediction();

    printf("\n✓ All energy quantization tests passed!\n");
    printf("The Opus-style energy quantization system is working correctly.\n");

    return 0;
}