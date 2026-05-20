/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "opcodec/plc.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

/* Generate a test sine wave */
static void generate_sine_wave(float *output, int samples, float freq, float sample_rate)
{
    for (int i = 0; i < samples; i++) {
        output[i] = 0.5f * sinf(2.0f * M_PI * freq * i / sample_rate);
    }
}

/* Calculate RMS energy of a signal */
static float calculate_rms(const float *signal, int samples)
{
    float sum = 0.0f;
    for (int i = 0; i < samples; i++) {
        sum += signal[i] * signal[i];
    }
    return sqrtf(sum / samples);
}

/* Test basic PLC initialization and cleanup */
static void test_plc_init(void)
{
    plc_ctx_t ctx;
    int result;

    printf("Testing PLC initialization...\n");

    /* Test normal initialization */
    result = plc_init(&ctx, 480);
    assert(result == 0);
    assert(ctx.initialized == true);
    assert(ctx.frame_size == 480);
    assert(ctx.fade_gain == 1.0f);
    assert(ctx.consecutive_losses == 0);

    /* Test invalid parameters */
    result = plc_init(NULL, 480);
    assert(result == -1);

    result = plc_init(&ctx, 0);
    assert(result == -1);

    result = plc_init(&ctx, PLC_MAX_FRAME + 1);
    assert(result == -1);

    printf("PLC initialization tests passed!\n");
}

/* Test good frame updates and history management */
static void test_plc_good_frame_update(void)
{
    plc_ctx_t ctx;
    float test_frame[480];

    printf("Testing good frame updates...\n");

    plc_init(&ctx, 480);

    /* Generate test audio frames */
    generate_sine_wave(test_frame, 480, 440.0f, 48000.0f);

    /* Test first frame update */
    plc_update_good(&ctx, test_frame, 480, 100, 0.8f);
    assert(ctx.history_len == 480);
    assert(ctx.last_pitch == 100);
    assert(ctx.last_pitch_gain == 0.8f);
    assert(ctx.consecutive_losses == 0);

    /* Test second frame update */
    generate_sine_wave(test_frame, 480, 880.0f, 48000.0f);
    plc_update_good(&ctx, test_frame, 480, 90, 0.9f);
    assert(ctx.history_len == 960);
    assert(ctx.last_pitch == 90);
    assert(ctx.last_pitch_gain == 0.9f);

    /* Test history buffer overflow */
    for (int i = 0; i < 5; i++) {
        generate_sine_wave(test_frame, 480, 220.0f + i * 100.0f, 48000.0f);
        plc_update_good(&ctx, test_frame, 480, 80 + i, 0.7f);
    }

    /* Should be capped at max history */
    assert(ctx.history_len <= PLC_MAX_FRAME * PLC_HISTORY_FRAMES);

    printf("Good frame update tests passed!\n");
}

/* Test concealment for voiced speech */
static void test_plc_voiced_concealment(void)
{
    plc_ctx_t ctx;
    float test_frame[480];
    float concealed[480];

    printf("Testing voiced concealment...\n");

    plc_init(&ctx, 480);

    /* Setup history with periodic signal */
    generate_sine_wave(test_frame, 480, 100.0f, 48000.0f);
    plc_update_good(&ctx, test_frame, 480, 480, 0.9f); /* Strong pitch */

    /* Perform concealment */
    plc_conceal(&ctx, concealed, 480);

    /* Verify concealment has reasonable energy */
    float concealed_rms = calculate_rms(concealed, 480);
    assert(concealed_rms > 0.01f);  /* Should have significant energy */
    assert(concealed_rms < 1.0f);   /* But not clipping */

    /* Verify consecutive loss counter increased */
    assert(ctx.consecutive_losses == 1);

    printf("Voiced concealment tests passed!\n");
}

/* Test concealment for unvoiced speech */
static void test_plc_unvoiced_concealment(void)
{
    plc_ctx_t ctx;
    float test_frame[480];
    float concealed[480];

    printf("Testing unvoiced concealment...\n");

    plc_init(&ctx, 480);

    /* Setup history with noise-like signal */
    for (int i = 0; i < 480; i++) {
        test_frame[i] = ((float)rand() / (float)RAND_MAX - 0.5f) * 0.1f;
    }
    plc_update_good(&ctx, test_frame, 480, 0, 0.2f); /* Weak/no pitch */

    /* Perform concealment */
    plc_conceal(&ctx, concealed, 480);

    /* Verify concealment produces some output */
    float concealed_rms = calculate_rms(concealed, 480);
    assert(concealed_rms > 0.001f); /* Should have some energy */

    printf("Unvoiced concealment tests passed!\n");
}

/* Test progressive fading during extended loss */
static void test_plc_fading(void)
{
    plc_ctx_t ctx;
    float test_frame[480];
    float concealed1[480], concealed2[480], concealed_late[480];

    printf("Testing progressive fading...\n");

    plc_init(&ctx, 480);

    /* Setup history */
    generate_sine_wave(test_frame, 480, 100.0f, 48000.0f);
    plc_update_good(&ctx, test_frame, 480, 100, 0.8f);

    /* First few concealments should have similar energy */
    plc_conceal(&ctx, concealed1, 480);
    plc_conceal(&ctx, concealed2, 480);

    float rms1 = calculate_rms(concealed1, 480);
    float rms2 = calculate_rms(concealed2, 480);

    /* Should be similar before fading starts */
    assert(fabsf(rms1 - rms2) / rms1 < 0.5f);

    /* Perform several more concealments to trigger fading */
    for (int i = 0; i < 5; i++) {
        plc_conceal(&ctx, concealed_late, 480);
    }

    float rms_late = calculate_rms(concealed_late, 480);

    /* Late concealment should have lower energy due to fading */
    assert(rms_late < rms1 * 0.8f);

    printf("Progressive fading tests passed!\n");
}

/* Test overlap-add for smooth transitions */
static void test_plc_overlap_add(void)
{
    plc_ctx_t ctx;
    float test_frame[480];
    float concealed[480], good_frame[480];

    printf("Testing overlap-add transitions...\n");

    plc_init(&ctx, 480);

    /* Setup and perform concealment */
    generate_sine_wave(test_frame, 480, 100.0f, 48000.0f);
    plc_update_good(&ctx, test_frame, 480, 100, 0.8f);
    plc_conceal(&ctx, concealed, 480);

    /* Verify overlap buffer is set up */
    assert(ctx.overlap_len > 0);

    /* Create good frame and apply overlap-add */
    generate_sine_wave(good_frame, 480, 110.0f, 48000.0f);
    plc_overlap_add(&ctx, good_frame, 480);

    /* Overlap length should be cleared after use */
    assert(ctx.overlap_len == 0);

    printf("Overlap-add tests passed!\n");
}

/* Test maximum loss handling (silence after max losses) */
static void test_plc_max_losses(void)
{
    plc_ctx_t ctx;
    float test_frame[480];
    float concealed[480];

    printf("Testing maximum loss handling...\n");

    plc_init(&ctx, 480);

    /* Setup history */
    generate_sine_wave(test_frame, 480, 100.0f, 48000.0f);
    plc_update_good(&ctx, test_frame, 480, 100, 0.8f);

    /* Exceed maximum losses */
    for (int i = 0; i < PLC_MAX_LOSSES + 2; i++) {
        plc_conceal(&ctx, concealed, 480);
    }

    /* After max losses, should output silence */
    float rms_final = calculate_rms(concealed, 480);
    assert(rms_final < 0.001f); /* Essentially silence */

    printf("Maximum loss handling tests passed!\n");
}

int main(void)
{
    printf("=== OPCODEC PLC Module Tests ===\n\n");

    test_plc_init();
    test_plc_good_frame_update();
    test_plc_voiced_concealment();
    test_plc_unvoiced_concealment();
    test_plc_fading();
    test_plc_overlap_add();
    test_plc_max_losses();

    printf("\n=== All PLC tests passed! ===\n");
    return 0;
}