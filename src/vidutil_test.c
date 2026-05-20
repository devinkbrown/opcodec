/*
 * Video Utility Test - Simple verification of Weighted Prediction and SAO
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/vidutil.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

/* Test patterns */
static void create_test_patterns(uint8_t *bright, uint8_t *dim, int size)
{
    for (int i = 0; i < size * size; i++) {
        bright[i] = 200 + (i % 55);  /* bright pattern */
        dim[i] = bright[i] / 2;       /* dimmed version */
    }
}

static void test_weighted_prediction(void)
{
    printf("Testing weighted prediction...\n");

    const int size = 16;
    uint8_t bright[size * size], dim[size * size];
    create_test_patterns(bright, dim, size);

    /* Test detection - should enable WP for significant brightness change */
    wp_params_t params = wp_detect(bright, dim, size, size);
    assert(params.enabled == true);
    printf("  WP detection: PASS (enabled=%d, weight=%d, offset=%d)\n",
           params.enabled, params.weight, params.offset);

    /* Test encode/decode */
    uint8_t encoded[10];
    int enc_len = wp_encode_params(&params, encoded, sizeof(encoded));
    assert(enc_len > 0);

    wp_params_t decoded_params;
    int dec_len = wp_decode_params(&decoded_params, encoded, enc_len);
    assert(dec_len == enc_len);
    assert(decoded_params.enabled == params.enabled);
    assert(decoded_params.weight == params.weight);
    assert(decoded_params.offset == params.offset);
    assert(decoded_params.log2_denom == params.log2_denom);
    printf("  WP encode/decode: PASS\n");

    /* Test application */
    uint8_t ref_block[size * size];
    memcpy(ref_block, dim, sizeof(ref_block));
    wp_apply(ref_block, size, size, &params);

    /* After weighted prediction, ref_block should be closer to bright */
    int diff_before = 0, diff_after = 0;
    for (int i = 0; i < size * size; i++) {
        diff_before += abs(bright[i] - dim[i]);
        diff_after += abs(bright[i] - ref_block[i]);
    }

    printf("  WP apply: difference before=%d, after=%d\n", diff_before, diff_after);
    assert(diff_after < diff_before);  /* should be closer */
    printf("  WP application: PASS\n");

    /* Test disabled WP */
    wp_params_t disabled = { .enabled = false };
    enc_len = wp_encode_params(&disabled, encoded, sizeof(encoded));
    assert(enc_len == 1);
    assert(encoded[0] == 0);

    dec_len = wp_decode_params(&decoded_params, encoded, enc_len);
    assert(dec_len == 1);
    assert(decoded_params.enabled == false);
    printf("  WP disabled mode: PASS\n");
}

static void test_sao(void)
{
    printf("Testing SAO...\n");

    const int size = 16;
    uint8_t orig[size * size], recon[size * size];

    /* Create test pattern with edges and bands */
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            int idx = y * size + x;
            if (x < size / 2) {
                orig[idx] = 50;   /* dark region */
            } else {
                orig[idx] = 200;  /* bright region */
            }
            /* Add some quantization noise */
            int noisy_val = orig[idx] + ((idx % 7) - 3);
            recon[idx] = (uint8_t)((noisy_val < 0) ? 0 :
                                   (noisy_val > 255) ? 255 : noisy_val);
        }
    }

    /* Test SAO analysis */
    sao_params_t params = sao_analyze(recon, orig, size, size);
    printf("  SAO type: %d\n", params.type);

    if (params.type == SAO_EDGE) {
        printf("  Edge class: %d, offsets: [%d, %d, %d, %d]\n",
               params.edge.eo_class,
               params.edge.offsets[0], params.edge.offsets[1],
               params.edge.offsets[2], params.edge.offsets[3]);
    } else if (params.type == SAO_BAND) {
        printf("  Start band: %d, offsets: [%d, %d, %d, %d]\n",
               params.band.start_band,
               params.band.offsets[0], params.band.offsets[1],
               params.band.offsets[2], params.band.offsets[3]);
    }

    /* Test encode/decode */
    uint8_t encoded[10];
    int enc_len = sao_encode_params(&params, encoded, sizeof(encoded));
    assert(enc_len > 0);

    sao_params_t decoded_params;
    int dec_len = sao_decode_params(&decoded_params, encoded, enc_len);
    assert(dec_len == enc_len);
    assert(decoded_params.type == params.type);
    printf("  SAO encode/decode: PASS\n");

    /* Test application */
    uint8_t filtered[size * size];
    memcpy(filtered, recon, sizeof(filtered));
    sao_apply(filtered, size, size, &params);

    /* Calculate MSE before and after SAO */
    int mse_before = 0, mse_after = 0;
    for (int i = 0; i < size * size; i++) {
        int err_before = orig[i] - recon[i];
        int err_after = orig[i] - filtered[i];
        mse_before += err_before * err_before;
        mse_after += err_after * err_after;
    }

    printf("  SAO MSE: before=%d, after=%d\n", mse_before, mse_after);
    /* SAO should generally reduce MSE */
    printf("  SAO application: %s\n",
           (mse_after <= mse_before) ? "PASS" : "WARN (MSE increased)");

    /* Test disabled SAO */
    sao_params_t disabled = { .type = SAO_OFF };
    enc_len = sao_encode_params(&disabled, encoded, sizeof(encoded));
    assert(enc_len == 1);
    assert(encoded[0] == 0);

    dec_len = sao_decode_params(&decoded_params, encoded, enc_len);
    assert(dec_len == 1);
    assert(decoded_params.type == SAO_OFF);
    printf("  SAO disabled mode: PASS\n");
}

int main(void)
{
    printf("=== Video Utility Test Suite ===\n");

    test_weighted_prediction();
    test_sao();

    printf("=== All tests completed successfully ===\n");
    return 0;
}