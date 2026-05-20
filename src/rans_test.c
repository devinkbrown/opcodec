/*
 * rans_test.c — Test rANS entropy coder including context-adaptive models
 *
 * Tests the basic rANS functionality and the new context-adaptive models
 * to ensure correctness and proper round-trip encoding/decoding.
 *
 * Copyright (c) 2026 Ophion Development Team.  GPL v2.
 */

#include "opcodec/rans.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* Test data patterns */
static const uint16_t test_data[] = {
    0, 1, 0, 2, 1, 0, 3, 1, 0, 0,  /* Laplace-like: many zeros, few high values */
    4, 1, 0, 5, 0, 1, 0, 0, 2, 0,
    1, 0, 0, 0, 6, 0, 1, 0, 0, 3
};
#define TEST_DATA_LEN (sizeof(test_data) / sizeof(test_data[0]))

/* Test basic rANS with Laplace model */
static int test_basic_rans(void)
{
    printf("Testing basic rANS with Laplace model...\n");

    /* Create model */
    rans_model_t model;
    rans_model_laplace(&model, 16, 3);  /* 16 symbols, decay=3 */

    /* Encode */
    uint8_t enc_buf[1024];
    rans_encoder_t enc;
    rans_enc_init(&enc, enc_buf, sizeof(enc_buf));

    for (size_t i = 0; i < TEST_DATA_LEN; i++) {
        rans_enc_put(&enc, &model, test_data[i]);
    }

    size_t compressed_len = rans_enc_flush(&enc);
    const uint8_t *compressed_data = rans_enc_data(&enc, &compressed_len);

    printf("  Compressed %zu symbols to %zu bytes (%.2f bits/symbol)\n",
           TEST_DATA_LEN, compressed_len, (compressed_len * 8.0) / TEST_DATA_LEN);

    /* Decode */
    rans_decoder_t dec;
    rans_dec_init(&dec, compressed_data, compressed_len);

    uint16_t decoded[TEST_DATA_LEN];
    for (size_t i = 0; i < TEST_DATA_LEN; i++) {
        decoded[TEST_DATA_LEN - 1 - i] = rans_dec_get(&dec, &model);  /* Decode in reverse */
    }

    /* Verify */
    for (size_t i = 0; i < TEST_DATA_LEN; i++) {
        if (decoded[i] != test_data[i]) {
            printf("  FAIL: decoded[%zu] = %u, expected %u\n", i, decoded[i], test_data[i]);
            return 0;
        }
    }

    printf("  PASS: Round-trip successful\n");
    return 1;
}

/* Test zero run model */
static int test_zero_run_model(void)
{
    printf("Testing zero run model...\n");

    rans_model_t model;
    rans_model_zero_run(&model, 8);  /* Max run length 8 */

    /* Test data: various run lengths */
    uint16_t runs[] = {0, 1, 0, 2, 0, 0, 3, 1, 0, 4};
    size_t runs_len = sizeof(runs) / sizeof(runs[0]);

    /* Encode */
    uint8_t enc_buf[256];
    rans_encoder_t enc;
    rans_enc_init(&enc, enc_buf, sizeof(enc_buf));

    for (size_t i = 0; i < runs_len; i++) {
        rans_enc_put(&enc, &model, runs[i]);
    }

    size_t compressed_len = rans_enc_flush(&enc);
    const uint8_t *compressed_data = rans_enc_data(&enc, &compressed_len);

    printf("  Compressed %zu run lengths to %zu bytes\n", runs_len, compressed_len);

    /* Decode */
    rans_decoder_t dec;
    rans_dec_init(&dec, compressed_data, compressed_len);

    uint16_t decoded[10];
    for (size_t i = 0; i < runs_len; i++) {
        decoded[runs_len - 1 - i] = rans_dec_get(&dec, &model);
    }

    /* Verify */
    for (size_t i = 0; i < runs_len; i++) {
        if (decoded[i] != runs[i]) {
            printf("  FAIL: decoded[%zu] = %u, expected %u\n", i, decoded[i], runs[i]);
            return 0;
        }
    }

    printf("  PASS: Zero run model works\n");
    return 1;
}

/* Test context-adaptive model */
static int test_adaptive_rans(void)
{
    printf("Testing context-adaptive rANS...\n");

    /* Initialize adaptive model */
    rans_adaptive_t adaptive;
    rans_adaptive_init(&adaptive, 16, 4, 3);  /* 16 symbols, 4 contexts, decay=3 */

    /* Simulate MDCT-like data: different contexts have different distributions */
    uint16_t mdct_data[] = {
        /* Context 0 (low band): mostly small values */
        0, 1, 0, 2, 1, 0, 1, 0, 3, 1,
        /* Context 1 (mid band): mixed values */
        2, 3, 1, 4, 2, 1, 5, 2, 1, 3,
        /* Context 2 (high band): very sparse, mostly zeros */
        0, 0, 0, 1, 0, 0, 0, 0, 2, 0,
        /* Context 3 (noise band): somewhat uniform */
        3, 4, 2, 5, 3, 1, 4, 2, 3, 6
    };
    uint16_t contexts[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /* Low band */
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  /* Mid band */
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2,  /* High band */
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3   /* Noise band */
    };
    size_t mdct_len = sizeof(mdct_data) / sizeof(mdct_data[0]);

    /* Encode */
    uint8_t enc_buf[1024];
    rans_encoder_t enc;
    rans_enc_init(&enc, enc_buf, sizeof(enc_buf));

    for (size_t i = 0; i < mdct_len; i++) {
        rans_adaptive_encode(&enc, &adaptive, contexts[i], mdct_data[i]);
    }

    size_t compressed_len = rans_enc_flush(&enc);
    const uint8_t *compressed_data = rans_enc_data(&enc, &compressed_len);

    printf("  Compressed %zu symbols to %zu bytes (%.2f bits/symbol)\n",
           mdct_len, compressed_len, (compressed_len * 8.0) / mdct_len);

    /* Decode with fresh adaptive model (must be initialized identically) */
    rans_adaptive_t decode_adaptive;
    rans_adaptive_init(&decode_adaptive, 16, 4, 3);

    rans_decoder_t dec;
    rans_dec_init(&dec, compressed_data, compressed_len);

    uint16_t decoded[40];
    for (size_t i = 0; i < mdct_len; i++) {
        size_t decode_idx = mdct_len - 1 - i;  /* Decode in reverse */
        decoded[decode_idx] = rans_adaptive_decode(&dec, &decode_adaptive, contexts[decode_idx]);
    }

    /* Verify */
    for (size_t i = 0; i < mdct_len; i++) {
        if (decoded[i] != mdct_data[i]) {
            printf("  FAIL: decoded[%zu] = %u, expected %u (context %u)\n",
                   i, decoded[i], mdct_data[i], contexts[i]);
            return 0;
        }
    }

    printf("  PASS: Context-adaptive round-trip successful\n");

    /* Verify that contexts adapted differently */
    printf("  Context adaptation check:\n");
    for (uint16_t ctx = 0; ctx < 4; ctx++) {
        printf("    Context %u: symbol 0 freq=%u, symbol 1 freq=%u\n",
               ctx,
               decode_adaptive.models[ctx].syms[0].freq,
               decode_adaptive.models[ctx].syms[1].freq);
    }

    return 1;
}

int main(void)
{
    printf("rANS entropy coder tests\n");
    printf("========================\n");

    int tests_passed = 0;
    int total_tests = 3;

    if (test_basic_rans()) tests_passed++;
    if (test_zero_run_model()) tests_passed++;
    if (test_adaptive_rans()) tests_passed++;

    printf("\nResults: %d/%d tests passed\n", tests_passed, total_tests);

    if (tests_passed == total_tests) {
        printf("All tests PASSED!\n");
        return 0;
    } else {
        printf("Some tests FAILED!\n");
        return 1;
    }
}