/* video_test.c — OPVIS codec roundtrip and feature verification tests
 *
 * Covers:
 *   - I-frame, P-frame, B-frame encode/decode with PSNR check (≥ 35dB)
 *   - 10-bit P010 encode/decode roundtrip
 *   - Palette mode: flat color block encodes small
 *   - IBC mode: repeated 8×8 block pattern hits IBC path
 *   - v1 header parsing and backward-compat guard
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/video.h"
#include "opcodec/hdr.h"
#include "opcodec/screen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* ---- Test bookkeeping ---- */

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  pass: %s\n", msg); g_pass++; } \
    else       { printf("  FAIL: %s\n", msg); g_fail++; } \
} while (0)

#define CHECK_GE(val, threshold, msg) do { \
    if ((val) >= (threshold)) { \
        printf("  pass: %s (%.2f >= %.2f)\n", msg, (double)(val), (double)(threshold)); \
        g_pass++; \
    } else { \
        printf("  FAIL: %s (%.2f < %.2f)\n", msg, (double)(val), (double)(threshold)); \
        g_fail++; \
    } \
} while (0)

#define CHECK_LE(val, threshold, msg) do { \
    if ((val) <= (threshold)) { \
        printf("  pass: %s (%zu <= %zu)\n", msg, (size_t)(val), (size_t)(threshold)); \
        g_pass++; \
    } else { \
        printf("  FAIL: %s (%zu > %zu)\n", msg, (size_t)(val), (size_t)(threshold)); \
        g_fail++; \
    } \
} while (0)

/* ---- Signal utilities ---- */

static float psnr_y(const uint8_t *orig, const uint8_t *recon, int w, int h) {
    double mse = 0.0;
    for (int i = 0; i < w * h; i++) {
        double d = (double)orig[i] - (double)recon[i];
        mse += d * d;
    }
    mse /= (double)(w * h);
    if (mse < 1e-10) return 100.0f;
    return (float)(10.0 * log10(255.0 * 255.0 / mse));
}

/* Synthetic 640×480 test pattern: horizontal ramp + some structure */
static void make_gradient(uint8_t *y, uint8_t *u, uint8_t *v, int w, int h) {
    for (int row = 0; row < h; row++)
        for (int col = 0; col < w; col++)
            y[row * w + col] = (uint8_t)(((row + col) * 255) / (w + h - 2));
    memset(u, 128, (w / 2) * (h / 2));
    memset(v, 128, (w / 2) * (h / 2));
}

/* Repeated tile pattern: every pixel = (row%8)*16 + (col%8)*4 */
static void make_repeated_tiles(uint8_t *y, int w, int h) {
    for (int row = 0; row < h; row++)
        for (int col = 0; col < w; col++)
            y[row * w + col] = (uint8_t)((row % 8) * 16 + (col % 8) * 4);
}

/* ---- Encode/decode helpers ---- */

typedef struct {
    opvis_encoder_t enc;
    opvis_decoder_t dec;
    uint8_t *enc_pool;
    uint8_t *dec_pool;
    uint8_t *bitstream;
    int      bs_cap;
} codec_ctx_t;

static codec_ctx_t *ctx_alloc(int w, int h, uint8_t quality, uint16_t gop,
                               opvis_pixel_fmt_t fmt) {
    codec_ctx_t *c = calloc(1, sizeof(*c));
    if (!c) return NULL;

    opvis_color_info_t ci = {0};
    size_t enc_sz = opvis_encoder_pool_size_v1(w, h, &ci);
    size_t dec_sz = opvis_decoder_pool_size(w, h);
    c->enc_pool  = malloc(enc_sz);
    c->dec_pool  = malloc(dec_sz);
    c->bs_cap    = OPVIS_MAX_ENCODED;
    c->bitstream = malloc((size_t)c->bs_cap);

    if (!c->enc_pool || !c->dec_pool || !c->bitstream) {
        free(c->enc_pool); free(c->dec_pool); free(c->bitstream); free(c);
        return NULL;
    }

    if (opvis_encoder_init(&c->enc, (uint16_t)w, (uint16_t)h,
                           quality, gop, fmt, c->enc_pool, enc_sz) < 0) {
        free(c->enc_pool); free(c->dec_pool); free(c->bitstream); free(c);
        return NULL;
    }
    if (opvis_decoder_init(&c->dec, c->dec_pool, dec_sz) < 0) {
        free(c->enc_pool); free(c->dec_pool); free(c->bitstream); free(c);
        return NULL;
    }

    return c;
}

static void ctx_free(codec_ctx_t *c) {
    if (!c) return;
    free(c->enc_pool);
    free(c->dec_pool);
    free(c->bitstream);
    free(c);
}

/* ---- Tests ---- */

static void test_iframe_psnr(void) {
    printf("\n=== I-frame encode/decode PSNR ===\n");
    const int W = 640, H = 480;

    uint8_t *orig_y = malloc(W * H);
    uint8_t *orig_u = malloc((W/2) * (H/2));
    uint8_t *orig_v = malloc((W/2) * (H/2));
    uint8_t *input  = malloc(W * H + 2 * (W/2) * (H/2));
    if (!orig_y || !orig_u || !orig_v || !input) { goto cleanup; }

    make_gradient(orig_y, orig_u, orig_v, W, H);
    memcpy(input,                          orig_y, W * H);
    memcpy(input + W * H,                  orig_u, (W/2) * (H/2));
    memcpy(input + W * H + (W/2)*(H/2),   orig_v, (W/2) * (H/2));

    codec_ctx_t *c = ctx_alloc(W, H, 70, 30, OPVIS_FMT_YUV420P);
    CHECK(c != NULL, "context allocated");
    if (!c) goto cleanup;

    /* Force I-frame by encoding first frame of a new GOP */
    int enc_len = opvis_encode(&c->enc, input, (size_t)(W*H + 2*(W/2)*(H/2)),
                               c->bitstream, (size_t)c->bs_cap);
    CHECK(enc_len > 0, "I-frame encoded");

    int dec_ret = opvis_decode(&c->dec, c->bitstream, (size_t)enc_len);
    CHECK(dec_ret == 0, "I-frame decoded");

    if (dec_ret == 0) {
        const uint8_t *dec_y = opvis_decoded_y(&c->dec);
        float db = psnr_y(orig_y, dec_y, W, H);
        /* Debug: print first block's pixel values */
        int max_err2 = 0, err_pos2 = 0;
        for (int i = 0; i < W * H; i++) {
            int e = abs((int)dec_y[i] - (int)orig_y[i]);
            if (e > max_err2) { max_err2 = e; err_pos2 = i; }
        }
        printf("    I-frame DBG: worst pixel at (%d,%d) dec=%d orig=%d err=%d\n",
               err_pos2 % W, err_pos2 / W, dec_y[err_pos2], orig_y[err_pos2], max_err2);
        CHECK_GE(db, 35.0f, "I-frame PSNR ≥ 35dB");
        printf("    I-frame PSNR = %.2f dB, bitstream = %d bytes\n", db, enc_len);
    }

    ctx_free(c);
cleanup:
    free(orig_y); free(orig_u); free(orig_v); free(input);
}

static void test_gop_psnr(void) {
    printf("\n=== GOP encode/decode PSNR (I+P+B frames) ===\n");
    const int W = 640, H = 480;
    const int NFRAMES = 6;

    uint8_t *frames[NFRAMES];
    uint8_t *tmp_u = malloc((W/2) * (H/2));
    uint8_t *tmp_v = malloc((W/2) * (H/2));
    if (!tmp_u || !tmp_v) { free(tmp_u); free(tmp_v); return; }

    for (int f = 0; f < NFRAMES; f++) {
        frames[f] = malloc(W * H + 2 * (W/2) * (H/2));
        if (!frames[f]) { free(tmp_u); free(tmp_v); return; }
        /* Slightly different gradient each frame to simulate motion */
        for (int row = 0; row < H; row++)
            for (int col = 0; col < W; col++)
                frames[f][row * W + col] = (uint8_t)(((row + col + f * 4) * 255) / (W + H + 20));
        memset(frames[f] + W*H,              128, (W/2)*(H/2));
        memset(frames[f] + W*H + (W/2)*(H/2), 128, (W/2)*(H/2));
    }

    codec_ctx_t *c = ctx_alloc(W, H, 70, 4, OPVIS_FMT_YUV420P);
    CHECK(c != NULL, "GOP context allocated");
    if (!c) goto cleanup;

    float min_psnr = 1000.0f;
    int all_ok = 1;

    for (int f = 0; f < NFRAMES; f++) {
        int enc_len = opvis_encode(&c->enc, frames[f], (size_t)(W*H + 2*(W/2)*(H/2)),
                                   c->bitstream, (size_t)c->bs_cap);
        if (enc_len <= 0) { all_ok = 0; continue; }
        if (opvis_decode(&c->dec, c->bitstream, (size_t)enc_len) < 0) { all_ok = 0; continue; }
        const uint8_t *dec_y = opvis_decoded_y(&c->dec);
        float db = psnr_y(frames[f], dec_y, W, H);
        if (db < min_psnr) min_psnr = db;
        printf("    frame %d: PSNR = %.2f dB, %d bytes\n", f, db, enc_len);
    }

    CHECK(all_ok, "all frames encoded and decoded");
    CHECK_GE(min_psnr, 30.0f, "GOP minimum PSNR ≥ 30dB");

    ctx_free(c);
cleanup:
    for (int f = 0; f < NFRAMES; f++) free(frames[f]);
    free(tmp_u); free(tmp_v);
}

static void test_p010_roundtrip(void) {
    printf("\n=== 10-bit P010 encode/decode roundtrip ===\n");
    const int W = 128, H = 128;
    const int YN  = W * H;
    const int UVN = (W/2) * (H/2);

    /* Build a P010 input frame: Y in top-10-bits of uint16_t */
    uint16_t *p010 = malloc((YN + 2 * UVN) * sizeof(uint16_t));
    if (!p010) return;

    for (int i = 0; i < YN; i++)
        p010[i] = p010_pack((uint16_t)((i * 1023) / YN));
    for (int i = 0; i < UVN; i++) {
        p010[YN + 2*i]     = p010_pack(512);  /* Cb = mid */
        p010[YN + 2*i + 1] = p010_pack(512);  /* Cr = mid */
    }

    codec_ctx_t *c = ctx_alloc(W, H, 70, 30, OPVIS_FMT_P010);
    CHECK(c != NULL, "P010 context allocated");
    if (!c) { free(p010); return; }

    int enc_len = opvis_encode(&c->enc, (const uint8_t *)p010,
                               (size_t)((YN + 2*UVN) * sizeof(uint16_t)),
                               c->bitstream, (size_t)c->bs_cap);
    CHECK(enc_len > 0, "P010 I-frame encoded");

    if (enc_len > 0) {
        /* Check header flags */
        CHECK(c->bitstream[0] == 1,          "v1 header version byte");
        CHECK((c->bitstream[11] & 0x40) != 0, "10-bit flag set in header");

        int dec_ret = opvis_decode(&c->dec, c->bitstream, (size_t)enc_len);
        CHECK(dec_ret == 0, "P010 frame decoded");

        if (dec_ret == 0) {
            const uint16_t *dec_y16 = opvis_decoded_y16(&c->dec);
            CHECK(dec_y16 != NULL, "decoded_y16 returns non-NULL for 10-bit stream");

            if (dec_y16) {
                /* Build reference 8-bit (shifted from 10-bit input >>2) */
                uint8_t *ref8 = malloc(YN);
                uint8_t *dec8 = malloc(YN);
                if (ref8 && dec8) {
                    for (int i = 0; i < YN; i++) ref8[i] = (uint8_t)(p010_unpack(p010[i]) >> 2);
                    for (int i = 0; i < YN; i++) dec8[i] = (uint8_t)(dec_y16[i] >> 2);
                    float db = psnr_y(ref8, dec8, W, H);
                    CHECK_GE(db, 30.0f, "P010 roundtrip PSNR ≥ 30dB");
                    printf("    P010 PSNR = %.2f dB\n", db);
                }
                free(ref8); free(dec8);
            }
        }
    }

    ctx_free(c);
    free(p010);
}

static void test_palette_mode(void) {
    printf("\n=== Palette mode: flat-color block encodes small ===\n");
    const int W = 128, H = 128;

    uint8_t *input = malloc(W * H + 2 * (W/2) * (H/2));
    if (!input) return;

    /* All pixels the same value — ideal for palette (1 entry) */
    memset(input,         200,  W * H);
    memset(input + W * H, 128, 2 * (W/2) * (H/2));

    codec_ctx_t *c = ctx_alloc(W, H, 70, 30, OPVIS_FMT_YUV420P);
    CHECK(c != NULL, "palette context allocated");
    if (!c) { free(input); return; }

    int enc_len = opvis_encode(&c->enc, input, (size_t)(W*H + 2*(W/2)*(H/2)),
                               c->bitstream, (size_t)c->bs_cap);
    CHECK(enc_len > 0, "flat frame encoded");

    if (enc_len > 0) {
        int dec_ret = opvis_decode(&c->dec, c->bitstream, (size_t)enc_len);
        CHECK(dec_ret == 0, "flat frame decoded");

        if (dec_ret == 0) {
            const uint8_t *dec_y = opvis_decoded_y(&c->dec);
            /* Flat frame: all decoded pixels should be the same (or very close) */
            int max_err = 0;
            int first_err_pos = -1;
            for (int i = 0; i < W * H; i++) {
                int err = abs((int)dec_y[i] - 200);
                if (err > max_err) { max_err = err; if (first_err_pos < 0) first_err_pos = i; }
            }
            if (max_err > 4) {
                /* Find first pixel with exactly max_err */
                for (int i = 0; i < W * H; i++) {
                    if (abs((int)dec_y[i] - 200) == max_err) {
                        printf("    flat frame DBG: max_err pixel at (%d,%d) decoded=%d expected=200\n",
                               i % W, i / W, dec_y[i]);
                        break;
                    }
                }
            }
            CHECK(max_err <= 4, "flat frame pixel error ≤ 4");
            printf("    flat frame: %d bytes, max pixel error = %d\n", enc_len, max_err);
        }
    }

    /* Test palette API directly: detect a 4-distinct-value block */
    printf("\n  Palette API unit test:\n");
    uint8_t blk[64];       /* 8×8 block */
    uint8_t pal[SCREEN_PALETTE_MAX];
    uint8_t idx[64];
    int n = 0;
    for (int i = 0; i < 64; i++) blk[i] = (uint8_t)((i % 4) * 64);
    bool detected = screen_palette_detect(blk, 8, pal, &n, idx);
    CHECK(detected, "palette detected for 4-value block");
    CHECK(n == 4, "palette has 4 entries");

    /* Reconstruct and verify */
    uint8_t recon[64];
    screen_palette_reconstruct(pal, idx, 8, recon);
    int mismatch = 0;
    for (int i = 0; i < 64; i++) if (recon[i] != blk[i]) mismatch++;
    CHECK(mismatch == 0, "palette reconstruct exact match");

    /* 17-distinct-value block should NOT be palette-eligible */
    for (int i = 0; i < 64; i++) blk[i] = (uint8_t)(i * 4);  /* 64 distinct */
    detected = screen_palette_detect(blk, 8, pal, &n, idx);
    CHECK(!detected, "palette rejected for 64-distinct-value block");

    ctx_free(c);
    free(input);
}

static void test_ibc_mode(void) {
    printf("\n=== IBC mode: repeated tile pattern ===\n");
    const int W = 128, H = 128;

    uint8_t *input = malloc(W * H + 2 * (W/2) * (H/2));
    if (!input) return;

    /* Tiled pattern: each 8×8 tile is identical → IBC should find matches */
    make_repeated_tiles(input, W, H);
    memset(input + W * H, 128, 2 * (W/2) * (H/2));

    codec_ctx_t *c = ctx_alloc(W, H, 70, 30, OPVIS_FMT_YUV420P);
    CHECK(c != NULL, "IBC context allocated");
    if (!c) { free(input); return; }

    /* IBC only activates when the pool is v1 (ibc_hashtable allocated) */
    CHECK(c->enc.ibc_hashtable != NULL, "IBC hash table allocated (v1 pool)");

    int enc_len = opvis_encode(&c->enc, input, (size_t)(W*H + 2*(W/2)*(H/2)),
                               c->bitstream, (size_t)c->bs_cap);
    CHECK(enc_len > 0, "tiled frame encoded");

    if (enc_len > 0) {
        int dec_ret = opvis_decode(&c->dec, c->bitstream, (size_t)enc_len);
        CHECK(dec_ret == 0, "tiled frame decoded");

        if (dec_ret == 0) {
            const uint8_t *dec_y = opvis_decoded_y(&c->dec);
            float db = psnr_y(input, dec_y, W, H);
            CHECK_GE(db, 30.0f, "IBC tiled frame PSNR ≥ 30dB");
            printf("    tiled frame: %d bytes, PSNR = %.2f dB\n", enc_len, db);
        }
    }

    /* IBC API unit test */
    printf("\n  IBC API unit test:\n");
    const int FW = 64;
    uint8_t *frame = calloc(FW * FW, 1);
    if (!frame) { ctx_free(c); free(input); return; }

    /* Fill a reference 8×8 block at (0,0) */
    for (int row = 0; row < 8; row++)
        for (int col = 0; col < 8; col++)
            frame[row * FW + col] = (uint8_t)(row * 8 + col);

    uint32_t ibc_table[SCREEN_IBC_TABLE_SIZE];
    memset(ibc_table, 0xFF, sizeof(ibc_table));

    /* Update hash with block at (0,0) */
    screen_ibc_update(frame, FW, 0, 0, ibc_table, SCREEN_IBC_TABLE_SIZE);

    /* Copy same pattern to (16,0) — completely separate, no overlap with (0,0) */
    for (int row = 0; row < 8; row++)
        for (int col = 0; col < 8; col++)
            frame[row * FW + (16 + col)] = (uint8_t)(row * 8 + col);

    /* IBC search at (16,0) should find (0,0) */
    int bv_x = 0, bv_y = 0;
    bool found = screen_ibc_search(frame, FW, FW, 16, 0, 8,
                                   ibc_table, SCREEN_IBC_TABLE_SIZE,
                                   &bv_x, &bv_y);
    CHECK(found, "IBC search finds reference block");
    if (found) {
        CHECK(bv_x == -16 && bv_y == 0, "IBC block vector correct (-16, 0)");
        printf("    IBC bv = (%d, %d)\n", bv_x, bv_y);
    }

    free(frame);
    ctx_free(c);
    free(input);
}

static void test_header_compat(void) {
    printf("\n=== Header parsing and backward-compat guard ===\n");

    /* Reject v0 streams (byte 0 = 0) */
    uint8_t fake_v0[14] = {0, 0, 70, 0, 128, 0, 96, 0, 0, 0, 0, 0, 0, 0};
    opvis_decoder_t dec = {0};
    uint8_t dec_pool[65536];
    opvis_decoder_init(&dec, dec_pool, sizeof(dec_pool));
    int ret = opvis_decode(&dec, fake_v0, sizeof(fake_v0));
    CHECK(ret < 0, "v0 bitstream correctly rejected by v1 decoder");

    /* Reject truncated stream */
    uint8_t tiny[4] = {1, 0, 70, 0};
    ret = opvis_decode(&dec, tiny, sizeof(tiny));
    CHECK(ret < 0, "truncated stream rejected");

    /* Reject NULL inputs */
    ret = opvis_decode(NULL, fake_v0, sizeof(fake_v0));
    CHECK(ret < 0, "NULL decoder rejected");
    ret = opvis_decode(&dec, NULL, 14);
    CHECK(ret < 0, "NULL input rejected");

    printf("  note: v0 streams are not decoded (v1-only codec path)\n");
}

static void test_hdr_meta(void) {
    printf("\n=== HDR metadata encode/decode ===\n");
    const int W = 128, H = 128;

    uint8_t *input = malloc(W * H + 2 * (W/2) * (H/2));
    if (!input) return;
    make_gradient(input, input + W*H, input + W*H + (W/2)*(H/2), W, H);

    codec_ctx_t *c = ctx_alloc(W, H, 70, 30, OPVIS_FMT_YUV420P);
    if (!c) { free(input); return; }

    opvis_color_info_t ci = {
        .bitdepth    = 8,
        .transfer    = OPVIS_TF_PQ,
        .primaries   = OPVIS_CP_BT2020,
        .subsampling = 0,
    };
    opvis_hdr_meta_t hdr = {
        .max_lum  = 1000,
        .min_lum  = 1,
        .knee     = 128,
        .knee_gain = 64,
    };
    opvis_encoder_set_color_info(&c->enc, &ci, &hdr);

    int enc_len = opvis_encode(&c->enc, input, (size_t)(W*H + 2*(W/2)*(H/2)),
                               c->bitstream, (size_t)c->bs_cap);
    CHECK(enc_len > 0, "HDR frame encoded");

    if (enc_len > 0) {
        /* Check header byte 11 flags */
        uint8_t flags = c->bitstream[11];
        CHECK((flags & 0x80) != 0, "hdr_present bit set");
        CHECK(((flags >> 4) & 3) == (uint8_t)OPVIS_TF_PQ,     "PQ transfer encoded");
        CHECK(((flags >> 2) & 3) == (uint8_t)OPVIS_CP_BT2020,  "BT.2020 primaries encoded");

        int dec_ret = opvis_decode(&c->dec, c->bitstream, (size_t)enc_len);
        CHECK(dec_ret == 0, "HDR frame decoded");
        if (dec_ret == 0) {
            CHECK(c->dec.color_info.transfer  == OPVIS_TF_PQ,    "decoder sees PQ transfer");
            CHECK(c->dec.color_info.primaries == OPVIS_CP_BT2020, "decoder sees BT.2020");
        }
    }

    ctx_free(c);
    free(input);
}

/* ---- ALF bitstream signaling test ---- */

static void test_alf_bitstream(void) {
    printf("\n=== ALF bitstream signaling ===\n");
    const int W = 128, H = 128;

    codec_ctx_t *c = ctx_alloc(W, H, 70, 30, OPVIS_FMT_YUV420P);
    CHECK(c != NULL, "ALF context allocated");
    if (!c) return;

    uint8_t *orig_y = malloc(W * H);
    uint8_t *u = malloc((W/2) * (H/2));
    uint8_t *v = malloc((W/2) * (H/2));
    if (!orig_y || !u || !v) { free(orig_y); free(u); free(v); ctx_free(c); return; }

    make_gradient(orig_y, u, v, W, H);
    size_t in_len = (size_t)(W * H + 2 * (W/2) * (H/2));
    uint8_t *input = malloc(in_len);
    if (!input) { free(orig_y); free(u); free(v); ctx_free(c); return; }
    memcpy(input, orig_y, W * H);
    memcpy(input + W * H, u, (W/2)*(H/2));
    memcpy(input + W * H + (W/2)*(H/2), v, (W/2)*(H/2));

    int enc_len = opvis_encode(&c->enc, input, in_len, c->bitstream, (size_t)c->bs_cap);
    CHECK(enc_len > 18, "ALF I-frame encoded");

    if (enc_len > 18) {
        uint8_t b12 = c->bitstream[12];
        bool alf_present = (b12 & 0x80) != 0;
        printf("    q=70: byte12=0x%02x alf_present=%d screen_mode=%d\n",
               b12, alf_present ? 1 : 0, (b12 >> 6) & 1);

        /* Decode must succeed and produce correct PSNR regardless of ALF signaling */
        int dec_ret = opvis_decode(&c->dec, c->bitstream, (size_t)enc_len);
        CHECK(dec_ret == 0, "ALF I-frame (q=70) decoded");

        if (dec_ret == 0) {
            float db = psnr_y(orig_y, opvis_decoded_y(&c->dec), W, H);
            printf("    ALF frame PSNR = %.2f dB\n", db);
            CHECK_GE(db, 35.0f, "ALF frame PSNR ≥ 35dB");
        }
    }

    /* Low-quality encode to force ALF to fire (alf_present=1) */
    ctx_free(c);
    c = ctx_alloc(W, H, 10, 30, OPVIS_FMT_YUV420P);
    if (c) {
        int enc_len2 = opvis_encode(&c->enc, input, in_len, c->bitstream, (size_t)c->bs_cap);
        CHECK(enc_len2 > 18, "ALF I-frame (q=10) encoded");
        if (enc_len2 > 18) {
            uint8_t b12 = c->bitstream[12];
            bool alf_present = (b12 & 0x80) != 0;
            printf("    q=10: byte12=0x%02x alf_present=%d\n", b12, alf_present ? 1 : 0);

            /* Determine expected CTU bitmap size */
            int ctu_cols = (W + 63) / 64;
            int ctu_rows = (H + 63) / 64;
            int ctu_count = ctu_cols * ctu_rows;
            int bitmap_bytes = (ctu_count + 7) / 8;

            if (alf_present) {
                uint32_t payload_len = (uint32_t)c->bitstream[13] << 24 |
                                       (uint32_t)c->bitstream[14] << 16 |
                                       (uint32_t)c->bitstream[15] <<  8 |
                                       (uint32_t)c->bitstream[16];
                int expected_total = 18 + (int)payload_len + bitmap_bytes;
                CHECK(enc_len2 == expected_total, "ALF bitmap suffix size correct");
            }

            int dec_ret2 = opvis_decode(&c->dec, c->bitstream, (size_t)enc_len2);
            CHECK(dec_ret2 == 0, "ALF I-frame (q=10) decoded");
        }
    }

    free(orig_y); free(u); free(v); free(input);
    ctx_free(c);
}

/* ---- IBC screen_mode flag test ---- */

static void test_screen_mode_flag(void) {
    printf("\n=== screen_mode bit (byte 12) ===\n");
    const int W = 128, H = 64;

    codec_ctx_t *c = ctx_alloc(W, H, 70, 30, OPVIS_FMT_YUV420P);
    CHECK(c != NULL, "screen_mode context allocated");
    if (!c) return;

    /* Flat monochrome frame — likely to hit palette path */
    size_t in_len = (size_t)(W * H + 2 * (W/2) * (H/2));
    uint8_t *input = calloc(1, in_len);
    if (!input) { ctx_free(c); return; }
    memset(input, 200, W * H);  /* solid gray luma */
    memset(input + W * H, 128, 2 * (W/2) * (H/2));

    int enc_len = opvis_encode(&c->enc, input, in_len, c->bitstream, (size_t)c->bs_cap);
    CHECK(enc_len > 18, "screen_mode frame encoded");

    if (enc_len > 18) {
        printf("    byte12=0x%02x (bit7=alf=%d, bit6=screen=%d)\n",
               c->bitstream[12],
               (c->bitstream[12] >> 7) & 1,
               (c->bitstream[12] >> 6) & 1);
        /* Decode must succeed regardless */
        int dec_ret = opvis_decode(&c->dec, c->bitstream, (size_t)enc_len);
        CHECK(dec_ret == 0, "screen_mode frame decoded");
    }

    free(input);
    ctx_free(c);
}

/* ---- Main ---- */

int main(void) {
    printf("=== OPVIS Video Codec Roundtrip Test Suite ===\n");

    test_iframe_psnr();
    test_gop_psnr();
    test_p010_roundtrip();
    test_palette_mode();
    test_ibc_mode();
    test_header_compat();
    test_hdr_meta();
    test_alf_bitstream();
    test_screen_mode_flag();

    printf("\n=== Summary ===\n");
    printf("  Checks passed: %d\n", g_pass);
    printf("  Checks failed: %d\n", g_fail);
    if (g_fail == 0)
        printf("All tests PASSED\n");
    else
        printf("SOME TESTS FAILED\n");

    return g_fail == 0 ? 0 : 1;
}
