/*
 * opcodec/lvc.c — Latent Video Codec implementation
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/lvc.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define BS LVC_BLOCK_SIZE
#define BS2 (BS * BS)

/* ── Synthetic codebook generation ────────────────────────────────────── */

/* Generate a DCT-II basis function as a codebook entry.
 * k: frequency index (0 = DC, 1–15 = AC in row/column)
 * Pattern: 2D separable DCT basis, stored as uint8_t offsets around 128. */
static void gen_dct_basis(float *entry, int ku, int kv)
{
    const float scale_u = (ku == 0) ? 1.0f / sqrtf((float)BS)
                                     : sqrtf(2.0f / (float)BS);
    const float scale_v = (kv == 0) ? 1.0f / sqrtf((float)BS)
                                     : sqrtf(2.0f / (float)BS);
    const float pi_over_2N = (float)M_PI / (2.0f * (float)BS);

    for (int y = 0; y < BS; y++) {
        for (int x = 0; x < BS; x++) {
            float cu = scale_u * cosf(pi_over_2N * (float)(2 * x + 1) * (float)ku);
            float cv = scale_v * cosf(pi_over_2N * (float)(2 * y + 1) * (float)kv);
            /* Scale to roughly [0, 255] range centered at 128 */
            entry[y * BS + x] = 128.0f + 64.0f * cu * cv;
        }
    }
}

/* Build 256 codebook entries from 2D DCT basis functions (16×16 grid) */
static void build_codebook(float cb[LVC_CODEBOOK_N][BS2])
{
    for (int ku = 0; ku < 16; ku++) {
        for (int kv = 0; kv < 16; kv++) {
            int idx = ku * 16 + kv;
            gen_dct_basis(cb[idx], ku, kv);
        }
    }
}

/* ── VQ search: find best codebook entry for a 16×16 block ── */
static int vq_search(const float cb[LVC_CODEBOOK_N][BS2],
                     const uint8_t *block, int stride,
                     float *best_gain_out)
{
    int best_idx = 0;
    float best_gain = 1.0f;
    float best_dist = 1e20f;

    for (int i = 0; i < LVC_CODEBOOK_N; i++) {
        /* Estimate optimal gain: g = <block, cb_i> / <cb_i, cb_i> */
        float num = 0.0f, den = 1e-6f;
        for (int k = 0; k < BS2; k++) {
            int y = k / BS, x = k % BS;
            float bv = (float)block[y * stride + x];
            float cv = cb[i][k];
            num += bv * cv;
            den += cv * cv;
        }
        float gain = num / den;
        /* Clamp gain to valid range */
        if (gain < LVC_GAIN_MIN) gain = LVC_GAIN_MIN;
        if (gain > LVC_GAIN_MAX) gain = LVC_GAIN_MAX;

        /* Compute distortion with this gain */
        float dist = 0.0f;
        for (int k = 0; k < BS2; k++) {
            int y = k / BS, x = k % BS;
            float bv = (float)block[y * stride + x];
            float cv = gain * cb[i][k];
            float d  = bv - cv;
            dist += d * d;
        }
        if (dist < best_dist) {
            best_dist = dist;
            best_idx  = i;
            best_gain = gain;
        }
    }

    if (best_gain_out) *best_gain_out = best_gain;
    return best_idx;
}

/* Quantize gain to 4-bit code (0–15) */
static uint8_t gain_to_4bit(float gain)
{
    float norm = (gain - LVC_GAIN_MIN) / (LVC_GAIN_MAX - LVC_GAIN_MIN);
    int code = (int)(norm * (LVC_GAIN_STEPS - 1) + 0.5f);
    if (code < 0)               code = 0;
    if (code > LVC_GAIN_STEPS - 1) code = LVC_GAIN_STEPS - 1;
    return (uint8_t)code;
}

static float gain_from_4bit(uint8_t code)
{
    return LVC_GAIN_MIN + (float)code * (LVC_GAIN_MAX - LVC_GAIN_MIN)
                          / (float)(LVC_GAIN_STEPS - 1);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int lvc_enc_init(lvc_enc_t *ctx)
{
    if (!ctx) return -1;
    memset(ctx, 0, sizeof(*ctx));
    build_codebook(ctx->codebook);
    ctx->codebook_trained = true;
    ctx->initialized = true;
    return 0;
}

int lvc_encode(lvc_enc_t *ctx, const uint8_t *luma,
               uint16_t width, uint16_t height,
               uint8_t *out, int out_cap)
{
    if (!ctx || !ctx->initialized || !luma || !out) return -1;

    int mb_cols = (width  + BS - 1) / BS;
    int mb_rows = (height + BS - 1) / BS;
    int n_mbs   = mb_cols * mb_rows;
    int needed  = LVC_PACKET_HDR + n_mbs + (n_mbs + 1) / 2;
    if (out_cap < needed) return -1;

    /* Header */
    uint8_t *p = out;
    *p++ = 1;              /* LVC version */
    *p++ = (uint8_t)(width  >> 8);
    *p++ = (uint8_t)(width  & 0xFF);
    *p++ = (uint8_t)(height >> 8);
    *p++ = (uint8_t)(height & 0xFF);
    *p++ = 0x01u;          /* flags: gain_present */

    /* VQ index table */
    uint8_t *idx_table = p;   p += n_mbs;

    /* Gain nibble table (packed 4+4 bits per byte) */
    uint8_t *gain_table = p;  p += (n_mbs + 1) / 2;
    memset(gain_table, 0, (size_t)((n_mbs + 1) / 2));

    int mb = 0;
    for (int by = 0; by < mb_rows; by++) {
        for (int bx = 0; bx < mb_cols; bx++, mb++) {
            int px = bx * BS, py = by * BS;
            /* The block may extend past frame bounds; use clamped addressing */
            uint8_t block[BS2];
            for (int y = 0; y < BS; y++) {
                int fy = py + y;
                if (fy >= height) fy = height - 1;
                for (int x = 0; x < BS; x++) {
                    int fx = px + x;
                    if (fx >= width) fx = width - 1;
                    block[y * BS + x] = luma[fy * width + fx];
                }
            }

            float gain;
            int idx = vq_search(ctx->codebook, block, BS, &gain);
            idx_table[mb] = (uint8_t)idx;

            uint8_t gc = gain_to_4bit(gain);
            if (mb & 1)
                gain_table[mb / 2] |= (gc << 4);
            else
                gain_table[mb / 2] |= gc;
        }
    }

    return (int)(p - out);
}

int lvc_dec_init(lvc_dec_t *ctx)
{
    if (!ctx) return -1;
    memset(ctx, 0, sizeof(*ctx));
    build_codebook(ctx->codebook);
    ctx->codebook_trained = true;
    ctx->initialized = true;
    return 0;
}

int lvc_decode(lvc_dec_t *ctx, const uint8_t *in, int in_len,
               uint8_t *luma_out, uint16_t *width_out, uint16_t *height_out)
{
    if (!ctx || !ctx->initialized || !in || in_len < LVC_PACKET_HDR) return -1;

    const uint8_t *p = in;
    uint8_t version  = *p++;
    uint16_t width   = ((uint16_t)*p++ << 8); width  |= *p++;
    uint16_t height  = ((uint16_t)*p++ << 8); height |= *p++;
    uint8_t flags    = *p++;

    if (version != 1 || width == 0 || height == 0) return -1;

    if (width_out)  *width_out  = width;
    if (height_out) *height_out = height;
    if (!luma_out) return 0;  /* caller just wants dimensions */

    int mb_cols = (width  + BS - 1) / BS;
    int mb_rows = (height + BS - 1) / BS;
    int n_mbs   = mb_cols * mb_rows;
    int remaining = in_len - (int)(p - in);
    if (remaining < n_mbs) return -1;

    const uint8_t *idx_table  = p;  p += n_mbs;
    const uint8_t *gain_table = (flags & 1) ? p : NULL;

    int mb = 0;
    for (int by = 0; by < mb_rows; by++) {
        for (int bx = 0; bx < mb_cols; bx++, mb++) {
            uint8_t idx = idx_table[mb];
            float gain = 1.0f;
            if (gain_table) {
                uint8_t nibble = (mb & 1) ? (gain_table[mb / 2] >> 4)
                                           : (gain_table[mb / 2] & 0x0F);
                gain = gain_from_4bit(nibble);
            }

            int px = bx * BS, py = by * BS;
            for (int y = 0; y < BS; y++) {
                int fy = py + y;
                if (fy >= height) continue;
                for (int x = 0; x < BS; x++) {
                    int fx = px + x;
                    if (fx >= width) continue;
                    float v = gain * ctx->codebook[idx][y * BS + x];
                    int iv = (int)(v + 0.5f);
                    if (iv < 0)   iv = 0;
                    if (iv > 255) iv = 255;
                    luma_out[fy * width + fx] = (uint8_t)iv;
                }
            }
        }
    }
    return 0;
}
