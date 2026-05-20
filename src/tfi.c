/*
 * opcodec/tfi.c — Temporal Frame Interpolation
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/tfi.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Block-match SAD between two 8×8 patches */
static uint32_t sad8(const uint8_t *a, int a_stride,
                     const uint8_t *b, int b_stride)
{
    uint32_t s = 0;
    for (int y = 0; y < TFI_BLOCK_SIZE; y++) {
        for (int x = 0; x < TFI_BLOCK_SIZE; x++) {
            int d = (int)a[y * a_stride + x] - (int)b[y * b_stride + x];
            s += (uint32_t)(d < 0 ? -d : d);
        }
    }
    return s;
}

/* Find best matching 8×8 block in ref for the block at (bx,by) in src */
static void find_mv(const uint8_t *src, const uint8_t *ref,
                    int width, int height,
                    int bx, int by,
                    int16_t *mvx, int16_t *mvy)
{
    uint32_t best_sad = UINT32_MAX;
    int best_dx = 0, best_dy = 0;

    for (int dy = -TFI_SEARCH_RANGE; dy <= TFI_SEARCH_RANGE; dy++) {
        for (int dx = -TFI_SEARCH_RANGE; dx <= TFI_SEARCH_RANGE; dx++) {
            int rx = bx + dx, ry = by + dy;
            if (rx < 0 || ry < 0 ||
                rx + TFI_BLOCK_SIZE > width || ry + TFI_BLOCK_SIZE > height)
                continue;
            uint32_t s = sad8(src + by * width + bx, width,
                              ref + ry * width + rx, width);
            if (s < best_sad) {
                best_sad = s;
                best_dx  = dx;
                best_dy  = dy;
            }
        }
    }
    *mvx = (int16_t)best_dx;
    *mvy = (int16_t)best_dy;
}

int tfi_init(tfi_ctx_t *ctx, uint16_t width, uint16_t height)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->width  = width;
    ctx->height = height;
    ctx->mv_cols = (width  + TFI_BLOCK_SIZE - 1) / TFI_BLOCK_SIZE;
    ctx->mv_rows = (height + TFI_BLOCK_SIZE - 1) / TFI_BLOCK_SIZE;
    int n = ctx->mv_cols * ctx->mv_rows;

    ctx->fwd_mvx = calloc((size_t)n, sizeof(int16_t));
    ctx->fwd_mvy = calloc((size_t)n, sizeof(int16_t));
    ctx->bwd_mvx = calloc((size_t)n, sizeof(int16_t));
    ctx->bwd_mvy = calloc((size_t)n, sizeof(int16_t));

    if (!ctx->fwd_mvx || !ctx->fwd_mvy || !ctx->bwd_mvx || !ctx->bwd_mvy) {
        tfi_free(ctx);
        return -1;
    }
    ctx->initialized = true;
    return 0;
}

void tfi_free(tfi_ctx_t *ctx)
{
    if (!ctx) return;
    free(ctx->fwd_mvx); ctx->fwd_mvx = NULL;
    free(ctx->fwd_mvy); ctx->fwd_mvy = NULL;
    free(ctx->bwd_mvx); ctx->bwd_mvx = NULL;
    free(ctx->bwd_mvy); ctx->bwd_mvy = NULL;
    ctx->initialized = false;
}

int tfi_interpolate(tfi_ctx_t *ctx,
                    const uint8_t *prev, const uint8_t *next,
                    uint8_t *out, float alpha)
{
    if (!ctx || !ctx->initialized || !prev || !next || !out) return -1;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    float beta = 1.0f - alpha;

    int w = (int)ctx->width;
    int h = (int)ctx->height;

    /* Step 1: estimate forward MVs (prev → interpolated position) and
     *         backward MVs (next → interpolated position) */
    for (int br = 0; br < ctx->mv_rows; br++) {
        for (int bc = 0; bc < ctx->mv_cols; bc++) {
            int bx = bc * TFI_BLOCK_SIZE;
            int by = br * TFI_BLOCK_SIZE;
            int idx = br * ctx->mv_cols + bc;
            find_mv(prev, next, w, h, bx, by,
                    &ctx->fwd_mvx[idx], &ctx->fwd_mvy[idx]);
            find_mv(next, prev, w, h, bx, by,
                    &ctx->bwd_mvx[idx], &ctx->bwd_mvy[idx]);
        }
    }

    /* Step 2: synthesize output using motion-compensated blending */
    for (int br = 0; br < ctx->mv_rows; br++) {
        for (int bc = 0; bc < ctx->mv_cols; bc++) {
            int bx = bc * TFI_BLOCK_SIZE;
            int by = br * TFI_BLOCK_SIZE;
            int idx = br * ctx->mv_cols + bc;

            /* Forward reference: scale MV by alpha (how far along we are) */
            int fx = bx + (int)(ctx->fwd_mvx[idx] * alpha);
            int fy = by + (int)(ctx->fwd_mvy[idx] * alpha);
            /* Backward reference: scale by beta */
            int rx = bx - (int)(ctx->bwd_mvx[idx] * beta);
            int ry = by - (int)(ctx->bwd_mvy[idx] * beta);

            for (int y = 0; y < TFI_BLOCK_SIZE; y++) {
                int oy = by + y;
                if (oy >= h) break;
                for (int x = 0; x < TFI_BLOCK_SIZE; x++) {
                    int ox = bx + x;
                    if (ox >= w) break;

                    /* Clamp forward source coordinates */
                    int sfx = fx + x, sfy = fy + y;
                    if (sfx < 0)    sfx = 0;
                    if (sfx >= w)   sfx = w - 1;
                    if (sfy < 0)    sfy = 0;
                    if (sfy >= h)   sfy = h - 1;

                    /* Clamp backward source coordinates */
                    int sbx = rx + x, sby = ry + y;
                    if (sbx < 0)    sbx = 0;
                    if (sbx >= w)   sbx = w - 1;
                    if (sby < 0)    sby = 0;
                    if (sby >= h)   sby = h - 1;

                    int pv = (int)prev[sfy * w + sfx];
                    int nv = (int)next[sby * w + sbx];
                    int blended = (int)(beta * (float)pv + alpha * (float)nv + 0.5f);
                    if (blended < 0)   blended = 0;
                    if (blended > 255) blended = 255;
                    out[oy * w + ox] = (uint8_t)blended;
                }
            }
        }
    }
    return 0;
}

void tfi_write_hint(uint8_t *dst, uint8_t alpha_q8, uint8_t use_motion)
{
    dst[0] = 0x03;        /* OPVIS_FRAME_INTERP marker */
    dst[1] = alpha_q8;
    dst[2] = use_motion;
}

bool tfi_read_hint(const uint8_t *src, uint8_t *alpha_q8, uint8_t *use_motion)
{
    if (!src || src[0] != 0x03) return false;
    if (alpha_q8)   *alpha_q8   = src[1];
    if (use_motion) *use_motion = src[2];
    return true;
}

uint32_t tfi_measure_quality(const uint8_t *synth, const uint8_t *truth,
                              int width, int height)
{
    uint32_t total = 0;
    int n = width * height;
    for (int i = 0; i < n; i++) {
        int d = (int)synth[i] - (int)truth[i];
        total += (uint32_t)(d < 0 ? -d : d);
    }
    return total / (uint32_t)(n > 0 ? n : 1);
}
