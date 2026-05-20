/*
 * opcodec/vplc.c — Video Packet Loss Concealment implementation
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/vplc.h"
#include "opcodec/video.h"    /* for opvis_mv_t */
#include <string.h>

#define BS   VPLC_MB_SIZE

/* ── Internal helpers ─────────────────────────────────────────────────── */

/* Clamped pixel read */
static inline uint8_t cpix(const uint8_t *plane, int w, int h, int x, int y)
{
    if (x < 0)     x = 0;
    if (x >= w)    x = w - 1;
    if (y < 0)     y = 0;
    if (y >= h)    y = h - 1;
    return plane[y * w + x];
}

/* ── Temporal copy concealment ─────────────────────────────────────────── */

static void conceal_temporal(uint8_t *cur, const uint8_t *ref,
                              int w, int h, int bx, int by, int bs)
{
    for (int y = 0; y < bs; y++) {
        int fy = by * bs + y;
        if (fy >= h) break;
        for (int x = 0; x < bs; x++) {
            int fx = bx * bs + x;
            if (fx >= w) break;
            cur[fy * w + fx] = ref[fy * w + fx];
        }
    }
}

/* ── Motion-copy concealment ────────────────────────────────────────────── */

static void conceal_motion(uint8_t *cur, const uint8_t *ref,
                            int w, int h, int bx, int by, int bs,
                            int mvx, int mvy)
{
    /* Quarter-pixel MVs → integer offset (round toward zero) */
    int dx = mvx / 4, dy = mvy / 4;

    for (int y = 0; y < bs; y++) {
        int fy = by * bs + y;
        if (fy >= h) break;
        for (int x = 0; x < bs; x++) {
            int fx = bx * bs + x;
            if (fx >= w) break;
            cur[fy * w + fx] = cpix(ref, w, h, fx + dx, fy + dy);
        }
    }
}

/* ── Boundary matching concealment ─────────────────────────────────────── */

static void conceal_boundary(uint8_t *cur, int w, int h,
                              int bx, int by, int bs)
{
    int px0 = bx * bs, py0 = by * bs;
    int px1 = px0 + bs - 1, py1 = py0 + bs - 1;

    /* Collect boundary rows/cols from available neighbors */
    /* Top boundary: row above block */
    /* Bottom boundary: row below block */
    /* Left boundary: column left of block */
    /* Right boundary: column right of block */

    /* For each pixel (x,y) in the lost block, interpolate from 4 boundaries */
    for (int y = 0; y < bs; y++) {
        int fy = py0 + y;
        if (fy >= h) break;
        for (int x = 0; x < bs; x++) {
            int fx = px0 + x;
            if (fx >= w) break;

            /* Sample from top, bottom, left, right boundary neighbors */
            float top    = (float)cpix(cur, w, h, fx, py0 - 1);
            float bottom = (float)cpix(cur, w, h, fx, py1 + 1);
            float left   = (float)cpix(cur, w, h, px0 - 1, fy);
            float right  = (float)cpix(cur, w, h, px1 + 1, fy);

            /* Distance-weighted blend (closer boundary gets more weight) */
            float wy_top    = (float)(bs - 1 - y) + 0.5f;
            float wy_bottom = (float)(y) + 0.5f;
            float wx_left   = (float)(bs - 1 - x) + 0.5f;
            float wx_right  = (float)(x) + 0.5f;

            float wsum = wy_top + wy_bottom + wx_left + wx_right;
            float val  = (wy_top * top + wy_bottom * bottom
                         + wx_left * left + wx_right * right) / wsum;
            int iv = (int)(val + 0.5f);
            if (iv < 0)   iv = 0;
            if (iv > 255) iv = 255;
            cur[fy * w + fx] = (uint8_t)iv;
        }
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void vplc_init(vplc_ctx_t *ctx, vplc_strategy_t strategy)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->default_strategy = strategy;
    ctx->initialized = true;
}

int vplc_conceal(vplc_ctx_t *ctx,
                  uint8_t *cur_y, uint8_t *cur_u, uint8_t *cur_v,
                  const uint8_t *ref_y, const uint8_t *ref_u, const uint8_t *ref_v,
                  const void *mvs_raw,
                  const vplc_mb_status_t *lost_map,
                  uint16_t width, uint16_t height)
{
    if (!ctx || !ctx->initialized || !cur_y || !lost_map) return 0;

    const opvis_mv_t *mvs = (const opvis_mv_t *)mvs_raw;
    const int mb_cols = (width  + BS - 1) / BS;
    const int mb_rows = (height + BS - 1) / BS;
    const int cw = width / 2, ch = height / 2;
    int concealed = 0;

    for (int by = 0; by < mb_rows; by++) {
        for (int bx = 0; bx < mb_cols; bx++) {
            int mb_idx = by * mb_cols + bx;
            if (!lost_map[mb_idx].lost) continue;

            vplc_strategy_t strat = ctx->default_strategy;

            /* Prefer motion copy when we have MVs and a reference frame */
            if (strat == VPLC_STRAT_MOTION_COPY && mvs && ref_y) {
                /* Use MV from a neighboring available block */
                int mvx = 0, mvy = 0;
                /* Try left, top, right, bottom neighbors */
                static const int dx4[] = { -1,  0,  1, 0 };
                static const int dy4[] = {  0, -1,  0, 1 };
                bool found_mv = false;
                for (int d = 0; d < 4; d++) {
                    int nx = bx + dx4[d], ny = by + dy4[d];
                    if (nx < 0 || nx >= mb_cols || ny < 0 || ny >= mb_rows) continue;
                    int nidx = ny * mb_cols + nx;
                    if (!lost_map[nidx].lost) {
                        mvx = mvs[nidx].x;
                        mvy = mvs[nidx].y;
                        found_mv = true;
                        break;
                    }
                }
                if (found_mv) {
                    conceal_motion(cur_y, ref_y, width, height, bx, by, BS, mvx, mvy);
                    if (cur_u && cur_v && ref_u && ref_v)
                        conceal_motion(cur_u, ref_u, cw, ch, bx, by, BS/2, mvx/2, mvy/2);
                    if (cur_v && ref_v)
                        conceal_motion(cur_v, ref_v, cw, ch, bx, by, BS/2, mvx/2, mvy/2);
                    ctx->stats.motion_copy_used++;
                    goto done_mb;
                }
                /* Fall through to boundary match */
                strat = VPLC_STRAT_BOUNDARY_MATCH;
            }

            if (strat == VPLC_STRAT_BOUNDARY_MATCH) {
                conceal_boundary(cur_y, width, height, bx, by, BS);
                if (cur_u) conceal_boundary(cur_u, cw, ch, bx, by, BS/2);
                if (cur_v) conceal_boundary(cur_v, cw, ch, bx, by, BS/2);
                ctx->stats.boundary_match_used++;
                goto done_mb;
            }

            /* Temporal copy fallback */
            if (ref_y) {
                conceal_temporal(cur_y, ref_y, width, height, bx, by, BS);
                if (cur_u && ref_u) conceal_temporal(cur_u, ref_u, cw, ch, bx, by, BS/2);
                if (cur_v && ref_v) conceal_temporal(cur_v, ref_v, cw, ch, bx, by, BS/2);
            }
            ctx->stats.temporal_copy_used++;

done_mb:
            ctx->stats.mbs_concealed++;
            concealed++;
        }
    }
    return concealed;
}

void vplc_reset_stats(vplc_ctx_t *ctx)
{
    if (ctx) memset(&ctx->stats, 0, sizeof(ctx->stats));
}
