/*
 * opcodec/gbs.c — Generative Background Synthesis implementation
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/gbs.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef CLAMP
#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#endif

/* ── Internal helpers ─────────────────────────────────────────────────── */

/* Convert 8-bit luma energy to log-domain band code */
static uint8_t energy_to_8bit_log(float energy)
{
    if (energy <= 0.0f) return 0;
    float db = 10.0f * log10f(energy + 1e-10f);
    int code = (int)((db + 60.0f) * 255.0f / 60.0f + 0.5f);
    if (code < 0)   code = 0;
    if (code > 255) code = 255;
    return (uint8_t)code;
}

static float energy_from_8bit_log(uint8_t code)
{
    float db = (float)code * 60.0f / 255.0f - 60.0f;
    return powf(10.0f, db / 10.0f);
}

/* Compute 32 Mel-band energies from a luma histogram approximation */
static void compute_bg_bands(const float *bg, int w4, int h4,
                              uint8_t *band_codes)
{
    int n_taps = w4 * h4;
    float log_min = logf(1.0f);
    float log_max = logf((float)(n_taps > 1 ? n_taps : 2));
    const int N_BANDS = 32;

    for (int b = 0; b < N_BANDS; b++) {
        float flo = expf(log_min + (float)b       * (log_max - log_min) / (float)N_BANDS);
        float fhi = expf(log_min + (float)(b + 1) * (log_max - log_min) / (float)N_BANDS);
        int t0 = (int)flo, t1 = (int)fhi;
        if (t0 >= n_taps) t0 = n_taps - 1;
        if (t1 >= n_taps) t1 = n_taps - 1;
        if (t1 < t0)      t1 = t0;
        float energy = 0.0f;
        for (int t = t0; t <= t1; t++)
            energy += bg[t] * bg[t];
        float avg = (t1 >= t0) ? energy / (float)(t1 - t0 + 1) : 0.0f;
        band_codes[b] = energy_to_8bit_log(avg);
    }
}

/* Compute dominant color in background (simplified: mean YUV of static area) */
static void compute_dominant_color(const float *bg_y4, int w4, int h4,
                                    uint8_t *hue, uint8_t *sat, uint8_t *lum)
{
    if (w4 == 0 || h4 == 0) { *hue = 0; *sat = 0; *lum = 128; return; }

    /* Estimate mean luma as a proxy for luminance */
    float sum = 0.0f;
    int n = w4 * h4;
    for (int i = 0; i < n; i++) sum += bg_y4[i];
    float mean_y = sum / (float)n;

    /* Use mean_y as lum; hue/sat are placeholders (full estimation needs color planes) */
    *lum = (uint8_t)CLAMP((int)mean_y, 0, 255);
    *hue = 0;   /* neutral */
    *sat = 0;   /* neutral */
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int gbs_enc_init(gbs_enc_t *ctx, uint16_t width, uint16_t height, gbs_mode_t mode)
{
    if (!ctx || width == 0 || height == 0) return -1;
    memset(ctx, 0, sizeof(*ctx));

    ctx->bg_width  = width;
    ctx->bg_height = height;
    ctx->bg_w4     = (width  + 3) / 4;
    ctx->bg_h4     = (height + 3) / 4;

    if ((size_t)ctx->bg_w4 * ctx->bg_h4 > sizeof(ctx->bg_y) / sizeof(ctx->bg_y[0]))
        return -1;

    /* Init background to neutral grey */
    for (size_t i = 0; i < (size_t)ctx->bg_w4 * ctx->bg_h4; i++)
        ctx->bg_y[i] = 128.0f;

    ctx->fg_mask.mb_cols = (width  + GBS_BLOCK_SIZE - 1) / GBS_BLOCK_SIZE;
    ctx->fg_mask.mb_rows = (height + GBS_BLOCK_SIZE - 1) / GBS_BLOCK_SIZE;
    memset(ctx->fg_mask.bits, 0, sizeof(ctx->fg_mask.bits));

    ctx->mode  = mode;
    ctx->bg_update_countdown = GBS_BG_UPDATE_INTERVAL_FRAMES;
    ctx->desc.flags = 0;
    ctx->initialized = true;
    return 0;
}

void gbs_enc_free(gbs_enc_t *ctx)
{
    (void)ctx;  /* no heap allocations */
}

void gbs_enc_update(gbs_enc_t *ctx,
                    const uint8_t *luma,
                    const uint32_t *mv_sad)
{
    if (!ctx || !ctx->initialized || !luma) return;

    const int mb_cols = ctx->fg_mask.mb_cols;
    const int mb_rows = ctx->fg_mask.mb_rows;
    const int w4  = ctx->bg_w4;
    const int h4  = ctx->bg_h4;
    const int w   = ctx->bg_width;

    /* Clear foreground mask */
    memset(ctx->fg_mask.bits, 0, sizeof(ctx->fg_mask.bits));

    /* Update 1/4-res background model and build foreground mask */
    for (int by = 0; by < mb_rows; by++) {
        for (int bx = 0; bx < mb_cols; bx++) {
            int mb_idx  = by * mb_cols + bx;
            uint32_t sad = mv_sad ? mv_sad[mb_idx] : 0;

            /* Foreground: block with significant motion/texture difference */
            bool is_fg = (sad > (uint32_t)(GBS_MOTION_THRESH * GBS_BLOCK_SIZE * GBS_BLOCK_SIZE));
            if (is_fg)
                ctx->fg_mask.bits[mb_idx / 8] |= (uint8_t)(1u << (mb_idx & 7));

            /* Update 1/4-res background only for static blocks */
            if (!is_fg) {
                /* Sample center pixel of block at 1/4 resolution */
                int px  = bx * GBS_BLOCK_SIZE + GBS_BLOCK_SIZE / 2;
                int py  = by * GBS_BLOCK_SIZE + GBS_BLOCK_SIZE / 2;
                int px4 = px / 4, py4 = py / 4;
                if (px4 < w4 && py4 < h4) {
                    float pixel = (float)luma[py * w + px];
                    ctx->bg_y[py4 * w4 + px4] =
                        (1.0f - GBS_BG_ALPHA) * ctx->bg_y[py4 * w4 + px4]
                        + GBS_BG_ALPHA * pixel;
                }
            }
        }
    }

    /* Update descriptor periodically */
    ctx->bg_update_countdown--;
    if (ctx->bg_update_countdown <= 0) {
        compute_bg_bands(ctx->bg_y, w4, h4, ctx->desc.band_energy);
        compute_dominant_color(ctx->bg_y, w4, h4,
                                &ctx->desc.dominant_hue,
                                &ctx->desc.dominant_sat,
                                &ctx->desc.dominant_lum);
        ctx->desc.flags |= 0x03u;  /* valid | update_needed */
        ctx->bg_update_countdown = GBS_BG_UPDATE_INTERVAL_FRAMES;
    }

    ctx->frame_count++;
}

int gbs_enc_serialize(gbs_enc_t *ctx, uint8_t *out, int out_cap)
{
    if (!ctx || !out || out_cap < GBS_DESCRIPTOR_BYTES) return 0;
    if (!(ctx->desc.flags & 0x02u)) return 0;  /* no update needed */

    uint8_t *p = out;
    *p++ = 1;  /* GBS descriptor version */
    memcpy(p, ctx->desc.band_energy, 32); p += 32;
    *p++ = ctx->desc.dominant_hue;
    *p++ = ctx->desc.dominant_sat;
    *p++ = ctx->desc.dominant_lum;
    *p++ = ctx->desc.flags & 0xFEu;  /* clear update_needed bit */

    ctx->desc.flags &= ~0x02u;  /* clear update_needed */
    return (int)(p - out);
}

int gbs_dec_init(gbs_dec_t *ctx, uint16_t width, uint16_t height,
                 uint8_t *bg_y, uint8_t *bg_u, uint8_t *bg_v)
{
    if (!ctx || !bg_y) return -1;
    memset(ctx, 0, sizeof(*ctx));
    ctx->width  = width;
    ctx->height = height;
    ctx->bg_y   = bg_y;
    ctx->bg_u   = bg_u;
    ctx->bg_v   = bg_v;

    /* Initialize background to neutral grey */
    memset(bg_y, 128, (size_t)width * height);
    if (bg_u) memset(bg_u, 128, (size_t)(width / 2) * (height / 2));
    if (bg_v) memset(bg_v, 128, (size_t)(width / 2) * (height / 2));

    ctx->initialized = true;
    return 0;
}

int gbs_dec_apply(gbs_dec_t *ctx, const uint8_t *in, int in_len)
{
    if (!ctx || !ctx->initialized || !in || in_len < GBS_DESCRIPTOR_BYTES) return -1;

    const uint8_t *p = in;
    if (*p++ != 1) return -1;  /* version check */

    memcpy(ctx->desc.band_energy, p, 32); p += 32;
    ctx->desc.dominant_hue = *p++;
    ctx->desc.dominant_sat = *p++;
    ctx->desc.dominant_lum = *p++;
    ctx->desc.flags        = *p++;

    /* Reconstruct background texture from band energies:
     * Fill background with a smooth gradient derived from the dominant luminance
     * plus low-frequency energy profile. This is a placeholder for a proper
     * learned synthesis model (e.g., ControlNet-based BG generation). */
    if (!ctx->bg_y) return 0;

    const int w = ctx->width, h = ctx->height;
    float base_lum = (float)ctx->desc.dominant_lum;

    /* Use first few band energies to add large-scale spatial variation */
    float e0 = energy_from_8bit_log(ctx->desc.band_energy[0]);
    float e1 = energy_from_8bit_log(ctx->desc.band_energy[1]);
    float scale = sqrtf(e0 + e1 + 1e-6f) * 20.0f;
    if (scale > 40.0f) scale = 40.0f;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            /* Smooth sinusoidal gradient (very low spatial frequency = BG texture) */
            float vx = (float)x / (float)(w > 1 ? w - 1 : 1) - 0.5f;
            float vy = (float)y / (float)(h > 1 ? h - 1 : 1) - 0.5f;
            float val = base_lum + scale * (0.5f * vx + 0.3f * vy);
            int iv = (int)(val + 0.5f);
            if (iv < 0)   iv = 0;
            if (iv > 255) iv = 255;
            ctx->bg_y[y * w + x] = (uint8_t)iv;
        }
    }

    /* Chroma: fill with dominant hue/sat approximation */
    if (ctx->bg_u && ctx->bg_v) {
        uint8_t cu = (uint8_t)(128 + (int)(ctx->desc.dominant_sat / 4));
        uint8_t cv = (uint8_t)(128 - (int)(ctx->desc.dominant_sat / 4));
        memset(ctx->bg_u, cu, (size_t)(w / 2) * (h / 2));
        memset(ctx->bg_v, cv, (size_t)(w / 2) * (h / 2));
    }

    return 0;
}

void gbs_dec_set_vbg(gbs_dec_t *ctx,
                     const uint8_t *y, const uint8_t *u, const uint8_t *v)
{
    if (!ctx) return;
    ctx->vbg_y   = y;
    ctx->vbg_u   = u;
    ctx->vbg_v   = v;
    ctx->use_vbg = (y != NULL);
}

void gbs_dec_composite(gbs_dec_t *ctx,
                        uint8_t *fg_y, uint8_t *fg_u, uint8_t *fg_v,
                        const gbs_mask_t *fg_mask)
{
    if (!ctx || !ctx->initialized || !fg_y || !fg_mask) return;

    const int w = ctx->width, h = ctx->height;
    const int mb_cols = fg_mask->mb_cols;
    const int mb_rows = fg_mask->mb_rows;
    const int BS = GBS_BLOCK_SIZE;

    const uint8_t *bg_y_src = ctx->use_vbg ? ctx->vbg_y : ctx->bg_y;
    const uint8_t *bg_u_src = ctx->use_vbg ? ctx->vbg_u : ctx->bg_u;
    const uint8_t *bg_v_src = ctx->use_vbg ? ctx->vbg_v : ctx->bg_v;

    for (int by = 0; by < mb_rows; by++) {
        for (int bx = 0; bx < mb_cols; bx++) {
            int mb_idx = by * mb_cols + bx;
            bool is_fg = (fg_mask->bits[mb_idx / 8] >> (mb_idx & 7)) & 1;
            if (is_fg) continue;  /* foreground: keep decoded pixels as-is */

            /* Background block: replace with synthesized/virtual background */
            if (bg_y_src) {
                for (int y = 0; y < BS; y++) {
                    int fy = by * BS + y;
                    if (fy >= h) break;
                    for (int x = 0; x < BS; x++) {
                        int fx = bx * BS + x;
                        if (fx >= w) break;
                        fg_y[fy * w + fx] = bg_y_src[fy * w + fx];
                    }
                }
            }
            /* Chroma replacement (4:2:0: one chroma sample per 2×2 luma block) */
            if (fg_u && bg_u_src && fg_v && bg_v_src) {
                int cw = w / 2, ch = h / 2;
                for (int y = 0; y < BS / 2; y++) {
                    int fy = by * BS / 2 + y;
                    if (fy >= ch) break;
                    for (int x = 0; x < BS / 2; x++) {
                        int fx = bx * BS / 2 + x;
                        if (fx >= cw) break;
                        fg_u[fy * cw + fx] = bg_u_src[fy * cw + fx];
                        fg_v[fy * cw + fx] = bg_v_src[fy * cw + fx];
                    }
                }
            }
        }
    }
}
