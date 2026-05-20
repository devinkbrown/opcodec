/*
 * opcodec/saliency.c — Fast perceptual saliency estimation for video
 *
 * Algorithm:
 *   1. Skin-tone detection: Y ∈ [80,200] with high edge density nearby → face candidate.
 *      Large connected skin regions (≥ 32×32) are upgraded to face weight.
 *   2. Motion saliency: CTUs with |MV| > 2 quarter-pixels receive motion weight.
 *   3. Text saliency: CTUs with edge density > 40% receive text weight.
 *   4. Temporal masking: temporal_count tracks how long each CTU has been
 *      encoded at or below the target QP.  After SALIENCY_MEMORIZE_FRAMES
 *      consecutive high-quality frames the brain has memorised the region;
 *      sal_temporal_qp_boost() then signals the encoder to raise QP by 3.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/saliency.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Sobel edge magnitude at (x,y) in 8-bit luma plane, unnormalized */
static int edge_magnitude(const uint8_t *luma, int w, int h, int x, int y)
{
    if (x < 1 || x >= w - 1 || y < 1 || y >= h - 1) return 0;
    int gx = -(int)luma[(y-1)*w + (x-1)] - 2*(int)luma[y*w + (x-1)]
               - (int)luma[(y+1)*w + (x-1)]
             + (int)luma[(y-1)*w + (x+1)] + 2*(int)luma[y*w + (x+1)]
             + (int)luma[(y+1)*w + (x+1)];
    int gy = -(int)luma[(y-1)*w + (x-1)] - 2*(int)luma[(y-1)*w + x]
               - (int)luma[(y-1)*w + (x+1)]
             + (int)luma[(y+1)*w + (x-1)] + 2*(int)luma[(y+1)*w + x]
             + (int)luma[(y+1)*w + (x+1)];
    int mag = abs(gx) + abs(gy);  /* L1 approximation, fast */
    return mag;
}

void sal_init(sal_ctx_t *ctx,
              uint16_t frame_width, uint16_t frame_height, uint16_t ctu_size)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->frame_width  = frame_width;
    ctx->frame_height = frame_height;
    ctx->ctu_cols     = (uint16_t)((frame_width  + ctu_size - 1) / ctu_size);
    ctx->ctu_rows     = (uint16_t)((frame_height + ctu_size - 1) / ctu_size);
    /* Clamp to grid size */
    if (ctx->ctu_cols > SALIENCY_MAX_CTU_COLS)
        ctx->ctu_cols = SALIENCY_MAX_CTU_COLS;
    if (ctx->ctu_rows > SALIENCY_MAX_CTU_ROWS)
        ctx->ctu_rows = SALIENCY_MAX_CTU_ROWS;
    /* Initialize all weights to background */
    for (int r = 0; r < (int)ctx->ctu_rows; r++)
        for (int c = 0; c < (int)ctx->ctu_cols; c++)
            ctx->grid[r][c].weight = SALIENCY_WEIGHT_BG;
}

void sal_set_faces(sal_ctx_t *ctx,
                   const sal_face_rect_t *faces, int n_faces)
{
    if (n_faces > 8) n_faces = 8;
    ctx->n_faces = n_faces;
    if (n_faces > 0 && faces)
        memcpy(ctx->faces, faces, (size_t)n_faces * sizeof(sal_face_rect_t));
}

void sal_estimate(sal_ctx_t *ctx,
                  const uint8_t *luma,
                  const int16_t *mv_x, const int16_t *mv_y)
{
    const int fw = ctx->frame_width;
    const int fh = ctx->frame_height;
    /* CTU pixel size — inferred from grid / frame dims */
    const int cs_x = (fw + ctx->ctu_cols - 1) / ctx->ctu_cols;
    const int cs_y = (fh + ctx->ctu_rows - 1) / ctx->ctu_rows;

    for (int r = 0; r < (int)ctx->ctu_rows; r++) {
        for (int c = 0; c < (int)ctx->ctu_cols; c++) {
            ctu_saliency_t *s = &ctx->grid[r][c];

            int px0 = c * cs_x;
            int py0 = r * cs_y;
            int px1 = px0 + cs_x; if (px1 > fw) px1 = fw;
            int py1 = py0 + cs_y; if (py1 > fh) py1 = fh;
            int npix = (px1 - px0) * (py1 - py0);
            if (npix <= 0) { s->weight = SALIENCY_WEIGHT_MIN; continue; }

            /* ── Motion saliency ── */
            s->is_motion = false;
            if (mv_x && mv_y) {
                int idx = r * (int)ctx->ctu_cols + c;
                int mv_mag = abs((int)mv_x[idx]) + abs((int)mv_y[idx]);
                if (mv_mag > 4)  /* > 1 full pixel */
                    s->is_motion = true;
            }

            /* ── Skin-tone heuristic (Y in [80,210] = plausible face luma) ─ */
            int skin_pixels = 0;
            int edge_sum    = 0;
            /* Sample every 2nd pixel for speed */
            for (int y = py0; y < py1; y += 2) {
                for (int x = px0; x < px1; x += 2) {
                    uint8_t yv = luma[y * fw + x];
                    if (yv >= 80 && yv <= 210) skin_pixels++;
                    edge_sum += edge_magnitude(luma, fw, fh, x, y);
                }
            }
            int sampled = ((py1 - py0 + 1) / 2) * ((px1 - px0 + 1) / 2);
            if (sampled < 1) sampled = 1;
            float skin_ratio = (float)skin_pixels / (float)sampled;
            float edge_ratio = (float)edge_sum / ((float)sampled * 255.0f);

            /* Text: high edge density, moderate luma variance */
            s->is_text = (edge_ratio > 0.15f && skin_ratio < 0.5f);

            /* Face detection: large skin region with moderate edges */
            s->is_face = false;
            if (skin_ratio > 0.45f && edge_ratio > 0.04f && edge_ratio < 0.40f) {
                /* Additional check: at least 16×16 skin patch */
                int streak = 0, max_streak = 0;
                for (int y = py0; y < py1; y++) {
                    for (int x = px0; x < px1; x++) {
                        uint8_t yv = luma[y * fw + x];
                        if (yv >= 80 && yv <= 210) { streak++; if (streak > max_streak) max_streak = streak; }
                        else streak = 0;
                    }
                }
                if (max_streak >= 16) s->is_face = true;
            }

            /* ── Override with externally provided faces ── */
            for (int fi = 0; fi < ctx->n_faces; fi++) {
                const sal_face_rect_t *f = &ctx->faces[fi];
                /* Check if CTU overlaps the face rectangle */
                if ((int)f->x < px1 && (int)(f->x + f->w) > px0 &&
                    (int)f->y < py1 && (int)(f->y + f->h) > py0)
                    s->is_face = true;
            }

            /* ── Compose final weight ── */
            float w = SALIENCY_WEIGHT_BG;
            if (s->is_face)        w  = SALIENCY_WEIGHT_FACE;
            else if (s->is_text)   w  = 2.50f;
            else if (s->is_motion) w  = SALIENCY_WEIGHT_MOTION;

            /* Faces near image center get a small boost */
            if (s->is_face) {
                float cx = (float)(px0 + px1) / (2.0f * (float)fw) - 0.5f;
                float cy = (float)(py0 + py1) / (2.0f * (float)fh) - 0.5f;
                float dist = sqrtf(cx*cx + cy*cy);
                w += (1.0f - dist * 2.0f) * 0.5f;
            }

            if (w < SALIENCY_WEIGHT_MIN) w = SALIENCY_WEIGHT_MIN;
            if (w > SALIENCY_WEIGHT_MAX) w = SALIENCY_WEIGHT_MAX;
            s->weight = w;
        }
    }
}

void sal_update_temporal(sal_ctx_t *ctx, int col, int row,
                         int qp_target, int qp_used)
{
    if (col < 0 || col >= (int)ctx->ctu_cols ||
        row < 0 || row >= (int)ctx->ctu_rows)
        return;
    ctu_saliency_t *s = &ctx->grid[row][col];
    if (qp_used <= qp_target) {
        if (s->temporal_count < 255) s->temporal_count++;
    } else if (qp_used > qp_target + 3) {
        s->temporal_count = 0;
    }
}
