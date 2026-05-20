/*
 * opcodec/saliency.h — Fast perceptual saliency estimation for video
 *
 * Estimates per-CTU visual importance (saliency weight) using lightweight
 * signal-analysis heuristics: skin-tone luma range, edge density, and motion
 * vector magnitude. No external ML inference needed; runs in < 0.5 ms/frame.
 *
 * Saliency weights drive SARDO (Saliency-Aware Rate-Distortion Optimization)
 * to allocate more bits to perceptually important regions (faces, motion,
 * text) and fewer bits to backgrounds the brain has already "memorized."
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_SALIENCY_H
#define OPCODEC_SALIENCY_H

#include <stdint.h>
#include <stdbool.h>

/* CTU grid: max 128×128 CTUs per frame (8192×8192 pixels / 64) */
#define SALIENCY_MAX_CTU_COLS  128
#define SALIENCY_MAX_CTU_ROWS  128

/* Saliency weight range: 0.1 (ignore) to 4.0 (face center) */
#define SALIENCY_WEIGHT_MIN    0.10f
#define SALIENCY_WEIGHT_MAX    4.00f
#define SALIENCY_WEIGHT_BG     0.20f   /* static background */
#define SALIENCY_WEIGHT_FACE   3.50f   /* face region */
#define SALIENCY_WEIGHT_MOTION 1.50f   /* moving non-face region */

/*
 * Temporal masking: after SALIENCY_MEMORIZE_FRAMES consecutive frames at
 * high quality, the brain has "memorized" the region. Boost its QP by
 * SALIENCY_MEMORIZE_QP_BOOST to save bits.
 */
#define SALIENCY_MEMORIZE_FRAMES    45   /* 1.5 seconds at 30 fps */
#define SALIENCY_MEMORIZE_QP_BOOST   3   /* quantizer parameter boost */

/* Per-CTU saliency record */
typedef struct {
    float   weight;          /* saliency weight [WEIGHT_MIN, WEIGHT_MAX] */
    uint8_t temporal_count;  /* frames this CTU has been high-quality (for temporal masking) */
    bool    is_face;         /* CTU overlaps a detected face region */
    bool    is_motion;       /* significant motion vectors in this CTU */
    bool    is_text;         /* high edge density consistent with text */
} ctu_saliency_t;

/* Detected face rectangle (in pixel coordinates) */
typedef struct {
    uint16_t x, y, w, h;
    float    confidence;     /* 0.0–1.0 */
} sal_face_rect_t;

/* Saliency context — holds per-CTU records and temporal history */
typedef struct {
    ctu_saliency_t grid[SALIENCY_MAX_CTU_ROWS][SALIENCY_MAX_CTU_COLS];
    uint16_t ctu_cols;
    uint16_t ctu_rows;
    uint16_t frame_width;
    uint16_t frame_height;

    /* Detected faces this frame */
    sal_face_rect_t faces[8];
    int             n_faces;

    /* Global motion confidence (0 = static scene, 1 = all moving) */
    float   global_motion;
} sal_ctx_t;

/*
 * Initialize saliency context for a given frame size.
 * ctu_size: 64 (standard) or 32/128 for other CTU modes.
 */
void sal_init(sal_ctx_t *ctx, uint16_t frame_width, uint16_t frame_height,
              uint16_t ctu_size);

/*
 * Estimate saliency for all CTUs in a frame.
 *
 * luma:    Y plane (frame_width × frame_height, row-major)
 * mv_x/y: per-(ctu_cols × ctu_rows) motion vectors in quarter-pixel units;
 *          NULL for I-frames (all motion treated as zero).
 *
 * Updates ctx->grid[row][col].weight and .is_face/.is_motion/.is_text.
 * Does NOT reset temporal_count (that is maintained across frames).
 */
void sal_estimate(sal_ctx_t *ctx,
                  const uint8_t *luma,
                  const int16_t *mv_x, const int16_t *mv_y);

/*
 * Inject externally-detected face rectangles (e.g., from a higher-level
 * face tracker). Overrides the internal skin-tone heuristic for those CTUs.
 * Call before sal_estimate() or immediately after.
 */
void sal_set_faces(sal_ctx_t *ctx,
                   const sal_face_rect_t *faces, int n_faces);

/*
 * Update temporal masking counters after encoding a CTU.
 * Call once per encoded CTU with the actual QP used.
 * qp_target: the nominal QP for this frame.
 * qp_used:   the QP actually applied to this CTU.
 * If qp_used <= qp_target (high quality), increment temporal_count.
 * If qp_used > qp_target + 3 (degraded), reset temporal_count.
 */
void sal_update_temporal(sal_ctx_t *ctx, int ctu_col, int ctu_row,
                         int qp_target, int qp_used);

/*
 * Get saliency weight for a CTU at (col, row).
 * Returns SALIENCY_WEIGHT_BG if out of bounds.
 */
static inline float sal_weight(const sal_ctx_t *ctx, int col, int row)
{
    if (col < 0 || col >= (int)ctx->ctu_cols ||
        row < 0 || row >= (int)ctx->ctu_rows)
        return SALIENCY_WEIGHT_BG;
    return ctx->grid[row][col].weight;
}

/*
 * Get temporal masking QP boost for a CTU.
 * Returns SALIENCY_MEMORIZE_QP_BOOST if temporal_count >= threshold, else 0.
 */
static inline int sal_temporal_qp_boost(const sal_ctx_t *ctx, int col, int row)
{
    if (col < 0 || col >= (int)ctx->ctu_cols ||
        row < 0 || row >= (int)ctx->ctu_rows)
        return 0;
    return (ctx->grid[row][col].temporal_count >= SALIENCY_MEMORIZE_FRAMES)
           ? SALIENCY_MEMORIZE_QP_BOOST : 0;
}

#endif /* OPCODEC_SALIENCY_H */
