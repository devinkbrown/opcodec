/*
 * opcodec/vplc.h — Video Packet Loss Concealment (VPLC)
 *
 * When one or more video packets are lost, VPLC conceals the damage by
 * synthesizing missing macroblocks from available neighboring data rather
 * than displaying frozen artifacts or hard cuts.
 *
 * Concealment strategies (applied in priority order):
 *
 *   1. MOTION_COPY (preferred for inter-frames):
 *      Copy the motion vector from a neighboring macroblock (spatial MV
 *      copy) and apply it to the reference frame. Works well for smooth
 *      translational motion.
 *
 *   2. BOUNDARY_MATCH (fallback for intra/I-frames):
 *      Fill the lost block by spatial interpolation from available neighbor
 *      pixels (boundary matching error concealment, H.264 Annex D style).
 *      Uses a weighted average of top, left, bottom, and right boundary rows.
 *
 *   3. TEMPORAL_COPY (last resort):
 *      Simply copy the corresponding block from the previous frame.
 *      Produces acceptable quality for static content.
 *
 * SARDO integration: the concealment strategy is biased by the saliency
 * map — foreground/face CTUs use MOTION_COPY, background CTUs accept
 * TEMPORAL_COPY to reduce compute.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_VPLC_H
#define OPCODEC_VPLC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define VPLC_MB_SIZE     16   /* macroblock size (matches OPVIS MB) */

typedef enum {
    VPLC_STRAT_MOTION_COPY   = 0,
    VPLC_STRAT_BOUNDARY_MATCH = 1,
    VPLC_STRAT_TEMPORAL_COPY  = 2,
} vplc_strategy_t;

/* Per-MB loss status passed by the jitter/RTP layer */
typedef struct {
    uint8_t lost;   /* 1 = this MB was lost and needs concealment */
} vplc_mb_status_t;

/* VPLC concealment result statistics */
typedef struct {
    uint32_t mbs_concealed;
    uint32_t motion_copy_used;
    uint32_t boundary_match_used;
    uint32_t temporal_copy_used;
} vplc_stats_t;

/* VPLC context (stateless per-frame, no heap alloc) */
typedef struct {
    vplc_strategy_t default_strategy;
    vplc_stats_t    stats;
    bool            initialized;
} vplc_ctx_t;

/* ── API ── */

void vplc_init(vplc_ctx_t *ctx, vplc_strategy_t strategy);

/*
 * Conceal lost macroblocks in a decoded frame.
 *
 * cur_y, cur_u, cur_v: current partially-decoded frame (lost MBs have
 *   undefined content — VPLC overwrites those positions).
 *   Sizes: y = width×height, u/v = (width/2)×(height/2)
 *
 * ref_y, ref_u, ref_v: previous (reference) frame — used for temporal copy.
 *
 * mvs: motion vector array from the bitstream (mb_cols × mb_rows opvis_mv_t).
 *   Pass NULL for I-frames (forces BOUNDARY_MATCH).
 *
 * lost_map: array of mb_cols × mb_rows vplc_mb_status_t indicating which MBs
 *   to conceal. Pass NULL to conceal nothing (stats update only).
 *
 * Returns number of MBs concealed.
 */
int vplc_conceal(vplc_ctx_t *ctx,
                  uint8_t *cur_y, uint8_t *cur_u, uint8_t *cur_v,
                  const uint8_t *ref_y, const uint8_t *ref_u, const uint8_t *ref_v,
                  const void *mvs,           /* opvis_mv_t* — declared void* to avoid dep */
                  const vplc_mb_status_t *lost_map,
                  uint16_t width, uint16_t height);

static inline const vplc_stats_t *vplc_get_stats(const vplc_ctx_t *ctx) {
    return &ctx->stats;
}

void vplc_reset_stats(vplc_ctx_t *ctx);

#endif /* OPCODEC_VPLC_H */
