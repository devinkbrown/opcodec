/* opcodec/tnr.h — Temporal Noise Reduction pre-filter for OPVIS encoder
 *
 * Motion-adaptive recursive temporal filter.  Applied to the luma plane
 * before encoding to suppress sensor/camera noise in static regions while
 * preserving motion edges (no ghosting on moving objects).
 *
 * Algorithm:
 *   For each pixel:
 *     if |cur - prev_filtered| < motion_thresh:    // static pixel
 *       filtered = alpha * prev_filtered + (1-alpha) * cur
 *     else:                                         // motion pixel
 *       filtered = cur                              // pass through unchanged
 *
 * Then apply a 2-pass morphological dilation to the motion mask to prevent
 * temporal smearing at motion boundaries (3-pixel border).
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_TNR_H
#define OPCODEC_TNR_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Opaque context that stores the previous filtered frame. */
typedef struct opvis_tnr_ctx {
    uint8_t *buf;       /* previous filtered luma plane (width * height bytes) */
    int      width;
    int      height;
    bool     initialized;
} opvis_tnr_ctx_t;

/*
 * Apply TNR to one luma plane in-place.
 *
 *  ctx          — TNR state (must remain alive between frames)
 *  luma         — current luma plane (width * height), modified in-place
 *  width/height — frame dimensions
 *  alpha        — temporal weight for previous frame (0.0 = no filtering,
 *                 0.5 = strong; recommended 0.4–0.6 for webcam video)
 *  motion_thresh — per-pixel difference threshold above which the pixel is
 *                  considered "moving" and TNR is suppressed (recommended 12–24)
 *
 * Returns 0 on success, -1 on allocation failure (first call only).
 */
int opvis_tnr_apply(opvis_tnr_ctx_t *ctx,
                    uint8_t *luma, int width, int height,
                    float alpha, uint8_t motion_thresh);

/* Free the internal buffer.  ctx itself is caller-owned. */
void opvis_tnr_free(opvis_tnr_ctx_t *ctx);

#endif /* OPCODEC_TNR_H */
