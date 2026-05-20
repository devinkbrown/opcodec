/* opcodec/tnr.c — Temporal Noise Reduction
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/tnr.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Apply one-pass 3×3 dilation to binary mask (mask[i]=1 means motion). */
static void dilate_motion_mask(uint8_t *mask, int width, int height) {
    /* Two-pass: horizontal then vertical, each radius-1.
     * Horizontal pass: mark pixel if any of its horizontal neighbors is set. */
    for (int y = 0; y < height; y++) {
        uint8_t *row = mask + y * width;
        uint8_t prev = 0;
        /* Forward */
        for (int x = 0; x < width; x++) {
            uint8_t cur = row[x];
            if (prev) row[x] = 1;
            prev = cur;
        }
        /* Backward (to propagate right-to-left too) */
        prev = 0;
        for (int x = width - 1; x >= 0; x--) {
            uint8_t cur = row[x];
            if (prev) row[x] = 1;
            prev = cur;
        }
    }
    /* Vertical pass */
    for (int x = 0; x < width; x++) {
        uint8_t prev = 0;
        for (int y = 0; y < height; y++) {
            uint8_t cur = mask[y * width + x];
            if (prev) mask[y * width + x] = 1;
            prev = cur;
        }
        prev = 0;
        for (int y = height - 1; y >= 0; y--) {
            uint8_t cur = mask[y * width + x];
            if (prev) mask[y * width + x] = 1;
            prev = cur;
        }
    }
}

int opvis_tnr_apply(opvis_tnr_ctx_t *ctx,
                    uint8_t *luma, int width, int height,
                    float alpha, uint8_t motion_thresh)
{
    if (!ctx || !luma || width <= 0 || height <= 0) return -1;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 0.95f) alpha = 0.95f;

    const int n = width * height;

    /* First call: allocate buffer and initialise to the first frame (no filtering). */
    if (!ctx->initialized || ctx->width != width || ctx->height != height) {
        free(ctx->buf);
        ctx->buf = malloc((size_t)n);
        if (!ctx->buf) { ctx->initialized = false; return -1; }
        memcpy(ctx->buf, luma, (size_t)n);
        ctx->width  = width;
        ctx->height = height;
        ctx->initialized = true;
        return 0;  /* first frame: nothing to filter against */
    }

    /* Build motion mask: 1 if pixel is above motion threshold. */
    uint8_t *mask = malloc((size_t)n);
    if (!mask) return -1;

    const uint8_t *prev = ctx->buf;
    for (int i = 0; i < n; i++) {
        int diff = (int)luma[i] - (int)prev[i];
        if (diff < 0) diff = -diff;
        mask[i] = (diff > (int)motion_thresh) ? 1u : 0u;
    }

    /* Dilate mask by 2 passes (each pass = radius 1) to get ~3px border. */
    dilate_motion_mask(mask, width, height);
    dilate_motion_mask(mask, width, height);

    /* Apply recursive temporal filter and update stored frame. */
    const float beta = 1.0f - alpha;
    for (int i = 0; i < n; i++) {
        if (mask[i]) {
            /* Moving pixel: pass through (no temporal blend). */
            ctx->buf[i] = luma[i];
        } else {
            /* Static pixel: recursive blend. */
            int blended = (int)(alpha * (float)prev[i] + beta * (float)luma[i] + 0.5f);
            if (blended < 0)   blended = 0;
            if (blended > 255) blended = 255;
            ctx->buf[i] = (uint8_t)blended;
            luma[i]     = (uint8_t)blended;
        }
    }

    free(mask);
    return 0;
}

void opvis_tnr_free(opvis_tnr_ctx_t *ctx)
{
    if (!ctx) return;
    free(ctx->buf);
    ctx->buf = NULL;
    ctx->initialized = false;
}
