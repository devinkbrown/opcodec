/*
 * opcodec/svf.c — Scene Video Fingerprinting implementation
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/svf.h"
#include <string.h>

/* ── Fingerprint computation ─────────────────────────────────────────── */

static void compute_fingerprint(const uint8_t *luma, uint16_t width, uint16_t height,
                                  uint8_t *fp)
{
    /* Divide frame into SVF_FP_GRID × SVF_FP_GRID cells.
     * Compute average luma per cell (average-pool). */
    for (int gy = 0; gy < SVF_FP_GRID; gy++) {
        for (int gx = 0; gx < SVF_FP_GRID; gx++) {
            int x0 = (int)((uint32_t)gx       * width  / SVF_FP_GRID);
            int x1 = (int)((uint32_t)(gx + 1) * width  / SVF_FP_GRID);
            int y0 = (int)((uint32_t)gy       * height / SVF_FP_GRID);
            int y1 = (int)((uint32_t)(gy + 1) * height / SVF_FP_GRID);
            if (x1 > width)  x1 = width;
            if (y1 > height) y1 = height;
            if (x1 <= x0 || y1 <= y0) { fp[gy * SVF_FP_GRID + gx] = 128; continue; }

            uint32_t sum = 0;
            int count = 0;
            for (int y = y0; y < y1; y++) {
                for (int x = x0; x < x1; x++) {
                    sum += luma[y * width + x];
                    count++;
                }
            }
            fp[gy * SVF_FP_GRID + gx] = (count > 0) ? (uint8_t)(sum / (uint32_t)count) : 128;
        }
    }
}

static uint32_t fp_sad(const uint8_t *a, const uint8_t *b)
{
    uint32_t sad = 0;
    for (int i = 0; i < SVF_FP_BYTES; i++) {
        int d = (int)a[i] - (int)b[i];
        sad += (uint32_t)(d < 0 ? -d : d);
    }
    return sad;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void svf_init(svf_ctx_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->initialized = true;
}

svf_result_t svf_analyze(svf_ctx_t *ctx,
                           const uint8_t *luma, uint16_t width, uint16_t height)
{
    if (!ctx || !ctx->initialized || !luma || width == 0 || height == 0)
        return SVF_NORMAL;

    uint8_t fp[SVF_FP_BYTES];
    compute_fingerprint(luma, width, height, fp);

    if (!ctx->has_prev) {
        memcpy(ctx->prev_fp, fp, SVF_FP_BYTES);
        memcpy(ctx->pprev_fp, fp, SVF_FP_BYTES);
        ctx->has_prev = true;
        ctx->prev_sad = 0;
        ctx->frames_since_iframe = 0;
        return SVF_NORMAL;
    }

    uint32_t sad = fp_sad(fp, ctx->prev_fp);
    ctx->prev_sad = sad;
    ctx->frames_since_iframe++;

    svf_result_t result = SVF_NORMAL;

    /* Hard cut detection */
    if (sad > SVF_HARD_CUT_THRESH) {
        ctx->grad_count  = 0;
        ctx->static_count = 0;
        ctx->hard_cuts_detected++;
        ctx->iframes_forced++;
        result = SVF_FORCE_IFRAME;
        goto update_fp;
    }

    /* Gradual transition detection */
    if (sad > SVF_GRAD_THRESH) {
        ctx->grad_count++;
        ctx->static_count = 0;
        if (ctx->grad_count >= SVF_GRAD_CONSEC) {
            ctx->grad_count = 0;
            ctx->grad_cuts_detected++;
            ctx->iframes_forced++;
            result = SVF_FORCE_IFRAME;
        }
        goto update_fp;
    }
    ctx->grad_count = 0;

    /* Static scene detection */
    if (sad < SVF_STATIC_THRESH) {
        ctx->static_count++;
        if (ctx->static_count >= 5 && ctx->frames_since_iframe < SVF_MAX_GOP)
            result = SVF_STATIC;
    } else {
        ctx->static_count = 0;
    }

update_fp:
    memcpy(ctx->pprev_fp, ctx->prev_fp, SVF_FP_BYTES);
    memcpy(ctx->prev_fp,  fp,           SVF_FP_BYTES);
    return result;
}

void svf_notify_iframe(svf_ctx_t *ctx)
{
    if (!ctx) return;
    ctx->frames_since_iframe = 0;
    ctx->grad_count  = 0;
    ctx->static_count = 0;
}
