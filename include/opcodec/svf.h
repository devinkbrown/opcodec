/*
 * opcodec/svf.h — Scene Video Fingerprinting (SVF)
 *
 * Detects hard cuts and gradual scene transitions in real-time video to
 * trigger adaptive I-frames and prevent visual corruption from mispredicted
 * inter-frames following a scene change.
 *
 * A fixed GOP structure (e.g., I every 30 frames) wastes bits with an I-frame
 * on a static scene, yet creates corruption when a cut happens mid-GOP.
 * SVF removes both problems:
 *   - On hard cut: immediately forces an I-frame (even mid-GOP).
 *   - On static scene: extends GOP up to SVF_MAX_GOP to save bits.
 *
 * Algorithm:
 *   1. Compute a compact 32-byte frame fingerprint: 8×8 downsampled luma
 *      (average pool 8×8 blocks), stored as uint8_t[64] grid.
 *   2. Scene change metric: SAD between current and previous fingerprint,
 *      normalized by mean brightness.
 *   3. Hard cut: metric > SVF_HARD_CUT_THRESH → force I-frame next.
 *   4. Gradual transition: metric > SVF_GRAD_THRESH for N consecutive frames
 *      → also force I-frame (catches fade-ins, dissolves).
 *   5. Static scene: metric < SVF_STATIC_THRESH for all frames → extend GOP.
 *
 * Integration: call svf_analyze() after converting each raw frame to YUV.
 * If it returns SVF_FORCE_IFRAME, set frame_type = OPVIS_FRAME_I for that frame.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_SVF_H
#define OPCODEC_SVF_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Fingerprint grid: 8×8 cells (luma average per cell) */
#define SVF_FP_GRID      8
#define SVF_FP_BYTES     (SVF_FP_GRID * SVF_FP_GRID)  /* 64 bytes */

/* Scene change thresholds (SAD of 64 uint8_t values, 0–255 each) */
#define SVF_HARD_CUT_THRESH  3200u   /* ~50/255 average difference */
#define SVF_GRAD_THRESH       800u   /* ~12/255 average for gradual transition */
#define SVF_STATIC_THRESH     100u   /* ~1.5/255 average for truly static scene */
#define SVF_GRAD_CONSEC          4   /* consecutive frames above GRAD_THRESH */
#define SVF_MAX_GOP           120    /* maximum GOP extension for static scenes */

typedef enum {
    SVF_NORMAL       = 0,   /* no scene change detected */
    SVF_FORCE_IFRAME = 1,   /* hard cut or gradual transition complete */
    SVF_STATIC       = 2,   /* scene is static, extend GOP */
} svf_result_t;

/* SVF context */
typedef struct {
    uint8_t prev_fp[SVF_FP_BYTES];   /* previous frame fingerprint */
    uint8_t pprev_fp[SVF_FP_BYTES];  /* frame before previous (for grad detect) */
    uint32_t prev_sad;               /* SAD from last frame */
    int      grad_count;             /* consecutive frames with grad motion */
    int      static_count;           /* consecutive static frames */
    uint32_t frames_since_iframe;    /* frames since last forced I-frame */
    bool     has_prev;
    bool     initialized;

    /* Statistics */
    uint32_t hard_cuts_detected;
    uint32_t grad_cuts_detected;
    uint32_t iframes_forced;
} svf_ctx_t;

/* ── API ── */

void svf_init(svf_ctx_t *ctx);

/*
 * Analyze a raw luma frame and detect scene changes.
 *
 * luma:   width × height uint8_t luma plane
 * width, height: frame dimensions
 * Returns SVF_FORCE_IFRAME if an I-frame should be inserted this frame,
 *         SVF_STATIC if GOP can be extended, SVF_NORMAL otherwise.
 *
 * After a forced I-frame, the encoder should pass is_forced_iframe=true
 * to svf_notify_iframe() to reset the transition state.
 */
svf_result_t svf_analyze(svf_ctx_t *ctx,
                          const uint8_t *luma, uint16_t width, uint16_t height);

/*
 * Notify SVF that an I-frame was just encoded (whether forced or periodic).
 * Resets transition counters.
 */
void svf_notify_iframe(svf_ctx_t *ctx);

/*
 * Get the current scene change metric (SAD value, 0 = identical).
 */
static inline uint32_t svf_get_sad(const svf_ctx_t *ctx) { return ctx->prev_sad; }

/*
 * Get number of consecutive static frames (useful for rate control).
 */
static inline int svf_static_frames(const svf_ctx_t *ctx) { return ctx->static_count; }

#endif /* OPCODEC_SVF_H */
