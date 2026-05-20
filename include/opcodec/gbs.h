/*
 * opcodec/gbs.h — Generative Background Synthesis (GBS)
 *
 * Separates each video frame into foreground (speaker/subject) and background,
 * transmits only the foreground at full quality, and synthesizes the background
 * at the receiver using a compact descriptor.
 *
 * Algorithm overview:
 *
 *   Encoder side:
 *     1. Background estimation: EWMA of past N frames in static regions
 *        (identified by near-zero motion vectors from the codec ME pass).
 *     2. Foreground mask: binary mask per 16×16 block (bit=1 if block has
 *        motion or texture significantly different from background model).
 *     3. Background descriptor: updated every GBS_BG_UPDATE_INTERVAL_FRAMES
 *        frames. Sent as: 32 DCT-domain band energies + 8-bit dominant hue/sat
 *        (= 5 bytes/update). Rate: 5 bytes × (30fps / 30) = 5 bytes/sec.
 *     4. Foreground-only bitstream: the OPVIS encoder is instructed to skip
 *        background blocks (replace with SKIP/palette-copy from ref), saving
 *        ~30–60% bitrate for talking-head video.
 *
 *   Decoder side:
 *     1. Background synthesizer: holds a low-frequency background texture
 *        updated from the descriptor. Background blocks are filled from this.
 *     2. Foreground blending: decoded foreground overlaid on synthesized BG.
 *     3. Optional virtual background: caller can inject a replacement background
 *        image; GBS clips the foreground mask and composites over it.
 *
 * Privacy modes:
 *   GBS_MODE_BLUR:    Replace background with blurred version of itself.
 *   GBS_MODE_REPLACE: Replace background with caller-provided RGBA image.
 *   GBS_MODE_ENCODE:  Compress background with descriptor (default).
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_GBS_H
#define OPCODEC_GBS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define GBS_BLOCK_SIZE    16    /* same as OPVIS MB size */
#define GBS_MAX_WIDTH    1920
#define GBS_MAX_HEIGHT   1080
#define GBS_MAX_MB_COLS  (GBS_MAX_WIDTH  / GBS_BLOCK_SIZE)
#define GBS_MAX_MB_ROWS  (GBS_MAX_HEIGHT / GBS_BLOCK_SIZE)
#define GBS_MAX_BLOCKS   (GBS_MAX_MB_COLS * GBS_MAX_MB_ROWS)
#define GBS_BG_UPDATE_INTERVAL_FRAMES 30   /* send BG descriptor once per second at 30fps */
#define GBS_MOTION_THRESH  12   /* pixel difference threshold for foreground detection */
#define GBS_BG_ALPHA      0.05f /* background EWMA learning rate (slow learner) */
#define GBS_DESCRIPTOR_BYTES 37 /* version(1)+bands(32)+hue(1)+sat(1)+lum(1)+flags(1) */

/* Background synthesis mode */
typedef enum {
    GBS_MODE_ENCODE  = 0,  /* transmit compact BG descriptor */
    GBS_MODE_BLUR    = 1,  /* blur background, don't transmit */
    GBS_MODE_REPLACE = 2,  /* caller-provided replacement BG */
} gbs_mode_t;

/* Per-block foreground flag (packed as bit array) */
typedef struct {
    uint8_t bits[(GBS_MAX_BLOCKS + 7) / 8];
    int     mb_cols, mb_rows;
} gbs_mask_t;

/* Background descriptor (compact BG state for transmission) */
typedef struct {
    uint8_t band_energy[32];  /* 8-bit log-energy per Mel band */
    uint8_t dominant_hue;     /* 0–255 hue of dominant BG color */
    uint8_t dominant_sat;     /* saturation */
    uint8_t dominant_lum;     /* luminance */
    uint8_t flags;            /* bit0=valid, bit1=update_needed */
} gbs_bg_desc_t;

/* GBS encoder context */
typedef struct {
    /* Background model: per-pixel float luma EWMA */
    float    bg_y[GBS_MAX_WIDTH * GBS_MAX_HEIGHT / 4]; /* 1/4-res model to save memory */
    uint16_t bg_width, bg_height;  /* actual dimensions */
    uint16_t bg_w4, bg_h4;         /* 1/4-res dimensions */

    /* Per-block motion flags (foreground mask) */
    gbs_mask_t fg_mask;

    /* Background descriptor */
    gbs_bg_desc_t desc;

    gbs_mode_t mode;
    int      frame_count;
    int      bg_update_countdown;
    bool     initialized;
} gbs_enc_t;

/* GBS decoder / compositor context */
typedef struct {
    /* Synthesized background texture (full resolution) */
    uint8_t *bg_y;          /* caller-provided buffer: width × height */
    uint8_t *bg_u, *bg_v;   /* chroma planes (width/2 × height/2) */

    /* Virtual background (optional replacement) */
    const uint8_t *vbg_y;
    const uint8_t *vbg_u, *vbg_v;
    bool     use_vbg;

    gbs_bg_desc_t desc;
    gbs_mode_t    mode;
    uint16_t      width, height;
    bool          initialized;
} gbs_dec_t;

/* ── Encoder API ── */

int gbs_enc_init(gbs_enc_t *ctx, uint16_t width, uint16_t height, gbs_mode_t mode);
void gbs_enc_free(gbs_enc_t *ctx);  /* no-op (no heap alloc, provided for symmetry) */

/*
 * Update background model from a decoded/reconstructed frame.
 * luma:    width × height uint8_t luma plane
 * mv_sad:  per-MB SAD values from motion estimation (NULL = assume all static)
 *          Higher SAD → more likely foreground. Array of mb_cols × mb_rows uint32_t.
 *
 * After this call, ctx->fg_mask reflects the current foreground segmentation.
 */
void gbs_enc_update(gbs_enc_t *ctx,
                    const uint8_t *luma,
                    const uint32_t *mv_sad);

/*
 * Serialize the background descriptor to at most GBS_DESCRIPTOR_BYTES bytes.
 * Only sends if the descriptor has changed or update is due.
 * Returns bytes written (0 if no update needed).
 */
int gbs_enc_serialize(gbs_enc_t *ctx, uint8_t *out, int out_cap);

/*
 * Get the foreground mask for the current frame.
 * Returns pointer to internal mask — valid until next gbs_enc_update() call.
 */
static inline const gbs_mask_t *gbs_enc_get_mask(const gbs_enc_t *ctx) {
    return &ctx->fg_mask;
}

/* Query whether a specific macroblock is foreground */
static inline bool gbs_is_foreground(const gbs_mask_t *mask, int mb_col, int mb_row) {
    int idx = mb_row * mask->mb_cols + mb_col;
    return (mask->bits[idx / 8] >> (idx & 7)) & 1;
}

/* ── Decoder API ── */

/*
 * Initialize GBS decoder. bg_y/u/v: caller-provided output buffers for the
 * synthesized background (size: y = w×h, u/v = w/2 × h/2).
 */
int gbs_dec_init(gbs_dec_t *ctx, uint16_t width, uint16_t height,
                 uint8_t *bg_y, uint8_t *bg_u, uint8_t *bg_v);

/*
 * Decode and apply a GBS background descriptor packet.
 * Returns 0 on success, -1 on parse error.
 */
int gbs_dec_apply(gbs_dec_t *ctx, const uint8_t *in, int in_len);

/*
 * Set a virtual background (caller-owned, must remain valid).
 * Pass NULL to disable virtual background.
 */
void gbs_dec_set_vbg(gbs_dec_t *ctx,
                     const uint8_t *y, const uint8_t *u, const uint8_t *v);

/*
 * Composite foreground over synthesized/virtual background in-place.
 * fg_y/u/v: decoded frame planes (modified in-place for background regions)
 * fg_mask:  per-block foreground mask received from encoder side-channel
 */
void gbs_dec_composite(gbs_dec_t *ctx,
                        uint8_t *fg_y, uint8_t *fg_u, uint8_t *fg_v,
                        const gbs_mask_t *fg_mask);

#endif /* OPCODEC_GBS_H */
