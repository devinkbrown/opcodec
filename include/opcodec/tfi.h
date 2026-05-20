/*
 * opcodec/tfi.h — Temporal Frame Interpolation
 *
 * Synthesizes intermediate video frames from two decoded reference frames
 * using motion-compensated blending. Enables the encoder to skip every
 * other frame ("INTERP" frame type) and save ~50% of video bitrate at
 * the cost of client-side synthesis.
 *
 * Algorithm:
 *   1. Divide each frame into 8×8 blocks.
 *   2. For each block: search both reference frames for best-match block
 *      (forward and backward motion estimation via block-matching SAD).
 *   3. Blend: output[i] = (alpha * fwd_block + (1−alpha) * bwd_block + 0.5)
 *      where alpha = temporal position of interpolated frame (default 0.5).
 *   4. Apply simple spatial smoothing at block boundaries to reduce artifacts.
 *
 * Bitstream:
 *   An INTERP frame emits only a 3-byte hint packet:
 *     byte 0: frame_type = OPVIS_FRAME_INTERP (0x03)
 *     byte 1: alpha_q8  (alpha * 256, 128 = midpoint)
 *     byte 2: quality hint (0 = skip synthesis, use blend; 1 = use motion)
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_TFI_H
#define OPCODEC_TFI_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define TFI_BLOCK_SIZE       8     /* block size for motion estimation */
#define TFI_SEARCH_RANGE    16     /* search range in pixels */
#define TFI_MAX_WIDTH     4096
#define TFI_MAX_HEIGHT    4096
#define TFI_HINT_SIZE        3     /* bytes in the hint packet */

/* TFI context — holds scratch buffers for block matching */
typedef struct {
    /* Scratch motion field: one MV per 8×8 block */
    int16_t *fwd_mvx;  /* forward  MV x components */
    int16_t *fwd_mvy;
    int16_t *bwd_mvx;  /* backward MV x components */
    int16_t *bwd_mvy;
    int      mv_cols;
    int      mv_rows;
    uint16_t width;
    uint16_t height;
    bool     initialized;
} tfi_ctx_t;

/*
 * Initialize TFI context for a given frame resolution.
 * Allocates motion-field scratch buffers.
 * Returns 0 on success, -1 on allocation failure.
 */
int tfi_init(tfi_ctx_t *ctx, uint16_t width, uint16_t height);

/*
 * Free resources held by TFI context.
 */
void tfi_free(tfi_ctx_t *ctx);

/*
 * Synthesize an interpolated frame between prev and next reference frames.
 *
 * prev:   luma plane of the earlier reference (width×height)
 * next:   luma plane of the later  reference  (width×height)
 * out:    output luma plane (caller-allocated, width×height)
 * alpha:  temporal position [0.0, 1.0]; 0.5 = exact midpoint
 *
 * Returns 0 on success.
 */
int tfi_interpolate(tfi_ctx_t *ctx,
                    const uint8_t *prev, const uint8_t *next,
                    uint8_t *out,
                    float alpha);

/*
 * Write the 3-byte INTERP hint packet to dst.
 * alpha_q8: alpha * 256 (128 = midpoint).
 * use_motion: 1 = decoder should run motion compensation,
 *             0 = simple blend (cheaper but lower quality).
 */
void tfi_write_hint(uint8_t *dst, uint8_t alpha_q8, uint8_t use_motion);

/*
 * Parse the 3-byte INTERP hint packet.
 * Returns false if dst does not contain a valid hint.
 */
bool tfi_read_hint(const uint8_t *src, uint8_t *alpha_q8, uint8_t *use_motion);

/*
 * Estimate whether interpolation quality is sufficient for a given block.
 * Returns the SAD between the synthesized block and a ground-truth block
 * (encoder-side quality check). If SAD > threshold, caller should encode
 * a real frame instead.
 */
uint32_t tfi_measure_quality(const uint8_t *synth, const uint8_t *truth,
                              int width, int height);

#endif /* OPCODEC_TFI_H */
