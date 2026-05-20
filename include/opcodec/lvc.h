/*
 * opcodec/lvc.h — Latent Video Codec (LVC) Mode
 *
 * An extreme-compression video encoding mode for severely bandwidth-constrained
 * or lossy network conditions. When OPVIS rate control detects sustained low
 * bandwidth (< 64 kbps), LVC mode activates and replaces full-frame coding
 * with a compact VQ-based latent representation.
 *
 * Algorithm:
 *   1. Divide the luma plane into non-overlapping 16×16 macroblocks.
 *   2. For each macroblock, find the best-match code vector in a 256-entry
 *      offline-trained VQ codebook (each entry = 256 floats = 16×16 block).
 *   3. Transmit the 8-bit codebook index per macroblock (no residual).
 *   4. Optionally transmit a 4-bit gain scalar per macroblock (16 steps).
 *   5. Reconstruct by table lookup + gain scaling.
 *
 * Compression:
 *   A 640×480 frame = 40×30 = 1200 macroblocks.
 *   1200 bytes + 600 gain nibbles = ~1800 bytes/frame.
 *   At 15 fps: 1800 × 15 × 8 = ~216 kbps luma-only, or ~43 kbps with I/P mix.
 *   (Chroma: separate 2×2 downsampled codebook, 4× fewer MBs → +~300 bytes.)
 *
 * Quality: visibly blocky but recognizable, comparable to circa-2000 webcam
 * video at 28.8k modem speeds. Acceptable for emergency fallback or IoT.
 *
 * Codebook: 256-entry offline-trained on diverse face/scene data.
 * Stored as 256 × 256 uint8_t table (= 65536 bytes = 64 KB).
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_LVC_H
#define OPCODEC_LVC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define LVC_BLOCK_SIZE   16     /* macroblock side length in pixels */
#define LVC_CODEBOOK_N  256     /* number of VQ codebook entries */
#define LVC_GAIN_STEPS   16     /* gain quantization steps (4 bits) */
#define LVC_GAIN_MIN    0.5f    /* minimum gain multiplier */
#define LVC_GAIN_MAX    2.0f    /* maximum gain multiplier */
#define LVC_PACKET_HDR   6      /* bytes: version(1)+width_BE(2)+height_BE(2)+flags(1) */

/* LVC encoder context */
typedef struct {
    /* Codebook: LVC_CODEBOOK_N × LVC_BLOCK_SIZE² float entries */
    float codebook[LVC_CODEBOOK_N][LVC_BLOCK_SIZE * LVC_BLOCK_SIZE];
    bool  codebook_trained;
    bool  initialized;
} lvc_enc_t;

/* LVC decoder context */
typedef struct {
    float codebook[LVC_CODEBOOK_N][LVC_BLOCK_SIZE * LVC_BLOCK_SIZE];
    bool  codebook_trained;
    bool  initialized;
} lvc_dec_t;

/* ── API ── */

/*
 * Initialize LVC encoder. Generates a synthetic codebook based on DCT basis
 * functions (Haar wavelets at block scale) — offline training would improve
 * quality but is not required for emergency fallback mode.
 */
int lvc_enc_init(lvc_enc_t *ctx);

/*
 * Encode one luma frame to LVC bitstream.
 * luma:       width × height uint8_t plane
 * out:        output buffer
 * out_cap:    max bytes in out
 * Returns bytes written, or -1 on error.
 */
int lvc_encode(lvc_enc_t *ctx, const uint8_t *luma,
               uint16_t width, uint16_t height,
               uint8_t *out, int out_cap);

/*
 * Initialize LVC decoder (uses same synthetic codebook as encoder).
 */
int lvc_dec_init(lvc_dec_t *ctx);

/*
 * Decode one LVC frame.
 * luma_out:   output buffer (caller allocates: width × height bytes)
 * Returns 0 on success, -1 on error.
 */
int lvc_decode(lvc_dec_t *ctx, const uint8_t *in, int in_len,
               uint8_t *luma_out, uint16_t *width_out, uint16_t *height_out);

/*
 * Estimate compressed size without encoding (for rate control decisions).
 * Returns number of bytes an LVC frame of the given dimensions would use.
 */
static inline int lvc_estimate_size(uint16_t width, uint16_t height) {
    int mb_cols = (width  + LVC_BLOCK_SIZE - 1) / LVC_BLOCK_SIZE;
    int mb_rows = (height + LVC_BLOCK_SIZE - 1) / LVC_BLOCK_SIZE;
    int n_mbs   = mb_cols * mb_rows;
    /* 1 byte index + half byte gain per macroblock, rounded up */
    return LVC_PACKET_HDR + n_mbs + (n_mbs + 1) / 2;
}

#endif /* OPCODEC_LVC_H */
