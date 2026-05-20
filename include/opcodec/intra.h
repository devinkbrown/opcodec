/* opcodec/intra.h — Intra prediction for OPVIS codec
 *
 * Provides two intra prediction APIs:
 *   - Legacy 6-mode H.264-style (DC, Horiz, Vert, DiagDL, DiagDR, Planar)
 *   - HEVC 35-mode (Planar=0, DC=1, Angular 2–34)
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_INTRA_H
#define OPCODEC_INTRA_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Legacy 6-mode API (backward compatible) ---- */

typedef enum {
    INTRA_DC        = 0,
    INTRA_HORIZ     = 1,
    INTRA_VERT      = 2,
    INTRA_DIAG_DL   = 3,
    INTRA_DIAG_DR   = 4,
    INTRA_PLANE     = 5,
    INTRA_NUM_MODES = 6
} intra_mode_t;

void intra_predict_16x16(intra_mode_t mode,
                         const uint8_t *top, const uint8_t *left,
                         uint8_t top_left,
                         uint8_t pred[16][16]);

intra_mode_t intra_find_best_mode(const uint8_t orig[16][16],
                                  const uint8_t *top, const uint8_t *left,
                                  uint8_t top_left);

void intra_predict_4x4(intra_mode_t mode,
                       const uint8_t *top, const uint8_t *left,
                       uint8_t top_left,
                       uint8_t pred[4][4]);

intra_mode_t intra_find_best_mode_4x4(const uint8_t orig[4][4],
                                       const uint8_t *top, const uint8_t *left,
                                       uint8_t top_left);

bool intra_get_neighbors_16x16(const uint8_t *frame, uint16_t width, uint16_t height,
                               uint16_t mb_x, uint16_t mb_y,
                               uint8_t top_out[16], uint8_t left_out[16],
                               uint8_t *top_left_out);

bool intra_get_neighbors_4x4(const uint8_t *frame, uint16_t width, uint16_t height,
                              uint16_t block_x, uint16_t block_y,
                              uint8_t top_out[4], uint8_t left_out[4],
                              uint8_t *top_left_out);

/* ---- HEVC 35-mode API ---- */

/* Mode assignments (matches HEVC spec):
 *   0        Planar  — weighted bilinear blend from top + left boundaries
 *   1        DC      — average of available top and left neighbors
 *   2–34     Angular — 33 directional modes spaced ~5.45° apart
 *              mode 2  ≈ vertical-like    (almost straight down)
 *              mode 18 = true vertical    (exactly top→down)
 *              mode 26 = true horizontal  (exactly left→right)
 *              mode 34 ≈ horizontal-like  (almost straight right)
 *
 * Mode is encoded in the bitstream as 6 bits (0–34; codes 35–63 reserved).
 */
#define INTRA_HEVC_PLANAR        0
#define INTRA_HEVC_DC            1
#define INTRA_HEVC_ANGULAR_FIRST 2
#define INTRA_HEVC_ANGULAR_LAST  34
#define INTRA_HEVC_VERT         10   /* angle=0 in vertical group — pure top→down */
#define INTRA_HEVC_HORIZ        26   /* angle=0 in horizontal group — pure left→right */
#define INTRA_NUM_MODES_HEVC    35

typedef uint8_t intra_mode_hevc_t;

/*
 * Predict an N×N block using HEVC 35-mode intra prediction.
 *
 * mode       — HEVC intra mode (0–34)
 * block_size — 4, 8, 16, 32, or 64
 * top        — top neighbor pixels (block_size values), NULL if top edge
 * top_right  — top-right extension (block_size values), NULL if unavailable
 * left       — left neighbor pixels (block_size values), NULL if left edge
 * top_left   — single top-left corner pixel
 * pred       — output prediction buffer [block_size * block_size], row-major
 */
void intra_predict_hevc(intra_mode_hevc_t mode, int block_size,
                        const uint8_t *top, const uint8_t *top_right,
                        const uint8_t *left, uint8_t top_left,
                        uint8_t *pred);

/*
 * Find the best HEVC intra mode for an N×N block.
 * Uses SATD for speed across all 35 modes.
 *
 * orig       — original pixels [block_size * block_size], row-major
 * block_size — 4, 8, 16, 32, or 64
 * top, top_right, left, top_left — neighbor pixels
 *
 * Returns best mode (0–34).
 */
intra_mode_hevc_t intra_find_best_mode_hevc(const uint8_t *orig, int block_size,
                                            const uint8_t *top,
                                            const uint8_t *top_right,
                                            const uint8_t *left,
                                            uint8_t top_left);

/*
 * Extract HEVC intra neighbors for an N×N block from a frame.
 *
 * frame        — luma plane
 * frame_width  — frame width
 * frame_height — frame height
 * bx, by       — block top-left in pixels
 * block_size   — block dimension in pixels
 * top_out      — output top neighbors (block_size pixels)
 * top_right_out — output top-right extension (block_size pixels, or NULL)
 * left_out     — output left neighbors (block_size pixels)
 * top_left_out — output corner pixel
 *
 * Returns true if any neighbors available.
 */
bool intra_get_neighbors_hevc(const uint8_t *frame,
                              uint16_t frame_width, uint16_t frame_height,
                              uint16_t bx, uint16_t by, int block_size,
                              uint8_t *top_out, uint8_t *top_right_out,
                              uint8_t *left_out, uint8_t *top_left_out);

#endif /* OPCODEC_INTRA_H */
