/* opcodec/screen.h — Screen content coding helpers for OPVIS codec
 *
 * Provides palette mode and Intra Block Copy (IBC) for screen-content frames.
 * Palette mode encodes blocks with ≤ 16 distinct luma values as index maps.
 * IBC copies already-decoded pixels from the current frame, exploiting
 * repeated UI elements (icons, text, window borders).
 *
 * Integration: palette is signaled as HEVC intra mode 35, IBC as mode 36.
 * The intra model must therefore be initialized with SCREEN_INTRA_NUM_MODES.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_SCREEN_H
#define OPCODEC_SCREEN_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Mode code slots (extend HEVC 35-mode intra) ---- */

#define SCREEN_MODE_PALETTE      35   /* intra mode slot for palette coding */
#define SCREEN_MODE_IBC          36   /* intra mode slot for intra block copy */
#define SCREEN_INTRA_NUM_MODES   37   /* total modes: 35 HEVC + palette + IBC */

/* ---- Palette parameters ---- */

#define SCREEN_PALETTE_MAX       16   /* max distinct Y values per block */
#define SCREEN_PALETTE_MIN_SIZE   8   /* min block size eligible for palette */

/* ---- IBC parameters ---- */

#define SCREEN_IBC_MIN_SIZE       8   /* min block size eligible for IBC */
#define SCREEN_IBC_TABLE_SIZE  4096   /* hash table entries (16 KB) */

/* ---- Palette API ---- */

/* Detect if an N×N luma block can be palette-coded (≤ SCREEN_PALETTE_MAX
 * distinct values).  On success, fills palette_y[0..n_entries-1] with
 * the sorted distinct values and indices[0..N*N-1] with per-pixel indices.
 * Returns false if the block has too many distinct values.
 */
bool screen_palette_detect(const uint8_t *block, int N,
                           uint8_t *palette_y, int *n_entries,
                           uint8_t *indices);

/* Reconstruct an N×N block from palette and indices into dst (row-major,
 * stride = N).
 */
void screen_palette_reconstruct(const uint8_t *palette_y, const uint8_t *indices,
                                int N, uint8_t *dst);

/* ---- IBC API ---- */

/* Compute the FNV-1a hash of the SCREEN_IBC_MIN_SIZE × SCREEN_IBC_MIN_SIZE
 * luma block at (bx, by) in frame (stride = fw pixels).
 */
uint32_t screen_ibc_hash(const uint8_t *frame, int fw, int bx, int by);

/* Search for a matching N×N block earlier in scan order.
 * Returns true and sets *bv_x, *bv_y (signed integer-pixel offsets) if a
 * good match is found; returns false if none.
 */
bool screen_ibc_search(const uint8_t *frame, int fw, int fh,
                       int bx, int by, int N,
                       const uint32_t *table, int table_size,
                       int *bv_x, int *bv_y);

/* Insert the block at (bx, by) into the hash table after it has been
 * processed (encoded or decoded) so later blocks can reference it.
 */
void screen_ibc_update(const uint8_t *frame, int fw,
                       int bx, int by,
                       uint32_t *table, int table_size);

/* Deprecated: same signature as screen_ibc_update but taking a frame pointer.
 * Use screen_ibc_update() directly. */
void screen_ibc_update_block(const uint8_t *frame, int fw,
                              int bx, int by,
                              uint32_t *table, int table_size);

#endif /* OPCODEC_SCREEN_H */
