/*
 * opcodec/energy.h — Band energy quantization for OPVOX audio codec
 *
 * Implements Opus-style two-pass (coarse/fine) energy quantization system
 * that provides 2-4 dB coding gain over simple approaches.
 *
 * Features:
 *   - Inter-band prediction (each band predicted from previous band)
 *   - Inter-frame prediction (temporal correlation with previous frame)
 *   - Coarse quantization: 6 dB resolution using Laplace-distributed residuals
 *   - Fine quantization: variable resolution (1-4 bits) using leftover bits
 *   - Optimal bit allocation prioritizing high-energy bands
 *
 * Algorithm overview:
 * 1. Convert band energies to dB: e_dB = 10 * log10(energy + epsilon)
 * 2. Predict from previous band and/or previous frame
 * 3. Quantize residual with 6 dB coarse steps
 * 4. Use remaining bits for fine refinement (doubles resolution per bit)
 * 5. Decoder reverses process to reconstruct linear energies
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_ENERGY_H
#define OPCODEC_ENERGY_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define ENERGY_MAX_BANDS  32
#define ENERGY_COARSE_RES 6.0f   /* 6 dB coarse resolution */

/* Energy quantization state */
typedef struct {
    float    prev_energy_dB[ENERGY_MAX_BANDS]; /* previous frame energies for inter-frame prediction */
    float    coarse_dB[ENERGY_MAX_BANDS];       /* quantized coarse energies (current frame) */
    uint16_t num_bands;
    bool     has_prev;                          /* whether previous frame data is available */
} energy_ctx_t;

/* Initialize energy quantization context */
void energy_init(energy_ctx_t *ctx, uint16_t num_bands);

/* Coarse energy quantization (encoder side).
 *
 * Input:  band_energy[num_bands] — raw band energies (linear scale)
 * Output: coarse_codes[num_bands] — quantized coarse codes (signed int8)
 *
 * Returns number of bits used for coarse encoding.
 */
int energy_encode_coarse(energy_ctx_t *ctx,
                         const float *band_energy, uint16_t num_bands,
                         int8_t *coarse_codes);

/* Fine energy refinement (encoder side).
 * Called after coarse encoding. Uses leftover bits to refine.
 *
 * Input:  band_energy[num_bands] — raw band energies
 *         fine_bits[num_bands]    — number of refinement bits per band (0-4)
 * Output: fine_codes[num_bands]   — refinement codes
 *
 * Returns number of bits used for fine encoding.
 */
int energy_encode_fine(energy_ctx_t *ctx,
                       const float *band_energy, uint16_t num_bands,
                       const uint8_t *fine_bits,
                       uint8_t *fine_codes);

/* Coarse energy decoding (decoder side).
 *
 * Input:  coarse_codes[num_bands] — quantized coarse codes
 * Output: band_energy[num_bands]  — reconstructed band energies (linear scale)
 */
void energy_decode_coarse(energy_ctx_t *ctx,
                          const int8_t *coarse_codes, uint16_t num_bands,
                          float *band_energy);

/* Fine energy decoding (decoder side).
 *
 * Input:  fine_codes[num_bands]   — refinement codes
 *         fine_bits[num_bands]    — bits per band
 * Output: band_energy[num_bands]  — refined band energies (updated in-place)
 */
void energy_decode_fine(energy_ctx_t *ctx,
                        const uint8_t *fine_codes, const uint8_t *fine_bits,
                        uint16_t num_bands,
                        float *band_energy);

/* Allocate fine bits across bands based on importance.
 *
 * total_fine_bits: total bits available for refinement
 * coarse_codes: from coarse encoding (bands with larger energy get priority)
 *
 * Output: fine_bits[num_bands] — bits allocated per band (0-4 each)
 */
void energy_allocate_fine_bits(uint16_t total_fine_bits,
                               const float *coarse_dB,
                               uint16_t num_bands,
                               uint8_t *fine_bits);

/* Commit fine-refined coarse_dB as the temporal prediction baseline for the
 * next frame.  Call after energy_encode_fine / energy_decode_fine. */
void energy_commit(energy_ctx_t *ctx);

/* Reset context (e.g., on channel change or codec reset) */
void energy_reset(energy_ctx_t *ctx);

#endif /* OPCODEC_ENERGY_H */