/*
 * opcodec/pns.h — Perceptual Noise Substitution (PNS)
 *
 * At very low bitrates, some frequency bands contain only noise-like content.
 * Instead of coding their exact coefficients, transmit only the energy level
 * and regenerate pseudorandom noise at the decoder. This saves significant
 * bits for noise-like bands.
 *
 * Algorithm:
 *   1. Analyze spectral flatness for each band (geometric/arithmetic mean)
 *   2. Mark noise-like bands (flatness > threshold)
 *   3. Store energy level for noise bands
 *   4. Zero coefficients in encoder, regenerate noise in decoder
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_PNS_H
#define OPCODEC_PNS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "opcodec/bwe.h"  /* For band_range_t */

/* Maximum bands for PNS processing */
#define PNS_MAX_BANDS 32

/* Tonality threshold: bands with spectral flatness above this are noise-like */
#define PNS_TONALITY_THRESHOLD 0.4f

/* PNS context — tracks which bands use noise substitution */
typedef struct {
    bool     is_noise[PNS_MAX_BANDS];     /* which bands use noise substitution */
    float    noise_energy[PNS_MAX_BANDS]; /* energy to reproduce */
    uint8_t  num_bands;
    uint32_t rng;                         /* decoder noise generator state */
} pns_ctx_t;

/* ---- PNS API ---- */

/* Initialize PNS context and seed RNG */
void pns_init(pns_ctx_t *ctx);

/* Analyze spectral characteristics and mark noise-like bands.
 *
 * For each band:
 *   - Compute spectral flatness (geometric mean / arithmetic mean of |coeffs|^2)
 *   - If flatness > PNS_TONALITY_THRESHOLD, mark as noise band
 *   - Store the band energy for transmission
 *
 * bands[] defines the start/end indices of each band.
 */
void pns_analyze(pns_ctx_t *ctx,
                 const float *mdct, int num_coeffs,
                 const band_range_t *bands, int num_bands);

/* Zero out MDCT coefficients in noise bands (encoder side).
 * Call after pns_analyze() so PVQ doesn't waste bits on noise bands.
 * Modifies mdct[] in-place.
 */
void pns_zero_noise_bands(const pns_ctx_t *ctx,
                          float *mdct, int num_coeffs,
                          const band_range_t *bands, int num_bands);

/* Fill noise bands with pseudorandom values (decoder side).
 * Generates noise scaled to match stored noise_energy values.
 * Uses LCG: rng = rng * 1664525 + 1013904223
 * Modifies mdct[] in-place.
 */
void pns_fill_noise(pns_ctx_t *ctx,
                    float *mdct, int num_coeffs,
                    const band_range_t *bands, int num_bands);

/* Encode noise band flags and energies to bitstream.
 * Format: 1 bit per band (is_noise flag) + 8 bits per noise band (energy)
 * Returns number of bytes written to `out`.
 */
int pns_encode(const pns_ctx_t *ctx,
               uint8_t *out, size_t out_cap);

/* Decode noise band flags and energies from bitstream.
 * Returns number of bytes consumed from `in`.
 */
int pns_decode(pns_ctx_t *ctx,
               const uint8_t *in, size_t in_len);

#endif /* OPCODEC_PNS_H */