/*
 * opcodec/bwe.h — Bandwidth Extension and Spectral Noise Shaping
 *
 * BWE (Bandwidth Extension) saves bits by coding only low frequency bands
 * and regenerating high bands via spectral folding with energy scaling.
 *
 * SNS (Spectral Noise Shaping) shapes quantization noise to follow the
 * spectral envelope, making it perceptually masked by the signal.
 *
 * These techniques work in the MDCT domain and can be used independently
 * or together to improve perceptual quality at low bitrates.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_BWE_H
#define OPCODEC_BWE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Maximum bands and MDCT coefficients */
#define BWE_MAX_BANDS   32
#define BWE_MAX_MDCT    480   /* 960/2 for 48kHz */

/* SNS context — per-frame spectral noise shaping state */
typedef struct {
    float    scale_factors[BWE_MAX_BANDS];   /* per-band scale factors (energy) */
    float    prev_scale[BWE_MAX_BANDS];      /* previous frame for inter-frame prediction */
    uint16_t num_bands;
    bool     has_prev;
} sns_ctx_t;

/* BWE context — bandwidth extension state */
typedef struct {
    uint16_t cutoff_band;                     /* first uncoded band index */
    uint16_t total_bands;
    float    high_energy[BWE_MAX_BANDS];      /* transmitted energies for high bands */
    uint16_t num_high_bands;                  /* number of uncoded bands */
} bwe_ctx_t;

/* Band definition: start and end MDCT coefficient indices */
typedef struct {
    uint16_t start;
    uint16_t end;
} band_range_t;

/* ---- SNS API ---- */

/* Initialize SNS context */
void sns_init(sns_ctx_t *ctx);

/* Compute scale factors from MDCT coefficients.
 * bands[] defines the start/end indices of each band.
 * Scale factors are stored in ctx->scale_factors. */
void sns_analyze(sns_ctx_t *ctx,
                 const float *mdct, int num_coeffs,
                 const band_range_t *bands, int num_bands);

/* Flatten spectrum: divide each coefficient by its band's scale factor.
 * Call after sns_analyze on the encoder side. Modifies mdct[] in-place. */
void sns_flatten(const sns_ctx_t *ctx,
                 float *mdct, int num_coeffs,
                 const band_range_t *bands, int num_bands);

/* Restore spectrum: multiply each coefficient by its band's scale factor.
 * Call on the decoder side after dequantization. Modifies mdct[] in-place. */
void sns_restore(const sns_ctx_t *ctx,
                 float *mdct, int num_coeffs,
                 const band_range_t *bands, int num_bands);

/* Encode scale factors to bitstream. Uses inter-band prediction and
 * optional inter-frame prediction for efficient coding.
 * Returns number of bytes written to `out`. */
int sns_encode_scales(const sns_ctx_t *ctx,
                      uint8_t *out, size_t out_cap);

/* Decode scale factors from bitstream.
 * Returns number of bytes consumed from `in`. */
int sns_decode_scales(sns_ctx_t *ctx,
                      const uint8_t *in, size_t in_len);

/* ---- BWE API ---- */

/* Initialize BWE context.
 * cutoff_band: first band index that won't be explicitly coded.
 * total_bands: total number of bands in the codec. */
void bwe_init(bwe_ctx_t *ctx, uint16_t cutoff_band, uint16_t total_bands);

/* Determine optimal cutoff band for given bitrate (bits per frame).
 * Higher bitrate = higher cutoff = more bands coded explicitly. */
uint16_t bwe_optimal_cutoff(int bits_per_frame, int num_bands,
                            uint32_t sample_rate);

/* Encode high-band energies for transmission.
 * Extracts energy from the original MDCT coefficients for uncoded bands.
 * Returns bytes written to `out`. */
int bwe_encode(bwe_ctx_t *ctx,
               const float *mdct, int num_coeffs,
               const band_range_t *bands, int num_bands,
               uint8_t *out, size_t out_cap);

/* Decode high-band energies from bitstream.
 * Returns bytes consumed from `in`. */
int bwe_decode(bwe_ctx_t *ctx,
               const uint8_t *in, size_t in_len);

/* Synthesize high bands by folding low-band coefficients.
 * After calling bwe_decode, call this to fill in the uncoded bands.
 * Copies MDCT coefficients from coded bands, scales to match
 * transmitted energies. Modifies mdct[] in-place. */
void bwe_synthesize(const bwe_ctx_t *ctx,
                    float *mdct, int num_coeffs,
                    const band_range_t *bands, int num_bands);

#endif /* OPCODEC_BWE_H */