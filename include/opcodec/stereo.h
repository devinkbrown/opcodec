/*
 * opcodec/stereo.h — Parametric Stereo Coding for OPVOX
 *
 * Parametric stereo dramatically reduces stereo bitrate by transmitting
 * a mono downmix plus spatial parameters (IID, ICC, IPD) that describe
 * the stereo image. At the decoder, the stereo signal is reconstructed
 * using the mono signal and spatial parameters.
 *
 * Parameters:
 *   IID: Inter-channel Intensity Difference (energy ratio in dB)
 *   ICC: Inter-channel Coherence (correlation coefficient 0-1)
 *   IPD: Inter-channel Phase Difference (phase offset -π to π)
 *   OPD: Overall Phase Difference (not currently used)
 *
 * This allows stereo coding at nearly mono bitrates while preserving
 * the essential spatial characteristics of the original stereo signal.
 * Particularly effective for speech and moderate complexity music.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_STEREO_H
#define OPCODEC_STEREO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "opcodec/bwe.h"  /* for band_range_t */

/* Maximum bands for parametric stereo analysis */
#define PS_MAX_BANDS 20

/* Number of parameter types */
#define PS_MAX_PARAMS 4  /* IID, ICC, IPD, OPD */

/* Parametric stereo parameters for one frame */
typedef struct {
    float iid[PS_MAX_BANDS];    /* Inter-channel Intensity Difference (dB) */
    float icc[PS_MAX_BANDS];    /* Inter-channel Coherence [0,1] */
    float ipd[PS_MAX_BANDS];    /* Inter-channel Phase Difference [-pi, pi] */
    bool  has_ipd;              /* IPD only needed for non-symmetric signals */
    uint8_t num_bands;          /* Number of frequency bands used */
} ps_params_t;

/* Parametric stereo encoder/decoder context */
typedef struct {
    ps_params_t prev_params;    /* Previous frame parameters for temporal smoothing */
    float prev_l[480];          /* Previous frame left MDCT coefficients */
    float prev_r[480];          /* Previous frame right MDCT coefficients */
    bool has_prev;              /* Whether previous frame data is valid */
    uint8_t num_bands;          /* Number of frequency bands configured */
} ps_ctx_t;

/* ---- Core API ---- */

/*
 * Initialize parametric stereo context.
 * num_bands: Number of frequency bands (1-PS_MAX_BANDS)
 */
void ps_init(ps_ctx_t *ctx, uint8_t num_bands);

/*
 * Analyze stereo MDCT coefficients to extract spatial parameters.
 * Computes IID, ICC, and optionally IPD for each frequency band.
 *
 * ctx: Parametric stereo context
 * mdct_l: Left channel MDCT coefficients
 * mdct_r: Right channel MDCT coefficients
 * num_coeffs: Number of MDCT coefficients (typically 480 for 48kHz)
 * bands: Band range definitions (start/end coefficient indices)
 * num_bands: Number of bands to analyze
 * params: Output parameters structure
 */
void ps_analyze(ps_ctx_t *ctx,
                const float *mdct_l, const float *mdct_r,
                int num_coeffs,
                const band_range_t *bands, int num_bands,
                ps_params_t *params);

/*
 * Create mono downmix from stereo MDCT coefficients.
 * Uses standard downmix formula: M = (L + R) / sqrt(2)
 *
 * mdct_l: Left channel MDCT coefficients
 * mdct_r: Right channel MDCT coefficients
 * mdct_mono: Output mono MDCT coefficients
 * num_coeffs: Number of MDCT coefficients
 */
void ps_downmix(const float *mdct_l, const float *mdct_r,
                float *mdct_mono, int num_coeffs);

/*
 * Reconstruct stereo MDCT coefficients from mono signal and spatial parameters.
 * Uses the spatial parameters to upmix the mono signal back to stereo.
 *
 * params: Spatial parameters from encoder
 * mdct_mono: Input mono MDCT coefficients
 * mdct_l: Output left channel MDCT coefficients
 * mdct_r: Output right channel MDCT coefficients
 * num_coeffs: Number of MDCT coefficients
 * bands: Band range definitions (must match encoder)
 * num_bands: Number of bands to process
 */
void ps_upmix(const ps_params_t *params,
              const float *mdct_mono,
              float *mdct_l, float *mdct_r,
              int num_coeffs,
              const band_range_t *bands, int num_bands);

/* ---- Bitstream API ---- */

/*
 * Encode spatial parameters to bitstream.
 * Quantizes and serializes IID (5 bits), ICC (3 bits), and optional IPD (5 bits).
 *
 * params: Spatial parameters to encode
 * out: Output buffer
 * out_cap: Output buffer capacity
 * Returns: Number of bytes written, or -1 on error
 */
int ps_encode_params(const ps_params_t *params,
                     uint8_t *out, size_t out_cap);

/*
 * Decode spatial parameters from bitstream.
 * Deserializes and dequantizes the spatial parameters.
 *
 * params: Output parameters structure
 * in: Input bitstream buffer
 * in_len: Input buffer length
 * Returns: Number of bytes consumed, or -1 on error
 */
int ps_decode_params(ps_params_t *params,
                     const uint8_t *in, size_t in_len);

#endif /* OPCODEC_STEREO_H */