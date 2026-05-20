#ifndef OPCODEC_TNS_H
#define OPCODEC_TNS_H

/*
 * Temporal Noise Shaping (TNS) for MDCT-based audio codecs
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 *
 * TNS prevents pre-echo artifacts in transient frames by shaping quantization
 * noise in the time domain using LPC prediction in the MDCT domain.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* TNS configuration */
#define TNS_MAX_ORDER    12    /* maximum LPC order */
#define TNS_MAX_BANDS    32    /* maximum spectral bands */
#define TNS_MAX_COEFFS   480   /* maximum MDCT coefficients (48kHz) */

/* TNS filter parameters (transmitted as side info) */
typedef struct {
    int8_t   coeffs[TNS_MAX_ORDER];  /* quantized reflection coefficients (4-bit each) */
    uint8_t  order;                  /* LPC order used (0 = TNS disabled) */
    uint16_t start_band;             /* first band to filter */
    uint16_t stop_band;              /* last band to filter (exclusive) */
} tns_params_t;

/* TNS encoder/decoder context */
typedef struct {
    float    lpc[TNS_MAX_ORDER + 1]; /* LPC coefficients (a[0]=1.0) */
    uint8_t  order;
    bool     active;                 /* whether TNS is applied this frame */
} tns_ctx_t;

/* Band range definition */
typedef struct {
    uint16_t start;
    uint16_t end;
} tns_band_t;

/* Initialize TNS context */
void tns_init(tns_ctx_t *ctx);

/* Analyze MDCT coefficients and decide whether TNS should be applied.
 * Computes LPC model of the MDCT spectrum.
 *
 * Returns true if TNS is beneficial (strong temporal transients detected).
 * The params structure is filled with quantized filter parameters for
 * transmission as side information.
 *
 * Decision criterion: apply TNS when the spectral prediction gain > 2 dB
 * (i.e., the MDCT spectrum has strong spectral structure that TNS can whiten).
 */
bool tns_analyze(tns_ctx_t *ctx,
                 const float *mdct, int num_coeffs,
                 const tns_band_t *bands, int num_bands,
                 tns_params_t *params);

/* Apply TNS analysis filter to MDCT coefficients (encoder side).
 * Filters the MDCT spectrum with the LPC analysis filter to whiten it.
 * Modifies mdct[] in-place. Must be called after tns_analyze returns true. */
void tns_filter_encode(const tns_ctx_t *ctx,
                       float *mdct, int num_coeffs,
                       const tns_band_t *bands, int num_bands,
                       const tns_params_t *params);

/* Decode TNS parameters and set up synthesis filter (decoder side).
 * Reconstructs LPC coefficients from quantized reflection coefficients. */
void tns_decode_params(tns_ctx_t *ctx, const tns_params_t *params);

/* Apply TNS synthesis filter to MDCT coefficients (decoder side).
 * Inverse of the analysis filter — restores temporal envelope shape.
 * Modifies mdct[] in-place. */
void tns_filter_decode(const tns_ctx_t *ctx,
                       float *mdct, int num_coeffs,
                       const tns_band_t *bands, int num_bands,
                       const tns_params_t *params);

/* Encode TNS parameters to bitstream (compact binary format).
 * Returns bytes written. */
int tns_encode_params(const tns_params_t *params,
                      uint8_t *out, size_t out_cap);

/* Decode TNS parameters from bitstream.
 * Returns bytes consumed. */
int tns_decode_params_from_stream(tns_params_t *params,
                                 const uint8_t *in, size_t in_len);

#endif /* OPCODEC_TNS_H */