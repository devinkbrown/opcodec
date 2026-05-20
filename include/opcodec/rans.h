/*
 * opcodec/rans.h — Range Asymmetric Numeral Systems (rANS) entropy coder
 *
 * High-performance entropy coding for opcodec library.
 * Replaces Rice/Golomb coding with faster, higher-compression entropy coding.
 *
 * Based on Fabian Giesen's public-domain rANS reference.
 * 32-bit portable implementation (no 64-bit division needed).
 * Byte-aligned output for fast streaming.
 *
 * Features:
 *   - 32-bit state machine, no 64-bit arithmetic
 *   - Portable C11, no platform dependencies
 *   - Byte-aligned output streams
 *   - Static probability models for speed
 *   - Laplace distribution for audio residuals
 *   - Zig-zag encoding for signed values
 *
 * Usage:
 *   1. Build probability model from histogram or use predefined distributions
 *   2. Encode symbols with rans_enc_put()
 *   3. Flush encoder and get compressed data
 *   4. Initialize decoder and decode with rans_dec_get()
 *
 * Copyright (c) 2026 Ophion Development Team.  GPL v2.
 */

#ifndef OPCODEC_RANS_H
#define OPCODEC_RANS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Probability scale — all symbol frequencies must sum to (1 << RANS_PROB_BITS) */
#define RANS_PROB_BITS   14
#define RANS_PROB_SCALE  (1u << RANS_PROB_BITS)

/* Maximum symbols for static probability tables.
 * Must cover the full zig-zag encoded range of quantized wavelet coefficients
 * AND the last-significant-position alphabet for the largest CU (64×64 = 4096). */
#define RANS_MAX_SYMBOLS 4096

/* Context-adaptive model: maintains per-context probability tables */
#define RANS_MAX_CONTEXTS 16
#define RANS_ADAPT_RATE   8    /* adaptation speed: lower = faster */

/* rANS state bounds for 32-bit implementation */
#define RANS_L           (1u << 23)  /* Lower bound for renormalization */

/* Probability table entry: cumulative frequency + frequency */
typedef struct {
    uint16_t  cum_freq;   /* cumulative frequency (start) */
    uint16_t  freq;       /* symbol frequency (width) */
} rans_sym_t;

/* Probability model — holds frequency table for an alphabet */
typedef struct {
    rans_sym_t  syms[RANS_MAX_SYMBOLS];
    uint16_t    num_symbols;
} rans_model_t;

/* Context-adaptive model: maintains per-context probability tables */
typedef struct {
    rans_model_t models[RANS_MAX_CONTEXTS];
    uint32_t     counts[RANS_MAX_CONTEXTS][RANS_MAX_SYMBOLS]; /* running symbol counts */
    uint32_t     total[RANS_MAX_CONTEXTS];                     /* total count per context */
    uint16_t     num_symbols;
    uint16_t     num_contexts;
    uint32_t     adapt_interval;  /* renormalize every N symbols */
    uint32_t     symbols_coded;   /* running count for renormalization */
} rans_adaptive_t;

/* Encoder state */
typedef struct {
    uint32_t  state;      /* rANS state machine */
    uint8_t  *buf;        /* output buffer (written backwards) */
    size_t    buf_cap;    /* buffer capacity */
    size_t    buf_pos;    /* current write position (decreasing) */
} rans_encoder_t;

/* Decoder state */
typedef struct {
    uint32_t       state;   /* rANS state machine */
    const uint8_t *buf;     /* input buffer */
    size_t         buf_pos; /* current read position (increasing) */
    size_t         buf_len; /* buffer length */
} rans_decoder_t;

/* ---- Model construction ---- */

/* Initialize a model from raw frequency counts. Counts are normalized
 * to sum to RANS_PROB_SCALE. Zero-frequency symbols get freq=0.  */
void rans_model_init(rans_model_t *model, const uint32_t *counts, uint16_t num_symbols);

/* Build a Laplace distribution model centered at 0 for signed values.
 * decay controls how fast probability drops (higher = steeper).
 * Good for MDCT coefficient residuals and band energy residuals. */
void rans_model_laplace(rans_model_t *model, uint16_t num_symbols, int decay);

/* Build a uniform distribution model */
void rans_model_uniform(rans_model_t *model, uint16_t num_symbols);

/* Build a geometric distribution model for zero runs (biased toward short runs) */
void rans_model_zero_run(rans_model_t *model, uint16_t max_run);

/* ---- Encoder ---- */

/* Initialize encoder with output buffer */
void rans_enc_init(rans_encoder_t *enc, uint8_t *buf, size_t cap);

/* Encode a symbol using the given probability model */
void rans_enc_put(rans_encoder_t *enc, const rans_model_t *model, uint16_t symbol);

/* Flush remaining encoder state to output buffer. Returns total bytes written. */
size_t rans_enc_flush(rans_encoder_t *enc);

/* Get pointer to start of encoded data and set output length */
const uint8_t *rans_enc_data(const rans_encoder_t *enc, size_t *out_len);

/* ---- Decoder ---- */

/* Initialize decoder with input buffer */
void rans_dec_init(rans_decoder_t *dec, const uint8_t *buf, size_t len);

/* Decode one symbol using the given probability model */
uint16_t rans_dec_get(rans_decoder_t *dec, const rans_model_t *model);

/* ---- Context-adaptive models ---- */

/* Initialize adaptive model with Laplace distributions for all contexts */
void rans_adaptive_init(rans_adaptive_t *adp, uint16_t num_symbols, uint16_t num_contexts, int initial_decay);

/* Encode a symbol using context's current model, then update the context's counts */
void rans_adaptive_encode(rans_encoder_t *enc, rans_adaptive_t *adp, uint16_t context, uint16_t symbol);

/* Decode a symbol using context's model, update counts identically to encoder */
uint16_t rans_adaptive_decode(rans_decoder_t *dec, rans_adaptive_t *adp, uint16_t context);

/* Update counts for a symbol in a context (internal function) */
void rans_adaptive_update(rans_adaptive_t *adp, uint16_t context, uint16_t symbol);

/* Rebuild the rans_model_t from current counts (internal function) */
void rans_adaptive_rebuild_model(rans_adaptive_t *adp, uint16_t context);

/* ---- Convenience functions for signed integers ---- */

/* Map signed integer to unsigned (zig-zag encoding): 0,-1,1,-2,2,... -> 0,1,2,3,4... */
static inline uint16_t rans_zigzag_enc(int16_t val)
{
    /* Cast to unsigned before the left shift to avoid signed-overflow UB. */
    return (uint16_t)(((uint16_t)val << 1) ^ (uint16_t)(val >> 15));
}

/* Map unsigned integer back to signed (zig-zag decoding) */
static inline int16_t rans_zigzag_dec(uint16_t val)
{
    return (int16_t)((val >> 1) ^ -(int16_t)(val & 1));
}

#endif /* OPCODEC_RANS_H */