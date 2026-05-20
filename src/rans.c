/*
 * opcodec/rans.c — Range Asymmetric Numeral Systems (rANS) entropy coder
 *
 * High-performance entropy coding for opcodec library.
 * Replaces Rice/Golomb coding with faster, higher-compression entropy coding.
 *
 * Implementation based on Fabian Giesen's public-domain rANS reference.
 * 32-bit portable implementation optimized for audio/video compression.
 *
 * Copyright (c) 2026 Ophion Development Team.  GPL v2.
 */

#include "opcodec/rans.h"
#include <string.h>
#include <assert.h>

/* ---- Internal helpers ---- */

/* Write a byte to the encoder output buffer (backwards) */
static inline void rans_enc_put_byte(rans_encoder_t *enc, uint8_t byte)
{
    assert(enc->buf_pos > 0);
    enc->buf[--enc->buf_pos] = byte;
}

/* Read a byte from the decoder input buffer (forwards) */
static inline uint8_t rans_dec_get_byte(rans_decoder_t *dec)
{
    assert(dec->buf_pos < dec->buf_len);
    return dec->buf[dec->buf_pos++];
}

/* Renormalize encoder state: push bytes to output when state gets too large */
static void rans_enc_renorm(rans_encoder_t *enc, uint32_t freq)
{
    uint32_t x_max = ((RANS_L >> RANS_PROB_BITS) << 8) * freq;
    while (enc->state >= x_max) {
        rans_enc_put_byte(enc, enc->state & 0xFF);
        enc->state >>= 8;
    }
}

/* Renormalize decoder state: pull bytes from input when state gets too small */
static void rans_dec_renorm(rans_decoder_t *dec)
{
    while (dec->state < RANS_L && dec->buf_pos < dec->buf_len) {
        dec->state = (dec->state << 8) | rans_dec_get_byte(dec);
    }
}

/* Find symbol containing the given cumulative frequency */
static uint16_t rans_find_symbol(const rans_model_t *model, uint16_t cum)
{
    /* Linear search is fine for small alphabets (<= 256 symbols) */
    for (uint16_t s = 0; s < model->num_symbols; s++) {
        if (model->syms[s].freq == 0) continue;  /* Skip zero-frequency symbols */

        uint16_t sym_start = model->syms[s].cum_freq;
        uint16_t sym_end = sym_start + model->syms[s].freq;
        if (cum >= sym_start && cum < sym_end) {
            return s;
        }
    }
    /* Should never happen with a valid model */
    assert(0);
    return 0;
}

/* ---- Model construction ---- */

void rans_model_init(rans_model_t *model, const uint32_t *counts, uint16_t num_symbols)
{
    assert(model && counts && num_symbols <= RANS_MAX_SYMBOLS);

    model->num_symbols = num_symbols;

    /* Calculate total count */
    uint64_t total_count = 0;
    for (uint16_t i = 0; i < num_symbols; i++) {
        total_count += counts[i];
    }

    /* Handle edge case: all symbols have zero count */
    if (total_count == 0) {
        rans_model_uniform(model, num_symbols);
        return;
    }

    /* Normalize frequencies to sum to RANS_PROB_SCALE */
    uint32_t freq_sum = 0;
    uint16_t max_freq_idx = 0;
    uint16_t max_freq = 0;

    for (uint16_t i = 0; i < num_symbols; i++) {
        uint32_t freq;
        if (counts[i] == 0) {
            freq = 0;
        } else {
            /* Proportional scaling with minimum frequency of 1 */
            freq = (uint32_t)(((uint64_t)counts[i] * RANS_PROB_SCALE) / total_count);
            if (freq == 0) freq = 1;  /* Ensure non-zero symbols get at least 1 */
        }

        model->syms[i].freq = (uint16_t)freq;
        freq_sum += freq;

        /* Track symbol with highest frequency for adjustment */
        if (freq > max_freq) {
            max_freq = (uint16_t)freq;
            max_freq_idx = i;
        }
    }

    /* Adjust the most frequent symbol to ensure exact sum */
    if (freq_sum != RANS_PROB_SCALE) {
        int32_t adjustment = (int32_t)RANS_PROB_SCALE - (int32_t)freq_sum;
        int32_t new_freq = (int32_t)model->syms[max_freq_idx].freq + adjustment;
        if (new_freq > 0 && new_freq <= UINT16_MAX) {
            model->syms[max_freq_idx].freq = (uint16_t)new_freq;
        }
    }

    /* Compute cumulative frequencies */
    uint16_t cum_freq = 0;
    for (uint16_t i = 0; i < num_symbols; i++) {
        model->syms[i].cum_freq = cum_freq;
        cum_freq += model->syms[i].freq;
    }
}

void rans_model_laplace(rans_model_t *model, uint16_t num_symbols, int decay)
{
    assert(model && num_symbols <= RANS_MAX_SYMBOLS && decay > 0);

    model->num_symbols = num_symbols;

    /* Build Laplace distribution: freq[i] ∝ exp(-decay * i / num_symbols)
     * For zig-zag encoded values, small magnitudes (indices 0,1,2...) get highest probability */

    uint32_t freqs[RANS_MAX_SYMBOLS];
    uint64_t total_freq = 0;

    for (uint16_t i = 0; i < num_symbols; i++) {
        /* Use fixed-point arithmetic to avoid floating point */
        /* freq = exp(-decay * i / num_symbols) * scale_factor */
        int32_t exp_arg = -(decay * (int32_t)i) / (int32_t)num_symbols;

        /* Approximate exp() using integer arithmetic */
        uint32_t freq;
        if (exp_arg < -20) {
            freq = 1;  /* Very small probability, but not zero */
        } else {
            /* Simple approximation: freq = max(1, scale >> (decay*i/8)) */
            int32_t shift = (decay * (int32_t)i) / (8 * (int32_t)num_symbols);
            if (shift > 15) shift = 15;  /* Prevent underflow */
            freq = RANS_PROB_SCALE >> shift;
            if (freq < 1) freq = 1;
        }

        freqs[i] = freq;
        total_freq += freq;
    }

    /* Normalize to RANS_PROB_SCALE */
    for (uint16_t i = 0; i < num_symbols; i++) {
        freqs[i] = (uint32_t)(((uint64_t)freqs[i] * RANS_PROB_SCALE) / total_freq);
        if (freqs[i] < 1) freqs[i] = 1;
    }

    rans_model_init(model, freqs, num_symbols);
}

void rans_model_uniform(rans_model_t *model, uint16_t num_symbols)
{
    assert(model && num_symbols > 0 && num_symbols <= RANS_MAX_SYMBOLS);

    model->num_symbols = num_symbols;

    /* Equal probability for all symbols */
    uint32_t counts[RANS_MAX_SYMBOLS];
    for (uint16_t i = 0; i < num_symbols; i++) {
        counts[i] = 1;
    }

    rans_model_init(model, counts, num_symbols);
}

void rans_model_zero_run(rans_model_t *model, uint16_t max_run)
{
    assert(model && max_run > 0 && max_run <= RANS_MAX_SYMBOLS);

    model->num_symbols = max_run;

    /* Build geometric distribution: P(X = k) = (1-p)^k * p where p = 0.3
     * This biases toward short runs which is common for zero runs in quantized data */

    uint32_t counts[RANS_MAX_SYMBOLS];

    /* Use fixed-point arithmetic: p = 0.3 ≈ 307/1024 */
    const uint32_t p_num = 307;    /* p numerator */
    const uint32_t p_den = 1024;   /* p denominator */
    const uint32_t q_num = p_den - p_num;  /* (1-p) numerator */

    uint64_t prob_scale = p_den;  /* Start with p */

    for (uint16_t run = 0; run < max_run; run++) {
        /* P(run) = (1-p)^run * p */
        uint64_t count = (prob_scale * p_num) / p_den;
        if (count == 0) count = 1;  /* Ensure minimum probability */

        counts[run] = (uint32_t)count;

        /* Update prob_scale for next iteration: prob_scale *= (1-p) */
        prob_scale = (prob_scale * q_num) / p_den;
        if (prob_scale == 0) break;  /* Prevent underflow for very long runs */
    }

    rans_model_init(model, counts, max_run);
}

/* ---- Encoder ---- */

void rans_enc_init(rans_encoder_t *enc, uint8_t *buf, size_t cap)
{
    assert(enc && buf && cap >= 8);  /* Need space for at least state flush */

    enc->state = RANS_L;  /* Initialize to lower bound */
    enc->buf = buf;
    enc->buf_cap = cap;
    enc->buf_pos = cap;   /* Start writing from the end */
}

void rans_enc_put(rans_encoder_t *enc, const rans_model_t *model, uint16_t symbol)
{
    assert(enc && model && symbol < model->num_symbols);
    assert(model->syms[symbol].freq > 0);  /* Can't encode zero-probability symbols */

    uint32_t freq = model->syms[symbol].freq;
    uint32_t cum_freq = model->syms[symbol].cum_freq;

    /* Renormalize before encoding to prevent overflow */
    rans_enc_renorm(enc, freq);

    /* rANS encoding step: C(s,x) = (x/freq)*M + (x%freq) + cum_freq */
    enc->state = ((enc->state / freq) << RANS_PROB_BITS) + (enc->state % freq) + cum_freq;
}

size_t rans_enc_flush(rans_encoder_t *enc)
{
    assert(enc);

    /* Output the final state as 4 bytes (write in reverse order so they're correct when read forward) */
    for (int i = 3; i >= 0; i--) {
        rans_enc_put_byte(enc, (enc->state >> (i * 8)) & 0xFF);
    }

    return enc->buf_cap - enc->buf_pos;
}

const uint8_t *rans_enc_data(const rans_encoder_t *enc, size_t *out_len)
{
    assert(enc && out_len);

    *out_len = enc->buf_cap - enc->buf_pos;
    return enc->buf + enc->buf_pos;
}

/* ---- Decoder ---- */

void rans_dec_init(rans_decoder_t *dec, const uint8_t *buf, size_t len)
{
    assert(dec && buf && len >= 4);

    dec->buf = buf;
    dec->buf_len = len;

    /* Read initial state from FIRST 4 bytes (they are the state, written reversed) */
    dec->state = 0;
    for (int i = 3; i >= 0; i--) {
        dec->state = (dec->state << 8) | buf[i];
    }

    /* Start reading from after the state */
    dec->buf_pos = 4;
}

uint16_t rans_dec_get(rans_decoder_t *dec, const rans_model_t *model)
{
    assert(dec && model);

    /* Renormalize before decoding */
    rans_dec_renorm(dec);

    /* Extract cumulative frequency from state */
    uint32_t cum = dec->state & (RANS_PROB_SCALE - 1);

    /* Find symbol whose range contains cum */
    uint16_t symbol = rans_find_symbol(model, (uint16_t)cum);

    uint32_t freq = model->syms[symbol].freq;
    uint32_t sym_cum_freq = model->syms[symbol].cum_freq;

    /* rANS decoding step: D(x) -> (s, x') where x' = freq*(x>>prob_bits) + (x&mask) - cum_freq */
    dec->state = freq * (dec->state >> RANS_PROB_BITS) + cum - sym_cum_freq;

    return symbol;
}

/* ---- Context-adaptive models ---- */

void rans_adaptive_init(rans_adaptive_t *adp, uint16_t num_symbols, uint16_t num_contexts, int initial_decay)
{
    assert(adp && num_symbols > 0 && num_symbols <= RANS_MAX_SYMBOLS);
    assert(num_contexts > 0 && num_contexts <= RANS_MAX_CONTEXTS);
    assert(initial_decay > 0);

    adp->num_symbols = num_symbols;
    adp->num_contexts = num_contexts;
    adp->adapt_interval = 256;  /* Default: renormalize every 256 symbols */
    adp->symbols_coded = 0;

    /* Initialize all contexts with Laplace distributions */
    for (uint16_t ctx = 0; ctx < num_contexts; ctx++) {
        rans_model_laplace(&adp->models[ctx], num_symbols, initial_decay);

        /* Initialize running counts from the Laplace frequencies */
        adp->total[ctx] = 0;
        for (uint16_t sym = 0; sym < num_symbols; sym++) {
            /* Use the frequency as initial count (scaled down to prevent overflow) */
            uint32_t count = adp->models[ctx].syms[sym].freq;
            adp->counts[ctx][sym] = count;
            adp->total[ctx] += count;
        }
    }
}

void rans_adaptive_encode(rans_encoder_t *enc, rans_adaptive_t *adp, uint16_t context, uint16_t symbol)
{
    assert(enc && adp && context < adp->num_contexts && symbol < adp->num_symbols);

    /* Encode using current model for this context */
    rans_enc_put(enc, &adp->models[context], symbol);

    /* Update counts for this context */
    rans_adaptive_update(adp, context, symbol);

    /* Check if we need to renormalize any models */
    adp->symbols_coded++;
    if (adp->symbols_coded >= adp->adapt_interval) {
        /* Rebuild models from accumulated counts for all contexts */
        for (uint16_t ctx = 0; ctx < adp->num_contexts; ctx++) {
            rans_adaptive_rebuild_model(adp, ctx);
        }
        adp->symbols_coded = 0;
    }
}

uint16_t rans_adaptive_decode(rans_decoder_t *dec, rans_adaptive_t *adp, uint16_t context)
{
    assert(dec && adp && context < adp->num_contexts);

    /* Decode using current model for this context */
    uint16_t symbol = rans_dec_get(dec, &adp->models[context]);

    /* Update counts for this context (must match encoder exactly) */
    rans_adaptive_update(adp, context, symbol);

    /* Check if we need to renormalize any models (must match encoder) */
    adp->symbols_coded++;
    if (adp->symbols_coded >= adp->adapt_interval) {
        /* Rebuild models from accumulated counts for all contexts */
        for (uint16_t ctx = 0; ctx < adp->num_contexts; ctx++) {
            rans_adaptive_rebuild_model(adp, ctx);
        }
        adp->symbols_coded = 0;
    }

    return symbol;
}

void rans_adaptive_update(rans_adaptive_t *adp, uint16_t context, uint16_t symbol)
{
    assert(adp && context < adp->num_contexts && symbol < adp->num_symbols);

    /* Increment count for this symbol in this context */
    adp->counts[context][symbol]++;
    adp->total[context]++;

    /* Prevent count overflow by halving all counts when total gets too large */
    if (adp->total[context] > 32768) {
        /* Halve all counts (minimum 1 to preserve non-zero symbols) */
        uint32_t new_total = 0;
        for (uint16_t sym = 0; sym < adp->num_symbols; sym++) {
            adp->counts[context][sym] = (adp->counts[context][sym] + 1) >> 1;
            new_total += adp->counts[context][sym];
        }
        adp->total[context] = new_total;
    }
}

void rans_adaptive_rebuild_model(rans_adaptive_t *adp, uint16_t context)
{
    assert(adp && context < adp->num_contexts);

    /* Rebuild the rans_model_t from current counts using existing normalization logic */
    rans_model_init(&adp->models[context], adp->counts[context], adp->num_symbols);
}

/*
 * Context selection strategies for different data types:
 *
 * For MDCT coefficients:
 *   context = band_index % num_contexts
 *   Different frequency bands have different probability distributions.
 *   Low bands tend to have higher energy, high bands are sparser.
 *
 * For motion vectors:
 *   context = (prev_mv_magnitude > threshold) ? 1 : 0
 *   Large previous motion suggests large current motion.
 *   Could be extended to more contexts based on direction or temporal patterns.
 *
 * For quantized energy levels:
 *   context = min(previous_coarse_code, num_contexts - 1)
 *   Adjacent energy levels in time/frequency tend to be correlated.
 *   Higher previous energy suggests higher current energy.
 *
 * For zero run lengths:
 *   context = (last_nonzero_distance < 4) ? 0 : 1
 *   Short vs long runs have different distributions.
 *   Use rans_model_zero_run() for the models.
 */