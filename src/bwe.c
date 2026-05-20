/*
 * opcodec/bwe.c — Bandwidth Extension and Spectral Noise Shaping
 *
 * Implementation of BWE (spectral folding for high frequencies) and
 * SNS (noise shaping via scale factors) for the OPVOX audio codec.
 *
 * BWE saves bits by not coding high frequency bands, instead regenerating
 * them by copying low frequency content and scaling to match transmitted
 * energy values.
 *
 * SNS shapes quantization noise to follow the spectral envelope by
 * flattening the spectrum before quantization (encoder) and restoring
 * it after quantization (decoder).
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/bwe.h"
#include <string.h>
#include <math.h>
#include <assert.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Constants */
#define SNS_SCALE_MIN        1e-10f    /* Minimum scale factor to prevent div by zero */
#define SNS_SCALE_MAX        100.0f    /* Maximum scale factor for stability */
#define SNS_INTER_ALPHA      0.5f      /* Inter-frame prediction coefficient */
#define BWE_ENERGY_MIN       1e-12f    /* Minimum energy for BWE bands */
#define BWE_RANDOM_SIGN_MASK 0x55555555u /* Alternating bit pattern for sign randomization */

/* SNS scale factor quantization: 6-bit signed, 1.0 dB resolution, ±32 dB range.
 * Using 1 dB steps (vs 0.5 dB) doubles the representable range to ±32 dB so
 * near-silent bands decode to ≤ 0.025 linear (vs 0.158), cutting noise leakage
 * from those bands by 40× with no extra bit cost. */
#define SNS_QUANT_BITS       6
#define SNS_QUANT_RANGE      (1 << (SNS_QUANT_BITS - 1))  /* ±32 */
#define SNS_QUANT_STEP       1.0f      /* dB per step */
#define SNS_DB_FACTOR        (1.0f / (20.0f * logf(10.0f)))  /* Convert to natural log */

/* BWE energy quantization: 8-bit, 1 dB resolution */
#define BWE_QUANT_BITS       8
#define BWE_QUANT_RANGE      (1 << BWE_QUANT_BITS)  /* 0-255 */
#define BWE_QUANT_STEP       1.0f      /* dB per step */
#define BWE_MIN_DB           -60.0f    /* Minimum dB value */

/* Utility functions */
static float
clampf(float x, float min, float max)
{
    return x < min ? min : (x > max ? max : x);
}

static float
db_to_linear(float db)
{
    return expf(db * logf(10.0f) / 20.0f);
}

static float
linear_to_db(float linear)
{
    return 20.0f * log10f(clampf(linear, 1e-10f, 1e10f));
}

/* Bit writer/reader for entropy coding */
typedef struct {
    uint8_t *buf;
    size_t pos;
    uint8_t bit;
    size_t cap;
} bit_writer_t;

typedef struct {
    const uint8_t *buf;
    size_t pos;
    uint8_t bit;
    size_t len;
} bit_reader_t;

static void
bit_writer_init(bit_writer_t *w, uint8_t *buf, size_t cap)
{
    w->buf = buf;
    w->pos = 0;
    w->bit = 0;
    w->cap = cap;
    if (cap > 0)
        memset(buf, 0, cap);
}

static int
bit_writer_write(bit_writer_t *w, uint32_t value, uint8_t bits)
{
    if (bits > 32)
        return -1;

    for (int i = bits - 1; i >= 0; i--) {
        if (w->pos >= w->cap)
            return -1;

        if ((value >> i) & 1)
            w->buf[w->pos] |= (1u << (7 - w->bit));

        w->bit++;
        if (w->bit == 8) {
            w->bit = 0;
            w->pos++;
        }
    }
    return 0;
}

static size_t
bit_writer_bytes(const bit_writer_t *w)
{
    return w->pos + (w->bit > 0 ? 1 : 0);
}

static void
bit_reader_init(bit_reader_t *r, const uint8_t *buf, size_t len)
{
    r->buf = buf;
    r->pos = 0;
    r->bit = 0;
    r->len = len;
}

static uint32_t
bit_reader_read(bit_reader_t *r, uint8_t bits)
{
    uint32_t value = 0;

    for (int i = 0; i < bits; i++) {
        if (r->pos >= r->len)
            return 0;

        if (r->buf[r->pos] & (1u << (7 - r->bit)))
            value |= (1u << (bits - 1 - i));

        r->bit++;
        if (r->bit == 8) {
            r->bit = 0;
            r->pos++;
        }
    }
    return value;
}

/* ---- SNS Implementation ---- */

void
sns_init(sns_ctx_t *ctx)
{
    if (!ctx)
        return;

    memset(ctx, 0, sizeof(*ctx));
    ctx->has_prev = false;
}

void
sns_analyze(sns_ctx_t *ctx,
            const float *mdct, int num_coeffs,
            const band_range_t *bands, int num_bands)
{
    if (!ctx || !mdct || !bands || num_bands <= 0 || num_bands > BWE_MAX_BANDS)
        return;

    ctx->num_bands = num_bands;

    for (int b = 0; b < num_bands; b++) {
        uint16_t start = bands[b].start;
        uint16_t end = bands[b].end;

        if (end > num_coeffs)
            end = num_coeffs;
        if (start >= end) {
            ctx->scale_factors[b] = SNS_SCALE_MIN;
            continue;
        }

        /* Compute RMS energy for this band */
        float energy = 0.0f;
        uint16_t count = end - start;

        for (uint16_t i = start; i < end; i++) {
            energy += mdct[i] * mdct[i];
        }

        /* RMS = sqrt(energy / count) */
        float rms = sqrtf(energy / count);

        /* Clamp to valid range */
        ctx->scale_factors[b] = clampf(rms, SNS_SCALE_MIN, SNS_SCALE_MAX);
    }
}

void
sns_flatten(const sns_ctx_t *ctx,
            float *mdct, int num_coeffs,
            const band_range_t *bands, int num_bands)
{
    if (!ctx || !mdct || !bands || num_bands != ctx->num_bands)
        return;

    for (int b = 0; b < num_bands; b++) {
        uint16_t start = bands[b].start;
        uint16_t end = bands[b].end;

        if (end > num_coeffs)
            end = num_coeffs;
        if (start >= end)
            continue;

        float inv_scale = 1.0f / ctx->scale_factors[b];

        for (uint16_t i = start; i < end; i++) {
            mdct[i] *= inv_scale;
        }
    }
}

void
sns_restore(const sns_ctx_t *ctx,
            float *mdct, int num_coeffs,
            const band_range_t *bands, int num_bands)
{
    if (!ctx || !mdct || !bands || num_bands != ctx->num_bands)
        return;

    for (int b = 0; b < num_bands; b++) {
        uint16_t start = bands[b].start;
        uint16_t end = bands[b].end;

        if (end > num_coeffs)
            end = num_coeffs;
        if (start >= end)
            continue;

        float scale = ctx->scale_factors[b];

        for (uint16_t i = start; i < end; i++) {
            mdct[i] *= scale;
        }
    }
}

int
sns_encode_scales(const sns_ctx_t *ctx,
                  uint8_t *out, size_t out_cap)
{
    if (!ctx || !out || out_cap < (size_t)(ctx->num_bands * SNS_QUANT_BITS / 8 + 2))
        return -1;

    bit_writer_t writer;
    bit_writer_init(&writer, out, out_cap);

    /* Convert scale factors to log domain */
    float log_scales[BWE_MAX_BANDS];
    for (int i = 0; i < ctx->num_bands; i++) {
        log_scales[i] = linear_to_db(ctx->scale_factors[i]);
    }

    /* Encode first band absolutely */
    float first_quantized = roundf(log_scales[0] / SNS_QUANT_STEP);
    int first_quant = (int)clampf(first_quantized, -SNS_QUANT_RANGE, SNS_QUANT_RANGE - 1);

    /* Convert signed to unsigned for transmission */
    uint32_t first_unsigned = (uint32_t)(first_quant + SNS_QUANT_RANGE);
    if (bit_writer_write(&writer, first_unsigned, SNS_QUANT_BITS + 1) < 0)
        return -1;

    /* Encode remaining bands as deltas with inter-band prediction */
    float prev_log = first_quant * SNS_QUANT_STEP;

    for (int i = 1; i < ctx->num_bands; i++) {
        float predicted = prev_log;

        /* Optional inter-frame prediction if previous frame available */
        if (ctx->has_prev) {
            float prev_frame_log = linear_to_db(ctx->prev_scale[i]);
            predicted = predicted * (1.0f - SNS_INTER_ALPHA) +
                       prev_frame_log * SNS_INTER_ALPHA;
        }

        float delta = log_scales[i] - predicted;
        float delta_quantized = roundf(delta / SNS_QUANT_STEP);
        int delta_quant = (int)clampf(delta_quantized, -SNS_QUANT_RANGE, SNS_QUANT_RANGE - 1);

        /* Convert signed to unsigned */
        uint32_t delta_unsigned = (uint32_t)(delta_quant + SNS_QUANT_RANGE);
        if (bit_writer_write(&writer, delta_unsigned, SNS_QUANT_BITS + 1) < 0)
            return -1;

        prev_log = predicted + delta_quant * SNS_QUANT_STEP;
    }

    return bit_writer_bytes(&writer);
}

int
sns_decode_scales(sns_ctx_t *ctx,
                  const uint8_t *in, size_t in_len)
{
    if (!ctx || !in || in_len < (size_t)(ctx->num_bands * SNS_QUANT_BITS / 8))
        return -1;

    bit_reader_t reader;
    bit_reader_init(&reader, in, in_len);

    /* Copy previous scale factors for inter-frame prediction */
    if (ctx->has_prev) {
        memcpy(ctx->prev_scale, ctx->scale_factors,
               ctx->num_bands * sizeof(float));
    }

    /* Decode first band */
    uint32_t first_unsigned = bit_reader_read(&reader, SNS_QUANT_BITS + 1);
    int first_quant = (int)first_unsigned - SNS_QUANT_RANGE;
    float prev_log = first_quant * SNS_QUANT_STEP;

    ctx->scale_factors[0] = db_to_linear(prev_log);

    /* Decode remaining bands as deltas */
    for (int i = 1; i < ctx->num_bands; i++) {
        uint32_t delta_unsigned = bit_reader_read(&reader, SNS_QUANT_BITS + 1);
        int delta_quant = (int)delta_unsigned - SNS_QUANT_RANGE;

        float predicted = prev_log;

        /* Apply inter-frame prediction if available */
        if (ctx->has_prev) {
            float prev_frame_log = linear_to_db(ctx->prev_scale[i]);
            predicted = predicted * (1.0f - SNS_INTER_ALPHA) +
                       prev_frame_log * SNS_INTER_ALPHA;
        }

        float log_scale = predicted + delta_quant * SNS_QUANT_STEP;
        ctx->scale_factors[i] = db_to_linear(log_scale);

        prev_log = log_scale;
    }

    ctx->has_prev = true;
    return reader.pos + (reader.bit > 0 ? 1 : 0);
}

/* ---- BWE Implementation ---- */

void
bwe_init(bwe_ctx_t *ctx, uint16_t cutoff_band, uint16_t total_bands)
{
    if (!ctx)
        return;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cutoff_band = cutoff_band;
    ctx->total_bands = total_bands;
    ctx->num_high_bands = (cutoff_band < total_bands) ?
                         (total_bands - cutoff_band) : 0;
}

uint16_t
bwe_optimal_cutoff(int bits_per_frame, int num_bands, uint32_t sample_rate)
{
    /* Conservative thresholds to ensure quality */
    if (bits_per_frame >= 160) {
        return num_bands;  /* Code everything */
    }
    if (bits_per_frame >= 100) {
        return (num_bands * 3) / 4;  /* Code 75% */
    }
    if (bits_per_frame >= 60) {
        return num_bands / 2;  /* Code 50% */
    }

    /* Very low bitrate: code only 33% */
    uint16_t min_cutoff = num_bands / 3;

    /* For 8kHz, always code at least half the bands */
    if (sample_rate == 8000 && min_cutoff < num_bands / 2) {
        min_cutoff = num_bands / 2;
    }

    return min_cutoff;
}

int
bwe_encode(bwe_ctx_t *ctx,
           const float *mdct, int num_coeffs,
           const band_range_t *bands, int num_bands,
           uint8_t *out, size_t out_cap)
{
    if (!ctx || !mdct || !bands || !out || num_bands != ctx->total_bands)
        return -1;

    /* All bands coded by PVQ — no high bands for BWE to encode */
    if (ctx->cutoff_band >= num_bands) {
        ctx->num_high_bands = 0;
        return 0;
    }

    /* Check output capacity */
    if (out_cap < ctx->num_high_bands)
        return -1;

    bit_writer_t writer;
    bit_writer_init(&writer, out, out_cap);

    /* Extract and quantize energy for each uncoded band */
    for (uint16_t h = ctx->cutoff_band; h < ctx->total_bands; h++) {
        uint16_t start = bands[h].start;
        uint16_t end = bands[h].end;

        if (end > num_coeffs)
            end = num_coeffs;
        if (start >= end) {
            ctx->high_energy[h - ctx->cutoff_band] = BWE_ENERGY_MIN;
            continue;
        }

        /* Compute RMS energy */
        float energy = 0.0f;
        uint16_t count = end - start;

        for (uint16_t i = start; i < end; i++) {
            energy += mdct[i] * mdct[i];
        }

        float rms = sqrtf(energy / count);
        ctx->high_energy[h - ctx->cutoff_band] =
            clampf(rms, BWE_ENERGY_MIN, SNS_SCALE_MAX);

        /* Quantize to dB and encode */
        float db = linear_to_db(rms);
        float quantized_db = clampf(db, BWE_MIN_DB, BWE_MIN_DB + BWE_QUANT_RANGE - 1);
        uint32_t quant_val = (uint32_t)roundf((quantized_db - BWE_MIN_DB) / BWE_QUANT_STEP);
        quant_val = clampf(quant_val, 0, BWE_QUANT_RANGE - 1);

        if (bit_writer_write(&writer, quant_val, BWE_QUANT_BITS) < 0)
            return -1;
    }

    return bit_writer_bytes(&writer);
}

int
bwe_decode(bwe_ctx_t *ctx,
           const uint8_t *in, size_t in_len)
{
    if (!ctx || !in)
        return -1;
    if (ctx->num_high_bands == 0)
        return 0;
    if (in_len < ctx->num_high_bands)
        return -1;

    bit_reader_t reader;
    bit_reader_init(&reader, in, in_len);

    /* Decode energy values for each high band */
    for (uint16_t i = 0; i < ctx->num_high_bands; i++) {
        uint32_t quant_val = bit_reader_read(&reader, BWE_QUANT_BITS);
        float db = BWE_MIN_DB + quant_val * BWE_QUANT_STEP;
        ctx->high_energy[i] = db_to_linear(db);
    }

    return reader.pos + (reader.bit > 0 ? 1 : 0);
}

void
bwe_synthesize(const bwe_ctx_t *ctx,
               float *mdct, int num_coeffs,
               const band_range_t *bands, int num_bands)
{
    if (!ctx || !mdct || !bands ||
        num_bands != ctx->total_bands ||
        ctx->cutoff_band >= num_bands ||
        ctx->cutoff_band == 0)
        return;

    static uint32_t rng_state = 0x12345678u;  /* Simple PRNG state */

    /* Synthesize each uncoded band by folding from coded bands */
    for (uint16_t h = ctx->cutoff_band; h < ctx->total_bands; h++) {
        uint16_t start = bands[h].start;
        uint16_t end = bands[h].end;

        if (end > num_coeffs)
            end = num_coeffs;
        if (start >= end)
            continue;

        /* Choose source band cyclically from coded region */
        uint16_t src_idx = (h - ctx->cutoff_band) % ctx->cutoff_band;
        uint16_t src_start = bands[src_idx].start;
        uint16_t src_end = bands[src_idx].end;

        if (src_end > num_coeffs)
            src_end = num_coeffs;
        if (src_start >= src_end)
            continue;

        /* Copy coefficients with wrapping */
        uint16_t src_len = src_end - src_start;
        uint16_t dst_len = end - start;
        float copied_energy = 0.0f;

        for (uint16_t i = 0; i < dst_len; i++) {
            uint16_t src_offset = i % src_len;
            mdct[start + i] = mdct[src_start + src_offset];
            copied_energy += mdct[start + i] * mdct[start + i];
        }

        /* Apply sign randomization to reduce tonal artifacts */
        rng_state = rng_state * 1664525u + 1013904223u;  /* LCG */
        for (uint16_t i = 0; i < dst_len; i++) {
            if ((rng_state >> (i & 31u)) & 1u) {
                mdct[start + i] = -mdct[start + i];
            }
        }

        /* Scale to match transmitted energy */
        if (copied_energy > BWE_ENERGY_MIN) {
            float copied_rms = sqrtf(copied_energy / dst_len);
            float target_energy = ctx->high_energy[h - ctx->cutoff_band];
            float scale = target_energy / copied_rms;

            for (uint16_t i = start; i < end; i++) {
                mdct[i] *= scale;
            }
        }
    }
}