/*
 * opcodec/aec2.c — Acoustic Environment Codec v2 implementation
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/aec2.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Internal helpers ─────────────────────────────────────────────────── */

/* Map linear RIR energy to 6-bit log code (0=silent, 63=max) */
static uint8_t energy_to_6bit(float energy)
{
    if (energy <= 0.0f) return 0;
    /* log scale: 6 bits → 64 steps over 96 dB range (1.5 dB/step) */
    float db = 10.0f * log10f(energy + 1e-10f);
    /* Normalize to [0, 63]: -96 dB → 0, 0 dB → 63 */
    int code = (int)((db + 96.0f) * 63.0f / 96.0f + 0.5f);
    if (code < 0)  code = 0;
    if (code > 63) code = 63;
    return (uint8_t)code;
}

static float energy_from_6bit(uint8_t code)
{
    float db = (float)code * 96.0f / 63.0f - 96.0f;
    return powf(10.0f, db / 10.0f);
}

/* Compute per-band energy: divide AEC2_RIR_TAPS into n_bands Mel-spaced bands */
static void compute_band_energies(const float *rir, int n_taps, int n_bands,
                                   uint8_t *codes)
{
    float log_min = logf(1.0f);
    float log_max = logf((float)n_taps);
    for (int b = 0; b < n_bands; b++) {
        float flo = expf(log_min + (float)b       * (log_max - log_min) / (float)n_bands);
        float fhi = expf(log_min + (float)(b + 1) * (log_max - log_min) / (float)n_bands);
        int t0 = (int)flo;
        int t1 = (int)fhi;
        if (t0 >= n_taps) t0 = n_taps - 1;
        if (t1 >= n_taps) t1 = n_taps - 1;
        if (t1 < t0)      t1 = t0;
        float energy = 0.0f;
        for (int t = t0; t <= t1; t++)
            energy += rir[t] * rir[t];
        float avg = (t1 >= t0) ? energy / (float)(t1 - t0 + 1) : 0.0f;
        codes[b] = energy_to_6bit(avg);
    }
}

/* Pack/unpack 6-bit codes into a byte stream */
static int pack_6bit(const uint8_t *codes, int n, uint8_t *dst)
{
    int bits = 0, out_idx = 0;
    uint32_t acc = 0;
    for (int i = 0; i < n; i++) {
        acc = (acc << 6) | (codes[i] & 0x3Fu);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            dst[out_idx++] = (uint8_t)(acc >> bits);
            acc &= (1u << bits) - 1u;
        }
    }
    if (bits > 0)
        dst[out_idx++] = (uint8_t)(acc << (8 - bits));
    return out_idx;
}

static int unpack_6bit(const uint8_t *src, int n_bytes, int n_codes, uint8_t *codes)
{
    int bits = 0, in_idx = 0;
    uint32_t acc = 0;
    for (int i = 0; i < n_codes; i++) {
        while (bits < 6 && in_idx < n_bytes) {
            acc = (acc << 8) | src[in_idx++];
            bits += 8;
        }
        if (bits < 6) break;
        bits -= 6;
        codes[i] = (uint8_t)((acc >> bits) & 0x3Fu);
        acc &= (1u << bits) - 1u;
    }
    return in_idx;
}

/* Pack early reflection taps as 4-bit signed delta values */
static int pack_4bit_signed(const int8_t *taps, int n, uint8_t *dst)
{
    int out_idx = 0;
    for (int i = 0; i < n; i += 2) {
        uint8_t lo = (uint8_t)(taps[i] & 0x0F);
        uint8_t hi = (i + 1 < n) ? (uint8_t)(taps[i + 1] & 0x0F) : 0;
        dst[out_idx++] = (hi << 4) | lo;
    }
    return out_idx;
}

static void unpack_4bit_signed(const uint8_t *src, int n_bytes, int n_taps, int8_t *taps)
{
    for (int i = 0; i < n_taps; i++) {
        int byte_idx = i / 2;
        if (byte_idx >= n_bytes) { taps[i] = 0; continue; }
        uint8_t nibble = (i & 1) ? (src[byte_idx] >> 4) : (src[byte_idx] & 0x0F);
        /* Sign extend 4-bit value */
        taps[i] = (int8_t)((nibble & 8) ? (int8_t)(nibble | 0xF0) : (int8_t)nibble);
    }
}

/* Compute RIR change magnitude (normalized L2 difference) */
static float rir_delta(const float *a, const float *b, int n)
{
    float sum = 0.0f, norm = 1e-10f;
    for (int i = 0; i < n; i++) {
        float d = a[i] - b[i];
        sum  += d * d;
        norm += b[i] * b[i];
    }
    return sqrtf(sum / norm);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void aec2_enc_init(aec2_enc_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->wet_gain = AEC2_WET_DEFAULT;
    ctx->dry_gain = AEC2_DRY_DEFAULT;
}

void aec2_enc_set_rir(aec2_enc_t *ctx, const float *rir, float wet, float dry)
{
    if (!ctx || !rir) return;
    memcpy(ctx->rir, rir, AEC2_RIR_TAPS * sizeof(float));
    ctx->wet_gain = (wet < 0.0f) ? 0.0f : (wet > 1.0f ? 1.0f : wet);
    ctx->dry_gain = (dry < 0.0f) ? 0.0f : (dry > 1.0f ? 1.0f : dry);
    ctx->has_rir = true;
}

int aec2_encode(aec2_enc_t *ctx, uint8_t *out, int out_cap, uint32_t now_ms)
{
    if (!ctx || !out || out_cap < 4 || !ctx->has_rir) return 0;

    /* Rate limit: don't send more than one update per interval */
    if (ctx->last_update_ms != 0 &&
        (now_ms - ctx->last_update_ms) < AEC2_UPDATE_INTERVAL_MS)
        return 0;

    /* Check if change is significant enough to warrant transmission */
    if (ctx->last_update_ms != 0 &&
        rir_delta(ctx->rir, ctx->prev_rir, AEC2_RIR_TAPS) < AEC2_UPDATE_THRESH)
        return 0;

    const int n_bands = AEC2_MAX_BANDS;
    const int n_early = AEC2_MAX_EARLY;

    /* Compute band energy codes */
    uint8_t band_codes[AEC2_MAX_BANDS];
    compute_band_energies(ctx->rir, AEC2_RIR_TAPS, n_bands, band_codes);

    /* Early reflection deltas (quantize to 4-bit signed ±7) */
    int8_t early_deltas[AEC2_MAX_EARLY];
    for (int t = 0; t < n_early && t < AEC2_RIR_TAPS; t++) {
        float amp = ctx->rir[t] * 7.0f;  /* scale to ±7 */
        int v = (int)(amp + (amp >= 0 ? 0.5f : -0.5f));
        if (v > 7)  v = 7;
        if (v < -7) v = -7;
        early_deltas[t] = (int8_t)v;
    }

    /* Build packet */
    uint8_t *p = out;
    *p++ = 2;         /* version */
    *p++ = (uint8_t)n_bands;
    *p++ = (uint8_t)n_early;
    *p++ = 0x03u;     /* flags: wet_gain_present | dry_gain_present */

    /* Band energies: packed 6-bit codes */
    int band_bytes = pack_6bit(band_codes, n_bands, p);
    p += band_bytes;

    /* Early reflection taps: packed 4-bit signed */
    int early_bytes = pack_4bit_signed(early_deltas, n_early, p);
    p += early_bytes;

    /* Gain bytes */
    *p++ = (uint8_t)(ctx->wet_gain * 255.0f + 0.5f);
    *p++ = (uint8_t)(ctx->dry_gain * 255.0f + 0.5f);

    int written = (int)(p - out);
    if (written > out_cap) return 0;  /* safety check */

    memcpy(ctx->prev_rir, ctx->rir, AEC2_RIR_TAPS * sizeof(float));
    ctx->last_update_ms = now_ms;
    return written;
}

void aec2_dec_init(aec2_dec_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->wet_gain = AEC2_WET_DEFAULT;
    ctx->dry_gain = AEC2_DRY_DEFAULT;
    ctx->active   = false;  /* disabled until first RIR received */
}

int aec2_decode(aec2_dec_t *ctx, const uint8_t *in, int in_len)
{
    if (!ctx || !in || in_len < 4) return -1;

    const uint8_t *p = in;
    uint8_t version  = *p++;
    uint8_t n_bands  = *p++;
    uint8_t n_early  = *p++;
    uint8_t flags    = *p++;
    int remaining    = in_len - 4;

    if (version != 2) return -1;
    if (n_bands > AEC2_MAX_BANDS || n_early > AEC2_MAX_EARLY) return -1;

    /* Unpack band energies */
    int band_bytes = (n_bands * 6 + 7) / 8;
    if (remaining < band_bytes) return -1;
    uint8_t band_codes[AEC2_MAX_BANDS] = {0};
    unpack_6bit(p, band_bytes, n_bands, band_codes);
    p += band_bytes; remaining -= band_bytes;

    /* Unpack early taps */
    int early_bytes = (n_early + 1) / 2;
    if (remaining < early_bytes) return -1;
    int8_t early_deltas[AEC2_MAX_EARLY] = {0};
    unpack_4bit_signed(p, early_bytes, n_early, early_deltas);
    p += early_bytes; remaining -= early_bytes;

    /* Read gain bytes if present */
    if ((flags & 1) && remaining >= 1) {
        ctx->wet_gain = (float)*p++ / 255.0f;
        remaining--;
    }
    if ((flags & 2) && remaining >= 1) {
        ctx->dry_gain = (float)*p++ / 255.0f;
        remaining--;
    }

    /* Reconstruct RIR from band energies:
     * Fill each band with a constant spectral envelope amplitude.
     * This is a coarse but perceptually adequate approximation. */
    memset(ctx->rir, 0, sizeof(ctx->rir));
    float log_min = logf(1.0f);
    float log_max = logf((float)AEC2_RIR_TAPS);
    for (int b = 0; b < n_bands; b++) {
        float flo = expf(log_min + (float)b       * (log_max - log_min) / (float)n_bands);
        float fhi = expf(log_min + (float)(b + 1) * (log_max - log_min) / (float)n_bands);
        int t0 = (int)flo;
        int t1 = (int)fhi;
        if (t0 >= AEC2_RIR_TAPS) t0 = AEC2_RIR_TAPS - 1;
        if (t1 >= AEC2_RIR_TAPS) t1 = AEC2_RIR_TAPS - 1;
        if (t1 < t0)              t1 = t0;
        float amp = sqrtf(energy_from_6bit(band_codes[b]));
        /* Exponential decay within band to sound like a room tail */
        for (int t = t0; t <= t1; t++) {
            float decay = expf(-3.0f * (float)(t - t0) / (float)(t1 - t0 + 1));
            ctx->rir[t] = amp * decay;
        }
    }

    /* Overlay early reflections (overwrite first n_early taps) */
    for (int t = 0; t < n_early && t < AEC2_RIR_TAPS; t++)
        ctx->rir[t] = (float)early_deltas[t] / 7.0f;

    /* Normalize RIR to unit peak */
    float peak = 0.0f;
    for (int t = 0; t < AEC2_RIR_TAPS; t++) {
        float a = ctx->rir[t] < 0 ? -ctx->rir[t] : ctx->rir[t];
        if (a > peak) peak = a;
    }
    if (peak > 1e-6f)
        for (int t = 0; t < AEC2_RIR_TAPS; t++)
            ctx->rir[t] /= peak;

    ctx->active = true;
    return 0;
}

void aec2_apply(aec2_dec_t *ctx, float *pcm, int n_samples)
{
    if (!ctx || !pcm || n_samples <= 0 || !ctx->active) return;

    /* Split-buffer convolution: eliminates per-tap modulo arithmetic.
     *
     * We maintain a double-length linear scratch buffer of size 2×AEC2_RIR_TAPS
     * and copy the circular window into it once per n_samples batch.  The inner
     * FIR loop then reads a contiguous range, which the compiler can auto-vectorise
     * (no aliasing barriers from circular-buffer index wrapping).
     *
     * The copy is O(AEC2_RIR_TAPS) per batch, amortised over n_samples; the inner
     * loop saves one modulo per tap per sample = AEC2_RIR_TAPS × n_samples modulos.
     */
    float lin[AEC2_RIR_TAPS * 2];

    /* Build linear window: newest sample at lin[AEC2_RIR_TAPS-1], oldest at lin[0].
     * conv_head points to the *next* write slot, so the most-recent sample is at
     * (conv_head - 1 + AEC2_RIR_TAPS) % AEC2_RIR_TAPS.
     * We arrange lin[] so that lin[AEC2_RIR_TAPS - 1 - k] = x[n-k]. */
    int h = (ctx->conv_head > 0) ? ctx->conv_head - 1 : AEC2_RIR_TAPS - 1;

    for (int i = 0; i < n_samples; i++) {
        float dry = pcm[i];

        /* Write into circular buffer and advance head */
        ctx->conv_buf[ctx->conv_head] = dry;
        ctx->conv_head = (ctx->conv_head + 1 < AEC2_RIR_TAPS) ? ctx->conv_head + 1 : 0;

        /* Update linear view: insert new sample at the front */
        h = (h > 0) ? h - 1 : AEC2_RIR_TAPS - 1;

        /* Build contiguous segment starting at h (wraps once at most).
         * lin[k] = x[n-k] for k=0..AEC2_RIR_TAPS-1 */
        int tail = AEC2_RIR_TAPS - h;  /* samples from h to end of conv_buf */
        memcpy(lin,        ctx->conv_buf + h, (size_t)tail * sizeof(float));
        memcpy(lin + tail, ctx->conv_buf,     (size_t)h    * sizeof(float));

        /* Inner FIR: contiguous, compiler-vectorisable */
        float wet = 0.0f;
        for (int k = 0; k < AEC2_RIR_TAPS; k++)
            wet += ctx->rir[k] * lin[k];

        float out = ctx->dry_gain * dry + ctx->wet_gain * wet;
        if (out >  1.0f) out =  1.0f;
        if (out < -1.0f) out = -1.0f;
        pcm[i] = out;
    }
}
