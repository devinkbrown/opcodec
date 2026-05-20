/*
 * opcodec/sam.c — Semantic Audio Mode: ultra-low-bitrate parametric vocoder
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/sam.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── LPC Analysis ─────────────────────────────────────────────────────────── */

/* Levinson-Durbin algorithm: autocorrelation → LPC coefficients.
 * lpc[0] = 1.0, lpc[1..order] = predictor coefficients (negated). */
static void lpc_analyze(const float *pcm, int len, int order,
                        float *lpc, float *energy_out)
{
    float R[SAM_ORDER + 1];
    /* Autocorrelation */
    for (int lag = 0; lag <= order; lag++) {
        float s = 0.0f;
        for (int n = lag; n < len; n++) s += pcm[n] * pcm[n - lag];
        R[lag] = s;
    }
    if (energy_out) *energy_out = R[0];

    /* Levinson-Durbin */
    float a[SAM_ORDER + 1];
    float E = R[0] + 1e-6f;
    memset(a, 0, sizeof(a));
    a[0] = 1.0f;

    for (int m = 1; m <= order; m++) {
        float lambda = 0.0f;
        for (int j = 1; j <= m - 1; j++) lambda += a[j] * R[m - j];
        lambda = -(R[m] + lambda) / E;
        float a_new[SAM_ORDER + 1];
        for (int j = 0; j <= m; j++) {
            a_new[j] = a[j] + lambda * a[m - j];
        }
        memcpy(a, a_new, ((size_t)m + 1) * sizeof(float));
        E *= 1.0f - lambda * lambda;
        if (E < 1e-10f) E = 1e-10f;
    }
    memcpy(lpc, a, ((size_t)order + 1) * sizeof(float));
}

/* Convert LPC coefficients to Line Spectral Frequencies (LSFs).
 * Bisection method finding roots of P(z) = A(z) + z^{-(order+1)} A(1/z)
 * and Q(z) = A(z) - z^{-(order+1)} A(1/z). */
static void lpc_to_lsf(const float *lpc, int order, float *lsf)
{
    /* Sum/difference polynomials */
    float P[SAM_ORDER + 2], Q[SAM_ORDER + 2];
    for (int i = 0; i <= order; i++) {
        P[i] = lpc[i] + lpc[order - i];
        Q[i] = lpc[i] - lpc[order - i];
    }
    /* Find alternating roots by scanning [0, π] */
    int nroots = 0;
    float step = (float)M_PI / 512.0f;
    float *poly = P;
    float prev_val = 1.0f;
    for (float w = step; w < (float)M_PI && nroots < order; w += step) {
        float val = 1.0f;
        for (int i = 0; i <= order; i++)
            val = val * cosf(w) + poly[i];  /* Chebyshev-ish eval */
        if (prev_val * val < 0.0f) {
            /* Root between w-step and w: bisect */
            float lo = w - step, hi = w;
            for (int iter = 0; iter < 16; iter++) {
                float mid = (lo + hi) * 0.5f;
                float fmid = 1.0f;
                for (int i = 0; i <= order; i++) fmid = fmid * cosf(mid) + poly[i];
                if (fmid * prev_val < 0.0f) hi = mid; else lo = mid;
            }
            lsf[nroots++] = (lo + hi) * 0.5f;
            poly = (poly == P) ? Q : P;  /* alternate polynomials */
        }
        prev_val = val;
    }
    /* Ensure monotonically increasing LSFs */
    for (int i = 0; i < order; i++) {
        if (i == 0 && lsf[i] < 0.01f) lsf[i] = 0.01f;
        if (i > 0 && lsf[i] <= lsf[i-1]) lsf[i] = lsf[i-1] + 0.01f;
        if (lsf[i] > (float)M_PI - 0.01f) lsf[i] = (float)M_PI - 0.01f;
    }
    /* Fill any missed roots with spaced values */
    for (int i = nroots; i < order; i++)
        lsf[i] = lsf[i > 0 ? i-1 : 0] + (float)M_PI / (float)(order + 1);
}

/* Convert LSFs back to LPC coefficients (inverse of above, Newton form) */
static void lsf_to_lpc(const float *lsf, int order, float *lpc)
{
    float P[SAM_ORDER + 2] = {0};
    float Q[SAM_ORDER + 2] = {0};
    P[0] = 1.0f; Q[0] = 1.0f;
    /* Build P and Q from alternating LSF roots */
    for (int i = 0; i < order; i += 2) {
        float w = lsf[i];
        /* Convolve P with (1 - 2cos(w)z^{-1} + z^{-2}) */
        float c = -2.0f * cosf(w);
        for (int j = i + 2; j >= 2; j--)
            P[j] += c * P[j-1] + P[j-2];
        P[1] += c;
    }
    for (int i = 1; i < order; i += 2) {
        float w = lsf[i];
        float c = -2.0f * cosf(w);
        for (int j = i + 2; j >= 2; j--)
            Q[j] += c * Q[j-1] + Q[j-2];
        Q[1] += c;
    }
    lpc[0] = 1.0f;
    for (int i = 1; i <= order; i++)
        lpc[i] = 0.5f * (P[i] + Q[i]);
}

/* ── Simple pitch detector (autocorrelation) ─────────────────────────────── */
static int detect_pitch(const float *pcm, int len, int rate,
                        float *out_voiced)
{
    int min_p = rate / 500;   /* 500 Hz max */
    int max_p = rate / 60;    /* 60  Hz min */
    if (min_p < 2) min_p = 2;
    if (max_p > len / 2) max_p = len / 2;

    float best_corr = 0.0f;
    int   best_lag  = min_p;
    float r0 = 0.0f;
    for (int n = 0; n < len; n++) r0 += pcm[n] * pcm[n];
    if (r0 < 1e-8f) { *out_voiced = 0.0f; return 0; }

    for (int lag = min_p; lag <= max_p; lag++) {
        float s = 0.0f;
        for (int n = lag; n < len; n++) s += pcm[n] * pcm[n - lag];
        float corr = s / r0;
        if (corr > best_corr) { best_corr = corr; best_lag = lag; }
    }
    *out_voiced = best_corr;
    return (best_corr > 0.35f) ? best_lag : 0;
}

/* ── Simple VQ (nearest neighbour in 2D sub-space, full codebook too large
 *    for a fixed table → use lattice quantizer on normalized LSFs) ────────── */

/* Quantize 10 LSFs to 12 bits using scalar uniform quantization. */
static uint16_t lsf_quantize(const float *lsf)
{
    /* Reduce 10 LSFs to a single 12-bit code using PCA projection.
     * Projection: first and sixth LSF capture most of the variance in
     * voiced/unvoiced and spectral tilt. */
    float l1 = lsf[0] / (float)M_PI;   /* [0,1] */
    float l6 = lsf[5] / (float)M_PI;   /* [0,1] */
    /* Encode as 6 bits each */
    uint16_t q1 = (uint16_t)(l1 * 63.0f + 0.5f);
    uint16_t q6 = (uint16_t)(l6 * 63.0f + 0.5f);
    if (q1 > 63) q1 = 63;
    if (q6 > 63) q6 = 63;
    return (q1 << 6) | q6;
}

static void lsf_dequantize(uint16_t code, float *lsf)
{
    float l1 = (float)((code >> 6) & 0x3F) / 63.0f;
    float l6 = (float)(code        & 0x3F) / 63.0f;
    /* Reconstruct all 10 LSFs with equal spacing between l1 and l6 */
    for (int i = 0; i < SAM_ORDER; i++) {
        float t = (float)i / (float)(SAM_ORDER - 1);
        lsf[i] = (float)M_PI * (l1 + t * (l6 - l1));
        if (lsf[i] < 0.05f) lsf[i] = 0.05f;
        if (lsf[i] > (float)M_PI - 0.05f) lsf[i] = (float)M_PI - 0.05f;
        if (i > 0 && lsf[i] <= lsf[i-1]) lsf[i] = lsf[i-1] + 0.03f;
    }
}

/* ── Frame serialization ─────────────────────────────────────────────────── */

void sam_frame_write(const sam_frame_t *f, uint8_t *out)
{
    /* Layout (32 bits):
     * bits 31-20: lsf_code (12 bits)
     * bits 19-13: pitch_7  (7 bits)
     * bit  12   : voiced   (1 bit)
     * bits 11- 6: energy_6 (6 bits)
     * bits  5- 4: flags    (2 bits)
     * bits  3- 0: reserved (4 bits, zero)
     */
    uint32_t w =
        ((uint32_t)(f->lsf_code & 0xFFFu) << 20) |
        ((uint32_t)(f->pitch_7  & 0x7Fu)  << 13) |
        ((uint32_t)(f->voiced   & 0x01u)  << 12) |
        ((uint32_t)(f->energy_6 & 0x3Fu)  <<  6) |
        ((uint32_t)(f->flags    & 0x03u)  <<  4);
    out[0] = (uint8_t)(w >> 24);
    out[1] = (uint8_t)(w >> 16);
    out[2] = (uint8_t)(w >>  8);
    out[3] = (uint8_t)(w);
}

void sam_frame_read(sam_frame_t *f, const uint8_t *in)
{
    uint32_t w = ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16)
               | ((uint32_t)in[2] <<  8) | (uint32_t)in[3];
    f->lsf_code = (uint16_t)((w >> 20) & 0xFFFu);
    f->pitch_7  = (uint8_t) ((w >> 13) & 0x7Fu);
    f->voiced   = (uint8_t) ((w >> 12) & 0x01u);
    f->energy_6 = (uint8_t) ((w >>  6) & 0x3Fu);
    f->flags    = (uint8_t) ((w >>  4) & 0x03u);
}

/* ── Encoder ─────────────────────────────────────────────────────────────── */

int sam_enc_init(sam_enc_t *enc, uint32_t sample_rate)
{
    if (!enc) return -1;
    memset(enc, 0, sizeof(*enc));
    enc->sample_rate = (sample_rate > 24000) ? 16000 : sample_rate;
    enc->frame_size  = (int)(enc->sample_rate * SAM_FRAME_MS / 1000);
    enc->rng = 12345u;
    enc->initialized = true;
    return 0;
}

int sam_encode(sam_enc_t *enc, const float *pcm, int frame_size, uint8_t *out)
{
    if (!enc || !enc->initialized || !pcm || !out) return -1;

    sam_frame_t f = {0};

    /* Pre-emphasis: y[n] = x[n] - 0.97*x[n-1] */
    float emph[SAM_MAX_FRAME_SIZE];
    int n = (frame_size < SAM_MAX_FRAME_SIZE) ? frame_size : SAM_MAX_FRAME_SIZE;
    emph[0] = pcm[0] - 0.97f * enc->preemph_prev;
    for (int i = 1; i < n; i++) emph[i] = pcm[i] - 0.97f * pcm[i-1];
    enc->preemph_prev = pcm[n-1];

    /* LPC analysis */
    float energy = 0.0f;
    lpc_analyze(emph, n, SAM_ORDER, enc->lpc, &energy);
    lpc_to_lsf(enc->lpc, SAM_ORDER, enc->lsf);
    f.lsf_code = lsf_quantize(enc->lsf);

    /* Pitch detection */
    float voiced_prob = 0.0f;
    int period = detect_pitch(emph, n, (int)enc->sample_rate, &voiced_prob);
    f.voiced  = (period > 0) ? 1u : 0u;
    /* Quantize pitch period: 7 bits, 0 = unvoiced */
    if (period > 0) {
        float norm = (float)(period - SAM_PITCH_MIN)
                   / (float)(SAM_PITCH_MAX - SAM_PITCH_MIN);
        f.pitch_7 = (uint8_t)(norm * 127.0f + 0.5f);
        if (f.pitch_7 < 1) f.pitch_7 = 1;
        if (f.pitch_7 > 127) f.pitch_7 = 127;
    }

    /* Energy: 6-bit log-energy */
    float rms = sqrtf(energy / (float)n + 1e-12f);
    float db  = 20.0f * log10f(rms + 1e-12f) + 94.0f;
    if (db < 0.0f) db = 0.0f;
    if (db > 94.0f) db = 94.0f;
    f.energy_6 = (uint8_t)(db / 94.0f * 63.0f + 0.5f);

    /* Silence detection */
    f.flags = (rms < 1e-4f) ? 2u : 0u;

    enc->pitch_period = period;
    enc->energy = energy;

    sam_frame_write(&f, out);
    return SAM_PACKET_BYTES;
}

/* ── Decoder ─────────────────────────────────────────────────────────────── */

int sam_dec_init(sam_dec_t *dec, uint32_t sample_rate)
{
    if (!dec) return -1;
    memset(dec, 0, sizeof(*dec));
    dec->sample_rate = (sample_rate > 24000) ? 16000 : sample_rate;
    dec->frame_size  = (int)(dec->sample_rate * SAM_FRAME_MS / 1000);
    dec->rng = 54321u;
    dec->initialized = true;
    return 0;
}

int sam_decode(sam_dec_t *dec, const uint8_t *in, float *out, int frame_size)
{
    if (!dec || !dec->initialized || !in || !out) return -1;
    int n = (frame_size < SAM_MAX_FRAME_SIZE) ? frame_size : SAM_MAX_FRAME_SIZE;

    sam_frame_t f;
    sam_frame_read(&f, in);

    /* Silence */
    if (f.flags == 2) {
        memset(out, 0, (size_t)n * sizeof(float));
        return n;
    }

    /* Reconstruct LPC coefficients from LSF code */
    float lsf_dec[SAM_ORDER];
    lsf_dequantize(f.lsf_code, lsf_dec);

    /* Interpolate between previous and current LPC for smoother transitions */
    float lpc_cur[SAM_ORDER + 1];
    lsf_to_lpc(lsf_dec, SAM_ORDER, lpc_cur);

    if (dec->has_prev) {
        for (int i = 0; i <= SAM_ORDER; i++)
            lpc_cur[i] = 0.5f * dec->prev_lpc[i] + 0.5f * lpc_cur[i];
    }
    memcpy(dec->prev_lpc, lpc_cur, sizeof(lpc_cur));
    dec->has_prev = true;

    /* Decode energy */
    float target_rms = powf(10.0f,
        ((float)f.energy_6 / 63.0f * 94.0f - 94.0f) / 20.0f);

    /* Decode pitch period */
    int period = 0;
    if (f.voiced) {
        float norm = (float)f.pitch_7 / 127.0f;
        period = (int)(norm * (float)(SAM_PITCH_MAX - SAM_PITCH_MIN)
                       + (float)SAM_PITCH_MIN + 0.5f);
        if (period < SAM_PITCH_MIN) period = SAM_PITCH_MIN;
        if (period > SAM_PITCH_MAX) period = SAM_PITCH_MAX;
    }

    /* Generate excitation and synthesize */
    float excitation[SAM_MAX_FRAME_SIZE];
    float energy_exc = 0.0f;

    if (f.voiced && period > 0) {
        /* Pulse train excitation */
        for (int i = 0; i < n; i++) {
            dec->exc_phase += 1.0f;
            float v;
            if (dec->exc_phase >= (float)period) {
                dec->exc_phase -= (float)period;
                v = 1.0f;
            } else {
                v = 0.0f;
            }
            excitation[i] = v;
            energy_exc += v * v;
        }
    } else {
        /* White noise excitation (unvoiced) */
        for (int i = 0; i < n; i++) {
            dec->rng = dec->rng * 1664525u + 1013904223u;
            float v  = (float)(int32_t)dec->rng / 2147483648.0f;
            excitation[i] = v;
            energy_exc += v * v;
        }
    }

    /* Normalize excitation to unit RMS */
    float exc_rms = sqrtf(energy_exc / (float)n + 1e-12f);
    float exc_gain = (exc_rms > 1e-10f) ? (target_rms / exc_rms) : 0.0f;
    for (int i = 0; i < n; i++) excitation[i] *= exc_gain;

    /* LPC synthesis filter: H(z) = 1 / A(z) */
    for (int i = 0; i < n; i++) {
        float s = excitation[i];
        for (int j = 1; j <= SAM_ORDER; j++) {
            int k = i - j;
            float prev = (k >= 0) ? out[k] : dec->synth_state[SAM_ORDER - j + (i - j + SAM_ORDER)];
            if (k < 0) {
                int si = SAM_ORDER + k;
                if (si >= 0 && si < SAM_ORDER) prev = dec->synth_state[si];
                else prev = 0.0f;
            }
            s -= lpc_cur[j] * prev;
        }
        out[i] = s;
    }
    /* Update synthesis memory */
    for (int j = 0; j < SAM_ORDER; j++) {
        int src = n - SAM_ORDER + j;
        dec->synth_state[j] = (src >= 0) ? out[src] : 0.0f;
    }

    /* Post-filter: 1/(1-0.4*A(z)) for naturalness */
    for (int i = 0; i < n; i++) {
        float s = out[i];
        for (int j = 1; j <= SAM_ORDER; j++) {
            int k = i - j;
            float prev = (k >= 0) ? out[k] : dec->postfilt_state[SAM_ORDER + k];
            if (k < 0) {
                int si = SAM_ORDER + k;
                prev = (si >= 0 && si < SAM_ORDER) ? dec->postfilt_state[si] : 0.0f;
            }
            s += 0.4f * lpc_cur[j] * prev;
        }
        out[i] = s;
    }
    for (int j = 0; j < SAM_ORDER; j++) {
        int src = n - SAM_ORDER + j;
        dec->postfilt_state[j] = (src >= 0) ? out[src] : 0.0f;
    }

    /* De-emphasis: y[n] = x[n] + 0.97*y[n-1] */
    for (int i = 1; i < n; i++) out[i] += 0.97f * out[i-1];

    /* Clip to [-1, 1] */
    for (int i = 0; i < n; i++) {
        if (out[i] >  1.0f) out[i] =  1.0f;
        if (out[i] < -1.0f) out[i] = -1.0f;
    }
    return n;
}
