/*
 * opcodec/ns2.c — Noise Suppression v2 implementation
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/ns2.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Radix-2 DIT FFT, N=256 (Cooley-Tukey) ─────────────────────────────
 *
 * Replaces the O(N²) direct DFT (33K multiply-adds + 66K trig calls per
 * frame) with an O(N log N) FFT using precomputed twiddle factors
 * (1024 butterflies + 0 transcendental calls at runtime).
 *
 * At 187 frames/sec (48 kHz / 256 samples), this saves ~12 million
 * cosf/sinf calls per second in the noise suppression path.
 * ─────────────────────────────────────────────────────────────────────── */

#define FFT_N     256
#define FFT_HALF  128

/* Twiddle factors: W_k = e^{-j 2π k / N}  for k = 0..N/2-1 */
static float fft_wr[FFT_HALF];  /* cos(-2π k / 256) */
static float fft_wi[FFT_HALF];  /* sin(-2π k / 256) */
static bool  fft_lut_ready = false;

static void fft_init_lut(void)
{
    if (fft_lut_ready) return;
    for (int k = 0; k < FFT_HALF; k++) {
        float angle = -2.0f * (float)M_PI * (float)k / (float)FFT_N;
        fft_wr[k] = cosf(angle);
        fft_wi[k] = sinf(angle);
    }
    fft_lut_ready = true;
}

/* In-place radix-2 DIT FFT, N=256 complex samples. */
static void fft256(float *xr, float *xi)
{
    /* Bit-reversal permutation (N=256, 8-bit reversal) */
    int j = 0;
    for (int i = 1; i < FFT_N - 1; i++) {
        int bit = FFT_N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t;
            t = xr[i]; xr[i] = xr[j]; xr[j] = t;
            t = xi[i]; xi[i] = xi[j]; xi[j] = t;
        }
    }

    /* Butterfly stages: log2(256) = 8 */
    for (int len = 2; len <= FFT_N; len <<= 1) {
        int half_len = len >> 1;
        int step     = FFT_N / len;   /* twiddle index step */
        for (int i = 0; i < FFT_N; i += len) {
            for (int k = 0; k < half_len; k++) {
                float wr = fft_wr[k * step];
                float wi = fft_wi[k * step];
                float ur = xr[i + k],           ui = xi[i + k];
                float vr = xr[i + k + half_len] * wr - xi[i + k + half_len] * wi;
                float vi = xr[i + k + half_len] * wi + xi[i + k + half_len] * wr;
                xr[i + k]           = ur + vr;
                xi[i + k]           = ui + vi;
                xr[i + k + half_len] = ur - vr;
                xi[i + k + half_len] = ui - vi;
            }
        }
    }
}

/* Real DFT: N=256 real input → N/2+1 = 129 complex bins. */
static void rdft256(const float *in, float *re, float *im)
{
    float xr[FFT_N], xi[FFT_N];
    memcpy(xr, in, FFT_N * sizeof(float));
    memset(xi, 0,  FFT_N * sizeof(float));
    fft_init_lut();
    fft256(xr, xi);
    for (int k = 0; k < NS2_FREQ_BINS; k++) {
        re[k] = xr[k];
        im[k] = xi[k];
    }
}

/* Inverse real DFT: N/2+1 complex bins → N=256 real output.
 * Uses the conjugate-FFT-conjugate-scale identity. */
static void irdft256(const float *re, const float *im, float *out)
{
    float xr[FFT_N], xi[FFT_N];

    /* Reconstruct full conjugate-symmetric spectrum */
    xr[0] = re[0]; xi[0] = 0.0f;
    for (int k = 1; k < NS2_FREQ_BINS - 1; k++) {
        xr[k]           = re[k];
        xi[k]           = im[k];
        xr[FFT_N - k]   = re[k];
        xi[FFT_N - k]   = -im[k];
    }
    xr[FFT_HALF] = re[NS2_FREQ_BINS - 1];
    xi[FFT_HALF] = 0.0f;

    /* IFFT via conjugate trick: IFFT(X) = conj(FFT(conj(X))) / N */
    for (int i = 0; i < FFT_N; i++) xi[i] = -xi[i];
    fft_init_lut();
    fft256(xr, xi);
    const float scale = 1.0f / (float)FFT_N;
    for (int i = 0; i < FFT_N; i++) out[i] = xr[i] * scale;
}

/* ── MCRA noise PSD update ─────────────────────────────────────────────── */

#define MCRA_ALPHA_S  0.9f    /* smoothing for spectral floor */
#define MCRA_ALPHA_D  0.85f   /* smoothing for noise PSD */
#define MCRA_L        8       /* sub-window length (frames) */
#define MCRA_BETA     0.8f    /* minimum statistics bias compensation */

static void mcra_update(ns2_ctx_t *ctx, const float *power_spec)
{
    /* Update running minimum */
    ctx->min_sub_count++;
    if (ctx->min_sub_count >= MCRA_L) {
        /* Swap sub-window: new minimum starts from current sub-window min */
        for (int k = 0; k < NS2_FREQ_BINS; k++) {
            ctx->min_psd[k] = ctx->min_psd_sub[k];
            ctx->min_psd_sub[k] = power_spec[k];
        }
        ctx->min_sub_count = 0;
    } else {
        for (int k = 0; k < NS2_FREQ_BINS; k++) {
            if (power_spec[k] < ctx->min_psd_sub[k])
                ctx->min_psd_sub[k] = power_spec[k];
            if (ctx->min_psd_sub[k] < ctx->min_psd[k])
                ctx->min_psd[k] = ctx->min_psd_sub[k];
        }
    }

    /* Soft VAD + noise update */
    for (int k = 0; k < NS2_FREQ_BINS; k++) {
        /* Speech presence probability (simplified) */
        float sr = power_spec[k] / (MCRA_BETA * ctx->min_psd[k] + 1e-20f);
        float p_speech = (sr > 1.5f) ? 1.0f : 0.0f;  /* hard threshold */

        /* Update noise PSD only when speech is likely absent */
        if (p_speech < 0.5f) {
            ctx->noise_psd[k] = MCRA_ALPHA_D * ctx->noise_psd[k]
                                 + (1.0f - MCRA_ALPHA_D) * power_spec[k];
        }
        /* Guard against zero noise */
        if (ctx->noise_psd[k] < 1e-20f) ctx->noise_psd[k] = 1e-20f;
    }
}

/* ── Wiener gain computation ────────────────────────────────────────────── */

static void compute_wiener_gains(ns2_ctx_t *ctx,
                                  const float *re, const float *im,
                                  float *gains)
{
    const float dd_alpha = 0.98f;  /* decision-directed smoothing */
    float aggressiveness = (ctx->mode == NS2_MODE_STRONG) ? 1.5f
                         : (ctx->mode == NS2_MODE_VOICE)  ? 1.2f : 1.0f;

    for (int k = 0; k < NS2_FREQ_BINS; k++) {
        float power = re[k]*re[k] + im[k]*im[k];
        float noise = ctx->noise_psd[k] * aggressiveness;

        /* A posteriori SNR */
        float snr_post = power / noise;

        /* Decision-directed a priori SNR */
        float snr_prior = dd_alpha * (ctx->prev_clean_psd[k] / noise)
                          + (1.0f - dd_alpha) * fmaxf(0.0f, snr_post - 1.0f);
        ctx->snr_prior[k] = snr_prior;

        /* Wiener gain: G = snr_prior / (1 + snr_prior) */
        float g = snr_prior / (1.0f + snr_prior);

        /* Apply floor */
        if (g < ctx->attenuation_floor) g = ctx->attenuation_floor;
        gains[k] = g;

        /* Update previous clean PSD estimate */
        ctx->prev_clean_psd[k] = g * g * power;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int ns2_init(ns2_ctx_t *ctx, uint32_t sample_rate, ns2_mode_t mode)
{
    if (!ctx) return -1;
    memset(ctx, 0, sizeof(*ctx));

    ctx->sample_rate = sample_rate;
    ctx->mode = mode;
    ctx->attenuation_floor = (mode == NS2_MODE_STRONG) ? 0.02f : 0.05f;

    /* Hann window */
    for (int i = 0; i < NS2_FRAME_SIZE; i++)
        ctx->window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i
                                               / (float)(NS2_FRAME_SIZE - 1)));

    /* Bootstrap noise PSD to a small non-zero value */
    for (int k = 0; k < NS2_FREQ_BINS; k++) {
        ctx->noise_psd[k]    = 1e-6f;
        ctx->min_psd[k]      = 1e-6f;
        ctx->min_psd_sub[k]  = 1e-6f;
        ctx->prev_clean_psd[k] = 1e-6f;
        ctx->snr_prior[k]    = 1.0f;
    }

    ctx->initialized = true;
    return 0;
}

void ns2_noise_learn(ns2_ctx_t *ctx, const float *noise_pcm, int n_samples)
{
    if (!ctx || !ctx->initialized || !noise_pcm || n_samples < NS2_FRAME_SIZE) return;

    float frame[NS2_FRAME_SIZE];
    float re[NS2_FREQ_BINS], im[NS2_FREQ_BINS];

    for (int offset = 0; offset + NS2_FRAME_SIZE <= n_samples;
         offset += NS2_HOP_SIZE) {
        for (int i = 0; i < NS2_FRAME_SIZE; i++)
            frame[i] = noise_pcm[offset + i] * ctx->window[i];
        rdft256(frame, re, im);

        float power[NS2_FREQ_BINS];
        for (int k = 0; k < NS2_FREQ_BINS; k++)
            power[k] = re[k]*re[k] + im[k]*im[k];

        /* Directly set noise PSD from this segment */
        for (int k = 0; k < NS2_FREQ_BINS; k++) {
            ctx->noise_psd[k] = 0.9f * ctx->noise_psd[k] + 0.1f * power[k];
            ctx->min_psd[k]     = ctx->noise_psd[k];
            ctx->min_psd_sub[k] = ctx->noise_psd[k];
        }
        ctx->init_count++;
    }
}

void ns2_process(ns2_ctx_t *ctx, const float *in, float *out)
{
    if (!ctx || !ctx->initialized || !in || !out) return;

    /* frame_buf lives in ctx — safe for multi-instance use */
    float re[NS2_FREQ_BINS], im[NS2_FREQ_BINS];
    float gains[NS2_FREQ_BINS];
    float power[NS2_FREQ_BINS];
    float windowed[NS2_FRAME_SIZE];
    float synth[NS2_FRAME_SIZE];

    /* Shift buffer: move second half to first half, fill second half */
    memcpy(ctx->frame_buf, ctx->frame_buf + NS2_HOP_SIZE,
           NS2_HOP_SIZE * sizeof(float));
    memcpy(ctx->frame_buf + NS2_HOP_SIZE, in,
           NS2_HOP_SIZE * sizeof(float));

    /* Apply Hann window */
    for (int i = 0; i < NS2_FRAME_SIZE; i++)
        windowed[i] = ctx->frame_buf[i] * ctx->window[i];

    /* Analysis DFT */
    rdft256(windowed, re, im);

    /* Compute power spectrum */
    for (int k = 0; k < NS2_FREQ_BINS; k++)
        power[k] = re[k]*re[k] + im[k]*im[k];

    /* Update noise estimate (skip during bootstrap) */
    if (ctx->init_count < NS2_NOISE_INIT_FRAMES) {
        /* Bootstrap: assume silence, set noise from current spectrum */
        for (int k = 0; k < NS2_FREQ_BINS; k++) {
            ctx->noise_psd[k]    = 0.95f * ctx->noise_psd[k] + 0.05f * power[k];
            ctx->min_psd[k]      = ctx->noise_psd[k];
            ctx->min_psd_sub[k]  = ctx->noise_psd[k];
        }
        ctx->init_count++;
        /* Pass through during bootstrap */
        memcpy(out, in, NS2_HOP_SIZE * sizeof(float));
        return;
    }

    mcra_update(ctx, power);
    compute_wiener_gains(ctx, re, im, gains);

    /* Apply gains to spectrum */
    for (int k = 0; k < NS2_FREQ_BINS; k++) {
        re[k] *= gains[k];
        im[k] *= gains[k];
    }

    /* Synthesis IDFT */
    irdft256(re, im, synth);

    /* Overlap-add: output second half of previous + first half of new */
    for (int i = 0; i < NS2_HOP_SIZE; i++) {
        float s = ctx->overlap_buf[i] + synth[i] * ctx->window[i];
        if (s >  1.0f) s =  1.0f;
        if (s < -1.0f) s = -1.0f;
        out[i] = s;
    }
    /* Save second half for next overlap */
    for (int i = 0; i < NS2_HOP_SIZE; i++)
        ctx->overlap_buf[i] = synth[NS2_HOP_SIZE + i] * ctx->window[NS2_HOP_SIZE + i];
}

float ns2_get_noise_db(const ns2_ctx_t *ctx)
{
    if (!ctx || !ctx->initialized) return -96.0f;
    /* Average noise power across speech-relevant bins (2–32, ~125Hz–2kHz) */
    float sum = 0.0f;
    for (int k = 2; k < 32; k++)
        sum += ctx->noise_psd[k];
    float avg = sum / 30.0f;
    return 10.0f * log10f(avg + 1e-20f);
}
