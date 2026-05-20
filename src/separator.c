/*
 * opcodec/separator.c — Real-time Speaker Source Separation
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/separator.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Minimal DFT (real-input, output magnitude + phase) ─────────────────── */
/* We use a direct DFT for the STFT window size. For N=512 this is fast
 * enough for audio frame rates (< 2 ms on modern CPU). A proper FFT
 * could replace this for embedded targets. */

static void rdft(const float *in, int N, float *mag, float *phase)
{
    int half = N / 2 + 1;
    for (int k = 0; k < half; k++) {
        float re = 0.0f, im = 0.0f;
        float fk = 2.0f * (float)M_PI * (float)k / (float)N;
        for (int n = 0; n < N; n++) {
            re += in[n] * cosf(fk * (float)n);
            im -= in[n] * sinf(fk * (float)n);
        }
        mag[k]   = sqrtf(re*re + im*im);
        phase[k] = atan2f(im, re);
    }
}

static void irdft(const float *mag, const float *phase, int N, float *out)
{
    int half = N / 2 + 1;
    memset(out, 0, (size_t)N * sizeof(float));
    for (int n = 0; n < N; n++) {
        float fk = 2.0f * (float)M_PI * (float)n / (float)N;
        float v  = mag[0] * cosf(phase[0]);
        for (int k = 1; k < half - 1; k++) {
            float c = cosf(fk * (float)k + phase[k]);
            v += 2.0f * mag[k] * c;  /* conjugate symmetry */
        }
        /* Nyquist bin (no conjugate) */
        v += mag[half-1] * cosf((float)(half-1) * fk + phase[half-1]);
        out[n] = v / (float)N;
    }
}

/* ── Spectral profile extraction ──────────────────────────────────────────── */

static void compute_profile(const float *mag, float *profile)
{
    /* Map SEPARATOR_FREQ_BINS frequency bins to SEPARATOR_PROFILE_BINS
     * using logarithmic spacing (Mel-like) */
    int bins   = SEPARATOR_FREQ_BINS;
    int pbin   = SEPARATOR_PROFILE_BINS;
    float log_min = logf(1.0f);
    float log_max = logf((float)bins);

    for (int p = 0; p < pbin; p++) {
        float flo = expf(log_min + (float)p       * (log_max - log_min) / (float)pbin);
        float fhi = expf(log_min + (float)(p + 1) * (log_max - log_min) / (float)pbin);
        int b0 = (int)flo, b1 = (int)fhi;
        if (b0 >= bins) b0 = bins - 1;
        if (b1 >= bins) b1 = bins - 1;
        if (b1 < b0)    b1 = b0;
        float sum = 0.0f;
        int cnt = 0;
        for (int b = b0; b <= b1; b++) { sum += mag[b]; cnt++; }
        profile[p] = (cnt > 0) ? (sum / (float)cnt) : 0.0f;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int sep_init(sep_ctx_t *ctx, uint32_t sample_rate)
{
    if (!ctx) return -1;
    memset(ctx, 0, sizeof(*ctx));
    ctx->sample_rate = sample_rate;
    /* Hann window */
    for (int i = 0; i < SEPARATOR_FFT_SIZE; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i
                                       / (float)(SEPARATOR_FFT_SIZE - 1)));
        ctx->window[i] = w;
    }
    ctx->initialized = true;
    return 0;
}

void sep_free(sep_ctx_t *ctx)
{
    if (!ctx) return;
    ctx->initialized = false;
}

int sep_enroll_speaker(sep_ctx_t *ctx, int speaker_id,
                       const float *pcm, int n_samples)
{
    if (!ctx || !ctx->initialized) return -1;
    if (speaker_id < 0 || speaker_id >= SEPARATOR_MAX_SPEAKERS) return -1;
    if (!pcm || n_samples < SEPARATOR_FFT_SIZE) return -1;

    sep_speaker_t *sp = &ctx->speakers[speaker_id];
    memset(sp->profile, 0, sizeof(sp->profile));

    float frame[SEPARATOR_FFT_SIZE];
    float mag[SEPARATOR_FREQ_BINS];
    float ph[SEPARATOR_FREQ_BINS];
    float profile_acc[SEPARATOR_PROFILE_BINS] = {0};
    int   n_frames = 0;

    for (int offset = 0;
         offset + SEPARATOR_FFT_SIZE <= n_samples;
         offset += SEPARATOR_HOP_SIZE) {
        /* Apply Hann window */
        for (int i = 0; i < SEPARATOR_FFT_SIZE; i++)
            frame[i] = pcm[offset + i] * ctx->window[i];
        rdft(frame, SEPARATOR_FFT_SIZE, mag, ph);
        float local_profile[SEPARATOR_PROFILE_BINS];
        compute_profile(mag, local_profile);
        for (int p = 0; p < SEPARATOR_PROFILE_BINS; p++)
            profile_acc[p] += local_profile[p];
        n_frames++;
    }

    if (n_frames < 1) return -1;
    float total = 0.0f;
    for (int p = 0; p < SEPARATOR_PROFILE_BINS; p++) {
        sp->profile[p] = profile_acc[p] / (float)n_frames;
        total += sp->profile[p];
    }
    /* L1-normalize profile */
    if (total > 1e-8f)
        for (int p = 0; p < SEPARATOR_PROFILE_BINS; p++)
            sp->profile[p] /= total;

    sp->enrolled = true;
    sp->frames_seen = 0;
    if (speaker_id >= ctx->n_speakers)
        ctx->n_speakers = speaker_id + 1;
    return 0;
}

int sep_process(sep_ctx_t *ctx,
                const float *mixture, float **outputs, int n_out)
{
    if (!ctx || !ctx->initialized || !mixture) return -1;
    if (n_out != ctx->n_speakers) return -1;

    float frame[SEPARATOR_FFT_SIZE];
    float mix_mag[SEPARATOR_FREQ_BINS];
    float mix_ph[SEPARATOR_FREQ_BINS];
    float synth[SEPARATOR_MAX_SPEAKERS][SEPARATOR_FFT_SIZE];

    /* ── STFT of mixture ── */
    for (int i = 0; i < SEPARATOR_FFT_SIZE; i++)
        frame[i] = (i < SEPARATOR_FRAME_SIZE)
                   ? mixture[i] * ctx->window[i]
                   : 0.0f;
    rdft(frame, SEPARATOR_FFT_SIZE, mix_mag, mix_ph);
    memcpy(ctx->mixture_mag,   mix_mag, sizeof(mix_mag));
    memcpy(ctx->mixture_phase, mix_ph,  sizeof(mix_ph));

    /* ── Compute total profile of the mixture frame ── */
    float mix_profile[SEPARATOR_PROFILE_BINS];
    compute_profile(mix_mag, mix_profile);
    float mix_total = 0.0f;
    for (int p = 0; p < SEPARATOR_PROFILE_BINS; p++)
        mix_total += mix_profile[p];
    if (mix_total < 1e-8f) mix_total = 1e-8f;
    for (int p = 0; p < SEPARATOR_PROFILE_BINS; p++)
        mix_profile[p] /= mix_total;

    /* ── Compute per-speaker magnitude masks via spectral similarity ── */
    float total_weight[SEPARATOR_FREQ_BINS];
    memset(total_weight, 0, sizeof(total_weight));

    float speaker_gains[SEPARATOR_MAX_SPEAKERS];
    for (int k = 0; k < n_out; k++) {
        if (!ctx->speakers[k].enrolled) {
            speaker_gains[k] = 0.0f;
            continue;
        }
        /* Profile similarity: dot product between mixture profile and speaker
         * profile (both L1-normalized → bounded cosine approximation) */
        float dot = 0.0f;
        for (int p = 0; p < SEPARATOR_PROFILE_BINS; p++)
            dot += mix_profile[p] * ctx->speakers[k].profile[p];
        speaker_gains[k] = dot;
    }

    /* Normalise so gains sum to 1 */
    float sum_gain = 1e-8f;
    for (int k = 0; k < n_out; k++)
        sum_gain += speaker_gains[k];
    for (int k = 0; k < n_out; k++)
        speaker_gains[k] /= sum_gain;

    /* Build per-bin masks and reconstruct each speaker */
    for (int k = 0; k < n_out; k++) {
        float g = speaker_gains[k];
        float spk_mag[SEPARATOR_FREQ_BINS];
        for (int b = 0; b < SEPARATOR_FREQ_BINS; b++) {
            float mask = g; /* uniform mask weighted by speaker similarity */
            /* EWMA smoothing to reduce musical noise */
            ctx->speakers[k].mask_smooth[b] =
                0.7f * ctx->speakers[k].mask_smooth[b] + 0.3f * mask;
            spk_mag[b] = mix_mag[b] * ctx->speakers[k].mask_smooth[b];
            total_weight[b] += spk_mag[b];
        }
        irdft(spk_mag, mix_ph, SEPARATOR_FFT_SIZE, synth[k]);
    }

    /* ── Overlap-add to output buffers ── */
    for (int k = 0; k < n_out; k++) {
        if (!outputs[k]) continue;
        for (int i = 0; i < SEPARATOR_FRAME_SIZE; i++) {
            int hop_i = i % SEPARATOR_HOP_SIZE;
            float ola = (i < SEPARATOR_HOP_SIZE)
                        ? ctx->overlap_buf[k][hop_i] : 0.0f;
            float s = synth[k][i] + ola;
            if (s >  1.0f) s =  1.0f;
            if (s < -1.0f) s = -1.0f;
            outputs[k][i] = s;
        }
        /* Update overlap buffer for next hop */
        for (int i = 0; i < SEPARATOR_HOP_SIZE; i++)
            ctx->overlap_buf[k][i] = synth[k][SEPARATOR_FRAME_SIZE + i];
    }
    return 0;
}

void sep_reset_speaker(sep_ctx_t *ctx, int speaker_id)
{
    if (!ctx || speaker_id < 0 || speaker_id >= SEPARATOR_MAX_SPEAKERS) return;
    memset(&ctx->speakers[speaker_id], 0, sizeof(sep_speaker_t));
}
