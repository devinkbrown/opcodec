/*
 * opcodec/ns2.h — Noise Suppression v2 (Wiener Filter Spectral Subtraction)
 *
 * Removes stationary and quasi-stationary background noise from speech frames
 * without the musical noise artifacts of naive spectral subtraction.
 *
 * Algorithm (Ephraim–Malah MMSE-STSA variant):
 *   1. STFT of input frame (256-pt real DFT, 50% overlap).
 *   2. Noise power spectral density (PSD) estimate: MCRA (Minimum Controlled
 *      Recursive Averaging) — updates noise estimate only in frames flagged
 *      as speech-absent by a soft VAD.
 *   3. A priori SNR estimate: decision-directed approach with α=0.98 smoothing
 *      (avoids over-attenuation during speech onsets).
 *   4. Wiener gain: G[k] = SNR[k] / (1 + SNR[k]).
 *   5. Apply gain to spectrum, ISTFT, overlap-add.
 *
 * The Ephraim–Malah decision-directed estimator is long-established prior art
 * (1984); our MCRA implementation follows Cohen & Berdugo (2001) which is also
 * in the public domain / academic literature.
 *
 * Properties:
 *   - Latency: one frame (256 samples / 16 ms at 16 kHz)
 *   - Works at 8, 16, 32, 48 kHz (internally down-converts to 16 kHz for SNR
 *     estimation, applies gain at native rate)
 *   - No musical noise thanks to decision-directed a priori SNR
 *   - ~3–15 dB NR depending on stationarity of noise
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_NS2_H
#define OPCODEC_NS2_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define NS2_FRAME_SIZE    256   /* DFT size / analysis window */
#define NS2_HOP_SIZE      128   /* 50% overlap */
#define NS2_FREQ_BINS     (NS2_FRAME_SIZE / 2 + 1)  /* 129 real bins */
#define NS2_NOISE_INIT_FRAMES 20  /* frames used to bootstrap noise estimate */

/* Operating modes */
typedef enum {
    NS2_MODE_MODERATE = 0,   /* moderate suppression (~6 dB NR, low artefacts) */
    NS2_MODE_STRONG   = 1,   /* strong suppression (~12 dB NR, slight residual) */
    NS2_MODE_VOICE    = 2,   /* voice-optimized: preserves formants, heavy BG NR */
} ns2_mode_t;

/* NS2 context */
typedef struct {
    /* Noise PSD estimate (MCRA) */
    float noise_psd[NS2_FREQ_BINS];     /* smoothed noise power per bin */
    float min_psd[NS2_FREQ_BINS];       /* running minimum for MCRA */
    float min_psd_sub[NS2_FREQ_BINS];   /* sub-window minimum */
    int   min_sub_count;                /* frames since last sub-window reset */

    /* A priori SNR (decision-directed) */
    float snr_prior[NS2_FREQ_BINS];     /* ξ[k]: smoothed a priori SNR */
    float prev_clean_psd[NS2_FREQ_BINS];/* previous estimated clean PSD */

    /* OLA state */
    float overlap_buf[NS2_HOP_SIZE];    /* overlap-add buffer */
    float window[NS2_FRAME_SIZE];       /* Hann analysis window */
    float frame_buf[NS2_FRAME_SIZE];    /* analysis ring (moved from static in ns2_process) */

    /* Status */
    int   init_count;       /* frames seen (ramps up to INIT_FRAMES) */
    ns2_mode_t mode;
    float attenuation_floor; /* minimum gain floor (0.05 = -26 dB max attenuation) */

    uint32_t sample_rate;
    bool initialized;
} ns2_ctx_t;

/* ── API ── */

/*
 * Initialize noise suppressor.
 * sample_rate: 8000, 16000, 32000, or 48000
 * mode:        suppression aggressiveness
 */
int ns2_init(ns2_ctx_t *ctx, uint32_t sample_rate, ns2_mode_t mode);

/*
 * Process one block of NS2_HOP_SIZE float PCM samples (mono, normalized [-1,1]).
 * in:  NS2_HOP_SIZE input samples
 * out: NS2_HOP_SIZE output samples (denoised)
 *
 * Internally accumulates a full NS2_FRAME_SIZE window using overlap-add,
 * applies Wiener gain, and emits the denoised output at the same hop size.
 * Latency: one NS2_FRAME_SIZE block = 256 samples (16 ms at 16 kHz).
 */
void ns2_process(ns2_ctx_t *ctx, const float *in, float *out);

/*
 * Feed a noise-only segment to accelerate noise PSD estimation.
 * Call this during the first few frames when you know the speaker is silent.
 * n_samples: should be >= NS2_FRAME_SIZE.
 */
void ns2_noise_learn(ns2_ctx_t *ctx, const float *noise_pcm, int n_samples);

/*
 * Get current estimated noise level in dBFS (useful for UI noise indicator).
 */
float ns2_get_noise_db(const ns2_ctx_t *ctx);

#endif /* OPCODEC_NS2_H */
