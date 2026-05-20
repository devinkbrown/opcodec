/*
 * opcodec/separator.h — Real-time Speaker Source Separation
 *
 * Separates a monaural mixture of up to SEPARATOR_MAX_SPEAKERS voices into
 * individual speaker streams using magnitude-ratio masking in the STFT domain.
 *
 * Algorithm (lightweight Conv-TasNet-inspired, CPU-friendly):
 *   1. Compute short-time magnitude spectrum of the mixture (N-point STFT).
 *   2. For each speaker, estimate a soft magnitude mask M_k[t,f] ∈ [0,1]
 *      using a running speaker energy profile updated from previously clean
 *      segments (VAD-gated learning).
 *   3. Apply: Y_k[t,f] = M_k[t,f] × X[t,f]  (mask the mixture spectrum).
 *   4. Reconstruct waveform via overlap-add with the original mixture phase.
 *
 * Speaker identification:
 *   Each speaker must be enrolled (sep_enroll_speaker) from a short (≥ 1s)
 *   segment of clean audio. The profile is a per-band RMS energy histogram
 *   that captures spectral shape (a coarse voice fingerprint).
 *
 * Limitations:
 *   - Mono input only; stereo awareness planned.
 *   - Up to SEPARATOR_MAX_SPEAKERS simultaneous speakers.
 *   - Quality degrades for > 3 simultaneous speakers.
 *   - Not suitable for music separation (tuned for voice 100–8000 Hz).
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_SEPARATOR_H
#define OPCODEC_SEPARATOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SEPARATOR_MAX_SPEAKERS  4
#define SEPARATOR_FFT_SIZE    512      /* STFT window (10.67 ms at 48 kHz) */
#define SEPARATOR_HOP_SIZE    128      /* overlap-add hop (2.67 ms) */
#define SEPARATOR_FREQ_BINS   (SEPARATOR_FFT_SIZE / 2 + 1)
#define SEPARATOR_PROFILE_BINS 32      /* spectral profile resolution */
#define SEPARATOR_FRAME_SIZE   512     /* processing block size in samples */

/* Per-speaker model */
typedef struct {
    float    profile[SEPARATOR_PROFILE_BINS]; /* spectral shape signature */
    float    mask_smooth[SEPARATOR_FREQ_BINS];/* smoothed mask (EWMA) */
    float    energy_avg;                      /* average energy (for VAD) */
    bool     enrolled;
    uint32_t frames_seen;
} sep_speaker_t;

/* Separator context */
typedef struct {
    sep_speaker_t speakers[SEPARATOR_MAX_SPEAKERS];
    int           n_speakers;

    /* STFT workspace */
    float   window[SEPARATOR_FFT_SIZE];         /* analysis window (Hann) */
    float   overlap_buf[SEPARATOR_MAX_SPEAKERS][SEPARATOR_HOP_SIZE]; /* OLA state */
    float   mixture_mag[SEPARATOR_FREQ_BINS];
    float   mixture_phase[SEPARATOR_FREQ_BINS];

    /* STFT frame buffer */
    float   frame_buf[SEPARATOR_FFT_SIZE];
    int     buf_fill;                            /* samples in frame_buf */

    uint32_t sample_rate;
    bool     initialized;
} sep_ctx_t;

/*
 * Initialize separator context.
 * sample_rate: e.g. 48000.
 * Returns 0 on success, -1 on error.
 */
int sep_init(sep_ctx_t *ctx, uint32_t sample_rate);

/*
 * Free resources (currently a no-op since all storage is in-struct,
 * provided for API symmetry and future heap allocations).
 */
void sep_free(sep_ctx_t *ctx);

/*
 * Enroll a speaker from clean audio.
 * speaker_id: 0–(SEPARATOR_MAX_SPEAKERS-1)
 * pcm:        clean float samples for this speaker
 * n_samples:  should be >= sample_rate (at least 1 second)
 * Returns 0 on success, -1 on error (bad id, n_samples too small, etc.)
 */
int sep_enroll_speaker(sep_ctx_t *ctx, int speaker_id,
                       const float *pcm, int n_samples);

/*
 * Process one block of mixed audio and separate into per-speaker streams.
 *
 * mixture:     input mono PCM, exactly SEPARATOR_FRAME_SIZE samples
 * outputs:     array of n_speakers output buffers, each SEPARATOR_FRAME_SIZE
 *              samples. Caller allocates. Set to NULL to skip a speaker.
 * n_out:       must equal ctx->n_speakers
 *
 * Returns 0 on success, -1 on error.
 */
int sep_process(sep_ctx_t *ctx,
                const float *mixture,
                float **outputs, int n_out);

/*
 * Get number of enrolled speakers.
 */
static inline int sep_speaker_count(const sep_ctx_t *ctx) {
    return ctx->n_speakers;
}

/*
 * Reset a speaker's model (e.g., speaker left the channel).
 */
void sep_reset_speaker(sep_ctx_t *ctx, int speaker_id);

#endif /* OPCODEC_SEPARATOR_H */
