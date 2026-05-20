/*
 * opcodec/epsc.h — Emotional Prosody Side Channel
 *
 * Transmits a compact prosody descriptor alongside the main audio codec
 * to preserve emotional nuance at low bitrates. At 6–24 kbps, conventional
 * codecs destroy pitch variation, energy dynamics, and speaking rate — the
 * cues that convey urgency, humor, and emotion. EPSC transmits these
 * features at ~200 bps as a side channel; the decoder resynthesizes them.
 *
 * Features encoded per 20 ms frame:
 *   F0 (pitch): 7 bits  — MIDI-style semitone index 0–127 (0 = unvoiced)
 *   Energy:     5 bits  — log-energy in 3 dB steps over 96 dB range
 *   Voiced:     1 bit   — voiced/unvoiced flag
 *   Rate:       3 bits  — local speaking rate (syllables/sec quantized)
 *   Reserved:   0 bits
 * Total: 16 bits (2 bytes) per 20 ms frame = 800 bps.
 *
 * At 40 ms frames: 1 byte/frame = 200 bps — below the noise floor.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_EPSC_H
#define OPCODEC_EPSC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define EPSC_PACKET_BYTES   2    /* bytes per 20 ms frame */
#define EPSC_HISTORY_FRAMES 8   /* frames kept for rate estimation */
#define EPSC_F0_UNVOICED    0   /* F0 code for unvoiced frame */
#define EPSC_F0_MAX         127 /* maximum F0 code */

/* Per-frame prosody descriptor */
typedef struct {
    uint8_t f0_code;    /* 7-bit pitch code (0 = unvoiced, 1–127 = MIDI note) */
    uint8_t energy_db;  /* 5-bit log-energy code (0 = silence, 31 = 93 dB SPL) */
    bool    voiced;     /* voiced/unvoiced flag */
    uint8_t rate_code;  /* 3-bit speaking rate (0 = slow, 7 = fast) */
} epsc_frame_t;

/* EPSC encoder/decoder context */
typedef struct {
    /* History for speaking-rate estimation (voiced-onset ring buffer) */
    float   onset_times[EPSC_HISTORY_FRAMES]; /* times of recent voiced onsets */
    int     onset_head;
    int     onset_count;

    /* Previous frame state for delta coding */
    uint8_t prev_f0;
    float   prev_energy;
    bool    prev_voiced;

    /* Sample rate for Hz ↔ sample conversion */
    uint32_t sample_rate;
} epsc_ctx_t;

/*
 * Initialize EPSC context.
 * sample_rate: audio sample rate (e.g., 48000).
 */
void epsc_init(epsc_ctx_t *ctx, uint32_t sample_rate);

/*
 * Extract prosody features from a PCM float frame.
 *
 * pcm:        floating-point audio samples [-1.0, 1.0]
 * frame_size: number of samples
 * pitch_hz:   pitch frequency in Hz from pitch detector (0.0 = unvoiced)
 *
 * Returns the prosody descriptor for this frame.
 */
epsc_frame_t epsc_extract(epsc_ctx_t *ctx,
                           const float *pcm, int frame_size,
                           float pitch_hz);

/*
 * Encode one prosody frame to 2 bytes.
 * Returns number of bytes written (always EPSC_PACKET_BYTES).
 */
int epsc_encode(const epsc_frame_t *f, uint8_t *out);

/*
 * Decode 2 bytes into a prosody frame.
 * Returns number of bytes consumed (always EPSC_PACKET_BYTES).
 */
int epsc_decode(const uint8_t *in, epsc_frame_t *f);

/*
 * Apply prosody to decoded PCM samples.
 *
 * Adjusts pitch (via PSOLA pitch shift) and energy to match the
 * transmitted prosody descriptor. Operates in-place.
 *
 * pcm:         float samples to modify (in-place)
 * frame_size:  number of samples
 * target:      desired prosody (from epsc_decode)
 * actual:      prosody the codec actually produced (from epsc_extract
 *              on the decoded frame — NULL = skip energy normalization)
 *
 * Note: pitch resynthesis is a lightweight pitch-shift via resampling;
 * it is NOT a full vocoder. Quality is limited but fast (< 0.5 ms/frame).
 */
void epsc_apply(float *pcm, int frame_size,
                const epsc_frame_t *target, const epsc_frame_t *actual,
                uint32_t sample_rate);

/*
 * Convert F0 code to frequency in Hz.
 * code 0 → 0.0 Hz (unvoiced)
 * code 1 → ~55 Hz (MIDI note 33: A1)
 * code 127 → ~3136 Hz (MIDI note 127: G9)
 */
float epsc_f0_to_hz(uint8_t code);

/*
 * Convert pitch frequency (Hz) to 7-bit F0 code.
 * Returns EPSC_F0_UNVOICED for f0_hz <= 0 or > 3500.
 */
uint8_t epsc_hz_to_f0(float f0_hz);

#endif /* OPCODEC_EPSC_H */
