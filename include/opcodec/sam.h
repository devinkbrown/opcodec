/*
 * opcodec/sam.h — Semantic Audio Mode: ultra-low-bitrate parametric vocoder
 *
 * Encodes speech as a compact set of parametric descriptors and resynthesizes
 * it at the decoder using an LPC synthesis filter. Target bitrate: ~1200 bps
 * (compared to Opus minimum speech quality at ~6000 bps).
 *
 * Frame size: 40 ms (gives 25 frames/second).
 * Bits per frame:
 *   LSF coefficients (10 LSFs, VQ codebook index): 12 bits
 *   Pitch period (7 bits)                         :  7 bits
 *   Voiced/unvoiced flag                          :  1 bit
 *   Log-energy (6 bits, 1.5 dB steps, 96 dB range):  6 bits
 *   Frame-type flags (2 bits)                     :  2 bits
 *   Total                                         : 28 bits = 3.5 bytes/frame
 *   At 25 frames/sec: 700 bps                     ✓
 *
 * Line Spectral Frequency (LSF) coding:
 *   10th-order LPC → 10 LSFs → VQ against 4096-entry codebook (12 bits).
 *   Codebook is fixed (trained offline on read-aloud speech).
 *
 * Synthesis:
 *   Voiced: pulse-train excitation at pitch period, filtered through LPC.
 *   Unvoiced: spectrally shaped noise, filtered through LPC.
 *   Post-filter: 1/(1-0.4*A(z)) for voice naturalness.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_SAM_H
#define OPCODEC_SAM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SAM_ORDER          10       /* LPC order */
#define SAM_FRAME_MS       40       /* frame duration in ms */
#define SAM_MAX_SAMPLE_RATE 48000
#define SAM_MAX_FRAME_SIZE  (SAM_MAX_SAMPLE_RATE * SAM_FRAME_MS / 1000) /* 1920 */
#define SAM_PACKET_BITS    28       /* bits per 40 ms frame */
#define SAM_PACKET_BYTES    4       /* ceil(28/8) */
#define SAM_LSF_CODEBOOK_SIZE 4096  /* VQ codebook entries */
#define SAM_PITCH_MIN      20       /* min pitch period in samples (at 8 kHz) */
#define SAM_PITCH_MAX     320       /* max pitch period in samples (at 8 kHz) */

/* Encoded SAM frame (4 bytes) */
typedef struct {
    uint16_t lsf_code;  /* 12-bit VQ codebook index */
    uint8_t  pitch_7;   /* 7-bit quantized pitch period */
    uint8_t  voiced  : 1;
    uint8_t  energy_6: 6;
    uint8_t  flags   : 2; /* 0=normal, 1=onset, 2=silence, 3=rsvd */
} sam_frame_t;

/* SAM encoder context */
typedef struct {
    float    lpc[SAM_ORDER + 1];  /* current LPC coefficients */
    float    lsf[SAM_ORDER];      /* current LSFs */
    float    preemph_prev;        /* pre-emphasis filter state */
    int      pitch_period;        /* current pitch period (in samples at sample_rate) */
    float    energy;              /* current frame energy */
    uint32_t sample_rate;
    int      frame_size;          /* samples per 40 ms frame */
    bool     initialized;
    /* Excitation state for synthesis check */
    float    synth_state[SAM_ORDER]; /* LPC synthesis memory */
    float    exc_phase;              /* pulse train phase accumulator */
    uint32_t rng;                    /* noise generator state */
} sam_enc_t;

/* SAM decoder context */
typedef struct {
    float    lpc[SAM_ORDER + 1];  /* decoded LPC coefficients */
    float    synth_state[SAM_ORDER];
    float    postfilt_state[SAM_ORDER];
    float    exc_phase;
    uint32_t rng;
    uint32_t sample_rate;
    int      frame_size;
    bool     initialized;
    /* Smooth transitions */
    float    prev_lpc[SAM_ORDER + 1];
    bool     has_prev;
} sam_dec_t;

/* ── API ── */

/*
 * Initialize SAM encoder.
 * sample_rate: 8000, 16000, or 24000 (SAM targets narrowband/wideband voice).
 *   48000 is accepted but internally downsampled to 16000 for LPC analysis.
 */
int sam_enc_init(sam_enc_t *enc, uint32_t sample_rate);

/*
 * Encode one 40 ms frame of float PCM to a 4-byte SAM packet.
 * pcm:        frame_size float samples (frame_size = sample_rate * 40 / 1000).
 * out:        4-byte output buffer.
 * Returns bytes written (always SAM_PACKET_BYTES).
 */
int sam_encode(sam_enc_t *enc, const float *pcm, int frame_size, uint8_t *out);

/*
 * Initialize SAM decoder.
 */
int sam_dec_init(sam_dec_t *dec, uint32_t sample_rate);

/*
 * Decode one 4-byte SAM packet to float PCM.
 * out:        float buffer, at least frame_size samples.
 * frame_size: sample_rate * 40 / 1000.
 * Returns number of samples written, or -1 on error.
 */
int sam_decode(sam_dec_t *dec, const uint8_t *in, float *out, int frame_size);

/*
 * Serialize/deserialize sam_frame_t to/from 4 bytes.
 */
void sam_frame_write(const sam_frame_t *f, uint8_t *out);
void sam_frame_read(sam_frame_t *f, const uint8_t *in);

#endif /* OPCODEC_SAM_H */
