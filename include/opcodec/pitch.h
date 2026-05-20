#ifndef OPCODEC_PITCH_H
#define OPCODEC_PITCH_H

/*
 * OPCODEC Pitch Detection and Post-Filter
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 *
 * High-quality pitch detection using YIN algorithm and time-domain
 * comb filtering for enhanced voice quality at low bitrates.
 */

#include <stdint.h>
#include <stddef.h>

/* Pitch period range (in samples at the codec's sample rate) */
#define PITCH_MIN_PERIOD   20    /* ~2400 Hz at 48kHz, ~400 Hz at 8kHz */
#define PITCH_MAX_PERIOD   480   /* ~100 Hz at 48kHz, ~16.7 Hz at 8kHz */
#define PITCH_MAX_FRAME    960   /* max frame size (48kHz * 20ms) */

/* Pitch detector state */
typedef struct {
    float    prev_frame[PITCH_MAX_PERIOD]; /* overlap from previous frame for correlation */
    int      last_period;                   /* pitch period of previous frame (for smoothing) */
    float    last_gain;                     /* pitch correlation of previous frame */
    uint32_t sample_rate;
} pitch_detector_t;

/* Pitch post-filter state (applied after MDCT synthesis on decoder side,
 * and as pre-filter before MDCT analysis on encoder side) */
typedef struct {
    float    buf[PITCH_MAX_PERIOD + PITCH_MAX_FRAME]; /* delay line */
    int      period;                                    /* current pitch period */
    float    gain;                                      /* comb filter gain [0..1) */
} pitch_filter_t;

/* Pitch analysis result */
typedef struct {
    int      period;      /* detected pitch period in samples (0 = unvoiced) */
    float    correlation; /* normalized correlation [0..1], strength of pitch */
    float    gain;        /* optimal comb filter gain for this frame */
} pitch_info_t;

/* Initialize pitch detector */
void pitch_detect_init(pitch_detector_t *pd, uint32_t sample_rate);

/* Detect pitch in a frame of samples.
 * Returns pitch info with period, correlation, and optimal gain.
 * Period=0 means unvoiced (no pitch detected). */
pitch_info_t pitch_detect(pitch_detector_t *pd,
                          const float *samples, int frame_size);

/* Initialize pitch post-filter */
void pitch_filter_init(pitch_filter_t *pf);

/* Apply pitch pre-filter (encoder side, before MDCT).
 * Removes pitch periodicity so MDCT doesn't have to code it.
 * Modifies samples in-place. Transmit period and gain as side info. */
void pitch_prefilter(pitch_filter_t *pf,
                     float *samples, int frame_size,
                     int period, float gain);

/* Apply pitch post-filter (decoder side, after MDCT synthesis).
 * Re-introduces pitch periodicity using transmitted period and gain.
 * Modifies samples in-place. */
void pitch_postfilter(pitch_filter_t *pf,
                      float *samples, int frame_size,
                      int period, float gain);

/* Quantize pitch period for transmission.
 * Returns a compact code (8 bits) representing the period. */
uint8_t pitch_encode_period(int period, uint32_t sample_rate);

/* Decode pitch period from compact code. */
int pitch_decode_period(uint8_t code, uint32_t sample_rate);

/* Quantize pitch gain for transmission (4 bits, 16 levels). */
uint8_t pitch_encode_gain(float gain);

/* Decode pitch gain from 4-bit code. */
float pitch_decode_gain(uint8_t code);

#endif /* OPCODEC_PITCH_H */