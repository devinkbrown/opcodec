/*
 * OPCODEC Pitch Detection and Post-Filter Implementation
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 *
 * YIN-inspired pitch detection with time-domain comb filtering
 * for enhanced voice quality at low bitrates.
 */

#include "opcodec/pitch.h"
#include <math.h>
#include <string.h>
#include <assert.h>

/* YIN algorithm threshold for pitch detection */
#define YIN_THRESHOLD 0.25f

/* Minimum correlation for voiced speech */
#define MIN_CORRELATION 0.3f

/* Period change threshold for smoothing (percentage) */
#define PERIOD_CHANGE_THRESHOLD 0.3f

/* Maximum comb filter gain to prevent instability */
#define MAX_FILTER_GAIN 0.95f

/* Pitch encoding special value for unvoiced frames */
#define UNVOICED_CODE 0

/*
 * Initialize pitch detector
 */
void pitch_detect_init(pitch_detector_t *pd, uint32_t sample_rate)
{
    assert(pd != NULL);
    assert(sample_rate == 8000 || sample_rate == 16000 || sample_rate == 32000 || sample_rate == 48000);

    memset(pd->prev_frame, 0, sizeof(pd->prev_frame));
    pd->last_period = 0;
    pd->last_gain = 0.0f;
    pd->sample_rate = sample_rate;
}

/*
 * Compute autocorrelation r(tau) = sum(x[n] * x[n+tau])
 * for efficiency in difference function calculation
 */
static float compute_autocorr(const float *x, int len, int tau)
{
    float sum = 0.0f;
    for (int n = 0; n < len - tau; n++) {
        sum += x[n] * x[n + tau];
    }
    return sum;
}

/*
 * Compute sum of squares for normalization
 */
static float compute_sum_squares(const float *x, int len)
{
    float sum = 0.0f;
    for (int n = 0; n < len; n++) {
        sum += x[n] * x[n];
    }
    return sum;
}

/*
 * Apply parabolic interpolation to refine pitch period
 */
static float parabolic_interpolation(const float *cmnd, int tau)
{
    if (tau <= 0 || cmnd[tau - 1] == 0.0f || cmnd[tau + 1] == 0.0f) {
        return (float)tau;
    }

    float y1 = cmnd[tau - 1];
    float y2 = cmnd[tau];
    float y3 = cmnd[tau + 1];

    float denom = 2.0f * (y1 - 2.0f * y2 + y3);
    if (fabsf(denom) < 1e-6f) {
        return (float)tau;
    }

    float delta = (y1 - y3) / denom;
    return (float)tau + delta;
}

/*
 * YIN pitch detection algorithm
 */
pitch_info_t pitch_detect(pitch_detector_t *pd, const float *samples, int frame_size)
{
    assert(pd != NULL);
    assert(samples != NULL);
    assert(frame_size > 0 && frame_size <= PITCH_MAX_FRAME);

    pitch_info_t result = {0, 0.0f, 0.0f};

    /* Determine pitch search range based on sample rate */
    int min_period = PITCH_MIN_PERIOD;
    int max_period = PITCH_MAX_PERIOD;

    /* Adjust range for sample rate (target 60-400 Hz for voice) */
    if (pd->sample_rate == 8000) {
        min_period = 20;   /* 400 Hz */
        max_period = 133;  /* 60 Hz */
    } else if (pd->sample_rate == 16000) {
        min_period = 40;   /* 400 Hz */
        max_period = 267;  /* 60 Hz */
    } else if (pd->sample_rate == 32000) {
        min_period = 80;   /* 400 Hz */
        max_period = 533;  /* 60 Hz, clamped to PITCH_MAX_PERIOD */
    } else if (pd->sample_rate == 48000) {
        min_period = 120;  /* 400 Hz */
        max_period = 800;  /* 60 Hz, but clamped to PITCH_MAX_PERIOD */
    }

    /* Clamp to absolute limits */
    if (max_period > PITCH_MAX_PERIOD) {
        max_period = PITCH_MAX_PERIOD;
    }

    /* Create extended buffer with previous frame overlap */
    float extended_buf[PITCH_MAX_PERIOD + PITCH_MAX_FRAME];
    memcpy(extended_buf, pd->prev_frame, PITCH_MAX_PERIOD * sizeof(float));
    memcpy(extended_buf + PITCH_MAX_PERIOD, samples, frame_size * sizeof(float));

    const float *analysis_buf = extended_buf + PITCH_MAX_PERIOD;
    /* Step 1: Compute difference function d(tau) using autocorrelation */
    float diff[PITCH_MAX_PERIOD + 1];
    float r0 = compute_sum_squares(analysis_buf, frame_size);

    for (int tau = 0; tau <= max_period && tau < frame_size; tau++) {
        if (tau == 0) {
            diff[tau] = 0.0f;
        } else {
            float r_tau = compute_autocorr(analysis_buf, frame_size, tau);
            float r_shifted = compute_sum_squares(analysis_buf + tau, frame_size - tau);

            /* d(tau) = r(0) + r_shifted(0) - 2*r(tau) */
            diff[tau] = r0 + r_shifted - 2.0f * r_tau;

            /* Prevent negative values due to floating point errors */
            if (diff[tau] < 0.0f) {
                diff[tau] = 0.0f;
            }
        }
    }

    /* Step 2: Compute cumulative mean normalized difference d'(tau) */
    float cmnd[PITCH_MAX_PERIOD + 1];
    cmnd[0] = 1.0f;

    float cumulative_sum = 0.0f;
    for (int tau = 1; tau <= max_period && tau < frame_size; tau++) {
        cumulative_sum += diff[tau];

        if (cumulative_sum == 0.0f) {
            cmnd[tau] = 1.0f;
        } else {
            cmnd[tau] = diff[tau] / (cumulative_sum / (float)tau);
        }
    }

    /* Step 3: Find absolute threshold minimum */
    int best_period = 0;
    float best_cmnd = 1.0f;

    for (int tau = min_period; tau <= max_period && tau < frame_size; tau++) {
        if (cmnd[tau] < YIN_THRESHOLD && cmnd[tau] < best_cmnd) {
            /* Check for local minimum */
            if (tau > 0 && tau < frame_size - 1 &&
                cmnd[tau] <= cmnd[tau - 1] && cmnd[tau] <= cmnd[tau + 1]) {
                best_period = tau;
                best_cmnd = cmnd[tau];
                break;  /* Take first good minimum */
            }
        }
    }

    /* If no threshold crossing, find global minimum */
    if (best_period == 0) {
        for (int tau = min_period; tau <= max_period && tau < frame_size; tau++) {
            if (cmnd[tau] < best_cmnd) {
                best_period = tau;
                best_cmnd = cmnd[tau];
            }
        }

        /* Require minimum quality even for global minimum */
        if (best_cmnd > 0.8f) {
            best_period = 0;  /* Unvoiced */
        }
    }

    /* Step 4: Parabolic interpolation and correlation calculation */
    if (best_period > 0) {
        float refined_period = parabolic_interpolation(cmnd, best_period);

        /* Compute normalized correlation for the refined period */
        int int_period = (int)floorf(refined_period + 0.5f);
        if (int_period > 0 && int_period < frame_size) {
            float r_tau = compute_autocorr(analysis_buf, frame_size, int_period);
            float r_shifted = compute_sum_squares(analysis_buf + int_period, frame_size - int_period);

            float correlation = 0.0f;
            if (r0 > 0.0f && r_shifted > 0.0f) {
                correlation = r_tau / sqrtf(r0 * r_shifted);

                /* Clamp correlation to valid range */
                if (correlation < 0.0f) correlation = 0.0f;
                if (correlation > 1.0f) correlation = 1.0f;
            }

            /* Apply minimum correlation threshold */
            if (correlation >= MIN_CORRELATION) {
                /* Period smoothing: avoid wild jumps if correlation is weak */
                if (pd->last_period > 0 && correlation < 0.7f) {
                    float period_change = fabsf(refined_period - pd->last_period) / pd->last_period;
                    if (period_change > PERIOD_CHANGE_THRESHOLD) {
                        /* Use previous period with reduced gain */
                        result.period = pd->last_period;
                        result.correlation = correlation * 0.8f;
                    } else {
                        result.period = int_period;
                        result.correlation = correlation;
                    }
                } else {
                    result.period = int_period;
                    result.correlation = correlation;
                }

                /* Compute optimal comb filter gain */
                result.gain = result.correlation * 0.4f;
                if (result.gain > MAX_FILTER_GAIN) {
                    result.gain = MAX_FILTER_GAIN;
                }
            }
        }
    }

    /* Update state for next frame */
    pd->last_period = result.period;
    pd->last_gain = result.gain;

    /* Save current frame for next analysis (last PITCH_MAX_PERIOD samples) */
    int copy_start = (frame_size > PITCH_MAX_PERIOD) ?
                     frame_size - PITCH_MAX_PERIOD : 0;
    int copy_len = (frame_size > PITCH_MAX_PERIOD) ?
                   PITCH_MAX_PERIOD : frame_size;

    memset(pd->prev_frame, 0, sizeof(pd->prev_frame));
    memcpy(pd->prev_frame + PITCH_MAX_PERIOD - copy_len,
           samples + copy_start, copy_len * sizeof(float));

    return result;
}

/*
 * Initialize pitch post-filter
 */
void pitch_filter_init(pitch_filter_t *pf)
{
    assert(pf != NULL);

    memset(pf->buf, 0, sizeof(pf->buf));
    pf->period = 0;
    pf->gain = 0.0f;
}

/*
 * Apply pitch pre-filter (encoder side, FIR whitening filter)
 * y[n] = x[n] - gain * x[n - period]
 *
 * The delay buffer stores the ORIGINAL input x[n], not the filtered output.
 * For n < period: x[n-period] comes from pf->buf (previous frame's history).
 * For n >= period: x[n-period] comes from the current frame's original.
 */
void pitch_prefilter(pitch_filter_t *pf, float *samples, int frame_size,
                     int period, float gain)
{
    assert(pf != NULL);
    assert(samples != NULL);
    assert(frame_size > 0 && frame_size <= PITCH_MAX_FRAME);
    assert(period >= 0 && period <= PITCH_MAX_PERIOD);
    assert(gain >= 0.0f && gain <= 1.0f);

    /* Skip filtering for unvoiced frames */
    if (period == 0 || gain < 0.01f) {
        for (int n = 0; n < frame_size; n++)
            pf->buf[PITCH_MAX_PERIOD + n] = samples[n];
        memmove(pf->buf, pf->buf + frame_size,
                PITCH_MAX_PERIOD * sizeof(float));
        return;
    }

    /* Save original before modifying samples in-place */
    float original[PITCH_MAX_FRAME];
    memcpy(original, samples, frame_size * sizeof(float));

    /* Apply FIR pre-filter: y[n] = x[n] - gain * x[n-period]
     * For n < period: x[n-period] is in pf->buf (previous frame history).
     * For n >= period: x[n-period] is in the current frame's original.
     * Write original[n] into buf[PITCH_MAX_PERIOD+n] during the loop so
     * the memmove below places it correctly for the next frame's reads —
     * matching the post-filter's inline-write pattern. */
    for (int n = 0; n < frame_size; n++) {
        float delayed = (n < period)
            ? pf->buf[PITCH_MAX_PERIOD - period + n]
            : original[n - period];
        samples[n] -= gain * delayed;
        pf->buf[PITCH_MAX_PERIOD + n] = original[n];
    }

    /* Shift delay buffer for next frame */
    memmove(pf->buf, pf->buf + frame_size,
            PITCH_MAX_PERIOD * sizeof(float));

    pf->period = period;
    pf->gain = gain;
}

/*
 * Apply pitch post-filter (decoder side, IIR synthesis filter)
 * y[n] = x[n] + gain * y[n - period]
 */
void pitch_postfilter(pitch_filter_t *pf, float *samples, int frame_size,
                      int period, float gain)
{
    assert(pf != NULL);
    assert(samples != NULL);
    assert(frame_size > 0 && frame_size <= PITCH_MAX_FRAME);
    assert(period >= 0 && period <= PITCH_MAX_PERIOD);
    assert(gain >= 0.0f && gain <= 1.0f);

    /* Skip filtering for unvoiced frames */
    if (period == 0 || gain < 0.01f) {
        /* Still need to update delay buffer */
        memmove(pf->buf, pf->buf + frame_size,
                PITCH_MAX_PERIOD * sizeof(float));
        memcpy(pf->buf + PITCH_MAX_PERIOD, samples,
               frame_size * sizeof(float));
        return;
    }

    /* Apply IIR post-filter */
    for (int n = 0; n < frame_size; n++) {
        float delayed = pf->buf[PITCH_MAX_PERIOD - period + n];
        samples[n] = samples[n] + gain * delayed;

        /* Update delay buffer with filtered output as we compute it */
        pf->buf[PITCH_MAX_PERIOD + n] = samples[n];
    }

    /* Shift delay buffer for next frame */
    memmove(pf->buf, pf->buf + frame_size,
            PITCH_MAX_PERIOD * sizeof(float));

    pf->period = period;
    pf->gain = gain;
}

/*
 * Quantize pitch period for transmission (8 bits)
 */
uint8_t pitch_encode_period(int period, uint32_t sample_rate)
{
    if (period == 0) {
        return UNVOICED_CODE;  /* Special code for unvoiced */
    }

    /* Map to sample rate specific range */
    int min_period = PITCH_MIN_PERIOD;
    int max_period = PITCH_MAX_PERIOD;

    if (sample_rate == 8000) {
        min_period = 20;
        max_period = 133;
    } else if (sample_rate == 16000) {
        min_period = 40;
        max_period = 267;
    } else if (sample_rate == 32000) {
        min_period = 80;
        max_period = 533;
    } else if (sample_rate == 48000) {
        min_period = 120;
        max_period = 480;
    }

    /* Clamp period to valid range */
    if (period < min_period) period = min_period;
    if (period > max_period) period = max_period;

    /* Linear mapping to 1-255 range (0 reserved for unvoiced) */
    int range = max_period - min_period;
    int code = 1 + ((period - min_period) * 254) / range;

    /* Ensure valid range */
    if (code < 1) code = 1;
    if (code > 255) code = 255;

    return (uint8_t)code;
}

/*
 * Decode pitch period from 8-bit code
 */
int pitch_decode_period(uint8_t code, uint32_t sample_rate)
{
    if (code == UNVOICED_CODE) {
        return 0;  /* Unvoiced */
    }

    /* Map from sample rate specific range */
    int min_period = PITCH_MIN_PERIOD;
    int max_period = PITCH_MAX_PERIOD;

    if (sample_rate == 8000) {
        min_period = 20;
        max_period = 133;
    } else if (sample_rate == 16000) {
        min_period = 40;
        max_period = 267;
    } else if (sample_rate == 32000) {
        min_period = 80;
        max_period = 533;
    } else if (sample_rate == 48000) {
        min_period = 120;
        max_period = 480;
    }

    /* Linear mapping from 1-255 range */
    int range = max_period - min_period;
    int period = min_period + ((code - 1) * range) / 254;

    /* Clamp to valid range */
    if (period < min_period) period = min_period;
    if (period > max_period) period = max_period;

    return period;
}

/*
 * Quantize pitch gain for transmission (6 bits, 64 levels)
 */
uint8_t pitch_encode_gain(float gain)
{
    if (gain < 0.0f) gain = 0.0f;
    if (gain >= 1.0f) gain = 0.9999f;

    int code = (int)(gain * 64.0f);

    if (code > 63) code = 63;

    return (uint8_t)code;
}

/*
 * Decode pitch gain from 6-bit code
 */
float pitch_decode_gain(uint8_t code)
{
    if (code > 63) code = 63;

    return (float)code / 64.0f;
}