/*
 * opcodec/stereo.c — Parametric Stereo Coding Implementation
 *
 * Parametric stereo implementation for the OPVOX audio codec.
 * Reduces stereo bitrate by transmitting mono downmix plus spatial parameters.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/stereo.h"
#include <math.h>
#include <string.h>
#include <assert.h>

/* Mathematical constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Temporal smoothing factor for parameter interpolation */
#define PS_SMOOTH_ALPHA 0.3f

/* ICC threshold for IPD transmission */
#define PS_IPD_THRESHOLD 0.8f

/* Minimum energy threshold to avoid division by zero */
#define PS_MIN_ENERGY 1e-12f

/* IID quantization parameters (5 bits = 32 levels) */
#define PS_IID_BITS 5
#define PS_IID_LEVELS (1 << PS_IID_BITS)
#define PS_IID_MIN_DB -15.0f
#define PS_IID_MAX_DB 15.0f
#define PS_IID_STEP ((PS_IID_MAX_DB - PS_IID_MIN_DB) / (PS_IID_LEVELS - 1))

/* ICC quantization parameters (3 bits = 8 levels) */
#define PS_ICC_BITS 3
#define PS_ICC_LEVELS (1 << PS_ICC_BITS)
#define PS_ICC_STEP (1.0f / (PS_ICC_LEVELS - 1))

/* IPD quantization parameters (5 bits = 32 levels) */
#define PS_IPD_BITS 5
#define PS_IPD_LEVELS (1 << PS_IPD_BITS)
#define PS_IPD_STEP ((2.0f * M_PI) / PS_IPD_LEVELS)

/* Helper function to clamp values to range */
static inline float
ps_clamp(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/* Helper function to compute energy in a frequency band */
static float
ps_band_energy(const float *mdct, const band_range_t *band)
{
    float energy = 0.0f;
    for (uint16_t i = band->start; i < band->end && i < 480; i++) {
        energy += mdct[i] * mdct[i];
    }
    return energy + PS_MIN_ENERGY; /* Avoid division by zero */
}

/* Helper function to compute correlation between L and R in a band */
static float
ps_band_correlation(const float *mdct_l, const float *mdct_r, const band_range_t *band)
{
    float cross = 0.0f, energy_l = 0.0f, energy_r = 0.0f;

    for (uint16_t i = band->start; i < band->end && i < 480; i++) {
        cross += mdct_l[i] * mdct_r[i];
        energy_l += mdct_l[i] * mdct_l[i];
        energy_r += mdct_r[i] * mdct_r[i];
    }

    float denom = sqrtf((energy_l + PS_MIN_ENERGY) * (energy_r + PS_MIN_ENERGY));
    return ps_clamp(cross / denom, -1.0f, 1.0f);
}

/* Helper function to compute phase difference in a band */
static float
ps_band_phase_diff(const float *mdct_l, const float *mdct_r, const band_range_t *band)
{
    float real_cross = 0.0f, imag_cross = 0.0f;

    /* Approximate phase difference using adjacent coefficient pairs */
    for (uint16_t i = band->start; i < band->end - 1 && i < 479; i += 2) {
        float l_real = mdct_l[i], l_imag = mdct_l[i + 1];
        float r_real = mdct_r[i], r_imag = mdct_r[i + 1];

        /* Cross-correlation in complex domain: L * conj(R) */
        real_cross += l_real * r_real + l_imag * r_imag;
        imag_cross += l_imag * r_real - l_real * r_imag;
    }

    return atan2f(imag_cross, real_cross + PS_MIN_ENERGY);
}

/* Quantize IID value to 5-bit integer */
static uint8_t
ps_quantize_iid(float iid_db)
{
    float clamped = ps_clamp(iid_db, PS_IID_MIN_DB, PS_IID_MAX_DB);
    int quantized = (int)roundf((clamped - PS_IID_MIN_DB) / PS_IID_STEP);
    return (uint8_t)ps_clamp((float)quantized, 0.0f, (float)(PS_IID_LEVELS - 1));
}

/* Dequantize IID value from 5-bit integer */
static float
ps_dequantize_iid(uint8_t iid_q)
{
    return PS_IID_MIN_DB + (float)iid_q * PS_IID_STEP;
}

/* Quantize ICC value to 3-bit integer */
static uint8_t
ps_quantize_icc(float icc)
{
    float clamped = ps_clamp(icc, 0.0f, 1.0f);
    int quantized = (int)roundf(clamped / PS_ICC_STEP);
    return (uint8_t)ps_clamp((float)quantized, 0.0f, (float)(PS_ICC_LEVELS - 1));
}

/* Dequantize ICC value from 3-bit integer */
static float
ps_dequantize_icc(uint8_t icc_q)
{
    return (float)icc_q * PS_ICC_STEP;
}

/* Quantize IPD value to 5-bit integer */
static uint8_t
ps_quantize_ipd(float ipd)
{
    /* Wrap phase to [-π, π] */
    while (ipd > M_PI) ipd -= 2.0f * M_PI;
    while (ipd < -M_PI) ipd += 2.0f * M_PI;

    /* Map to [0, 2π] for quantization */
    float normalized = ipd + M_PI;
    int quantized = (int)roundf(normalized / PS_IPD_STEP);
    return (uint8_t)(quantized % PS_IPD_LEVELS);
}

/* Dequantize IPD value from 5-bit integer */
static float
ps_dequantize_ipd(uint8_t ipd_q)
{
    float normalized = (float)ipd_q * PS_IPD_STEP;
    return normalized - M_PI; /* Map back to [-π, π] */
}

/* ---- Public API Implementation ---- */

void
ps_init(ps_ctx_t *ctx, uint8_t num_bands)
{
    assert(ctx != NULL);
    assert(num_bands > 0 && num_bands <= PS_MAX_BANDS);

    memset(ctx, 0, sizeof(ps_ctx_t));
    ctx->num_bands = num_bands;
    ctx->has_prev = false;
}

void
ps_analyze(ps_ctx_t *ctx,
           const float *mdct_l, const float *mdct_r,
           int num_coeffs,
           const band_range_t *bands, int num_bands,
           ps_params_t *params)
{
    assert(ctx != NULL);
    assert(mdct_l != NULL && mdct_r != NULL);
    assert(params != NULL);
    assert(bands != NULL);
    assert(num_bands > 0 && num_bands <= PS_MAX_BANDS);
    assert(num_coeffs > 0);

    params->num_bands = (uint8_t)num_bands;
    params->has_ipd = false;

    for (int band = 0; band < num_bands; band++) {
        /* Compute energy in each channel for this band */
        float energy_l = ps_band_energy(mdct_l, &bands[band]);
        float energy_r = ps_band_energy(mdct_r, &bands[band]);

        /* Compute IID: Inter-channel Intensity Difference */
        float iid_ratio = energy_l / energy_r;
        params->iid[band] = 10.0f * log10f(iid_ratio);

        /* Compute ICC: Inter-channel Coherence */
        params->icc[band] = fabsf(ps_band_correlation(mdct_l, mdct_r, &bands[band]));

        /* Compute IPD: Inter-channel Phase Difference (only if needed) */
        if (params->icc[band] < PS_IPD_THRESHOLD) {
            params->ipd[band] = ps_band_phase_diff(mdct_l, mdct_r, &bands[band]);
            params->has_ipd = true;
        } else {
            params->ipd[band] = 0.0f;
        }

        /* Temporal smoothing with previous frame */
        if (ctx->has_prev) {
            float alpha = PS_SMOOTH_ALPHA;
            params->iid[band] = alpha * params->iid[band] + (1.0f - alpha) * ctx->prev_params.iid[band];
            params->icc[band] = alpha * params->icc[band] + (1.0f - alpha) * ctx->prev_params.icc[band];
            if (params->has_ipd) {
                params->ipd[band] = alpha * params->ipd[band] + (1.0f - alpha) * ctx->prev_params.ipd[band];
            }
        }
    }

    /* Store parameters for next frame */
    ctx->prev_params = *params;

    /* Store MDCT coefficients for potential cross-frame processing */
    int copy_coeffs = num_coeffs < 480 ? num_coeffs : 480;
    memcpy(ctx->prev_l, mdct_l, copy_coeffs * sizeof(float));
    memcpy(ctx->prev_r, mdct_r, copy_coeffs * sizeof(float));
    ctx->has_prev = true;
}

void
ps_downmix(const float *mdct_l, const float *mdct_r,
           float *mdct_mono, int num_coeffs)
{
    assert(mdct_l != NULL && mdct_r != NULL && mdct_mono != NULL);
    assert(num_coeffs > 0);

    /* Standard downmix formula: M = (L + R) / sqrt(2) */
    const float scale = 1.0f / sqrtf(2.0f);

    for (int i = 0; i < num_coeffs; i++) {
        mdct_mono[i] = (mdct_l[i] + mdct_r[i]) * scale;
    }
}

void
ps_upmix(const ps_params_t *params,
          const float *mdct_mono,
          float *mdct_l, float *mdct_r,
          int num_coeffs,
          const band_range_t *bands, int num_bands)
{
    assert(params != NULL);
    assert(mdct_mono != NULL && mdct_l != NULL && mdct_r != NULL);
    assert(bands != NULL);
    assert(num_bands > 0 && num_coeffs > 0);

    for (int band = 0; band < num_bands && band < PS_MAX_BANDS; band++) {
        /* Convert IID from dB to linear scale */
        float iid_linear = powf(10.0f, params->iid[band] / 10.0f);

        /* Compute left/right energy scaling factors */
        float scale_l = sqrtf((1.0f + iid_linear) / 2.0f);
        float scale_r = sqrtf((1.0f / iid_linear + 1.0f) / 2.0f);

        /* Apply coherence-based stereo width control */
        float icc = ps_clamp(params->icc[band], 0.0f, 1.0f);
        float decorr_factor = sqrtf(1.0f - icc * icc);

        /* Basic phase rotation if IPD is available */
        float cos_ipd = 1.0f;
        if (params->has_ipd) {
            cos_ipd = cosf(params->ipd[band] * 0.5f);
        }

        /* Apply upmixing to each coefficient in the band */
        for (uint16_t i = bands[band].start; i < bands[band].end && i < (uint16_t)num_coeffs; i++) {
            float mono = mdct_mono[i];

            /* Create decorrelated component for stereo width */
            float decorr = mono * decorr_factor;
            float corr = mono * icc;

            /* Apply scaling and phase rotation */
            mdct_l[i] = (corr + decorr) * scale_l * cos_ipd;
            mdct_r[i] = (corr - decorr) * scale_r * cos_ipd;
        }
    }
}

int
ps_encode_params(const ps_params_t *params,
                 uint8_t *out, size_t out_cap)
{
    assert(params != NULL && out != NULL);
    assert(params->num_bands <= PS_MAX_BANDS);

    /* Calculate required space: header + IID + ICC + optional IPD */
    size_t base_size = 1 + /* header byte */
                      (params->num_bands * PS_IID_BITS + 7) / 8 + /* IID */
                      (params->num_bands * PS_ICC_BITS + 7) / 8;  /* ICC */

    size_t ipd_size = 0;
    if (params->has_ipd) {
        ipd_size = (params->num_bands * PS_IPD_BITS + 7) / 8;
    }

    size_t total_size = base_size + ipd_size;
    if (total_size > out_cap) {
        return -1; /* Not enough space */
    }

    uint8_t *ptr = out;

    /* Header byte: [num_bands:5][has_ipd:1][reserved:2] */
    *ptr++ = (params->num_bands << 3) | (params->has_ipd ? 0x04 : 0x00);

    /* Pack IID values (5 bits each) */
    uint32_t bit_buffer = 0;
    int bit_count = 0;

    for (int i = 0; i < params->num_bands; i++) {
        uint8_t iid_q = ps_quantize_iid(params->iid[i]);
        bit_buffer = (bit_buffer << PS_IID_BITS) | iid_q;
        bit_count += PS_IID_BITS;

        while (bit_count >= 8) {
            *ptr++ = (uint8_t)(bit_buffer >> (bit_count - 8));
            bit_count -= 8;
        }
    }

    /* Pack ICC values (3 bits each) */
    for (int i = 0; i < params->num_bands; i++) {
        uint8_t icc_q = ps_quantize_icc(params->icc[i]);
        bit_buffer = (bit_buffer << PS_ICC_BITS) | icc_q;
        bit_count += PS_ICC_BITS;

        while (bit_count >= 8) {
            *ptr++ = (uint8_t)(bit_buffer >> (bit_count - 8));
            bit_count -= 8;
        }
    }

    /* Pack IPD values if present (5 bits each) */
    if (params->has_ipd) {
        for (int i = 0; i < params->num_bands; i++) {
            uint8_t ipd_q = ps_quantize_ipd(params->ipd[i]);
            bit_buffer = (bit_buffer << PS_IPD_BITS) | ipd_q;
            bit_count += PS_IPD_BITS;

            while (bit_count >= 8) {
                *ptr++ = (uint8_t)(bit_buffer >> (bit_count - 8));
                bit_count -= 8;
            }
        }
    }

    /* Flush remaining bits */
    if (bit_count > 0) {
        *ptr++ = (uint8_t)(bit_buffer << (8 - bit_count));
    }

    return (int)(ptr - out);
}

int
ps_decode_params(ps_params_t *params,
                 const uint8_t *in, size_t in_len)
{
    assert(params != NULL && in != NULL);

    if (in_len < 1) {
        return -1; /* Need at least header byte */
    }

    const uint8_t *ptr = in;
    const uint8_t *end = in + in_len;

    /* Parse header byte */
    uint8_t header = *ptr++;
    params->num_bands = (header >> 3) & 0x1F;
    params->has_ipd = (header & 0x04) != 0;

    if (params->num_bands == 0 || params->num_bands > PS_MAX_BANDS) {
        return -1; /* Invalid band count */
    }

    /* Unpack bit stream */
    uint32_t bit_buffer = 0;
    int bit_count = 0;

    /* Fill initial bit buffer */
    while (ptr < end && bit_count < 32) {
        bit_buffer = (bit_buffer << 8) | *ptr++;
        bit_count += 8;
    }

    /* Extract IID values */
    for (int i = 0; i < params->num_bands; i++) {
        if (bit_count < PS_IID_BITS) {
            if (ptr < end) {
                bit_buffer = (bit_buffer << 8) | *ptr++;
                bit_count += 8;
            } else if (bit_count < PS_IID_BITS) {
                return -1; /* Not enough data */
            }
        }

        uint8_t iid_q = (bit_buffer >> (bit_count - PS_IID_BITS)) & ((1 << PS_IID_BITS) - 1);
        params->iid[i] = ps_dequantize_iid(iid_q);
        bit_count -= PS_IID_BITS;
    }

    /* Extract ICC values */
    for (int i = 0; i < params->num_bands; i++) {
        if (bit_count < PS_ICC_BITS) {
            if (ptr < end) {
                bit_buffer = (bit_buffer << 8) | *ptr++;
                bit_count += 8;
            } else if (bit_count < PS_ICC_BITS) {
                return -1; /* Not enough data */
            }
        }

        uint8_t icc_q = (bit_buffer >> (bit_count - PS_ICC_BITS)) & ((1 << PS_ICC_BITS) - 1);
        params->icc[i] = ps_dequantize_icc(icc_q);
        bit_count -= PS_ICC_BITS;
    }

    /* Extract IPD values if present */
    if (params->has_ipd) {
        for (int i = 0; i < params->num_bands; i++) {
            if (bit_count < PS_IPD_BITS) {
                if (ptr < end) {
                    bit_buffer = (bit_buffer << 8) | *ptr++;
                    bit_count += 8;
                } else if (bit_count < PS_IPD_BITS) {
                    return -1; /* Not enough data */
                }
            }

            uint8_t ipd_q = (bit_buffer >> (bit_count - PS_IPD_BITS)) & ((1 << PS_IPD_BITS) - 1);
            params->ipd[i] = ps_dequantize_ipd(ipd_q);
            bit_count -= PS_IPD_BITS;
        }
    }

    return (int)(ptr - in);
}