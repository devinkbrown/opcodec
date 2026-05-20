/*
 * Temporal Noise Shaping (TNS) for MDCT-based audio codecs
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 *
 * TNS prevents pre-echo artifacts in transient frames by shaping quantization
 * noise in the time domain using LPC prediction in the MDCT domain.
 */

#include <opcodec/tns.h>
#include <math.h>
#include <string.h>

/* TNS analysis thresholds */
#define TNS_GAIN_THRESHOLD    1.5f    /* minimum prediction gain to enable TNS */
#define TNS_ENERGY_THRESHOLD  1e-10f  /* minimum energy for stability */
#define TNS_REFLECT_THRESHOLD 0.01f   /* threshold for trimming reflection coeffs */

/* Initialize TNS context */
void tns_init(tns_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->lpc[0] = 1.0f;  /* a[0] = 1.0 always */
}

/* Compute autocorrelation of MDCT spectrum */
static void compute_autocorr(const float *mdct, int start, int end,
                            float *r, int max_order)
{
    /* Initialize autocorrelation array */
    for (int k = 0; k <= max_order; k++) {
        r[k] = 0.0f;
    }

    /* Compute r[k] = sum(mdct[i] * mdct[i+k]) */
    for (int k = 0; k <= max_order; k++) {
        for (int i = start; i < end - k; i++) {
            r[k] += mdct[i] * mdct[i + k];
        }
    }
}

/* Levinson-Durbin recursion to compute LPC coefficients from autocorrelation */
static int levinson_durbin(const float *r, float *a, float *k, int max_order)
{
    float E[TNS_MAX_ORDER + 1];
    float a_tmp[TNS_MAX_ORDER + 1];

    /* Check for numerical stability */
    if (r[0] < TNS_ENERGY_THRESHOLD) {
        return 0;
    }

    /* Initialize */
    a[0] = 1.0f;
    E[0] = r[0];

    int order = 0;

    for (int m = 1; m <= max_order; m++) {
        /* Compute reflection coefficient */
        float lambda = 0.0f;
        for (int j = 0; j < m; j++) {
            lambda += a[j] * r[m - j];
        }
        lambda = -lambda / E[m - 1];
        k[m] = lambda;

        /* Check stability constraint */
        if (fabsf(lambda) >= 1.0f) {
            break;
        }

        /* Update LPC coefficients */
        a_tmp[0] = 1.0f;
        a_tmp[m] = lambda;
        for (int j = 1; j < m; j++) {
            a_tmp[j] = a[j] + lambda * a[m - j];
        }

        /* Copy back */
        for (int j = 0; j <= m; j++) {
            a[j] = a_tmp[j];
        }

        /* Update prediction error */
        E[m] = E[m - 1] * (1.0f - lambda * lambda);

        /* Check for numerical stability */
        if (E[m] <= TNS_ENERGY_THRESHOLD) {
            break;
        }

        order = m;
    }

    return order;
}

/* Quantize reflection coefficient to 4-bit signed integer */
static int8_t quantize_reflect_coeff(float k)
{
    int quant = (int)roundf(k * 8.0f);
    if (quant < -8) quant = -8;
    if (quant > 7) quant = 7;
    return (int8_t)quant;
}

/* Dequantize 4-bit reflection coefficient */
static float dequantize_reflect_coeff(int8_t quant)
{
    return (float)quant / 8.0f;
}

/* Convert reflection coefficients to LPC coefficients using step-up recursion */
static void reflect_to_lpc(const float *k, float *a, int order)
{
    float a_tmp[TNS_MAX_ORDER + 1];

    a[0] = 1.0f;

    for (int m = 1; m <= order; m++) {
        a_tmp[0] = 1.0f;
        a_tmp[m] = k[m];

        for (int j = 1; j < m; j++) {
            a_tmp[j] = a[j] + k[m] * a[m - j];
        }

        for (int j = 0; j <= m; j++) {
            a[j] = a_tmp[j];
        }
    }
}

/* Determine optimal TNS order by trimming trailing weak coefficients */
static int determine_order(const float *k, int max_order)
{
    int order = max_order;

    while (order > 0 && fabsf(k[order]) < TNS_REFLECT_THRESHOLD) {
        order--;
    }

    return order;
}

/* Analyze MDCT coefficients and decide whether TNS should be applied */
bool tns_analyze(tns_ctx_t *ctx,
                 const float *mdct, int num_coeffs,
                 const tns_band_t *bands, int num_bands,
                 tns_params_t *params)
{
    float r[TNS_MAX_ORDER + 1];
    float k[TNS_MAX_ORDER + 1];

    /* Reset context */
    tns_init(ctx);
    memset(params, 0, sizeof(*params));

    /* Determine analysis region (typically full spectrum) */
    int start = 0;
    int end = num_coeffs;

    if (num_bands > 0 && bands != NULL) {
        start = bands[0].start;
        end = bands[num_bands - 1].end;

        /* Clamp to valid range */
        if (end > num_coeffs) end = num_coeffs;
        if (start >= end) return false;
    }

    /* Need minimum length for meaningful analysis */
    if (end - start < TNS_MAX_ORDER + 1) {
        return false;
    }

    /* Compute autocorrelation of MDCT spectrum */
    compute_autocorr(mdct, start, end, r, TNS_MAX_ORDER);

    /* Apply Levinson-Durbin recursion */
    int max_order = levinson_durbin(r, ctx->lpc, k, TNS_MAX_ORDER);
    if (max_order == 0) {
        return false;
    }

    /* Calculate prediction gain */
    float E_final = r[0];
    for (int m = 1; m <= max_order; m++) {
        E_final *= (1.0f - k[m] * k[m]);
        if (E_final <= TNS_ENERGY_THRESHOLD) break;
    }

    float prediction_gain = r[0] / E_final;

    /* Check if TNS is beneficial */
    if (prediction_gain < TNS_GAIN_THRESHOLD) {
        return false;
    }

    /* Determine optimal order */
    int order = determine_order(k, max_order);
    if (order == 0) {
        return false;
    }

    /* Fill parameters structure */
    params->order = (uint8_t)order;
    params->start_band = (uint16_t)start;
    params->stop_band = (uint16_t)end;

    /* Quantize reflection coefficients */
    for (int i = 1; i <= order; i++) {
        params->coeffs[i - 1] = quantize_reflect_coeff(k[i]);
    }

    ctx->order = (uint8_t)order;
    ctx->active = true;

    return true;
}

/* Apply TNS analysis filter (FIR) to MDCT coefficients */
void tns_filter_encode(const tns_ctx_t *ctx,
                       float *mdct, int num_coeffs,
                       const tns_band_t *bands __attribute__((unused)),
                       int num_bands __attribute__((unused)),
                       const tns_params_t *params)
{
    if (!ctx->active || ctx->order == 0) {
        return;
    }

    int start = params->start_band;
    int end = params->stop_band;

    /* Clamp to valid range */
    if (end > num_coeffs) end = num_coeffs;
    if (start >= end) return;

    /* Create copy of original coefficients for FIR filtering */
    static float original[TNS_MAX_COEFFS];
    if (end - start > TNS_MAX_COEFFS) return;

    memcpy(original, mdct + start, (end - start) * sizeof(float));

    /* Apply FIR analysis filter: y[n] = sum(a[k] * x[n-k]) */
    for (int i = start; i < end; i++) {
        float filtered = ctx->lpc[0] * original[i - start];

        for (int j = 1; j <= ctx->order && (i - start - j) >= 0; j++) {
            filtered += ctx->lpc[j] * original[i - start - j];
        }

        mdct[i] = filtered;
    }
}

/* Decode TNS parameters and set up synthesis filter */
void tns_decode_params(tns_ctx_t *ctx, const tns_params_t *params)
{
    tns_init(ctx);

    if (params->order == 0) {
        ctx->active = false;
        return;
    }

    /* Dequantize reflection coefficients */
    float k[TNS_MAX_ORDER + 1];
    k[0] = 0.0f;  /* not used */

    for (int i = 0; i < params->order; i++) {
        k[i + 1] = dequantize_reflect_coeff(params->coeffs[i]);
    }

    /* Convert to LPC coefficients */
    reflect_to_lpc(k, ctx->lpc, params->order);

    ctx->order = params->order;
    ctx->active = true;
}

/* Apply TNS synthesis filter (IIR) to MDCT coefficients */
void tns_filter_decode(const tns_ctx_t *ctx,
                       float *mdct, int num_coeffs,
                       const tns_band_t *bands __attribute__((unused)),
                       int num_bands __attribute__((unused)),
                       const tns_params_t *params)
{
    if (!ctx->active || ctx->order == 0) {
        return;
    }

    int start = params->start_band;
    int end = params->stop_band;

    /* Clamp to valid range */
    if (end > num_coeffs) end = num_coeffs;
    if (start >= end) return;

    /* Apply IIR synthesis filter: y[n] = x[n] - sum(a[k] * y[n-k]) */
    for (int i = start; i < end; i++) {
        float val = mdct[i];

        for (int j = 1; j <= ctx->order && (i - j) >= start; j++) {
            val -= ctx->lpc[j] * mdct[i - j];
        }

        mdct[i] = val;
    }
}

/* Encode TNS parameters to bitstream */
int tns_encode_params(const tns_params_t *params,
                      uint8_t *out, size_t out_cap)
{
    if (out_cap < 8) return -1;  /* need at least 8 bytes for worst case */

    uint32_t bits = 0;
    size_t bit_pos = 0;
    size_t byte_pos = 0;

    /* Helper function to write bits */
    bool write_bits_failed = false;

    #define WRITE_BITS(value, num_bits) do { \
        if (!write_bits_failed) { \
            uint32_t val = (value); \
            size_t nbits = (size_t)(num_bits); \
            bits |= (val << bit_pos); \
            bit_pos += nbits; \
            while (bit_pos >= 8) { \
                if (byte_pos >= out_cap) { \
                    write_bits_failed = true; \
                    break; \
                } \
                out[byte_pos++] = (uint8_t)(bits & 0xFF); \
                bits >>= 8; \
                bit_pos -= 8; \
            } \
        } \
    } while(0)

    /* Write TNS active flag */
    bool active = (params->order > 0);
    WRITE_BITS(active ? 1 : 0, 1);
    if (write_bits_failed) return -1;

    if (!active) {
        /* Flush remaining bits */
        if (bit_pos > 0) {
            if (byte_pos >= out_cap) return -1;
            out[byte_pos++] = (uint8_t)(bits & 0xFF);
        }
        return byte_pos;
    }

    /* Write order (4 bits) */
    WRITE_BITS(params->order, 4);
    if (write_bits_failed) return -1;

    /* Write start_band (5 bits) */
    WRITE_BITS(params->start_band, 5);
    if (write_bits_failed) return -1;

    /* Write stop_band (5 bits) */
    WRITE_BITS(params->stop_band, 5);
    if (write_bits_failed) return -1;

    /* Write quantized reflection coefficients (4 bits each) */
    for (int i = 0; i < params->order; i++) {
        uint8_t coeff = (uint8_t)(params->coeffs[i] & 0x0F);
        WRITE_BITS(coeff, 4);
        if (write_bits_failed) return -1;
    }

    /* Flush remaining bits */
    if (bit_pos > 0) {
        if (byte_pos >= out_cap) return -1;
        out[byte_pos++] = (uint8_t)(bits & 0xFF);
    }

    #undef WRITE_BITS
    return byte_pos;
}

/* Decode TNS parameters from bitstream */
int tns_decode_params_from_stream(tns_params_t *params,
                                 const uint8_t *in, size_t in_len)
{
    if (in_len < 1) return -1;

    memset(params, 0, sizeof(*params));

    uint32_t bits = 0;
    size_t bit_pos = 0;
    size_t byte_pos = 0;

    /* Helper function to read bits */
    bool read_bits_failed = false;

    #define READ_BITS(num_bits, result) do { \
        if (!read_bits_failed) { \
            size_t nbits = (size_t)(num_bits); \
            while (bit_pos < nbits) { \
                if (byte_pos >= in_len) { \
                    read_bits_failed = true; \
                    break; \
                } \
                bits |= ((uint32_t)in[byte_pos++] << bit_pos); \
                bit_pos += 8; \
            } \
            if (!read_bits_failed) { \
                uint32_t mask = (1U << nbits) - 1; \
                uint32_t value = bits & mask; \
                bits >>= nbits; \
                bit_pos -= nbits; \
                (result) = (int32_t)value; \
            } \
        } \
    } while(0)

    /* Read TNS active flag */
    int32_t active;
    READ_BITS(1, active);
    if (read_bits_failed) return -1;

    if (active == 0) {
        return byte_pos;
    }

    /* Read order */
    int32_t order;
    READ_BITS(4, order);
    if (read_bits_failed || order > TNS_MAX_ORDER) return -1;
    params->order = (uint8_t)order;

    /* Read start_band */
    int32_t start_band;
    READ_BITS(5, start_band);
    if (read_bits_failed) return -1;
    params->start_band = (uint16_t)start_band;

    /* Read stop_band */
    int32_t stop_band;
    READ_BITS(5, stop_band);
    if (read_bits_failed) return -1;
    params->stop_band = (uint16_t)stop_band;

    /* Read quantized reflection coefficients */
    for (int i = 0; i < order; i++) {
        int32_t coeff;
        READ_BITS(4, coeff);
        if (read_bits_failed) return -1;

        /* Sign extend 4-bit value */
        if (coeff >= 8) coeff -= 16;
        params->coeffs[i] = (int8_t)coeff;
    }

    #undef READ_BITS
    return byte_pos;
}