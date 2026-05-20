/* opcodec/hdr.c — HDR / color-science utilities
 *
 * Pure math: no codec state, no I/O.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/hdr.h"
#include <math.h>
#include <stdint.h>

/* ---- SMPTE ST 2084 PQ constants ---- */

#define PQ_M1  (2610.0f / 16384.0f)
#define PQ_M2  (2523.0f / 4096.0f * 128.0f)
#define PQ_C1  (3424.0f / 4096.0f)
#define PQ_C2  (2413.0f / 4096.0f * 32.0f)
#define PQ_C3  (2392.0f / 4096.0f * 32.0f)
#define PQ_MAX_NITS 10000.0f

float pq_eotf(float signal)
{
    if (signal <= 0.0f) return 0.0f;
    if (signal >= 1.0f) return PQ_MAX_NITS;

    float e_pow = powf(signal, 1.0f / PQ_M2);
    float num = e_pow - PQ_C1;
    if (num < 0.0f) num = 0.0f;
    float den = PQ_C2 - PQ_C3 * e_pow;
    if (den <= 0.0f) return 0.0f;

    return PQ_MAX_NITS * powf(num / den, 1.0f / PQ_M1);
}

float pq_oetf(float nits)
{
    if (nits <= 0.0f) return 0.0f;
    if (nits >= PQ_MAX_NITS) return 1.0f;

    float Lp = nits / PQ_MAX_NITS;
    float Lp_pow = powf(Lp, PQ_M1);
    float num = PQ_C1 + PQ_C2 * Lp_pow;
    float den = 1.0f + PQ_C3 * Lp_pow;
    return powf(num / den, PQ_M2);
}

/* ---- BT.2100 HLG constants ---- */

#define HLG_A  0.17883277f
#define HLG_B  0.28466892f   /* 1 - 4*A */
#define HLG_C  0.55991073f   /* 0.5 - A*ln(4*A) */

float hlg_oetf(float scene_linear)
{
    if (scene_linear < 0.0f) return 0.0f;
    if (scene_linear <= (1.0f / 12.0f))
        return sqrtf(3.0f * scene_linear);
    return HLG_A * logf(12.0f * scene_linear - HLG_B) + HLG_C;
}

float hlg_eotf(float signal)
{
    if (signal < 0.0f) return 0.0f;
    if (signal <= 0.5f)
        return (signal * signal) / 3.0f;
    return (expf((signal - HLG_C) / HLG_A) + HLG_B) / 12.0f;
}

/* ---- BT.2020 ↔ BT.709 color matrix ---- */

/* BT.2020 → BT.709 (linear light)
 * Derived from: M_709 * M_2020^-1
 * Values from ITU-R BT.2087 Table 1.
 */
void bt2020_to_bt709(float r2020, float g2020, float b2020,
                     float *r709, float *g709, float *b709)
{
    *r709 =  1.6605f * r2020 - 0.5876f * g2020 - 0.0728f * b2020;
    *g709 = -0.1246f * r2020 + 1.1329f * g2020 - 0.0083f * b2020;
    *b709 = -0.0182f * r2020 - 0.1006f * g2020 + 1.1187f * b2020;

    /* soft clip out-of-gamut values */
    if (*r709 < 0.0f) *r709 = 0.0f; else if (*r709 > 1.0f) *r709 = 1.0f;
    if (*g709 < 0.0f) *g709 = 0.0f; else if (*g709 > 1.0f) *g709 = 1.0f;
    if (*b709 < 0.0f) *b709 = 0.0f; else if (*b709 > 1.0f) *b709 = 1.0f;
}

void bt709_to_bt2020(float r709, float g709, float b709,
                     float *r2020, float *g2020, float *b2020)
{
    *r2020 =  0.6274f * r709 + 0.3293f * g709 + 0.0433f * b709;
    *g2020 =  0.0691f * r709 + 0.9195f * g709 + 0.0114f * b709;
    *b2020 =  0.0164f * r709 + 0.0880f * g709 + 0.8956f * b709;

    /* Clamp — BT.2020 is a wider gamut but BT.709 primaries always land inside it;
     * floating-point rounding can still produce tiny negative or >1.0 values. */
    if (*r2020 < 0.0f) *r2020 = 0.0f; else if (*r2020 > 1.0f) *r2020 = 1.0f;
    if (*g2020 < 0.0f) *g2020 = 0.0f; else if (*g2020 > 1.0f) *g2020 = 1.0f;
    if (*b2020 < 0.0f) *b2020 = 0.0f; else if (*b2020 > 1.0f) *b2020 = 1.0f;
}

/* ---- YCbCr ↔ RGB ---- */

/* BT.709 limited-range: Y=[16,235], Cb/Cr=[16,240] */
void yuv_bt709_to_rgb(uint8_t y, uint8_t cb, uint8_t cr,
                      float *r, float *g, float *b)
{
    float Y  = (y  - 16.0f)  / 219.0f;
    float Pb = (cb - 128.0f) / 224.0f;
    float Pr = (cr - 128.0f) / 224.0f;

    *r = Y              + 1.5748f * Pr;
    *g = Y - 0.1873f * Pb - 0.4681f * Pr;
    *b = Y + 1.8556f * Pb;

    if (*r < 0.0f) *r = 0.0f; else if (*r > 1.0f) *r = 1.0f;
    if (*g < 0.0f) *g = 0.0f; else if (*g > 1.0f) *g = 1.0f;
    if (*b < 0.0f) *b = 0.0f; else if (*b > 1.0f) *b = 1.0f;
}

/* BT.2020 limited-range: same quantization, different matrix */
void yuv_bt2020_to_rgb(uint8_t y, uint8_t cb, uint8_t cr,
                       float *r, float *g, float *b)
{
    float Y  = (y  - 16.0f)  / 219.0f;
    float Pb = (cb - 128.0f) / 224.0f;
    float Pr = (cr - 128.0f) / 224.0f;

    *r = Y              + 1.4746f * Pr;
    *g = Y - 0.1645f * Pb - 0.5713f * Pr;
    *b = Y + 1.8814f * Pb;

    if (*r < 0.0f) *r = 0.0f; else if (*r > 1.0f) *r = 1.0f;
    if (*g < 0.0f) *g = 0.0f; else if (*g > 1.0f) *g = 1.0f;
    if (*b < 0.0f) *b = 0.0f; else if (*b > 1.0f) *b = 1.0f;
}

void rgb_to_yuv_bt2020(float r, float g, float b,
                       uint8_t *y, uint8_t *cb, uint8_t *cr)
{
    /* BT.2020 coefficients: Kr=0.2627, Kg=0.6780, Kb=0.0593 */
    float Y  =  0.2627f * r + 0.6780f * g + 0.0593f * b;
    float Pb = (b - Y) / (2.0f * (1.0f - 0.0593f));
    float Pr = (r - Y) / (2.0f * (1.0f - 0.2627f));

    int yi  = (int)(Y  * 219.0f + 16.5f);
    int cbi = (int)(Pb * 224.0f + 128.5f);
    int cri = (int)(Pr * 224.0f + 128.5f);

    *y  = (uint8_t)(yi  < 16 ? 16 : yi  > 235 ? 235 : yi);
    *cb = (uint8_t)(cbi < 16 ? 16 : cbi > 240 ? 240 : cbi);
    *cr = (uint8_t)(cri < 16 ? 16 : cri > 240 ? 240 : cri);
}

/* ---- P010 ↔ YUV420P10 ---- */

void p010_to_yuv420p10(const uint16_t *p010_y, const uint16_t *p010_uv,
                       uint16_t *dst_y, uint16_t *dst_u, uint16_t *dst_v,
                       int width, int height)
{
    int luma_pixels = width * height;
    for (int i = 0; i < luma_pixels; i++)
        dst_y[i] = p010_unpack(p010_y[i]);

    int chroma_pixels = (width / 2) * (height / 2);
    for (int i = 0; i < chroma_pixels; i++) {
        dst_u[i] = p010_unpack(p010_uv[2 * i]);
        dst_v[i] = p010_unpack(p010_uv[2 * i + 1]);
    }
}

void yuv420p10_to_p010(const uint16_t *src_y, const uint16_t *src_u,
                       const uint16_t *src_v,
                       uint16_t *p010_y, uint16_t *p010_uv,
                       int width, int height)
{
    int luma_pixels = width * height;
    for (int i = 0; i < luma_pixels; i++)
        p010_y[i] = p010_pack(src_y[i]);

    int chroma_pixels = (width / 2) * (height / 2);
    for (int i = 0; i < chroma_pixels; i++) {
        p010_uv[2 * i]     = p010_pack(src_u[i]);
        p010_uv[2 * i + 1] = p010_pack(src_v[i]);
    }
}

/* ---- Tone mapping ---- */

float tone_map_knee(float x, float knee_point, float gain_above_knee)
{
    if (x <= 0.0f) return 0.0f;
    if (x <= knee_point) return x;
    float above = (x - knee_point) * gain_above_knee;
    float result = knee_point + above;
    return result > 1.0f ? 1.0f : result;
}
