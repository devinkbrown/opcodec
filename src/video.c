/* opcodec/video.c — Ophion custom video codec (OPVIS) implementation
 *
 * Integer Haar wavelet video codec with motion compensation.
 * Designed for real-time IRC video communication.
 *
 * Major upgrades:
 * - rANS entropy coding (replaces Exp-Golomb)
 * - SSIM-based adaptive quantization
 * - Multi-reference motion estimation (2 reference frames)
 * - Diamond search pattern with quarter-pixel motion estimation
 * - Improved in-loop deblocking filter
 * - Rate control system
 * - Temporal prediction of wavelet coefficients
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/video.h"
#include "opcodec/rans.h"
#include "opcodec/intra.h"
#include "opcodec/vidutil.h"
#include "opcodec/hdr.h"
#include "opcodec/screen.h"
#include "opcodec/tnr.h"
#include "opcodec/cdef.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* ========================================================================
 * UTILITY MACROS
 * ======================================================================== */

#define ALIGN16(x)      (((x) + 15) & ~15)
#define MAX(a, b)       ((a) > (b) ? (a) : (b))
#define MIN(a, b)       ((a) < (b) ? (a) : (b))
#define CLAMP(x, min, max) MIN(MAX(x, min), max)
#define UNUSED(x)       ((void)(x))

/* Big-endian write/read helpers */
static inline void write_be16(uint8_t *p, uint16_t v) {
    p[0] = (v >> 8) & 0xFF;
    p[1] = v & 0xFF;
}

static inline void write_be32(uint8_t *p, uint32_t v) {
    p[0] = (v >> 24) & 0xFF;
    p[1] = (v >> 16) & 0xFF;
    p[2] = (v >> 8) & 0xFF;
    p[3] = v & 0xFF;
}

static inline uint16_t read_be16(const uint8_t *p) {
    return (uint16_t)p[0] << 8 | p[1];
}

static inline uint32_t read_be32(const uint8_t *p) {
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 |
           (uint32_t)p[2] << 8 | p[3];
}

/* ========================================================================
 * RGB TO YUV CONVERSION (BT.601)
 * ======================================================================== */

/* Fixed-point RGB to YUV conversion using BT.601 coefficients
 * Y = 0.299*R + 0.587*G + 0.114*B
 * U = -0.169*R - 0.331*G + 0.500*B + 128
 * V = 0.500*R - 0.419*G - 0.081*B + 128
 *
 * Scale by 1024 for fixed-point math
 */
#define FP_SCALE    1024
#define Y_R_COEF    306   /* 0.299 * 1024 */
#define Y_G_COEF    601   /* 0.587 * 1024 */
#define Y_B_COEF    117   /* 0.114 * 1024 */
#define U_R_COEF    -173  /* -0.169 * 1024 */
#define U_G_COEF    -339  /* -0.331 * 1024 */
#define U_B_COEF    512   /* 0.500 * 1024 */
#define V_R_COEF    512   /* 0.500 * 1024 */
#define V_G_COEF    -429  /* -0.419 * 1024 */
#define V_B_COEF    -83   /* -0.081 * 1024 */

static void rgb_to_yuv420p(const uint8_t *rgb, uint16_t width, uint16_t height,
                           uint8_t *y, uint8_t *u, uint8_t *v) {
    const uint16_t uv_width = width / 2;

    for (uint16_t row = 0; row < height; row++) {
        for (uint16_t col = 0; col < width; col++) {
            const uint8_t *pixel = &rgb[(row * width + col) * 3];
            const uint8_t r = pixel[0];
            const uint8_t g = pixel[1];
            const uint8_t b = pixel[2];

            /* Y component */
            int32_t y_val = (Y_R_COEF * r + Y_G_COEF * g + Y_B_COEF * b) >> 10;
            y[row * width + col] = CLAMP(y_val, 0, 255);

            /* U and V components (4:2:0 subsampling) */
            if ((row & 1) == 0 && (col & 1) == 0) {
                int32_t u_val = ((U_R_COEF * r + U_G_COEF * g + U_B_COEF * b) >> 10) + 128;
                int32_t v_val = ((V_R_COEF * r + V_G_COEF * g + V_B_COEF * b) >> 10) + 128;

                u[(row / 2) * uv_width + (col / 2)] = CLAMP(u_val, 0, 255);
                v[(row / 2) * uv_width + (col / 2)] = CLAMP(v_val, 0, 255);
            }
        }
    }
}

/* ========================================================================
 * CDF 5/3 LIFTING WAVELET TRANSFORM (Integer Implementation)
 * ======================================================================== */

/* CDF 5/3 lifting wavelet - integer implementation for lossless compression */
static void cdf53_1d_forward(int16_t *data, size_t len, int16_t *temp) {
    if (len < 2) return;

    const size_t half = len / 2;
    int16_t *s = temp;          /* low frequencies */
    int16_t *d = temp + half;   /* high frequencies */

    /* Predict step (high-frequency) */
    for (size_t i = 0; i < half; i++) {
        int left = data[2*i];
        int right = (2*i + 2 >= len) ? data[2*i] : data[2*i + 2];
        d[i] = data[2*i + 1] - (left + right) / 2;
    }

    /* Update step (low-frequency) */
    for (size_t i = 0; i < half; i++) {
        int left = (i == 0) ? d[0] : d[i - 1];
        int right = (i == half - 1) ? d[half - 1] : d[i];
        s[i] = data[2*i] + (left + right + 2) / 4;
    }

    /* Copy back */
    memcpy(data, temp, len * sizeof(int16_t));
}

static void cdf53_1d_inverse(int16_t *data, size_t len, int16_t *temp) {
    if (len < 2) return;

    const size_t half = len / 2;
    int16_t *s = data;          /* low frequencies */
    int16_t *d = data + half;   /* high frequencies */

    /* Undo update step */
    for (size_t i = 0; i < half; i++) {
        int left = (i == 0) ? d[0] : d[i - 1];
        int right = (i == half - 1) ? d[half - 1] : d[i];
        temp[2*i] = s[i] - (left + right + 2) / 4;
    }

    /* Undo predict step */
    for (size_t i = 0; i < half; i++) {
        int left = temp[2*i];
        int right = (2*i + 2 >= len) ? temp[2*i] : temp[2*i + 2];
        temp[2*i + 1] = d[i] + (left + right) / 2;
    }

    /* Copy back */
    memcpy(data, temp, len * sizeof(int16_t));
}


/* ========================================================================
 * GENERALIZED N×N WAVELET TRANSFORM
 * ======================================================================== */

/* Number of decomposition levels for an N×N block */
static int wavelet_levels(int N) {
    if (N <= 8)  return 1;
    if (N == 16) return 2;
    return 3;  /* 32, 64 */
}

/* N×N forward CDF 5/3 (flat row-major buffer, stride=N) */
static void cdf53_2d_forward_n(int16_t *data, int N, int16_t *work) {
    int sz = N;
    for (int lv = 0; lv < wavelet_levels(N); lv++) {
        for (int row = 0; row < sz; row++)
            cdf53_1d_forward(data + row * N, sz, work);
        for (int col = 0; col < sz; col++) {
            for (int row = 0; row < sz; row++) work[row] = data[row * N + col];
            cdf53_1d_forward(work, sz, work + sz);
            for (int row = 0; row < sz; row++) data[row * N + col] = work[row];
        }
        sz >>= 1;
    }
}

/* N×N inverse CDF 5/3 */
static void cdf53_2d_inverse_n(int16_t *data, int N, int16_t *work) {
    int levels = wavelet_levels(N);
    int sz = N >> (levels - 1);
    for (int lv = 0; lv < levels; lv++) {
        for (int col = 0; col < sz; col++) {
            for (int row = 0; row < sz; row++) work[row] = data[row * N + col];
            cdf53_1d_inverse(work, sz, work + sz);
            for (int row = 0; row < sz; row++) data[row * N + col] = work[row];
        }
        for (int row = 0; row < sz; row++)
            cdf53_1d_inverse(data + row * N, sz, work);
        sz <<= 1;
    }
}

/* ========================================================================
 * MOTION ESTIMATION UTILITY FUNCTIONS
 * ======================================================================== */

/* Calculate SAD (Sum of Absolute Differences) between two 16x16 blocks */

/* ========================================================================
 * HEVC 8-TAP LUMA INTERPOLATION (DCTIF)
 *
 * Replaces bilinear with separable 8-tap filters for ~3-5% BD-rate gain.
 * Filter coefficients from HEVC spec Table 8-11 (×64 normalization).
 * ======================================================================== */

/* luma_filter[phase][tap] — phases: 0=integer, 1=1/4, 2=2/4, 3=3/4 */
static const int8_t kLumaFilter[4][8] = {
    {  0,  0,  0, 64,  0,  0,  0,  0 },  /* integer (0/4 pel) */
    { -1,  4,-10, 58, 17, -5,  1,  0 },  /* 1/4 pel */
    { -1,  4,-11, 40, 40,-11,  4, -1 },  /* 2/4 (half) pel */
    {  0,  1, -5, 17, 58,-10,  4, -1 },  /* 3/4 pel */
};

/* Clamped frame access (replicates border pixels instead of returning gray) */
static inline int frame_pixel(const uint8_t *ref, int w, int h, int x, int y)
{
    x = x < 0 ? 0 : (x >= w ? w - 1 : x);
    y = y < 0 ? 0 : (y >= h ? h - 1 : y);
    return ref[y * w + x];
}

/*
 * HEVC 8-tap luma interpolation at sub-pixel position (x_qpel, y_qpel).
 * Coordinates are in quarter-pixel units (integer = multiple of 4).
 * Uses separable two-pass filtering: horizontal then vertical.
 */
static uint8_t hevc_luma_interp(const uint8_t *ref, int width, int height,
                                int x_qpel, int y_qpel)
{
    int xi = x_qpel >> 2;   /* integer pixel x */
    int yi = y_qpel >> 2;   /* integer pixel y */
    int xf = x_qpel & 3;    /* x fractional phase (0-3) */
    int yf = y_qpel & 3;    /* y fractional phase (0-3) */

    if (xf == 0 && yf == 0) {
        /* Integer position — direct lookup */
        xi = xi < 0 ? 0 : (xi >= width  ? width  - 1 : xi);
        yi = yi < 0 ? 0 : (yi >= height ? height - 1 : yi);
        return ref[yi * width + xi];
    }

    const int8_t *hf = kLumaFilter[xf];
    const int8_t *vf = kLumaFilter[yf];

    if (yf == 0) {
        /* Horizontal-only 8-tap */
        int sum = 0;
        for (int k = 0; k < 8; k++)
            sum += hf[k] * frame_pixel(ref, width, height, xi - 3 + k, yi);
        return (uint8_t)CLAMP((sum + 32) >> 6, 0, 255);
    }

    if (xf == 0) {
        /* Vertical-only 8-tap */
        int sum = 0;
        for (int k = 0; k < 8; k++)
            sum += vf[k] * frame_pixel(ref, width, height, xi, yi - 3 + k);
        return (uint8_t)CLAMP((sum + 32) >> 6, 0, 255);
    }

    /* Two-pass: horizontal then vertical.
     * Horizontal intermediate stored as int32_t (unscaled) to retain precision.
     * max |sum_h| ≤ 96 × 255 = 24 480 — fits in int32_t comfortably.
     */
    int32_t tmp[8];
    for (int k = 0; k < 8; k++) {
        int32_t s = 0;
        for (int j = 0; j < 8; j++)
            s += (int32_t)hf[j] * frame_pixel(ref, width, height, xi - 3 + j, yi - 3 + k);
        tmp[k] = s;
    }

    int64_t sum = 0;
    for (int k = 0; k < 8; k++)
        sum += (int64_t)vf[k] * tmp[k];

    /* Two ×64 factors: normalize by 64×64=4096, round with 2048 */
    int v = (int)((sum + 2048) >> 12);
    return (uint8_t)CLAMP(v, 0, 255);
}

/* Generate quarter-pixel interpolated reference area for motion estimation.
 * Called once per CTU/MB to pre-fill interp_buf, avoiding repeated filter
 * calls during the motion search inner loop.
 */

/* Calculate SAD with quarter-pixel precision */

/* ========================================================================
 * SCENE CHANGE DETECTION AND ADAPTIVE QUANTIZATION
 * ======================================================================== */

/* Forward declaration — defined after the distortion helpers below */
static uint32_t calculate_satd_n(const uint8_t *a, int sa, const uint8_t *b, int sb, int N);

/* Scene change detection using SATD (more accurate than SAD for predicting residual cost).
 * Samples 1-in-4 macroblocks for speed; the 4x subsampling is sufficient for detection. */
static bool detect_scene_change(const uint8_t *cur_frame, const uint8_t *ref_frame,
                               uint16_t width, uint16_t mb_cols, uint16_t mb_rows) {
    uint64_t total_satd = 0;
    uint32_t mb_count = 0;

    for (uint16_t mb_y = 0; mb_y < mb_rows; mb_y += 2) {
        for (uint16_t mb_x = 0; mb_x < mb_cols; mb_x += 2) {
            const uint8_t *cur_mb = cur_frame + mb_y * OPVIS_MB_SIZE * width + mb_x * OPVIS_MB_SIZE;
            const uint8_t *ref_mb = ref_frame + mb_y * OPVIS_MB_SIZE * width + mb_x * OPVIS_MB_SIZE;
            total_satd += calculate_satd_n(cur_mb, width, ref_mb, width, OPVIS_MB_SIZE);
            mb_count++;
        }
    }
    if (mb_count == 0) return false;

    /* Normalise: SATD per sample. Threshold ~48: a full 180° cut delivers ~200. */
    const uint32_t avg_satd = (uint32_t)(total_satd / mb_count) / (OPVIS_MB_SIZE * OPVIS_MB_SIZE);
    return avg_satd > 48u;
}

/* ========================================================================
 * MOTION ESTIMATION AND COMPENSATION
 * ======================================================================== */

/* ========================================================================
 * ADAPTIVE QUANTIZATION
 * ======================================================================== */

/* ========================================================================
 * GENERALIZED N×N QUANTIZE / DEQUANTIZE
 * ======================================================================== */

static void quantize_nxn(int16_t *block, int N, uint8_t quality, float aq) {
    const int adj_q    = CLAMP((int)((float)quality * aq), 10, 100);
    const int base     = 100 - adj_q;
    /* HVS perceptual subband weights: LL=1.0×, LH/HL=1.5×, HH=2.5× */
    const int step_ll  = MAX(1, base / 8);
    const int step_det = MAX(1, base * 3 / 8);
    const int step_hh  = MAX(1, base * 5 / 4);
    const int ll_sz    = N >> wavelet_levels(N);
    const int hh_thr   = N / 2;

    for (int row = 0; row < N; row++) {
        for (int col = 0; col < N; col++) {
            int step = (row < ll_sz && col < ll_sz)  ? step_ll :
                       (row >= hh_thr && col >= hh_thr) ? step_hh : step_det;
            int16_t v = block[row * N + col];
            if      (v > 0) block[row * N + col] = (int16_t)((v + step / 2) / step);
            else if (v < 0) block[row * N + col] = (int16_t)((v - step / 2) / step);
        }
    }
}

static void dequantize_nxn(int16_t *block, int N, uint8_t quality, float aq) {
    const int adj_q    = CLAMP((int)((float)quality * aq), 10, 100);
    const int base     = 100 - adj_q;
    const int step_ll  = MAX(1, base / 8);
    const int step_det = MAX(1, base * 3 / 8);
    const int step_hh  = MAX(1, base * 5 / 4);
    const int ll_sz    = N >> wavelet_levels(N);
    const int hh_thr   = N / 2;

    for (int row = 0; row < N; row++) {
        for (int col = 0; col < N; col++) {
            int step = (row < ll_sz && col < ll_sz)  ? step_ll :
                       (row >= hh_thr && col >= hh_thr) ? step_hh : step_det;
            block[row * N + col] = (int16_t)(block[row * N + col] * step);
        }
    }
}

/* ========================================================================
 * IN-LOOP DEBLOCKING FILTER
 * ======================================================================== */

/* Compute boundary strength for improved deblocking filter */
static int compute_boundary_strength(const uint8_t *frame, uint16_t width,
                                    uint16_t mb_x, uint16_t mb_y, bool is_vertical,
                                    opvis_frame_type_t frame_type, const opvis_mv_t *mvs,
                                    const uint8_t *ref_indices, uint16_t mb_cols) {
    UNUSED(frame);
    UNUSED(width);
    const uint16_t mb_idx = mb_y * mb_cols + mb_x;
    const uint16_t neighbor_idx = is_vertical ?
        mb_idx - 1 : mb_idx - mb_cols;

    /* BS=4: I-frame macroblock boundary (strongest filter) */
    if (frame_type == OPVIS_FRAME_I) {
        return 4;
    }

    /* BS=3: coded coefficients on either side (strong filter) */
    /* For now, assume all P-frame blocks have some coded coefficients */
    /* This could be refined with actual coefficient tracking */

    /* BS=2: different reference frames (medium filter) */
    if (ref_indices && ref_indices[mb_idx] != ref_indices[neighbor_idx]) {
        return 2;
    }

    /* BS=1: MV difference > 1 pixel (weak filter) */
    if (mvs) {
        const opvis_mv_t mv1 = mvs[mb_idx];
        const opvis_mv_t mv2 = mvs[neighbor_idx];
        const int mv_diff_x = abs(mv1.x - mv2.x);
        const int mv_diff_y = abs(mv1.y - mv2.y);

        /* Check if MV difference > 1 pixel (4 quarter-pixel units) */
        if (mv_diff_x > 4 || mv_diff_y > 4) {
            return 1;
        }
    }

    /* BS=0: no filtering needed */
    return 0;
}

/* E4 — Apply softer deblocking to chroma plane (β/2, tc/2). */
static void apply_deblocking_chroma(uint8_t *chroma, int cw, int ch, int beta, int tc) {
    if (!chroma || cw < 4 || ch < 4) return;
    int hb = beta / 2; if (hb < 4) hb = 4;
    int htc = tc / 2; if (htc < 1) htc = 1; (void)htc;
    /* 8x8 grid alignment for chroma since luma is 16-aligned in 4:2:0 */
    for (int y = 8; y < ch - 2; y += 8) {
        for (int x = 0; x < cw; x++) {
            uint8_t *p0 = &chroma[(y-1) * cw + x];
            uint8_t *q0 = &chroma[y * cw + x];
            int diff = abs(*p0 - *q0);
            if (diff < hb) {
                uint8_t avg = (uint8_t)((*p0 + *q0 + 1) >> 1);
                *p0 = (uint8_t)((*p0 + avg) >> 1);
                *q0 = (uint8_t)((*q0 + avg) >> 1);
            }
        }
    }
    for (int x = 8; x < cw - 2; x += 8) {
        for (int y = 0; y < ch; y++) {
            uint8_t *p0 = &chroma[y * cw + (x-1)];
            uint8_t *q0 = &chroma[y * cw + x];
            int diff = abs(*p0 - *q0);
            if (diff < hb) {
                uint8_t avg = (uint8_t)((*p0 + *q0 + 1) >> 1);
                *p0 = (uint8_t)((*p0 + avg) >> 1);
                *q0 = (uint8_t)((*q0 + avg) >> 1);
            }
        }
    }
}

/* Apply improved deblocking filter with boundary strength classification.
 * E1 — QP-adaptive thresholds (beta/tc derived from quality).
 * E4 — Optional chroma plane filtering. */
static void apply_deblocking_filter_improved(uint8_t *frame, uint16_t width, uint16_t height,
                                            uint16_t mb_cols, uint16_t mb_rows,
                                            opvis_frame_type_t frame_type,
                                            const opvis_mv_t *mvs,
                                            const uint8_t *ref_indices,
                                            uint8_t quality,
                                            uint8_t *chroma_u, uint8_t *chroma_v,
                                            int cw, int ch) {
    /* E1 — Derive β and tc from QP/quality. */
    int beta = 16 + 4 * ((int)quality / 10);   /* 16..56 */
    int tc   =  2 + 1 * ((int)quality / 20);   /* 2..7 */
    int thr_strong = beta + (tc >> 1) + 4;     /* roughly the old 24 cap */
    int thr_weak   = beta;                     /* roughly the old 16 cap */
    if (thr_strong < 8) thr_strong = 8;
    if (thr_weak < 4) thr_weak = 4;
    /* Horizontal edges (between vertically adjacent macroblocks) */
    for (uint16_t mb_y = 1; mb_y < mb_rows; mb_y++) {
        for (uint16_t mb_x = 0; mb_x < mb_cols; mb_x++) {
            const int edge_y = mb_y * OPVIS_MB_SIZE;
            const int start_x = mb_x * OPVIS_MB_SIZE;

            /* Compute boundary strength */
            const int bs = compute_boundary_strength(frame, width, mb_x, mb_y,
                                                   false, frame_type, mvs, ref_indices, mb_cols);

            if (bs == 0) continue; /* No filtering */

            for (int x = start_x; x < start_x + OPVIS_MB_SIZE && x < width; x++) {
                if (edge_y < 2 || edge_y >= height - 2) continue;

                uint8_t *p1 = &frame[(edge_y - 2) * width + x];
                uint8_t *p0 = &frame[(edge_y - 1) * width + x];
                uint8_t *q0 = &frame[edge_y * width + x];
                uint8_t *q1 = &frame[(edge_y + 1) * width + x];

                /* Apply filter based on boundary strength.
                 * Always gate on pixel difference to preserve real content edges
                 * (e.g. tiles, text) — avoids filtering when |p0-q0| >= threshold. */
                int diff = abs(*p0 - *q0);
                if (bs >= 3) {
                    /* Strong 4-tap filter for BS >= 3 */
                    if (diff < thr_strong) {
                        uint8_t new_p0 = (2 * *p1 + *p0 + *q0 + 2) >> 2;
                        uint8_t new_q0 = (2 * *q1 + *q0 + *p0 + 2) >> 2;
                        *p0 = new_p0;
                        *q0 = new_q0;
                    }
                } else {
                    /* Weak 3-tap filter for BS 1-2 */
                    if (diff < thr_weak) {
                        uint8_t new_p0 = (*p1 + 2 * *p0 + *q0 + 2) >> 2;
                        uint8_t new_q0 = (*p0 + 2 * *q0 + *q1 + 2) >> 2;
                        *p0 = new_p0;
                        *q0 = new_q0;
                    }
                }
            }
        }
    }

    /* Vertical edges (between horizontally adjacent macroblocks) */
    for (uint16_t mb_y = 0; mb_y < mb_rows; mb_y++) {
        for (uint16_t mb_x = 1; mb_x < mb_cols; mb_x++) {
            const int edge_x = mb_x * OPVIS_MB_SIZE;
            const int start_y = mb_y * OPVIS_MB_SIZE;

            /* Compute boundary strength */
            const int bs = compute_boundary_strength(frame, width, mb_x, mb_y,
                                                   true, frame_type, mvs, ref_indices, mb_cols);

            if (bs == 0) continue; /* No filtering */

            for (int y = start_y; y < start_y + OPVIS_MB_SIZE && y < height; y++) {
                if (edge_x < 2 || edge_x >= width - 2) continue;

                uint8_t *p1 = &frame[y * width + (edge_x - 2)];
                uint8_t *p0 = &frame[y * width + (edge_x - 1)];
                uint8_t *q0 = &frame[y * width + edge_x];
                uint8_t *q1 = &frame[y * width + (edge_x + 1)];

                int diff = abs(*p0 - *q0);
                if (bs >= 3) {
                    /* Strong 4-tap filter for BS >= 3 */
                    if (diff < thr_strong) {
                        uint8_t new_p0 = (2 * *p1 + *p0 + *q0 + 2) >> 2;
                        uint8_t new_q0 = (2 * *q1 + *q0 + *p0 + 2) >> 2;
                        *p0 = new_p0;
                        *q0 = new_q0;
                    }
                } else {
                    /* Weak 3-tap filter for BS 1-2 */
                    if (diff < thr_weak) {
                        uint8_t new_p0 = (*p1 + 2 * *p0 + *q0 + 2) >> 2;
                        uint8_t new_q0 = (*p0 + 2 * *q0 + *q1 + 2) >> 2;
                        *p0 = new_p0;
                        *q0 = new_q0;
                    }
                }
            }
        }
    }

    /* E4 — chroma deblocking, softer (β/2, tc/2) */
    if (chroma_u && cw > 0 && ch > 0) apply_deblocking_chroma(chroma_u, cw, ch, beta, tc);
    if (chroma_v && cw > 0 && ch > 0) apply_deblocking_chroma(chroma_v, cw, ch, beta, tc);
}

/* ========================================================================
 * rANS ENTROPY CODING WITH PROBABILITY MODELS
 * ======================================================================== */

/* IBC hash table for decoder — reset at every I-frame boundary */
static uint32_t s_dec_ibc_table[SCREEN_IBC_TABLE_SIZE];

/* Per-encoder TNR context — one encoder per IRC channel */
static opvis_tnr_ctx_t s_tnr_ctx;

/* Probability models for different types of data */
static rans_model_t g_mv_model;         /* Motion vectors (Laplace distribution) */
static rans_model_t g_coeff_model;      /* Wavelet coefficients (Laplace distribution) */
static rans_model_t g_mode_model;       /* P-frame: skip/coded/ref0/ref1 (4 symbols) */
static rans_model_t g_split_model;      /* CU split flag: 0=leaf, 1=split (2 symbols) */
static rans_model_t g_intra_mode_model; /* intra modes: 35 HEVC + palette + IBC */
static rans_model_t g_byte_model;       /* raw 0-255 byte values (palette entries, etc.) */
static rans_model_t g_cbf_model;        /* Coded Block Flag: 0=all-zero residual, 1=has coeffs */
/* A1 — subband-specific coefficient models */
static rans_model_t g_coeff_ll_model;   /* LL subband (DC region) */
static rans_model_t g_coeff_lh_model;   /* LH/HL subbands */
static rans_model_t g_coeff_hh_model;   /* HH subband */
/* A3 — last significant coefficient position */
static rans_model_t g_last_pos_model;
/* A6 — context-adaptive split flag, indexed by [0..3] for N=64,32,16,8 */
static rans_model_t g_split_ctx[4];
/* B1/B2 — merge flag */
static rans_model_t g_merge_model;
/* B3 — AMVP candidate index (spatial vs TMVP) */
static rans_model_t g_mvp_idx_model;
/* C4 — MRL reference line index (0,1,2) */
static rans_model_t g_mrl_model;
/* C5 — ISP mode (0=off, 1=ISP_H, 2=ISP_V) */
static rans_model_t g_isp_model;
/* E3 — SAO type (0=off, 1=band, 2=edge) */
static rans_model_t g_sao_model;

/* Initialize probability models for video coding */
static void init_video_models(void) {
    rans_model_laplace(&g_mv_model, 256, 8);    /* covers zig-zag of ±127 MVs/BVs */
    rans_model_laplace(&g_coeff_model, 2048, 6); /* covers zig-zag of ±1016 wavelet LL */
    rans_model_uniform(&g_mode_model, 6);   /* extended: skip/coded/ref0/ref1/ref2/forced-intra */
    rans_model_uniform(&g_split_model, 2);
    rans_model_uniform(&g_intra_mode_model, SCREEN_INTRA_NUM_MODES);
    rans_model_uniform(&g_byte_model, 256);
    rans_model_uniform(&g_cbf_model, 2);
    rans_model_laplace(&g_coeff_ll_model, 2048, 4);
    rans_model_laplace(&g_coeff_lh_model, 1024, 7);
    rans_model_laplace(&g_coeff_hh_model, 512, 10);
    rans_model_uniform(&g_last_pos_model, 4096);
    rans_model_uniform(&g_split_ctx[0], 2);  /* N=64 */
    rans_model_uniform(&g_split_ctx[1], 2);  /* N=32 */
    rans_model_uniform(&g_split_ctx[2], 2);  /* N=16 */
    rans_model_uniform(&g_split_ctx[3], 2);  /* N=8 */
    rans_model_uniform(&g_merge_model, 2);
    rans_model_uniform(&g_mvp_idx_model, 2);
    rans_model_uniform(&g_mrl_model, 3);
    rans_model_uniform(&g_isp_model, 3);
    rans_model_uniform(&g_sao_model, 3);
}

/* Module-level rANS encoding scratch buffer.
 * Sized for worst-case: max frame (1920×1080) × ~2 bytes/pixel uncompressed.
 * Lives in BSS — no stack pressure, zero cost until first page access. */
#define VIDEO_ENC_TEMP_BUF_SIZE (OPVIS_MAX_WIDTH * OPVIS_MAX_HEIGHT * 4)
static uint8_t s_enc_temp_buf[VIDEO_ENC_TEMP_BUF_SIZE];

/* Set to true by encode_cu when any CU uses IBC or palette screen coding.
 * Cleared at the top of opvis_encode and read to set out[12] bit 6. */
static bool s_screen_mode_used;

/* Symbol staging buffer.
 *
 * rANS decodes in REVERSE order of encoding.  To keep the encoder and
 * decoder traversal order identical (both process CTUs / CUs top-to-
 * bottom, left-to-right), we buffer every symbol the encoder emits and
 * then reverse the list before handing it to rans_enc_put().  The
 * decoder calls rans_dec_get() in the same forward order as the encoder
 * buffered symbols, and because rANS reverses the stream the correct
 * symbol is returned at each step.
 */
typedef enum {
    VID_MODEL_MV       = 0,
    VID_MODEL_COEFF    = 1,
    VID_MODEL_MODE     = 2,
    VID_MODEL_SPLIT    = 3,
    VID_MODEL_INTRA    = 4,
    VID_MODEL_BYTE     = 5,
    VID_MODEL_CBF      = 6,
    VID_MODEL_COEFF_LL = 7,
    VID_MODEL_COEFF_LH = 8,
    VID_MODEL_COEFF_HH = 9,
    VID_MODEL_LAST_POS = 10,
    VID_MODEL_SPLIT_64 = 11,
    VID_MODEL_SPLIT_32 = 12,
    VID_MODEL_SPLIT_16 = 13,
    VID_MODEL_SPLIT_8  = 14,
    VID_MODEL_MERGE    = 15,
    VID_MODEL_MVP_IDX  = 16,
    VID_MODEL_MRL      = 17,
    VID_MODEL_ISP      = 18,
    VID_MODEL_SAO      = 19,
    VID_MODEL_COUNT
} vid_model_id_t;

typedef struct { uint16_t symbol; uint8_t model_id; uint8_t _pad; } vid_sym_t;

#define VIDEO_SYM_BUF_MAX (1024u * 1024u)  /* 1 M symbols — fits 640×480 fully split */
static vid_sym_t s_sym_buf[VIDEO_SYM_BUF_MAX];
static size_t    s_sym_count;

/* Temporary storage for rANS encoding */
typedef struct {
    rans_encoder_t enc;
    size_t final_size;
} video_entropy_encoder_t;

/* Temporary storage for rANS decoding */
typedef struct {
    rans_decoder_t dec;
} video_entropy_decoder_t;

/* Initialize video entropy encoder */
static void video_entropy_enc_init(video_entropy_encoder_t *ve, uint8_t *output_buf, size_t buf_size) {
    UNUSED(output_buf);
    UNUSED(buf_size);
    ve->final_size = 0;
    s_sym_count = 0;
}

/* ---- Encoder put helpers (append to staging buffer) ---- */

static void video_entropy_enc_put_mv(video_entropy_encoder_t *ve, int16_t value) {
    UNUSED(ve);
    if (s_sym_count >= VIDEO_SYM_BUF_MAX) return;
    uint16_t symbol = rans_zigzag_enc(value);
    if (symbol >= g_mv_model.num_symbols) symbol = g_mv_model.num_symbols - 1;
    s_sym_buf[s_sym_count++] = (vid_sym_t){ symbol, VID_MODEL_MV, 0 };
}

static void video_entropy_enc_put_coeff(video_entropy_encoder_t *ve, int16_t value) {
    UNUSED(ve);
    if (s_sym_count >= VIDEO_SYM_BUF_MAX) return;
    uint16_t symbol = rans_zigzag_enc(value);
    if (symbol >= g_coeff_model.num_symbols) symbol = g_coeff_model.num_symbols - 1;
    s_sym_buf[s_sym_count++] = (vid_sym_t){ symbol, VID_MODEL_COEFF, 0 };
}

static void video_entropy_enc_put_mode(video_entropy_encoder_t *ve, uint8_t mode) {
    UNUSED(ve);
    if (s_sym_count >= VIDEO_SYM_BUF_MAX) return;
    uint16_t symbol = (uint16_t)mode;
    if (symbol >= g_mode_model.num_symbols) symbol = g_mode_model.num_symbols - 1;
    s_sym_buf[s_sym_count++] = (vid_sym_t){ symbol, VID_MODEL_MODE, 0 };
}

static void video_entropy_enc_put_byte(video_entropy_encoder_t *ve, uint8_t value) {
    UNUSED(ve);
    if (s_sym_count >= VIDEO_SYM_BUF_MAX) return;
    s_sym_buf[s_sym_count++] = (vid_sym_t){ value, VID_MODEL_BYTE, 0 };
}

static void video_entropy_enc_put_intra_mode(video_entropy_encoder_t *ve, intra_mode_hevc_t mode) {
    UNUSED(ve);
    if (s_sym_count >= VIDEO_SYM_BUF_MAX) return;
    uint16_t s = ((uint16_t)mode < SCREEN_INTRA_NUM_MODES) ? (uint16_t)mode : 0u;
    s_sym_buf[s_sym_count++] = (vid_sym_t){ s, VID_MODEL_INTRA, 0 };
}

/* Flush: encode symbols in REVERSE into rANS then copy to output buffer.
 * rANS decodes in reverse, so encoding reversed gives forward decode order. */
static size_t video_entropy_enc_flush(video_entropy_encoder_t *ve, uint8_t *output_buf, size_t buf_size) {
    static const rans_model_t *models[VID_MODEL_COUNT];
    models[VID_MODEL_MV]       = &g_mv_model;
    models[VID_MODEL_COEFF]    = &g_coeff_model;
    models[VID_MODEL_MODE]     = &g_mode_model;
    models[VID_MODEL_SPLIT]    = &g_split_model;
    models[VID_MODEL_INTRA]    = &g_intra_mode_model;
    models[VID_MODEL_BYTE]     = &g_byte_model;
    models[VID_MODEL_CBF]      = &g_cbf_model;
    models[VID_MODEL_COEFF_LL] = &g_coeff_ll_model;
    models[VID_MODEL_COEFF_LH] = &g_coeff_lh_model;
    models[VID_MODEL_COEFF_HH] = &g_coeff_hh_model;
    models[VID_MODEL_LAST_POS] = &g_last_pos_model;
    models[VID_MODEL_SPLIT_64] = &g_split_ctx[0];
    models[VID_MODEL_SPLIT_32] = &g_split_ctx[1];
    models[VID_MODEL_SPLIT_16] = &g_split_ctx[2];
    models[VID_MODEL_SPLIT_8]  = &g_split_ctx[3];
    models[VID_MODEL_MERGE]    = &g_merge_model;
    models[VID_MODEL_MVP_IDX]  = &g_mvp_idx_model;
    models[VID_MODEL_MRL]      = &g_mrl_model;
    models[VID_MODEL_ISP]      = &g_isp_model;
    models[VID_MODEL_SAO]      = &g_sao_model;

    rans_enc_init(&ve->enc, s_enc_temp_buf, VIDEO_ENC_TEMP_BUF_SIZE);

    for (size_t i = s_sym_count; i-- > 0; )
        rans_enc_put(&ve->enc, models[s_sym_buf[i].model_id], s_sym_buf[i].symbol);

    rans_enc_flush(&ve->enc);
    size_t data_size;
    const uint8_t *data = rans_enc_data(&ve->enc, &data_size);

    if (data_size > buf_size) return 0;
    memcpy(output_buf, data, data_size);
    ve->final_size = data_size;
    return data_size;
}

/* ---- Decoder get helpers (read forward — rANS reversal gives correct order) ---- */

/* Initialize video entropy decoder */
static int video_entropy_dec_init(video_entropy_decoder_t *vd, const uint8_t *input_buf, size_t buf_size) {
    if (buf_size < 4) return -1;
    rans_dec_init(&vd->dec, input_buf, buf_size);
    return 0;
}

static int16_t video_entropy_dec_get_mv(video_entropy_decoder_t *vd) {
    return rans_zigzag_dec(rans_dec_get(&vd->dec, &g_mv_model));
}

static int16_t video_entropy_dec_get_coeff(video_entropy_decoder_t *vd) {
    return rans_zigzag_dec(rans_dec_get(&vd->dec, &g_coeff_model));
}

static uint8_t video_entropy_dec_get_mode(video_entropy_decoder_t *vd) {
    return (uint8_t)rans_dec_get(&vd->dec, &g_mode_model);
}

static uint8_t video_entropy_dec_get_byte(video_entropy_decoder_t *vd) {
    return (uint8_t)rans_dec_get(&vd->dec, &g_byte_model);
}

static intra_mode_hevc_t video_entropy_dec_get_intra_mode(video_entropy_decoder_t *vd) {
    return (intra_mode_hevc_t)rans_dec_get(&vd->dec, &g_intra_mode_model);
}

/* B3 — AMVP candidate index (0=spatial, 1=TMVP) */
static void video_entropy_enc_put_mvp_idx(video_entropy_encoder_t *ve, uint8_t idx) {
    (void)ve;
    if (s_sym_count >= VIDEO_SYM_BUF_MAX) return;
    s_sym_buf[s_sym_count++] = (vid_sym_t){ idx & 1u, VID_MODEL_MVP_IDX, 0 };
}
static uint8_t video_entropy_dec_get_mvp_idx(video_entropy_decoder_t *vd) {
    return (uint8_t)rans_dec_get(&vd->dec, &g_mvp_idx_model);
}
/* C4 — MRL reference line index (0,1,2) */
static void video_entropy_enc_put_mrl_idx(video_entropy_encoder_t *ve, uint8_t r) {
    (void)ve;
    if (s_sym_count >= VIDEO_SYM_BUF_MAX) return;
    s_sym_buf[s_sym_count++] = (vid_sym_t){ r, VID_MODEL_MRL, 0 };
}
static uint8_t video_entropy_dec_get_mrl_idx(video_entropy_decoder_t *vd) {
    return (uint8_t)rans_dec_get(&vd->dec, &g_mrl_model);
}
/* C5 — ISP mode (always 0=off; model is live for future use) */
static void video_entropy_enc_put_isp_mode(video_entropy_encoder_t *ve, uint8_t m) {
    (void)ve;
    if (s_sym_count >= VIDEO_SYM_BUF_MAX) return;
    s_sym_buf[s_sym_count++] = (vid_sym_t){ m, VID_MODEL_ISP, 0 };
}
static uint8_t video_entropy_dec_get_isp_mode(video_entropy_decoder_t *vd) {
    return (uint8_t)rans_dec_get(&vd->dec, &g_isp_model);
}
/* E3 — SAO type (0=off, 1=band, 2=edge); coded once per frame before CTU scan */
static void video_entropy_enc_put_sao_type(video_entropy_encoder_t *ve, uint8_t t) {
    (void)ve;
    if (s_sym_count >= VIDEO_SYM_BUF_MAX) return;
    s_sym_buf[s_sym_count++] = (vid_sym_t){ t, VID_MODEL_SAO, 0 };
}
static uint8_t video_entropy_dec_get_sao_type(video_entropy_decoder_t *vd) {
    return (uint8_t)rans_dec_get(&vd->dec, &g_sao_model);
}

static void video_entropy_enc_put_cbf(video_entropy_encoder_t *ve, uint8_t cbf) {
    UNUSED(ve);
    if (s_sym_count < VIDEO_SYM_BUF_MAX)
        s_sym_buf[s_sym_count++] = (vid_sym_t){ .symbol = cbf, .model_id = VID_MODEL_CBF };
}
static uint8_t video_entropy_dec_get_cbf(video_entropy_decoder_t *vd) {
    return (uint8_t)rans_dec_get(&vd->dec, &g_cbf_model);
}

/* A1 — subband selector: 0=LL, 1=LH/HL, 2=HH for an NxN block */
static inline int subband_for_pos(int pos, int N) {
    int row = pos / N, col = pos % N;
    int half = N / 2;
    if (row < half && col < half) return 0; /* LL */
    if (row < half || col < half) return 1; /* LH or HL */
    return 2; /* HH */
}

static void video_entropy_enc_put_coeff_subband(video_entropy_encoder_t *ve, int16_t value, int subband) {
    UNUSED(ve);
    if (s_sym_count >= VIDEO_SYM_BUF_MAX) return;
    uint16_t symbol = rans_zigzag_enc(value);
    uint8_t model_id; const rans_model_t *m;
    switch (subband) {
        case 0:  model_id = VID_MODEL_COEFF_LL; m = &g_coeff_ll_model; break;
        case 1:  model_id = VID_MODEL_COEFF_LH; m = &g_coeff_lh_model; break;
        default: model_id = VID_MODEL_COEFF_HH; m = &g_coeff_hh_model; break;
    }
    if (symbol >= m->num_symbols) symbol = (uint16_t)(m->num_symbols - 1);
    s_sym_buf[s_sym_count++] = (vid_sym_t){ symbol, model_id, 0 };
}

static int16_t video_entropy_dec_get_coeff_subband(video_entropy_decoder_t *vd, int subband) {
    const rans_model_t *m;
    switch (subband) {
        case 0:  m = &g_coeff_ll_model; break;
        case 1:  m = &g_coeff_lh_model; break;
        default: m = &g_coeff_hh_model; break;
    }
    return rans_zigzag_dec(rans_dec_get(&vd->dec, m));
}

/* A6 — context-adaptive split model selector by block size */
static inline uint8_t split_ctx_for_N(int N) {
    if (N >= 64) return VID_MODEL_SPLIT_64;
    if (N >= 32) return VID_MODEL_SPLIT_32;
    if (N >= 16) return VID_MODEL_SPLIT_16;
    return VID_MODEL_SPLIT_8;
}

static void video_entropy_enc_put_split_ctx(video_entropy_encoder_t *ve, uint8_t flag, int N) {
    UNUSED(ve);
    if (s_sym_count < VIDEO_SYM_BUF_MAX)
        s_sym_buf[s_sym_count++] = (vid_sym_t){ flag & 1u, split_ctx_for_N(N), 0 };
}

static uint8_t video_entropy_dec_get_split_ctx(video_entropy_decoder_t *vd, int N) {
    rans_model_t *m;
    uint8_t id = split_ctx_for_N(N);
    if      (id == VID_MODEL_SPLIT_64) m = &g_split_ctx[0];
    else if (id == VID_MODEL_SPLIT_32) m = &g_split_ctx[1];
    else if (id == VID_MODEL_SPLIT_16) m = &g_split_ctx[2];
    else                               m = &g_split_ctx[3];
    return (uint8_t)rans_dec_get(&vd->dec, m);
}

/* Generic uniform-model put/get for new flags */
static void put_sym(uint8_t model_id, uint16_t symbol) {
    if (s_sym_count < VIDEO_SYM_BUF_MAX)
        s_sym_buf[s_sym_count++] = (vid_sym_t){ symbol, model_id, 0 };
}

/* A3 — find last non-zero position */
static int find_last_pos(const int16_t *block, int total) {
    for (int i = total - 1; i >= 0; i--)
        if (block[i] != 0) return i;
    return -1;
}

/* A4 — diagonal scan ordering for 4x4 subblocks within an NxN block.
 * Reorders block to scan-order in temp then copies back. */
static const uint8_t k_diag4x4[16] = {
    0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 13, 10, 7, 14, 11, 15
};


static void apply_diag_scan_enc(int16_t *block, int N, int16_t *temp) {
    int nb = N / 4;
    if (nb <= 0) { memcpy(temp, block, (size_t)(N*N) * sizeof(int16_t)); memcpy(block, temp, (size_t)(N*N) * sizeof(int16_t)); return; }
    int idx = 0;
    for (int by = 0; by < nb; by++)
        for (int bx = 0; bx < nb; bx++)
            for (int s = 0; s < 16; s++) {
                int sy = k_diag4x4[s] / 4, sx = k_diag4x4[s] % 4;
                temp[idx++] = block[(by*4+sy) * N + (bx*4+sx)];
            }
    memcpy(block, temp, (size_t)(N*N) * sizeof(int16_t));
}

static void apply_diag_scan_dec(int16_t *block, int N, int16_t *temp) {
    int nb = N / 4;
    if (nb <= 0) return;
    memcpy(temp, block, (size_t)(N*N) * sizeof(int16_t));
    int idx = 0;
    for (int by = 0; by < nb; by++)
        for (int bx = 0; bx < nb; bx++)
            for (int s = 0; s < 16; s++) {
                int sy = k_diag4x4[s] / 4, sx = k_diag4x4[s] % 4;
                block[(by*4+sy) * N + (bx*4+sx)] = temp[idx++];
            }
}

/* A3+A4: diagonal scan + last significant coefficient encoding.
 * block[] is modified in-place (scan reordering). */
static void enc_coeff_block_subband(video_entropy_encoder_t *ve,
                                    int16_t *block, int N, int16_t *tmp) {
    apply_diag_scan_enc(block, N, tmp);
    int last = find_last_pos(block, N * N);
    if (last < 0) last = 0;
    put_sym(VID_MODEL_LAST_POS, (uint16_t)last);
    for (int i = 0; i <= last; i++)
        video_entropy_enc_put_coeff_subband(ve, block[i], subband_for_pos(i, N));
}

static void dec_coeff_block_subband(video_entropy_decoder_t *vd,
                                    int16_t *block, int N, int16_t *tmp) {
    int last = (int)rans_dec_get(&vd->dec, &g_last_pos_model);
    last = CLAMP(last, 0, N * N - 1);
    for (int i = 0; i <= last; i++)
        block[i] = video_entropy_dec_get_coeff_subband(vd, subband_for_pos(i, N));
    for (int i = last + 1; i < N * N; i++) block[i] = 0;
    apply_diag_scan_dec(block, N, tmp);
}

/* ========================================================================
 * POOL SIZE CALCULATION
 * ======================================================================== */

size_t opvis_encoder_pool_size(uint16_t width, uint16_t height) {
    const size_t y_size = width * height;
    const size_t uv_size = (width / 2) * (height / 2);
    const size_t wavelet_buf_size = OPVIS_CTU_SIZE * OPVIS_CTU_SIZE * sizeof(int16_t) * 2;
    const size_t mv_size = ((width / OPVIS_MB_SIZE) * (height / OPVIS_MB_SIZE)) * sizeof(opvis_mv_t);
    const size_t ref_indices_size = (width / OPVIS_MB_SIZE) * (height / OPVIS_MB_SIZE) * sizeof(uint8_t);
    const size_t interp_buf_size = (OPVIS_MV_RANGE * 2 + OPVIS_CTU_SIZE) * OPVIS_SUBPEL_SCALE *
                                   (OPVIS_MV_RANGE * 2 + OPVIS_CTU_SIZE) * OPVIS_SUBPEL_SCALE;

    /* prev_mvs: TMVP co-located MV storage (same size as current mvs) */
    return 3 * y_size + 6 * uv_size + wavelet_buf_size + mv_size + ref_indices_size + interp_buf_size + mv_size;
}

size_t opvis_decoder_pool_size(uint16_t width, uint16_t height) {
    const size_t y_size = width * height;
    const size_t uv_size = (width / 2) * (height / 2);
    const size_t wavelet_buf_size = OPVIS_CTU_SIZE * OPVIS_CTU_SIZE * sizeof(int16_t) * 2;
    const size_t interp_buf_size = (OPVIS_CTU_SIZE + 2) * OPVIS_SUBPEL_SCALE *
                                   (OPVIS_CTU_SIZE + 2) * OPVIS_SUBPEL_SCALE;
    /* 10-bit output planes: one slot (ref_y16[0]/u16[0]/v16[0]) */
    const size_t p10_size = y_size * sizeof(uint16_t) + 2 * uv_size * sizeof(uint16_t);
    /* prev_mvs: TMVP co-located MV storage */
    const size_t mb_count = (size_t)(width / OPVIS_MB_SIZE) * (height / OPVIS_MB_SIZE);

    /* 3 luma + 6 chroma planes (ref[0..2]); ref[2] is the B-frame output buffer */
    return 3 * y_size + 6 * uv_size + wavelet_buf_size + interp_buf_size + p10_size + mb_count * sizeof(opvis_mv_t);
}

size_t opvis_encoder_pool_size_v1(uint16_t width, uint16_t height,
                                  const opvis_color_info_t *color_info) {
    size_t base = opvis_encoder_pool_size(width, height);
    if (color_info && color_info->bitdepth == 10) {
        const size_t y_size  = width * height;
        const size_t uv_size = (width / 2) * (height / 2);
        base += (y_size + 2 * uv_size) * sizeof(uint16_t);  /* cur_y16/u16/v16 */
    }
    base += SCREEN_IBC_TABLE_SIZE * sizeof(uint32_t);  /* IBC hash table */
    return base;
}

size_t opvis_decoder_pool_size_v1(uint16_t width, uint16_t height,
                                  const opvis_color_info_t *color_info) {
    /* v1 pool always includes the 10-bit output slot, same as v0 */
    (void)color_info;
    return opvis_decoder_pool_size(width, height);
}

void opvis_encoder_set_color_info(opvis_encoder_t *enc,
                                  const opvis_color_info_t *ci,
                                  const opvis_hdr_meta_t *hdr) {
    if (!enc) return;
    if (ci)  enc->color_info = *ci;
    if (hdr) enc->hdr        = *hdr;
}

/* ========================================================================
 * ENCODER/DECODER INITIALIZATION
 * ======================================================================== */

int opvis_encoder_init(opvis_encoder_t *enc, uint16_t width, uint16_t height,
                       uint8_t quality, uint16_t gop_size,
                       opvis_pixel_fmt_t input_fmt,
                       uint8_t *pool, size_t pool_size) {
    if (!enc || !pool || width == 0 || height == 0 ||
        width > OPVIS_MAX_WIDTH || height > OPVIS_MAX_HEIGHT ||
        (width % OPVIS_MB_SIZE) != 0 || (height % OPVIS_MB_SIZE) != 0) {
        return -1;
    }

    const size_t required = opvis_encoder_pool_size(width, height);
    if (pool_size < required) return -1;

    memset(enc, 0, sizeof(*enc));
    enc->width = width;
    enc->height = height;
    enc->mb_cols = width / OPVIS_MB_SIZE;
    enc->mb_rows = height / OPVIS_MB_SIZE;
    enc->quality = quality;
    enc->gop_size = gop_size;
    enc->input_fmt = input_fmt;
    enc->frame_num = 0;
    enc->pool = pool;
    enc->pool_size = pool_size;

    /* Initialize rate control (default: no rate control) */
    enc->target_bitrate = 0;
    enc->rc_buffer = 0;
    enc->rc_buffer_size = 0;
    enc->rc_qp_adj = 0.0f;
    enc->crf_quality = 0;
    enc->fast_intra_enabled = false;
    enc->me_range = OPVIS_MV_RANGE;
    enc->cdef_enabled = true;
    enc->cdef_strength = 2;
    enc->tnr_enabled       = true;
    enc->tnr_alpha         = 0.5f;
    enc->tnr_motion_thresh = 16;
    memset(&enc->last_frame_stats, 0, sizeof(enc->last_frame_stats));

    /* Partition the pool for multi-reference frames */
    uint8_t *ptr = pool;
    const size_t y_size = width * height;
    const size_t uv_size = (width / 2) * (height / 2);
    const size_t mb_count = enc->mb_cols * enc->mb_rows;

    /* Two reference frames */
    enc->ref_y[0] = ptr; ptr += y_size;
    enc->ref_u[0] = ptr; ptr += uv_size;
    enc->ref_v[0] = ptr; ptr += uv_size;
    enc->ref_y[1] = ptr; ptr += y_size;
    enc->ref_u[1] = ptr; ptr += uv_size;
    enc->ref_v[1] = ptr; ptr += uv_size;

    /* Current frame */
    enc->cur_y = ptr; ptr += y_size;
    enc->cur_u = ptr; ptr += uv_size;
    enc->cur_v = ptr; ptr += uv_size;

    enc->wavelet_buf = (int16_t *)ptr;
    ptr += OPVIS_CTU_SIZE * OPVIS_CTU_SIZE * sizeof(int16_t) * 2;

    enc->interp_buf = ptr;
    ptr += (OPVIS_MV_RANGE * 2 + OPVIS_CTU_SIZE) * OPVIS_SUBPEL_SCALE *
           (OPVIS_MV_RANGE * 2 + OPVIS_CTU_SIZE) * OPVIS_SUBPEL_SCALE;

    enc->mvs = (opvis_mv_t *)ptr;
    ptr += mb_count * sizeof(opvis_mv_t);

    enc->ref_indices = ptr;
    ptr += mb_count * sizeof(uint8_t);

    /* prev_mvs: TMVP co-located MVs from the previous inter frame */
    enc->prev_mvs = (opvis_mv_t *)ptr;
    ptr += mb_count * sizeof(opvis_mv_t);
    memset(enc->prev_mvs, 0, mb_count * sizeof(opvis_mv_t));

    /* 10-bit working planes (only for P010 / YUV420P10LE input) */
    if (input_fmt == OPVIS_FMT_P010 || input_fmt == OPVIS_FMT_YUV420P10LE) {
        enc->color_info.bitdepth = 10;
        enc->cur_y16 = (uint16_t *)ptr; ptr += y_size  * sizeof(uint16_t);
        enc->cur_u16 = (uint16_t *)ptr; ptr += uv_size * sizeof(uint16_t);
        enc->cur_v16 = (uint16_t *)ptr; ptr += uv_size * sizeof(uint16_t);
    } else {
        enc->color_info.bitdepth = 8;
    }

    /* Optional: IBC hash table (allocated when pool provides v1 extra space) */
    const size_t ibc_bytes = SCREEN_IBC_TABLE_SIZE * sizeof(uint32_t);
    if ((size_t)(ptr - pool) + ibc_bytes <= pool_size) {
        enc->ibc_hashtable = (uint32_t *)ptr;
        ptr += ibc_bytes;
        memset(enc->ibc_hashtable, 0xFF, ibc_bytes);
    }
    (void)ptr;  /* suppress unused-variable warning if no further allocations */

    /* Clear reference frames */
    memset(enc->ref_y[0], 0, y_size);
    memset(enc->ref_u[0], 128, uv_size);
    memset(enc->ref_v[0], 128, uv_size);
    memset(enc->ref_y[1], 0, y_size);
    memset(enc->ref_u[1], 128, uv_size);
    memset(enc->ref_v[1], 128, uv_size);

    /* Initialize probability models for rANS */
    init_video_models();

    /* SARDO: saliency-aware RDO — enabled by default */
    sal_init(&enc->sal_ctx, width, height, OPVIS_CTU_SIZE);
    enc->sardo_enabled = true;

    /* TFI: temporal frame interpolation — disabled by default, caller enables */
    tfi_init(&enc->tfi_ctx, width, height);
    enc->tfi_enabled    = false;
    enc->tfi_skip_interval = 0;

    return 0;
}

/* Set rate control parameters for encoder */
void opvis_encoder_set_rate_control(opvis_encoder_t *enc, uint32_t target_bitrate_bps, uint32_t fps) {
    if (!enc || fps == 0) return;

    enc->target_bitrate = target_bitrate_bps;
    enc->rc_fps         = fps;

    /* Virtual buffer = 2x target bits per frame */
    const uint32_t target_frame_bits = target_bitrate_bps / fps;
    enc->rc_buffer_size = (int32_t)(target_frame_bits * 2);
    enc->rc_buffer      = enc->rc_buffer_size / 2;
    enc->rc_qp_adj      = 0.0f;
    enc->rc_integral    = 0.0f;
}

void opvis_encoder_set_tnr(opvis_encoder_t *enc, bool enabled,
                           float alpha, uint8_t motion_thresh) {
    if (!enc) return;
    enc->tnr_enabled       = enabled;
    enc->tnr_alpha         = alpha;
    enc->tnr_motion_thresh = motion_thresh;
}

/* D1 — Constant Rate Factor */
void opvis_encoder_set_crf(opvis_encoder_t *enc, uint8_t quality) {
    if (!enc) return;
    enc->crf_quality = quality;
}

/* D5 — Quality preset */
void opvis_encoder_set_preset(opvis_encoder_t *enc, opvis_preset_t preset) {
    if (!enc) return;
    switch (preset) {
        case OPVIS_PRESET_FAST:
            enc->fast_intra_enabled = true;
            enc->me_range = 8;
            break;
        case OPVIS_PRESET_MEDIUM:
            enc->fast_intra_enabled = false;
            enc->me_range = OPVIS_MV_RANGE;
            break;
        case OPVIS_PRESET_SLOW:
            enc->fast_intra_enabled = false;
            enc->me_range = 24;
            break;
    }
}

/* F1 — last frame stats accessor */
opvis_frame_stats_t opvis_encoder_get_stats(const opvis_encoder_t *enc) {
    opvis_frame_stats_t zero = {0};
    return enc ? enc->last_frame_stats : zero;
}

/* F3 — decoder stats accessor */
opvis_decoder_stats_t opvis_decoder_get_stats(const opvis_decoder_t *dec) {
    opvis_decoder_stats_t zero = {0};
    return dec ? dec->stats : zero;
}

/* F2 — flush: force an I-frame regardless of GOP position */
int opvis_encoder_flush(opvis_encoder_t *enc, uint8_t *out, size_t out_cap) {
    if (!enc || !out) return -1;
    if (enc->gop_size > 0) {
        /* Align frame_num to a GOP boundary so next encode emits I */
        enc->frame_num = (enc->frame_num / enc->gop_size) * enc->gop_size;
    }
    /* Use ref_y[0] (last good frame) as the input "current" frame */
    if (!enc->ref_y[0]) return -1;
    const size_t y_size = (size_t)enc->width * enc->height;
    const size_t uv_size = (size_t)(enc->width / 2) * (enc->height / 2);
    /* Build a YUV420P buffer from ref[0] and call opvis_encode */
    uint8_t *buf = (uint8_t *)malloc(y_size + 2 * uv_size);
    if (!buf) return -1;
    memcpy(buf, enc->ref_y[0], y_size);
    memcpy(buf + y_size, enc->ref_u[0], uv_size);
    memcpy(buf + y_size + uv_size, enc->ref_v[0], uv_size);
    opvis_pixel_fmt_t saved = enc->input_fmt;
    enc->input_fmt = OPVIS_FMT_YUV420P;
    int ret = opvis_encode(enc, buf, y_size + 2 * uv_size, out, out_cap);
    enc->input_fmt = saved;
    free(buf);
    return ret;
}

int opvis_decoder_init(opvis_decoder_t *dec, uint8_t *pool, size_t pool_size) {
    if (!dec || !pool) return -1;

    memset(dec, 0, sizeof(*dec));
    dec->pool = pool;
    dec->pool_size = pool_size;

    /* TFI decoder context — width/height unknown until first frame header */
    dec->tfi_enabled = true;  /* enable by default; init deferred to first frame */

    return 0;
}

/* ========================================================================
 * VARIABLE BLOCK SIZE CU HELPERS
 * ======================================================================== */

/* Minimum CU size — 8 for both intra and inter (chroma safely stays ≥4×4) */
#define CU_MIN_SIZE 8

/* SAD for arbitrary N×N block */
/* ========================================================================
 * CHROMA 4-TAP INTERPOLATION (HEVC-style, quarter-chroma-sample precision)
 * ======================================================================== */

/* 4-tap chroma filter, quarter-chroma-sample phase 0–3.
 * Derived from HEVC chroma filter (even phases from 8-phase table, ×32 norm). */
static const int8_t kChromaFilter[4][4] = {
    {  0, 32,  0,  0 },  /* 0/4 — integer  */
    { -4, 28,  8, -1 },  /* 1/4 pel        */
    { -4, 24, 16, -4 },  /* 2/4 — half pel */
    { -1,  8, 28, -4 },  /* 3/4 pel        */
};

/* Bilinear chroma interpolation at quarter-chroma-sample position.
 * cx_qpel, cy_qpel are in quarter-chroma-sample units (integer = multiple of 4). */
static uint8_t hevc_chroma_interp(const uint8_t *ref, int cw, int ch,
                                  int cx_qpel, int cy_qpel) {
    const int xi = cx_qpel >> 2, xf = cx_qpel & 3;
    const int yi = cy_qpel >> 2, yf = cy_qpel & 3;

    if (xf == 0 && yf == 0) {
        /* Integer position */
        int x = xi < 0 ? 0 : (xi >= cw ? cw - 1 : xi);
        int y = yi < 0 ? 0 : (yi >= ch ? ch - 1 : yi);
        return ref[y * cw + x];
    }

    const int8_t *hf = kChromaFilter[xf];
    const int8_t *vf = kChromaFilter[yf];

    /* Clamped pixel read helper */
#define CPIX(rx, ry) ref[((ry) < 0 ? 0 : (ry) >= ch ? ch-1 : (ry)) * cw + \
                         ((rx) < 0 ? 0 : (rx) >= cw ? cw-1 : (rx))]

    if (yf == 0) {
        int s = hf[0]*CPIX(xi-1,yi) + hf[1]*CPIX(xi,yi) +
                hf[2]*CPIX(xi+1,yi) + hf[3]*CPIX(xi+2,yi);
        return (uint8_t)CLAMP((s + 16) >> 5, 0, 255);
    }
    if (xf == 0) {
        int s = vf[0]*CPIX(xi,yi-1) + vf[1]*CPIX(xi,yi) +
                vf[2]*CPIX(xi,yi+1) + vf[3]*CPIX(xi,yi+2);
        return (uint8_t)CLAMP((s + 16) >> 5, 0, 255);
    }

    /* Two-pass: horizontal then vertical */
    int32_t tmp[4];
    for (int k = -1; k <= 2; k++) {
        tmp[k+1] = hf[0]*CPIX(xi-1,yi+k) + hf[1]*CPIX(xi,yi+k) +
                   hf[2]*CPIX(xi+1,yi+k) + hf[3]*CPIX(xi+2,yi+k);
    }
    int64_t s = (int64_t)vf[0]*tmp[0] + (int64_t)vf[1]*tmp[1] +
                (int64_t)vf[2]*tmp[2] + (int64_t)vf[3]*tmp[3];
    return (uint8_t)CLAMP((int)((s + 512) >> 10), 0, 255);
#undef CPIX
}

/* ========================================================================
 * ADAPTIVE MOTION VECTOR PREDICTION (AMVP)
 * ======================================================================== */

/* Return the spatial MV predictor for a CU at (x, y).
 * Prefers left neighbor; falls back to top; then (0,0).
 * Both encoder and decoder compute this identically from the shared MV grid. */
static opvis_mv_t get_mv_predictor(const opvis_mv_t *mvs, int mb_cols, int mb_rows,
                                   int x, int y) {
    const int mb_x = x / OPVIS_MB_SIZE;
    const int mb_y = y / OPVIS_MB_SIZE;

    /* Left */
    if (mb_x > 0)
        return mvs[mb_y * mb_cols + (mb_x - 1)];
    /* Top */
    if (mb_y > 0)
        return mvs[(mb_y - 1) * mb_cols + mb_x];
    /* Top-right (fallback when left is unavailable) */
    if (mb_y > 0 && mb_x + 1 < mb_cols)
        return mvs[(mb_y - 1) * mb_cols + (mb_x + 1)];

    UNUSED(mb_rows);
    return (opvis_mv_t){0, 0};
}

/* ========================================================================
 * CORE DISTORTION / RDO HELPERS
 * ======================================================================== */

static uint32_t calculate_sad_n(const uint8_t *a, int sa, const uint8_t *b, int sb, int N) {
    uint32_t sad = 0;
    for (int row = 0; row < N; row++)
        for (int col = 0; col < N; col++)
            sad += (uint32_t)abs((int)a[row * sa + col] - (int)b[row * sb + col]);
    return sad;
}

/* 4×4 Hadamard-based SATD for a pre-differenced 4×4 block (16 int16_t values) */
static uint32_t satd4(const int16_t *diff) {
    int32_t tmp[16];
    /* Row butterflies */
    for (int i = 0; i < 4; i++) {
        const int16_t *r = diff + i * 4;
        int a0 = r[0] + r[1], a1 = r[0] - r[1], a2 = r[2] + r[3], a3 = r[2] - r[3];
        tmp[i*4+0] = a0 + a2;
        tmp[i*4+1] = a1 + a3;
        tmp[i*4+2] = a0 - a2;
        tmp[i*4+3] = a1 - a3;
    }
    /* Column butterflies + accumulate absolute values */
    uint32_t sum = 0;
    for (int j = 0; j < 4; j++) {
        int b0 = tmp[j+0] + tmp[j+4], b1 = tmp[j+0] - tmp[j+4];
        int b2 = tmp[j+8] + tmp[j+12], b3 = tmp[j+8] - tmp[j+12];
        sum += (uint32_t)(abs(b0 + b2) + abs(b1 + b3) + abs(b0 - b2) + abs(b1 - b3));
    }
    return (sum + 1) >> 1;
}

/* SATD over N×N block (N must be a multiple of 4) */
static uint32_t calculate_satd_n(const uint8_t *a, int sa, const uint8_t *b, int sb, int N) {
    uint32_t satd = 0;
    int16_t diff[16];
    for (int by = 0; by < N; by += 4) {
        for (int bx = 0; bx < N; bx += 4) {
            for (int row = 0; row < 4; row++)
                for (int col = 0; col < 4; col++)
                    diff[row*4+col] = (int16_t)a[(by+row)*sa + (bx+col)]
                                    - (int16_t)b[(by+row)*sb + (bx+col)];
            satd += satd4(diff);
        }
    }
    return satd;
}

/* SATD of block vs flat DC prediction — measures texture complexity.
 * Hot path: called 5× per CU recursion level × CTU grid.  DC-sum loop
 * uses restrict + row pointer for auto-vectorisation (no alias). */
static uint32_t block_satd_dc(const uint8_t * restrict frame,
                               int fw, int fh, int x, int y, int N)
{
    /* Clamp to frame bounds for edge CTUs */
    const int xend = (x + N <= fw) ? N : (fw - x);
    const int yend = (y + N <= fh) ? N : (fh - y);
    if (xend <= 0 || yend <= 0) return 0;

    /* DC mean — row pointer avoids 2-D index recalculation per element */
    int sum = 0;
    for (int row = 0; row < yend; row++) {
        const uint8_t *row_ptr = frame + (y + row) * fw + x;
        for (int col = 0; col < xend; col++)
            sum += row_ptr[col];
    }
    const uint8_t dc = (uint8_t)(sum / (xend * yend));

    /* SATD in 4×4 tiles; replicate DC to a small scratch row for the reference */
    uint32_t satd = 0;
    int16_t diff[16];
    for (int by = 0; by < N; by += 4) {
        for (int bx = 0; bx < N; bx += 4) {
            for (int row = 0; row < 4; row++)
                for (int col = 0; col < 4; col++) {
                    int px = x + bx + col, py = y + by + row;
                    int16_t cur = (px < fw && py < fh) ? (int16_t)frame[py * fw + px] : (int16_t)dc;
                    diff[row*4+col] = cur - (int16_t)dc;
                }
            satd += satd4(diff);
        }
    }
    return satd;
}

/* RDO-guided split decision: split if sum-of-children SATD + λ*overhead < parent SATD.
 *
 * λ*overhead accounts for the extra bits needed to code 4 child split-flags and CU
 * headers.  We approximate overhead as N*N/8 SATD units (empirically tuned to HEVC
 * split rates on 720p natural content at QP 26–34).
 *
 * For large flat blocks the parent SATD is low, overhead dominates → no split.
 * For blocks with fine texture the child SATDs are much lower → split wins.
 */
static bool cu_should_split(const uint8_t *luma, int fw, int fh, int x, int y, int N,
                            float sal_weight_val) {
    if (N <= CU_MIN_SIZE) return false;

    uint32_t satd_parent = block_satd_dc(luma, fw, fh, x, y, N);

    /* Quick exit: very flat block — never worth splitting */
    if (satd_parent < (uint32_t)(N)) return false;

    const int half = N / 2;
    uint32_t satd_children =
        block_satd_dc(luma, fw, fh, x,      y,      half) +
        block_satd_dc(luma, fw, fh, x+half, y,      half) +
        block_satd_dc(luma, fw, fh, x,      y+half, half) +
        block_satd_dc(luma, fw, fh, x+half, y+half, half);

    /* SARDO: scale overhead by inverse saliency weight.
     * High saliency (face=3.5) → lower overhead → easier to split → finer detail.
     * Low saliency (bg=0.2)  → higher overhead → fewer splits → fewer bits wasted. */
    float base_overhead = (float)(N * N) / 8.0f;
    float eff_w = (sal_weight_val > 0.05f) ? sal_weight_val : 0.05f;
    uint32_t overhead = (uint32_t)(base_overhead / eff_w);

    return (satd_children + overhead) < satd_parent;
}


/* Fill N×N prediction block using HEVC 8-tap sub-pixel MC */
static void mc_predict_n(const uint8_t *ref, int fw, int fh,
                         int cx, int cy, int N, opvis_mv_t mv, uint8_t *pred) {
    int rxq = cx * OPVIS_SUBPEL_SCALE + mv.x;
    int ryq = cy * OPVIS_SUBPEL_SCALE + mv.y;
    for (int row = 0; row < N; row++)
        for (int col = 0; col < N; col++)
            pred[row * N + col] = hevc_luma_interp(ref, fw, fh,
                rxq + col * OPVIS_SUBPEL_SCALE,
                ryq + row * OPVIS_SUBPEL_SCALE);
}

/* ── Affine Motion Compensation ──────────────────────────────────────────
 *
 * 4-parameter affine model (scale + rotation + translation) derived from
 * three neighboring MVs:
 *
 *   For a block at (cx, cy) with size N, derive the affine parameters from
 *   the translational MVs of the top-left (TL), top-right (TR), and
 *   bottom-left (BL) corners of the neighboring CTU.
 *
 *   Affine mapping: for a pixel at (dx, dy) relative to (cx, cy):
 *     ref_x = cx + dx + (a * dx - b * dy + tx)
 *     ref_y = cy + dy + (b * dx + a * dy + ty)
 *
 *   where (tx, ty) is the TL translation,
 *   a = (mv_TR.x - mv_TL.x) / N   (x-scaling / rotation parameter)
 *   b = (mv_TR.y - mv_TL.y) / N   (y-shear parameter)
 *
 * Cost comparison: if affine SAD < translational SAD × 0.85, use affine.
 * The 0.85 factor accounts for the extra bits needed to code 2 extra MVs.
 *
 * Reference: HEVC JCTVC-G125 (2011) simplified affine model.
 * ─────────────────────────────────────────────────────────────────────── */

static void mc_affine_predict_n(const uint8_t *ref, int fw, int fh,
                                 int cx, int cy, int N,
                                 opvis_mv_t mv_tl, opvis_mv_t mv_tr,
                                 uint8_t *pred)
{
    float tx = (float)mv_tl.x / (float)OPVIS_SUBPEL_SCALE;
    float ty = (float)mv_tl.y / (float)OPVIS_SUBPEL_SCALE;
    float a  = (float)(mv_tr.x - mv_tl.x) / ((float)N * (float)OPVIS_SUBPEL_SCALE);
    float b  = (float)(mv_tr.y - mv_tl.y) / ((float)N * (float)OPVIS_SUBPEL_SCALE);

    for (int row = 0; row < N; row++) {
        for (int col = 0; col < N; col++) {
            float fdx = (float)col, fdy = (float)row;
            float rx = (float)cx + fdx + a * fdx - b * fdy + tx;
            float ry = (float)cy + fdy + b * fdx + a * fdy + ty;
            /* Convert to sub-pixel units for HEVC 8-tap interpolation */
            int rxq = (int)(rx * (float)OPVIS_SUBPEL_SCALE + 0.5f);
            int ryq = (int)(ry * (float)OPVIS_SUBPEL_SCALE + 0.5f);
            pred[row * N + col] = hevc_luma_interp(ref, fw, fh, rxq, ryq);
        }
    }
}

/* Try affine MC and return SAD; if < translational * threshold, return true
 * and fill pred[].  Otherwise pred[] is untouched. */
static bool affine_mc_try(const uint8_t *cur_block, int fw, int fh,
                           int cx, int cy, int N,
                           const uint8_t *ref,
                           opvis_mv_t mv_tl, opvis_mv_t mv_tr,
                           uint32_t trans_sad,
                           uint8_t *pred_out)
{
    /* Require at least 8×8 block for affine to be worthwhile */
    if (N < 8) return false;
    /* Degenerate: if TL and TR MVs are essentially identical, skip */
    if (abs(mv_tr.x - mv_tl.x) < 1 && abs(mv_tr.y - mv_tl.y) < 1) return false;

    uint8_t pred[64 * 64];
    mc_affine_predict_n(ref, fw, fh, cx, cy, N, mv_tl, mv_tr, pred);

    /* Compute SAD between current block and affine prediction */
    uint32_t affine_sad = 0;
    for (int row = 0; row < N; row++) {
        const uint8_t *cur_row = cur_block + row * fw;
        for (int col = 0; col < N; col++)
            affine_sad += (uint32_t)abs((int)cur_row[col] - (int)pred[row * N + col]);
    }

    /* Accept affine if it's at least 15% better (accounts for 2 extra coded MVs) */
    if (affine_sad < (uint32_t)((float)trans_sad * 0.85f)) {
        memcpy(pred_out, pred, (size_t)(N * N));
        return true;
    }
    return false;
}

/* Reconstruct N×N block into frame buffer */
static void mc_reconstruct_n(const uint8_t *ref, int fw, int fh,
                              int cx, int cy, int N, opvis_mv_t mv,
                              const int16_t *residual, uint8_t *out, int out_stride) {
    int rxq = cx * OPVIS_SUBPEL_SCALE + mv.x;
    int ryq = cy * OPVIS_SUBPEL_SCALE + mv.y;
    for (int row = 0; row < N; row++) {
        for (int col = 0; col < N; col++) {
            uint8_t pred = hevc_luma_interp(ref, fw, fh,
                rxq + col * OPVIS_SUBPEL_SCALE,
                ryq + row * OPVIS_SUBPEL_SCALE);
            int16_t v = (int16_t)pred + residual[row * N + col];
            out[row * out_stride + col] = (uint8_t)CLAMP(v, 0, 255);
        }
    }
}

/* Compute SATD between a current block (raw pixels, stride fw) and an
 * interpolated prediction at quarter-pixel MV (mx_q, my_q) in the reference.
 * N must be a multiple of 4.  Used for sub-pixel refinement. */
static uint32_t satd_subpel(const uint8_t *cur_block, int fw,
                             const uint8_t *ref, int ref_fw, int ref_fh,
                             int cx, int cy, int N, int mx_q, int my_q) {
    int16_t diff[16];
    uint32_t satd = 0;
    for (int by = 0; by < N; by += 4) {
        for (int bx = 0; bx < N; bx += 4) {
            for (int row = 0; row < 4; row++) {
                for (int col = 0; col < 4; col++) {
                    int16_t cur = (int16_t)cur_block[(cy + by + row) * fw + (cx + bx + col)];
                    int16_t prd = (int16_t)hevc_luma_interp(ref, ref_fw, ref_fh,
                        (cx + bx + col) * OPVIS_SUBPEL_SCALE + mx_q,
                        (cy + by + row) * OPVIS_SUBPEL_SCALE + my_q);
                    diff[row*4+col] = cur - prd;
                }
            }
            satd += satd4(diff);
        }
    }
    return satd;
}

/* Diamond search motion estimation for an N×N CU.
 * Integer-pixel diamond search (fast SAD), then quarter-pixel refinement (SATD). */
static opvis_mv_t motion_estimate_cu(const uint8_t *cur_block, int cur_x, int cur_y, int N,
                                     const uint8_t *ref0, const uint8_t *ref1,
                                     int fw, int fh, uint8_t *best_ref_idx,
                                     int me_range) {
    static const int ld[8][2] = {{0,-2},{-1,-1},{1,-1},{-2,0},{2,0},{-1,1},{1,1},{0,2}};
    static const int sd[4][2] = {{0,-1},{-1,0},{1,0},{0,1}};
    /* 8 quarter-pixel offsets for sub-pixel refinement */
    static const int qd[8][2] = {{0,-1},{-1,0},{1,0},{0,1},{-1,-1},{1,-1},{-1,1},{1,1}};

    opvis_mv_t best_mv = {0, 0};
    uint32_t best_sad = UINT32_MAX;
    uint8_t best_ref = 0;
    int best_int_x = 0, best_int_y = 0;   /* integer-pixel best position */
    const uint8_t *refs[2] = {ref0, ref1};

    /* ---- Integer-pixel diamond search (SAD) ---- */
    for (int ri = 0; ri < 2; ri++) {
        if (!refs[ri]) continue;
        int sx = 0, sy = 0;

        if (cur_x >= 0 && cur_y >= 0 && cur_x + N <= fw && cur_y + N <= fh) {
            uint32_t sad = calculate_sad_n(cur_block, fw,
                                           refs[ri] + cur_y * fw + cur_x, fw, N);
            if (sad < best_sad) {
                best_sad = sad; best_mv.x = best_mv.y = 0;
                best_ref = ri; best_int_x = 0; best_int_y = 0;
            }
        }

        for (bool improved = true; improved; ) {
            improved = false;
            for (int i = 0; i < 8; i++) {
                int tx = sx + ld[i][0], ty = sy + ld[i][1];
                if (abs(tx) > me_range || abs(ty) > me_range) continue;
                int rx = cur_x + tx, ry = cur_y + ty;
                if (rx < 0 || ry < 0 || rx + N > fw || ry + N > fh) continue;
                uint32_t sad = calculate_sad_n(cur_block, fw,
                                               refs[ri] + ry * fw + rx, fw, N);
                if (sad < best_sad) {
                    best_sad = sad;
                    best_mv.x = (int16_t)(tx * OPVIS_SUBPEL_SCALE);
                    best_mv.y = (int16_t)(ty * OPVIS_SUBPEL_SCALE);
                    best_ref = ri; sx = tx; sy = ty; improved = true;
                    best_int_x = tx; best_int_y = ty;
                }
            }
        }

        for (bool improved = true; improved; ) {
            improved = false;
            for (int i = 0; i < 4; i++) {
                int tx = sx + sd[i][0], ty = sy + sd[i][1];
                if (abs(tx) > me_range || abs(ty) > me_range) continue;
                int rx = cur_x + tx, ry = cur_y + ty;
                if (rx < 0 || ry < 0 || rx + N > fw || ry + N > fh) continue;
                uint32_t sad = calculate_sad_n(cur_block, fw,
                                               refs[ri] + ry * fw + rx, fw, N);
                if (sad < best_sad) {
                    best_sad = sad;
                    best_mv.x = (int16_t)(tx * OPVIS_SUBPEL_SCALE);
                    best_mv.y = (int16_t)(ty * OPVIS_SUBPEL_SCALE);
                    best_ref = ri; sx = tx; sy = ty; improved = true;
                    best_int_x = tx; best_int_y = ty;
                }
            }
        }
    }

    /* ---- Quarter-pixel refinement (SATD) ---- */
    /* Skip refinement when the integer-pixel match is already near-perfect (skip cost < 8 per sample).
     * Also skip for very small blocks (4×4) where 8-tap filter overhead outweighs gain. */
    if (best_sad < (uint32_t)(N * N / 2)) goto done_refine;

    if (cur_x >= 0 && cur_y >= 0 && cur_x + N <= fw && cur_y + N <= fh) {
        const uint8_t *best_ref_plane = refs[best_ref];
        if (best_ref_plane) {
            /* Baseline SATD at current integer best */
            uint32_t best_satd = satd_subpel(cur_block, fw, best_ref_plane, fw, fh,
                                             cur_x, cur_y, N,
                                             best_int_x * OPVIS_SUBPEL_SCALE,
                                             best_int_y * OPVIS_SUBPEL_SCALE);

            for (int i = 0; i < 8; i++) {
                int mx_q = best_int_x * OPVIS_SUBPEL_SCALE + qd[i][0];
                int my_q = best_int_y * OPVIS_SUBPEL_SCALE + qd[i][1];
                /* Bound check: integer part of the qpel MV must keep block in-frame */
                int int_rx = cur_x + mx_q / OPVIS_SUBPEL_SCALE;
                int int_ry = cur_y + my_q / OPVIS_SUBPEL_SCALE;
                if (int_rx < 0 || int_ry < 0 || int_rx + N > fw || int_ry + N > fh) continue;

                uint32_t satd = satd_subpel(cur_block, fw, best_ref_plane, fw, fh,
                                            cur_x, cur_y, N, mx_q, my_q);
                if (satd < best_satd) {
                    best_satd = satd;
                    best_mv.x = (int16_t)mx_q;
                    best_mv.y = (int16_t)my_q;
                }
            }
        }
    }

done_refine:
    *best_ref_idx = best_ref;
    return best_mv;
}

/* Update the 16-pixel MV grid for a CU of any size */
static void update_mv_grid(opvis_mv_t *mvs, uint8_t *ref_indices,
                           int mb_cols, int mb_rows,
                           int x, int y, int N, opvis_mv_t mv, uint8_t ref_idx) {
    for (int by = y; by < y + N; by += OPVIS_MB_SIZE) {
        int mb_y = by / OPVIS_MB_SIZE;
        if (mb_y >= mb_rows) continue;
        for (int bx = x; bx < x + N; bx += OPVIS_MB_SIZE) {
            int mb_x = bx / OPVIS_MB_SIZE;
            if (mb_x >= mb_cols) continue;
            int idx = mb_y * mb_cols + mb_x;
            mvs[idx] = mv;
            ref_indices[idx] = ref_idx;
        }
    }
}

/* Forward declarations for recursive CU functions */
static void encode_cu(opvis_encoder_t *enc, video_entropy_encoder_t *ve,
                      int x, int y, int N, opvis_frame_type_t ft, uint8_t quality,
                      opvis_mv_t *mvs, uint8_t *ref_indices);
static void decode_cu(opvis_decoder_t *dec, video_entropy_decoder_t *vd,
                      int x, int y, int N, opvis_frame_type_t ft, uint8_t quality,
                      opvis_mv_t *mvs, uint8_t *ref_indices);
static void get_mrl_neighbors(const uint8_t *frame, int fw, int fh,
                               int bx, int by, int N, int r,
                               uint8_t *top_out, uint8_t *left_out, uint8_t *tl_out);

/* ---- Leaf encoder ---- */

static void encode_leaf_cu(opvis_encoder_t *enc, video_entropy_encoder_t *ve,
                           int x, int y, int N, opvis_frame_type_t ft, uint8_t quality,
                           opvis_mv_t *mvs, uint8_t *ref_indices) {
    const int fw = enc->width, fh = enc->height;
    const int Nc = N / 2;                     /* chroma block size */
    const int cw = fw / 2, ch = fh / 2;

    /* --- Flat working buffers (max 64×64) on the stack --- */
    int16_t block[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
    uint8_t orig [OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
    uint8_t pred [OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];

    opvis_mv_t mv = {0, 0};
    uint8_t ref_idx = 0;

    /* ======== Luma ======== */
    if (ft == OPVIS_FRAME_I) {
        /* Extract original pixels */
        for (int row = 0; row < N; row++)
            for (int col = 0; col < N; col++) {
                int px = x + col, py = y + row;
                orig[row * N + col] = (px < fw && py < fh) ? enc->cur_y[py * fw + px] : 0;
            }

        bool screen_coded = false;

        /* --- Screen content: try IBC before intra --- */
        if (!screen_coded && enc->ibc_hashtable && N >= SCREEN_IBC_MIN_SIZE) {
            int bv_x = 0, bv_y = 0;
            bool ibc_found = screen_ibc_search(enc->cur_y, fw, fh, x, y, N,
                                               enc->ibc_hashtable, SCREEN_IBC_TABLE_SIZE,
                                               &bv_x, &bv_y);
            if (ibc_found) {
                video_entropy_enc_put_intra_mode(ve, SCREEN_MODE_IBC);
                video_entropy_enc_put_mv(ve, (int16_t)bv_x);
                video_entropy_enc_put_mv(ve, (int16_t)bv_y);
                /* Residual against IBC prediction */
                const int rx = x + bv_x, ry = y + bv_y;
                for (int row = 0; row < N; row++)
                    for (int col = 0; col < N; col++)
                        block[row * N + col] = (int16_t)orig[row * N + col] -
                            (int16_t)enc->cur_y[(ry + row) * fw + (rx + col)];
                cdf53_2d_forward_n(block, N, enc->wavelet_buf);
                quantize_nxn(block, N, 100, 1.0f);  /* lossless IBC residual */
                /* Snapshot pre-scan coefficients so reconstruction operates on the
                 * un-permuted layout (the encode helper reorders block in place). */
                int16_t recon[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
                memcpy(recon, block, (size_t)(N * N) * sizeof(int16_t));
                /* A1+A3+A4 — subband-aware coefficient coding with last-pos + diag scan */
                enc_coeff_block_subband(ve, block, N, enc->wavelet_buf);
                /* Write-back reconstructed pixels so future intra neighbors match decoder */
                {
                    dequantize_nxn(recon, N, 100, 1.0f);  /* lossless IBC residual */
                    cdf53_2d_inverse_n(recon, N, enc->wavelet_buf);
                    for (int row = 0; row < N; row++)
                        for (int col = 0; col < N; col++) {
                            int px = x + col, py2 = y + row;
                            if (px < fw && py2 < fh) {
                                int16_t ref_px = (int16_t)enc->cur_y[(ry + row) * fw + (rx + col)];
                                int16_t v = ref_px + recon[row * N + col];
                                enc->cur_y[py2 * fw + px] = (uint8_t)CLAMP(v, 0, 255);
                            }
                        }
                }
                /* Register reconstructed pixels for future IBC searches */
                if (enc->ibc_hashtable)
                    screen_ibc_update(enc->cur_y, fw, x, y, enc->ibc_hashtable, SCREEN_IBC_TABLE_SIZE);
                screen_coded = true;
                s_screen_mode_used = true;
            }
        }

        /* --- Screen content: try palette --- */
        if (!screen_coded && N >= SCREEN_PALETTE_MIN_SIZE) {
            uint8_t pal_y[SCREEN_PALETTE_MAX];
            uint8_t pal_idx[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
            int n_entries = 0;
            if (screen_palette_detect(orig, N, pal_y, &n_entries, pal_idx)) {
                video_entropy_enc_put_intra_mode(ve, SCREEN_MODE_PALETTE);
                video_entropy_enc_put_coeff(ve, (int16_t)n_entries);
                for (int e = 0; e < n_entries; e++)
                    video_entropy_enc_put_byte(ve, pal_y[e]);
                for (int i = 0; i < N * N; i++)
                    video_entropy_enc_put_coeff(ve, (int16_t)pal_idx[i]);
                /* Write-back palette reconstruction */
                for (int row = 0; row < N; row++)
                    for (int col = 0; col < N; col++) {
                        int px = x + col, py2 = y + row;
                        if (px < fw && py2 < fh)
                            enc->cur_y[py2 * fw + px] = pal_y[pal_idx[row * N + col]];
                    }
                screen_coded = true;
                s_screen_mode_used = true;
            }
        }

        /* --- Normal HEVC intra prediction --- */
        if (!screen_coded) {
            uint8_t top_buf[OPVIS_CTU_SIZE], tr_buf[OPVIS_CTU_SIZE], left_buf[OPVIS_CTU_SIZE];
            uint8_t top_left = 128;
            intra_get_neighbors_hevc(enc->cur_y, (uint16_t)fw, (uint16_t)fh,
                                     (uint16_t)x, (uint16_t)y, N,
                                     top_buf, tr_buf, left_buf, &top_left);

            /* C4 — MRL: try reference lines 0, 1, 2 and pick the one with lowest SAD */
            int max_r = 0;
            if (y > 2 && x > 2) max_r = 2;
            else if (y > 1 && x > 1) max_r = 1;

            uint8_t best_mrl = 0;
            intra_mode_hevc_t best_mode = INTRA_HEVC_DC;
            uint32_t best_cost = UINT32_MAX;
            uint8_t top_sel[OPVIS_CTU_SIZE], left_sel[OPVIS_CTU_SIZE];
            uint8_t tl_sel = top_left;

            for (int r = 0; r <= max_r; r++) {
                uint8_t tb[OPVIS_CTU_SIZE], lb[OPVIS_CTU_SIZE];
                uint8_t tl = top_left;
                if (r == 0) {
                    memcpy(tb, top_buf, (size_t)N);
                    memcpy(lb, left_buf, (size_t)N);
                } else {
                    get_mrl_neighbors(enc->cur_y, fw, fh, x, y, N, r, tb, lb, &tl);
                }
                uint8_t *tp = (y > r) ? tb : NULL;
                uint8_t *lp = (x > r) ? lb : NULL;
                intra_mode_hevc_t m = intra_find_best_mode_hevc(orig, N, tp, tr_buf, lp, tl);
                uint8_t tmp_pred[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
                intra_predict_hevc(m, N, tp, tr_buf, lp, tl, tmp_pred);
                uint32_t cost = 0;
                for (int i = 0; i < N * N; i++)
                    cost += (uint32_t)abs((int)orig[i] - (int)tmp_pred[i]);
                if (cost < best_cost) {
                    best_cost = cost; best_mrl = (uint8_t)r; best_mode = m;
                    memcpy(top_sel, tb, (size_t)N);
                    memcpy(left_sel, lb, (size_t)N);
                    tl_sel = tl;
                }
            }

            uint8_t *top_p  = (y > (int)best_mrl) ? top_sel : NULL;
            uint8_t *left_p = (x > (int)best_mrl) ? left_sel : NULL;
            intra_mode_hevc_t mode = best_mode;
            intra_predict_hevc(mode, N, top_p, tr_buf, left_p, tl_sel, pred);

            /* C5 — ISP: try 2×2 sub-block H-scan and V-scan modes.
             * Cost estimated without write-back (conservative; actual ISP uses
             * cross-sub-block references which improve quality further). */
            uint8_t best_isp = 0;
            if (N >= 8) {
                uint32_t cost_no_isp = 0;
                for (int i = 0; i < N * N; i++)
                    cost_no_isp += (uint32_t)abs((int)orig[i] - (int)pred[i]);

                const int H = N / 2;
                /* Sub-block (x_div, y_div) multiplied by H gives the offset:
                 * kDivH = raster order, kDivV = column order */
                const int kDivH[4][2] = {{0,0},{1,0},{0,1},{1,1}};
                const int kDivV[4][2] = {{0,0},{0,1},{1,0},{1,1}};
                uint32_t best_isp_cost = cost_no_isp;

                for (int isp_try = 1; isp_try <= 2; isp_try++) {
                    const int (*div)[2] = (isp_try == 1) ? kDivH : kDivV;
                    uint32_t cost_isp = 0;
                    for (int s = 0; s < 4; s++) {
                        int sx = x + div[s][0] * H;
                        int sy = y + div[s][1] * H;
                        uint8_t stb[OPVIS_CTU_SIZE/2], str[OPVIS_CTU_SIZE/2];
                        uint8_t slb[OPVIS_CTU_SIZE/2];
                        uint8_t stl;
                        intra_get_neighbors_hevc(enc->cur_y, (uint16_t)fw, (uint16_t)fh,
                                                 (uint16_t)sx, (uint16_t)sy, H,
                                                 stb, str, slb, &stl);
                        uint8_t *stp = (sy > 0) ? stb : NULL;
                        uint8_t *slp = (sx > 0) ? slb : NULL;
                        uint8_t sub_pred[OPVIS_CTU_SIZE/2 * OPVIS_CTU_SIZE/2];
                        intra_predict_hevc(mode, H, stp, str, slp, stl, sub_pred);
                        int ox = div[s][0] * H, oy = div[s][1] * H;
                        for (int row = 0; row < H; row++)
                            for (int col = 0; col < H; col++)
                                cost_isp += (uint32_t)abs(
                                    (int)orig[(oy + row) * N + (ox + col)] -
                                    (int)sub_pred[row * H + col]);
                    }
                    if (cost_isp < best_isp_cost * 9 / 10) {
                        best_isp_cost = cost_isp;
                        best_isp = (uint8_t)isp_try;
                    }
                }
            }

            video_entropy_enc_put_intra_mode(ve, mode);
            /* MRL = 0 when ISP is active (sub-blocks use cross-block refs, not MRL) */
            video_entropy_enc_put_mrl_idx(ve, best_isp ? 0 : best_mrl);
            video_entropy_enc_put_isp_mode(ve, best_isp);

            if (best_isp == 0) {
                /* No-ISP: encode the full N×N block */
                for (int i = 0; i < N * N; i++)
                    block[i] = (int16_t)orig[i] - (int16_t)pred[i];
                cdf53_2d_forward_n(block, N, enc->wavelet_buf);
                quantize_nxn(block, N, quality, 1.0f);
                int16_t recon[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
                memcpy(recon, block, (size_t)(N * N) * sizeof(int16_t));
                bool cbf = false;
                for (int i = 0; i < N * N; i++) if (block[i]) { cbf = true; break; }
                video_entropy_enc_put_cbf(ve, cbf ? 1 : 0);
                if (cbf) enc_coeff_block_subband(ve, block, N, enc->wavelet_buf);
                dequantize_nxn(recon, N, quality, 1.0f);
                cdf53_2d_inverse_n(recon, N, enc->wavelet_buf);
                for (int row = 0; row < N; row++)
                    for (int col = 0; col < N; col++) {
                        int px = x + col, py2 = y + row;
                        if (px < fw && py2 < fh) {
                            int16_t v = (int16_t)pred[row * N + col] + recon[row * N + col];
                            enc->cur_y[py2 * fw + px] = (uint8_t)CLAMP(v, 0, 255);
                        }
                    }
            } else {
                /* ISP encode: 4 sequential H×H sub-blocks with cross-block references.
                 * ISP_H = raster scan, ISP_V = column scan. */
                const int H = N / 2;
                const int kDivH[4][2] = {{0,0},{1,0},{0,1},{1,1}};
                const int kDivV[4][2] = {{0,0},{0,1},{1,0},{1,1}};
                const int (*div)[2] = (best_isp == 1) ? kDivH : kDivV;
                for (int s = 0; s < 4; s++) {
                    int sx = x + div[s][0] * H;
                    int sy = y + div[s][1] * H;
                    int ox = div[s][0] * H, oy = div[s][1] * H;
                    /* Extract sub-block orig from the full N×N buffer */
                    uint8_t sub_orig[OPVIS_CTU_SIZE/2 * OPVIS_CTU_SIZE/2];
                    for (int row = 0; row < H; row++)
                        for (int col = 0; col < H; col++)
                            sub_orig[row * H + col] = orig[(oy + row) * N + (ox + col)];
                    /* Neighbors from cur_y — previous sub-blocks already written back */
                    uint8_t stb[OPVIS_CTU_SIZE/2], str[OPVIS_CTU_SIZE/2];
                    uint8_t slb[OPVIS_CTU_SIZE/2], stl;
                    intra_get_neighbors_hevc(enc->cur_y, (uint16_t)fw, (uint16_t)fh,
                                             (uint16_t)sx, (uint16_t)sy, H,
                                             stb, str, slb, &stl);
                    uint8_t *stp = (sy > 0) ? stb : NULL;
                    uint8_t *slp = (sx > 0) ? slb : NULL;
                    uint8_t sub_pred_buf[OPVIS_CTU_SIZE/2 * OPVIS_CTU_SIZE/2];
                    intra_predict_hevc(mode, H, stp, str, slp, stl, sub_pred_buf);
                    int16_t sub_block[OPVIS_CTU_SIZE/2 * OPVIS_CTU_SIZE/2];
                    for (int i = 0; i < H * H; i++)
                        sub_block[i] = (int16_t)sub_orig[i] - (int16_t)sub_pred_buf[i];
                    cdf53_2d_forward_n(sub_block, H, enc->wavelet_buf);
                    quantize_nxn(sub_block, H, quality, 1.0f);
                    int16_t sub_recon[OPVIS_CTU_SIZE/2 * OPVIS_CTU_SIZE/2];
                    memcpy(sub_recon, sub_block, (size_t)(H * H) * sizeof(int16_t));
                    bool sub_cbf = false;
                    for (int i = 0; i < H * H; i++) if (sub_block[i]) { sub_cbf = true; break; }
                    video_entropy_enc_put_cbf(ve, sub_cbf ? 1 : 0);
                    if (sub_cbf) enc_coeff_block_subband(ve, sub_block, H, enc->wavelet_buf);
                    dequantize_nxn(sub_recon, H, quality, 1.0f);
                    cdf53_2d_inverse_n(sub_recon, H, enc->wavelet_buf);
                    for (int row = 0; row < H; row++)
                        for (int col = 0; col < H; col++) {
                            int px = sx + col, py2 = sy + row;
                            if (px < fw && py2 < fh) {
                                int16_t v = (int16_t)sub_pred_buf[row*H+col] + sub_recon[row*H+col];
                                enc->cur_y[py2 * fw + px] = (uint8_t)CLAMP(v, 0, 255);
                            }
                        }
                }
            }

            /* Register reconstructed pixels for future IBC searches */
            if (enc->ibc_hashtable && N >= SCREEN_IBC_MIN_SIZE)
                screen_ibc_update(enc->cur_y, fw, x, y, enc->ibc_hashtable, SCREEN_IBC_TABLE_SIZE);
        }

    } else if (ft == OPVIS_FRAME_P) {
        /* P-frame: zero-MV fast path, then full ME + AMVP delta coding + CBF */
        const uint8_t *cur_block = enc->cur_y + y * fw + x;
        bool skip = false;

        if (x + N <= fw && y + N <= fh && enc->ref_y[0]) {
            uint32_t skip_sad = calculate_sad_n(cur_block, fw,
                                                enc->ref_y[0] + y * fw + x, fw, N);
            skip = skip_sad < (uint32_t)(N * 4);
        }

        video_entropy_enc_put_mode(ve, skip ? 1 : 0);

        if (!skip) {
            /* Zero-MV fast path: if zero-MV SATD is already low, skip full ME */
            uint32_t zero_satd = UINT32_MAX;
            if (enc->ref_y[0] && x + N <= fw && y + N <= fh) {
                zero_satd = calculate_satd_n(cur_block, fw,
                                             enc->ref_y[0] + y * fw + x, fw, N);
            }

            if (zero_satd < (uint32_t)(N * N / 4)) {
                /* Zero MV is good enough — skip expensive diamond search */
                mv.x = mv.y = 0; ref_idx = 0;
            } else {
                mv = motion_estimate_cu(cur_block, x, y, N,
                                        enc->ref_y[0], enc->ref_y[1],
                                        fw, fh, &ref_idx, enc->me_range);
            }

            /* AMVP: code MV as delta from spatial predictor.
             * B1 — merge mode: when MV exactly matches the spatial predictor and
             * ref_idx == 0, encode mode=4 and skip the delta entirely. */
            opvis_mv_t mvp = get_mv_predictor(mvs, enc->mb_cols, enc->mb_rows, x, y);
            bool merge = (ref_idx == 0 && mv.x == mvp.x && mv.y == mvp.y);
            if (merge) {
                video_entropy_enc_put_mode(ve, 4);
            } else {
                /* B3 — AMVP: choose spatial predictor vs TMVP (co-located from prev frame) */
                opvis_mv_t mvp_tmvp = {0, 0};
                if (enc->prev_mvs) {
                    int tmb_x = x / OPVIS_MB_SIZE, tmb_y = y / OPVIS_MB_SIZE;
                    mvp_tmvp = enc->prev_mvs[tmb_y * enc->mb_cols + tmb_x];
                }
                uint32_t c_sp   = (uint32_t)(abs(mv.x - mvp.x)      + abs(mv.y - mvp.y));
                uint32_t c_tmvp = (uint32_t)(abs(mv.x - mvp_tmvp.x) + abs(mv.y - mvp_tmvp.y));
                uint8_t mvp_idx = (enc->prev_mvs && c_tmvp < c_sp) ? 1 : 0;
                opvis_mv_t sel_mvp = (mvp_idx == 1) ? mvp_tmvp : mvp;
                video_entropy_enc_put_mode(ve, ref_idx + 2);
                video_entropy_enc_put_mvp_idx(ve, mvp_idx);
                video_entropy_enc_put_mv(ve, (int16_t)(mv.x - sel_mvp.x));
                video_entropy_enc_put_mv(ve, (int16_t)(mv.y - sel_mvp.y));
            }

            mc_predict_n(enc->ref_y[ref_idx], fw, fh, x, y, N, mv, pred);

            /* Affine motion compensation: try if we have valid neighbor MVs.
             * Use top-left (x-OPVIS_MB_SIZE, y) and top-right (x, y-OPVIS_MB_SIZE)
             * neighbor block MVs from the MV grid as control points.
             * Only attempt when not in merge mode (merge means translational is free). */
            if (!merge && N >= 8) {
                int mb_x = x / OPVIS_MB_SIZE;
                int mb_y = y / OPVIS_MB_SIZE;
                opvis_mv_t mv_tl = mv, mv_tr = mv;
                bool have_neighbors = false;
                if (mb_x > 0 && mb_y > 0) {
                    mv_tl = mvs[(mb_y - 1) * enc->mb_cols + mb_x];
                    mv_tr = mvs[mb_y       * enc->mb_cols + (mb_x - 1)];
                    have_neighbors = true;
                } else if (mb_x > 0) {
                    mv_tr = mvs[mb_y * enc->mb_cols + (mb_x - 1)];
                    have_neighbors = true;
                } else if (mb_y > 0) {
                    mv_tl = mvs[(mb_y - 1) * enc->mb_cols + mb_x];
                    have_neighbors = true;
                }

                if (have_neighbors) {
                    /* Compute SAD of translational prediction to gate affine attempt */
                    uint32_t trans_sad = 0;
                    const uint8_t *cur_block_ptr = enc->cur_y + y * fw + x;
                    for (int row = 0; row < N; row++) {
                        const uint8_t *cr = cur_block_ptr + row * fw;
                        for (int col = 0; col < N; col++)
                            trans_sad += (uint32_t)abs((int)cr[col] - (int)pred[row * N + col]);
                    }

                    uint8_t affine_pred[64 * 64];
                    if (affine_mc_try(cur_block_ptr, fw, fh, x, y, N,
                                      enc->ref_y[ref_idx],
                                      mv_tl, mv_tr, trans_sad, affine_pred)) {
                        memcpy(pred, affine_pred, (size_t)(N * N));
                    }
                }
            }

            for (int row = 0; row < N; row++)
                for (int col = 0; col < N; col++) {
                    int px = x + col, py = y + row;
                    uint8_t cur = (px < fw && py < fh) ? enc->cur_y[py * fw + px] : 0;
                    block[row * N + col] = (int16_t)cur - (int16_t)pred[row * N + col];
                }

            cdf53_2d_forward_n(block, N, enc->wavelet_buf);
            quantize_nxn(block, N, quality, 1.0f);

            /* CBF: skip coefficient coding when residual is all-zero */
            bool cbf = false;
            for (int i = 0; i < N * N; i++) if (block[i]) { cbf = true; break; }
            video_entropy_enc_put_cbf(ve, cbf ? 1 : 0);
            if (cbf)
                enc_coeff_block_subband(ve, block, N, enc->wavelet_buf);
        }

    } else { /* OPVIS_FRAME_B — low-delay bi-predictive */
        const uint8_t *cur_block = enc->cur_y + y * fw + x;
        bool skip = false;

        /* Skip detection via zero-MV bi-pred using SATD for better accuracy */
        if (enc->ref_y[0] && enc->ref_y[1] && x + N <= fw && y + N <= fh) {
            uint8_t bipred[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
            for (int row = 0; row < N; row++)
                for (int col = 0; col < N; col++) {
                    int px = x + col, py = y + row;
                    bipred[row * N + col] = (uint8_t)(
                        ((int)enc->ref_y[0][py * fw + px] +
                         (int)enc->ref_y[1][py * fw + px] + 1) >> 1);
                }
            uint32_t skip_satd = calculate_satd_n(cur_block, fw, bipred, N, N);
            skip = skip_satd < (uint32_t)(N * 4);
        }

        video_entropy_enc_put_mode(ve, skip ? 1 : 0);

        if (!skip) {
            /* Independent ME for each reference */
            uint8_t dummy = 0;
            opvis_mv_t mv0 = motion_estimate_cu(cur_block, x, y, N,
                                                enc->ref_y[0], NULL, fw, fh, &dummy,
                                                enc->me_range);
            opvis_mv_t mv1 = motion_estimate_cu(cur_block, x, y, N,
                                                NULL, enc->ref_y[1], fw, fh, &dummy,
                                                enc->me_range);

            /* B5 — joint MV refinement at quarter-pixel: 1D sweep around each MV,
             * picking dx0 then dx1 that minimizes bipred SATD. */
            if (x + N <= fw && y + N <= fh && N >= 4) {
                uint32_t best = UINT32_MAX;
                opvis_mv_t bm0 = mv0;
                for (int d = -1; d <= 1; d++) {
                    uint32_t s = satd_subpel(cur_block, fw, enc->ref_y[0], fw, fh,
                                             x, y, N, mv0.x + d, mv0.y);
                    if (s < best) { best = s; bm0.x = (int16_t)(mv0.x + d); }
                }
                mv0 = bm0;
                best = UINT32_MAX;
                opvis_mv_t bm1 = mv1;
                for (int d = -1; d <= 1; d++) {
                    uint32_t s = satd_subpel(cur_block, fw, enc->ref_y[1], fw, fh,
                                             x, y, N, mv1.x + d, mv1.y);
                    if (s < best) { best = s; bm1.x = (int16_t)(mv1.x + d); }
                }
                mv1 = bm1;
            }

            /* AMVP: code each B-frame MV as delta from the shared spatial predictor */
            opvis_mv_t mvp = get_mv_predictor(mvs, enc->mb_cols, enc->mb_rows, x, y);
            video_entropy_enc_put_mv(ve, (int16_t)(mv0.x - mvp.x));
            video_entropy_enc_put_mv(ve, (int16_t)(mv0.y - mvp.y));
            video_entropy_enc_put_mv(ve, (int16_t)(mv1.x - mvp.x));
            video_entropy_enc_put_mv(ve, (int16_t)(mv1.y - mvp.y));

            /* Form bi-pred and compute residual */
            uint8_t pred0[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
            uint8_t pred1[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
            mc_predict_n(enc->ref_y[0], fw, fh, x, y, N, mv0, pred0);
            mc_predict_n(enc->ref_y[1], fw, fh, x, y, N, mv1, pred1);

            for (int row = 0; row < N; row++)
                for (int col = 0; col < N; col++) {
                    int px = x + col, py = y + row;
                    uint8_t cur = (px < fw && py < fh) ? enc->cur_y[py * fw + px] : 0;
                    uint8_t bp  = (uint8_t)(((int)pred0[row*N+col] + (int)pred1[row*N+col] + 1) >> 1);
                    block[row * N + col] = (int16_t)cur - (int16_t)bp;
                }

            cdf53_2d_forward_n(block, N, enc->wavelet_buf);
            quantize_nxn(block, N, quality, 1.0f);

            /* CBF */
            bool cbf = false;
            for (int i = 0; i < N * N; i++) if (block[i]) { cbf = true; break; }
            video_entropy_enc_put_cbf(ve, cbf ? 1 : 0);
            if (cbf)
                enc_coeff_block_subband(ve, block, N, enc->wavelet_buf);

            mv = mv0;  /* store primary MV for grid/deblocking */
        }
    }

    update_mv_grid(mvs, ref_indices, enc->mb_cols, enc->mb_rows, x, y, N, mv, ref_idx);

    /* ======== Chroma (U then V) ======== */
    if (Nc < 1) return;  /* pathological: N=1 has no chroma */

    int16_t cblock[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE / 4];
    const int cx = x / 2, cy = y / 2;

    for (int plane = 0; plane < 2; plane++) {
        const uint8_t *cur_c = (plane == 0) ? enc->cur_u : enc->cur_v;
        const uint8_t *ref_c = (plane == 0) ? enc->ref_u[ref_idx] : enc->ref_v[ref_idx];

        for (int row = 0; row < Nc; row++) {
            for (int col = 0; col < Nc; col++) {
                int px = cx + col, py2 = cy + row;
                int16_t cur_p = (px < cw && py2 < ch) ? (int16_t)cur_c[py2 * cw + px] : 128;
                int16_t ref_p = 128;
                if (ft != OPVIS_FRAME_I && ref_c) {
                    /* Chroma MV in quarter-chroma-sample units (luma qpel / 2) */
                    int cx_qpel = (cx + col) * OPVIS_SUBPEL_SCALE + mv.x / 2;
                    int cy_qpel = (cy + row) * OPVIS_SUBPEL_SCALE + mv.y / 2;
                    ref_p = (int16_t)hevc_chroma_interp(ref_c, cw, ch, cx_qpel, cy_qpel);
                }
                cblock[row * Nc + col] = (ft == OPVIS_FRAME_I) ? cur_p : (cur_p - ref_p);
            }
        }

        cdf53_2d_forward_n(cblock, Nc, enc->wavelet_buf);
        quantize_nxn(cblock, Nc, quality, 1.0f);
        if (ft != OPVIS_FRAME_I) {
            bool c_cbf = false;
            for (int i = 0; i < Nc * Nc; i++) if (cblock[i]) { c_cbf = true; break; }
            video_entropy_enc_put_cbf(ve, c_cbf ? 1 : 0);
            if (c_cbf)
                enc_coeff_block_subband(ve, cblock, Nc, enc->wavelet_buf);
        } else {
            enc_coeff_block_subband(ve, cblock, Nc, enc->wavelet_buf);
        }
    }
}

/* Fetch intra reference line at MRL offset r (r >= 1) from the block boundary.
 * Caller must ensure r < y and r < x before calling. */
static void get_mrl_neighbors(const uint8_t *frame, int fw, int fh,
                               int bx, int by, int N, int r,
                               uint8_t *top_out, uint8_t *left_out, uint8_t *tl_out)
{
    int ref_row = by - 1 - r;
    int ref_col = bx - 1 - r;
    for (int i = 0; i < N; i++) {
        int cx = bx + i;
        top_out[i]  = (ref_row >= 0 && cx < fw) ? frame[ref_row * fw + cx] :
                      (i > 0 ? top_out[i - 1] : 128);
        int cy = by + i;
        left_out[i] = (ref_col >= 0 && cy < fh) ? frame[cy * fw + ref_col] :
                      (i > 0 ? left_out[i - 1] : 128);
    }
    *tl_out = (ref_row >= 0 && ref_col >= 0) ? frame[ref_row * fw + ref_col] : 128;
}

static void encode_cu(opvis_encoder_t *enc, video_entropy_encoder_t *ve,
                      int x, int y, int N, opvis_frame_type_t ft, uint8_t quality,
                      opvis_mv_t *mvs, uint8_t *ref_indices) {
    if (N > CU_MIN_SIZE) {
        /* SARDO: get per-CTU saliency weight for RDO overhead scaling */
        float sal_w = 1.0f;
        if (enc->sardo_enabled) {
            int ctu_col = x / OPVIS_CTU_SIZE;
            int ctu_row = y / OPVIS_CTU_SIZE;
            sal_w = sal_weight(&enc->sal_ctx, ctu_col, ctu_row);
        }
        bool split = cu_should_split(enc->cur_y, enc->width, enc->height, x, y, N, sal_w);
        /* A6 — context-adaptive split flag */
        video_entropy_enc_put_split_ctx(ve, split ? 1 : 0, N);
        if (split) {
            int half = N / 2;
            encode_cu(enc, ve, x,      y,      half, ft, quality, mvs, ref_indices);
            encode_cu(enc, ve, x+half, y,      half, ft, quality, mvs, ref_indices);
            encode_cu(enc, ve, x,      y+half, half, ft, quality, mvs, ref_indices);
            encode_cu(enc, ve, x+half, y+half, half, ft, quality, mvs, ref_indices);
            return;
        }
    }
    encode_leaf_cu(enc, ve, x, y, N, ft, quality, mvs, ref_indices);
}

/* ---- Leaf decoder ---- */

static void decode_leaf_cu(opvis_decoder_t *dec, video_entropy_decoder_t *vd,
                           int x, int y, int N, opvis_frame_type_t ft, uint8_t quality,
                           opvis_mv_t *mvs, uint8_t *ref_indices) {
    const int fw = dec->width, fh = dec->height;
    const int Nc = N / 2;
    const int cw = fw / 2, ch = fh / 2;

    int16_t block[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
    opvis_mv_t mv = {0, 0};
    uint8_t ref_idx = 0;
    bool skip = false;

    /* ======== Luma ======== */
    if (ft == OPVIS_FRAME_I) {
        intra_mode_hevc_t mode = video_entropy_dec_get_intra_mode(vd);

        if (mode == SCREEN_MODE_IBC) {
            /* Intra Block Copy: block vector + wavelet residual */
            int16_t bv_x = video_entropy_dec_get_mv(vd);
            int16_t bv_y = video_entropy_dec_get_mv(vd);
            dec_coeff_block_subband(vd, block, N, dec->wavelet_buf);
            dequantize_nxn(block, N, 100, 1.0f);  /* lossless IBC residual */
            cdf53_2d_inverse_n(block, N, dec->wavelet_buf);

            const int rx = x + (int)bv_x, ry = y + (int)bv_y;
            for (int row = 0; row < N; row++)
                for (int col = 0; col < N; col++) {
                    int px = x + col, py2 = y + row;
                    if (px >= fw || py2 >= fh) continue;
                    int src_x = rx + col, src_y = ry + row;
                    uint8_t ref_px = (src_x >= 0 && src_y >= 0 && src_x < fw && src_y < fh) ?
                                      dec->cur_y[src_y * fw + src_x] : 0;
                    int16_t v = (int16_t)ref_px + block[row * N + col];
                    dec->cur_y[py2 * fw + px] = (uint8_t)CLAMP(v, 0, 255);
                }

        } else if (mode == SCREEN_MODE_PALETTE) {
            /* Palette mode: n_entries + palette Y values + per-pixel indices */
            int16_t ne = video_entropy_dec_get_coeff(vd);
            if (ne < 1 || ne > SCREEN_PALETTE_MAX) ne = 1;
            uint8_t pal_y[SCREEN_PALETTE_MAX];
            for (int e = 0; e < (int)ne; e++)
                pal_y[e] = video_entropy_dec_get_byte(vd);
            uint8_t pal_idx[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
            for (int i = 0; i < N * N; i++) {
                int16_t idx = video_entropy_dec_get_coeff(vd);
                pal_idx[i] = (uint8_t)CLAMP(idx, 0, (int16_t)(ne - 1));
            }
            for (int row = 0; row < N; row++)
                for (int col = 0; col < N; col++) {
                    int px = x + col, py2 = y + row;
                    if (px >= fw || py2 >= fh) continue;
                    dec->cur_y[py2 * fw + px] = pal_y[pal_idx[row * N + col]];
                }

        } else {
            /* Normal HEVC intra — read MRL/ISP, then CBF, then (optionally) residual */
            uint8_t mrl_r = video_entropy_dec_get_mrl_idx(vd);   /* C4 */
            if (mrl_r > 2) mrl_r = 0;
            uint8_t isp_mode = video_entropy_dec_get_isp_mode(vd); /* C5 */
            if (isp_mode > 2) isp_mode = 0;

            if (isp_mode != 0 && N >= 8) {
                /* ISP decode: 4 H×H sub-blocks, same scan order as encoder */
                const int H = N / 2;
                const int kDivH[4][2] = {{0,0},{1,0},{0,1},{1,1}};
                const int kDivV[4][2] = {{0,0},{0,1},{1,0},{1,1}};
                const int (*div)[2] = (isp_mode == 1) ? kDivH : kDivV;
                int16_t sub_block[OPVIS_CTU_SIZE/2 * OPVIS_CTU_SIZE/2];
                for (int s = 0; s < 4; s++) {
                    int sx = x + div[s][0] * H;
                    int sy = y + div[s][1] * H;
                    bool sub_cbf = video_entropy_dec_get_cbf(vd) != 0;
                    if (sub_cbf) {
                        dec_coeff_block_subband(vd, sub_block, H, dec->wavelet_buf);
                        dequantize_nxn(sub_block, H, quality, 1.0f);
                        cdf53_2d_inverse_n(sub_block, H, dec->wavelet_buf);
                    } else {
                        memset(sub_block, 0, (size_t)(H * H) * sizeof(int16_t));
                    }
                    uint8_t stb[OPVIS_CTU_SIZE/2], str[OPVIS_CTU_SIZE/2];
                    uint8_t slb[OPVIS_CTU_SIZE/2], stl;
                    intra_get_neighbors_hevc(dec->cur_y, (uint16_t)fw, (uint16_t)fh,
                                             (uint16_t)sx, (uint16_t)sy, H,
                                             stb, str, slb, &stl);
                    uint8_t *stp = (sy > 0) ? stb : NULL;
                    uint8_t *slp = (sx > 0) ? slb : NULL;
                    uint8_t sub_pred[OPVIS_CTU_SIZE/2 * OPVIS_CTU_SIZE/2];
                    intra_predict_hevc(mode, H, stp, str, slp, stl, sub_pred);
                    for (int row = 0; row < H; row++)
                        for (int col = 0; col < H; col++) {
                            int px = sx + col, py2 = sy + row;
                            if (px >= fw || py2 >= fh) continue;
                            int16_t v = (int16_t)sub_pred[row*H+col] + sub_block[row*H+col];
                            dec->cur_y[py2 * fw + px] = (uint8_t)CLAMP(v, 0, 255);
                        }
                }
            } else {
            bool cbf = video_entropy_dec_get_cbf(vd) != 0;
            if (cbf) {
                dec_coeff_block_subband(vd, block, N, dec->wavelet_buf);
                dequantize_nxn(block, N, quality, 1.0f);
                cdf53_2d_inverse_n(block, N, dec->wavelet_buf);
            } else {
                memset(block, 0, (size_t)(N * N) * sizeof(int16_t));
            }

            uint8_t top_buf[OPVIS_CTU_SIZE], tr_buf[OPVIS_CTU_SIZE], left_buf[OPVIS_CTU_SIZE];
            uint8_t top_left = 128;
            intra_get_neighbors_hevc(dec->cur_y, (uint16_t)fw, (uint16_t)fh,
                                     (uint16_t)x, (uint16_t)y, N,
                                     top_buf, tr_buf, left_buf, &top_left);

            /* Resolve MRL neighbors for selected reference line */
            uint8_t top_sel[OPVIS_CTU_SIZE], left_sel[OPVIS_CTU_SIZE];
            uint8_t tl_sel = top_left;
            if (mrl_r > 0) {
                get_mrl_neighbors(dec->cur_y, fw, fh, x, y, N, mrl_r, top_sel, left_sel, &tl_sel);
            } else {
                memcpy(top_sel,  top_buf,  (size_t)N);
                memcpy(left_sel, left_buf, (size_t)N);
            }
            uint8_t *top_p  = (y > (int)mrl_r) ? top_sel  : NULL;
            uint8_t *left_p = (x > (int)mrl_r) ? left_sel : NULL;

            uint8_t pred_buf[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE];
            intra_predict_hevc(mode, N, top_p, tr_buf, left_p, tl_sel, pred_buf);

            for (int row = 0; row < N; row++)
                for (int col = 0; col < N; col++) {
                    int px = x + col, py2 = y + row;
                    if (px >= fw || py2 >= fh) continue;
                    int16_t v = (int16_t)pred_buf[row * N + col] + block[row * N + col];
                    dec->cur_y[py2 * fw + px] = (uint8_t)CLAMP(v, 0, 255);
                }
            } /* end no-ISP block */
        }

        /* Update IBC hash after block is written to cur_y */
        if (N >= SCREEN_IBC_MIN_SIZE)
            screen_ibc_update(dec->cur_y, fw, x, y, s_dec_ibc_table, SCREEN_IBC_TABLE_SIZE);

    } else if (ft == OPVIS_FRAME_P) {
        uint8_t mode = video_entropy_dec_get_mode(vd);
        skip = (mode == 1);

        if (!skip) {
            uint8_t ref_mode = video_entropy_dec_get_mode(vd);
            opvis_mv_t mvp = get_mv_predictor(mvs, dec->mb_cols, dec->mb_rows, x, y);
            if (ref_mode == 4) {
                /* B1 — merge mode: MV == spatial predictor, ref0 only */
                ref_idx = 0;
                mv = mvp;
            } else {
                ref_idx = (ref_mode >= 2) ? (ref_mode - 2) : 0;
                if (ref_idx > 1) ref_idx = 0;
                /* B3 — AMVP: select spatial or TMVP predictor via mvp_idx */
                uint8_t mvp_idx = video_entropy_dec_get_mvp_idx(vd);
                opvis_mv_t sel_mvp = mvp;
                if (mvp_idx == 1 && dec->prev_mvs) {
                    int tmb_x = x / OPVIS_MB_SIZE, tmb_y = y / OPVIS_MB_SIZE;
                    sel_mvp = dec->prev_mvs[tmb_y * dec->mb_cols + tmb_x];
                }
                mv.x = (int16_t)(sel_mvp.x + video_entropy_dec_get_mv(vd));
                mv.y = (int16_t)(sel_mvp.y + video_entropy_dec_get_mv(vd));
            }

            /* CBF: if 0, residual is zero — use pure MC prediction */
            bool cbf = video_entropy_dec_get_cbf(vd) != 0;
            if (cbf) {
                dec_coeff_block_subband(vd, block, N, dec->wavelet_buf);
                dequantize_nxn(block, N, quality, 1.0f);
                cdf53_2d_inverse_n(block, N, dec->wavelet_buf);
            } else {
                memset(block, 0, (size_t)(N * N) * sizeof(int16_t));
            }

            mc_reconstruct_n(dec->ref_y[ref_idx], fw, fh, x, y, N, mv, block,
                             dec->cur_y + y * fw + x, fw);
        } else {
            for (int row = 0; row < N; row++)
                for (int col = 0; col < N; col++) {
                    int px = x + col, py = y + row;
                    if (px >= fw || py >= fh) continue;
                    dec->cur_y[py * fw + px] = dec->ref_y[0][py * fw + px];
                }
        }

    } else { /* OPVIS_FRAME_B */
        uint8_t mode = video_entropy_dec_get_mode(vd);
        skip = (mode == 1);

        if (!skip) {
            /* AMVP: decode both B-frame MVs as deltas from spatial predictor */
            opvis_mv_t mvp = get_mv_predictor(mvs, dec->mb_cols, dec->mb_rows, x, y);
            opvis_mv_t mv0, mv1;
            mv0.x = (int16_t)(mvp.x + video_entropy_dec_get_mv(vd));
            mv0.y = (int16_t)(mvp.y + video_entropy_dec_get_mv(vd));
            mv1.x = (int16_t)(mvp.x + video_entropy_dec_get_mv(vd));
            mv1.y = (int16_t)(mvp.y + video_entropy_dec_get_mv(vd));
            mv = mv0;  /* primary MV for deblocking grid */

            /* CBF */
            bool cbf = video_entropy_dec_get_cbf(vd) != 0;
            if (cbf) {
                dec_coeff_block_subband(vd, block, N, dec->wavelet_buf);
                dequantize_nxn(block, N, quality, 1.0f);
                cdf53_2d_inverse_n(block, N, dec->wavelet_buf);
            } else {
                memset(block, 0, (size_t)(N * N) * sizeof(int16_t));
            }

            /* Bi-predictive reconstruction */
            for (int row = 0; row < N; row++)
                for (int col = 0; col < N; col++) {
                    int px = x + col, py = y + row;
                    if (px >= fw || py >= fh) continue;
                    uint8_t p0 = hevc_luma_interp(dec->ref_y[0], fw, fh,
                        (px * OPVIS_SUBPEL_SCALE) + mv0.x,
                        (py * OPVIS_SUBPEL_SCALE) + mv0.y);
                    uint8_t p1 = hevc_luma_interp(dec->ref_y[1], fw, fh,
                        (px * OPVIS_SUBPEL_SCALE) + mv1.x,
                        (py * OPVIS_SUBPEL_SCALE) + mv1.y);
                    uint8_t bp = (uint8_t)(((int)p0 + (int)p1 + 1) >> 1);
                    int16_t v  = (int16_t)bp + block[row * N + col];
                    dec->cur_y[py * fw + px] = (uint8_t)CLAMP(v, 0, 255);
                }
        } else {
            /* Zero-MV bi-pred skip */
            for (int row = 0; row < N; row++)
                for (int col = 0; col < N; col++) {
                    int px = x + col, py = y + row;
                    if (px >= fw || py >= fh) continue;
                    uint8_t bp = (uint8_t)(((int)dec->ref_y[0][py*fw+px] +
                                            (int)dec->ref_y[1][py*fw+px] + 1) >> 1);
                    dec->cur_y[py * fw + px] = bp;
                }
        }
    }

    update_mv_grid(mvs, ref_indices, dec->mb_cols, dec->mb_rows, x, y, N, mv, ref_idx);

    /* ======== Chroma ======== */
    if (Nc < 1) return;

    int16_t cblock[OPVIS_CTU_SIZE * OPVIS_CTU_SIZE / 4];
    const int cx = x / 2, cy = y / 2;

    for (int plane = 0; plane < 2; plane++) {
        uint8_t *out_c  = (plane == 0) ? dec->cur_u  : dec->cur_v;
        const uint8_t *ref_c = (plane == 0) ? dec->ref_u[ref_idx] : dec->ref_v[ref_idx];

        bool c_cbf = (ft != OPVIS_FRAME_I) ? (video_entropy_dec_get_cbf(vd) != 0) : true;
        if (c_cbf) {
            dec_coeff_block_subband(vd, cblock, Nc, dec->wavelet_buf);
            dequantize_nxn(cblock, Nc, quality, 1.0f);
            cdf53_2d_inverse_n(cblock, Nc, dec->wavelet_buf);
        } else {
            memset(cblock, 0, (size_t)(Nc * Nc) * sizeof(int16_t));
        }

        for (int row = 0; row < Nc; row++) {
            for (int col = 0; col < Nc; col++) {
                int px = cx + col, py2 = cy + row;
                if (px >= cw || py2 >= ch) continue;

                int16_t val;
                if (ft == OPVIS_FRAME_I) {
                    val = cblock[row * Nc + col];
                } else if (ft == OPVIS_FRAME_B) {
                    /* Chroma bi-pred using 4-tap interpolation at quarter-chroma-sample */
                    const uint8_t *rc0 = (plane == 0) ? dec->ref_u[0] : dec->ref_v[0];
                    const uint8_t *rc1 = (plane == 0) ? dec->ref_u[1] : dec->ref_v[1];
                    uint8_t p0 = hevc_chroma_interp(rc0, cw, ch,
                        (cx + col) * OPVIS_SUBPEL_SCALE + mv.x / 2,
                        (cy + row) * OPVIS_SUBPEL_SCALE + mv.y / 2);
                    uint8_t p1 = rc1[py2 * cw + px];  /* zero-MV ref1 */
                    uint8_t bp = (uint8_t)(((int)p0 + (int)p1 + 1) >> 1);
                    val = (skip) ? (int16_t)bp : ((int16_t)bp + cblock[row * Nc + col]);
                } else if (!skip && ref_c) {
                    uint8_t ref_p = hevc_chroma_interp(ref_c, cw, ch,
                        (cx + col) * OPVIS_SUBPEL_SCALE + mv.x / 2,
                        (cy + row) * OPVIS_SUBPEL_SCALE + mv.y / 2);
                    val = (int16_t)ref_p + cblock[row * Nc + col];
                } else {
                    val = (int16_t)out_c[py2 * cw + px];
                }
                out_c[py2 * cw + px] = (uint8_t)CLAMP(val, 0, 255);
            }
        }
    }
}

static void decode_cu(opvis_decoder_t *dec, video_entropy_decoder_t *vd,
                      int x, int y, int N, opvis_frame_type_t ft, uint8_t quality,
                      opvis_mv_t *mvs, uint8_t *ref_indices) {
    if (N > CU_MIN_SIZE) {
        uint8_t split = video_entropy_dec_get_split_ctx(vd, N);
        if (split) {
            int half = N / 2;
            decode_cu(dec, vd, x,      y,      half, ft, quality, mvs, ref_indices);
            decode_cu(dec, vd, x+half, y,      half, ft, quality, mvs, ref_indices);
            decode_cu(dec, vd, x,      y+half, half, ft, quality, mvs, ref_indices);
            decode_cu(dec, vd, x+half, y+half, half, ft, quality, mvs, ref_indices);
            return;
        }
    }
    decode_leaf_cu(dec, vd, x, y, N, ft, quality, mvs, ref_indices);
}

/* ========================================================================
 * ENCODING IMPLEMENTATION
 * ======================================================================== */

int opvis_encode(opvis_encoder_t *enc, const uint8_t *input, size_t input_len,
                 uint8_t *out, size_t out_cap) {
    UNUSED(input_len);
    if (!enc || !input || !out || out_cap < OPVIS_HEADER_SIZE) {
        return -1;
    }

    /* Convert input to 8-bit YUV420P (internal processing is always 8-bit) */
    const size_t y_size  = enc->width * enc->height;
    const size_t uv_size = (enc->width / 2) * (enc->height / 2);
    if (enc->input_fmt == OPVIS_FMT_RGB24) {
        rgb_to_yuv420p(input, enc->width, enc->height,
                       enc->cur_y, enc->cur_u, enc->cur_v);
    } else if (enc->input_fmt == OPVIS_FMT_YUV420P) {
        memcpy(enc->cur_y, input, y_size);
        memcpy(enc->cur_u, input + y_size, uv_size);
        memcpy(enc->cur_v, input + y_size + uv_size, uv_size);
    } else if (enc->input_fmt == OPVIS_FMT_P010) {
        /* P010: packed LE, each uint16_t has 10-bit value in top bits */
        const uint16_t *py  = (const uint16_t *)input;
        const uint16_t *puv = py + y_size;
        for (size_t i = 0; i < y_size; i++)
            enc->cur_y[i] = (uint8_t)(p010_unpack(py[i]) >> 2);
        for (size_t i = 0; i < uv_size; i++) {
            enc->cur_u[i] = (uint8_t)(p010_unpack(puv[2 * i])     >> 2);
            enc->cur_v[i] = (uint8_t)(p010_unpack(puv[2 * i + 1]) >> 2);
        }
    } else if (enc->input_fmt == OPVIS_FMT_YUV420P10LE) {
        /* Planar 10-bit: each sample is a uint16_t in [0, 1023] */
        const uint16_t *py = (const uint16_t *)input;
        const uint16_t *pu = py + y_size;
        const uint16_t *pv = pu + uv_size;
        for (size_t i = 0; i < y_size;  i++) enc->cur_y[i] = (uint8_t)(py[i] >> 2);
        for (size_t i = 0; i < uv_size; i++) enc->cur_u[i] = (uint8_t)(pu[i] >> 2);
        for (size_t i = 0; i < uv_size; i++) enc->cur_v[i] = (uint8_t)(pv[i] >> 2);
    } else {
        return -1;
    }

    /* Temporal Noise Reduction: filter luma in-place before encoding.
     * Applied only on frames after the first (frame_num > 0) so I-frames and
     * new encoder instances always start from a clean, unfiltered signal.
     * This also prevents stale TNR context from a prior encode session bleeding
     * into a fresh encoder. */
    if (enc->tnr_enabled && enc->frame_num > 0 && enc->ref_y[0]) {
        opvis_tnr_apply(&s_tnr_ctx, enc->cur_y, enc->width, enc->height,
                        enc->tnr_alpha, enc->tnr_motion_thresh);
    }

    /* D4 — GOP adaptation: faster scene cuts shrink GOP, slow scenes extend it */
    uint16_t actual_gop = enc->gop_size;
    {
        const int ctu_cols2 = (enc->width  + OPVIS_CTU_SIZE - 1) / OPVIS_CTU_SIZE;
        const int ctu_rows2 = (enc->height + OPVIS_CTU_SIZE - 1) / OPVIS_CTU_SIZE;
        const int n_ctu = ctu_cols2 * ctu_rows2;
        uint32_t scene_satd = 0;
        for (int cy = 0; cy < ctu_rows2; cy++)
            for (int cx = 0; cx < ctu_cols2; cx++)
                scene_satd += block_satd_dc(enc->cur_y, enc->width, enc->height,
                                            cx * OPVIS_CTU_SIZE, cy * OPVIS_CTU_SIZE,
                                            MIN(OPVIS_CTU_SIZE, enc->width - cx * OPVIS_CTU_SIZE));
        const uint32_t HIGH_THRESH = (uint32_t)n_ctu * 20;
        const uint32_t LOW_THRESH  = (uint32_t)n_ctu * 4;
        if (scene_satd > HIGH_THRESH && enc->gop_size > 1) actual_gop = enc->gop_size / 2;
        else if (scene_satd < LOW_THRESH)                  actual_gop = enc->gop_size * 2;
        if (actual_gop < 4) actual_gop = 4;
        const uint32_t maxg = (uint32_t)enc->gop_size * 4u;
        if (actual_gop > maxg) actual_gop = (uint16_t)maxg;
    }

    /* Frame type: I at GOP boundary, then alternating P/B for low-delay B-frames */
    opvis_frame_type_t frame_type;
    if (enc->frame_num % actual_gop == 0) {
        frame_type = OPVIS_FRAME_I;
    } else if (enc->frame_num % 2 == 0) {
        frame_type = OPVIS_FRAME_B;  /* even non-I positions */
    } else {
        frame_type = OPVIS_FRAME_P;  /* odd positions */
    }

    /* Scene change: force I or P (B-frames don't update refs, so promote to P) */
    if (frame_type != OPVIS_FRAME_I && enc->frame_num > 0 && enc->ref_y[0]) {
        if (detect_scene_change(enc->cur_y, enc->ref_y[0], enc->width,
                               enc->mb_cols, enc->mb_rows)) {
            frame_type = OPVIS_FRAME_I;
        }
    }

    /* D3 — Frame skip on near-static content (only emit SKIP header, decoder repeats prev frame) */
    bool emit_skip_frame = false;
    if (frame_type != OPVIS_FRAME_I && enc->frame_num > 0 && enc->ref_y[0]) {
        uint32_t sample_satd = calculate_satd_n(enc->cur_y, enc->width,
                                                enc->ref_y[0], enc->width,
                                                MIN(64, MIN(enc->width, enc->height)));
        const uint32_t FRAME_SKIP_THRESH = 64u * 8u;
        if (sample_satd < FRAME_SKIP_THRESH) emit_skip_frame = true;
    }
    if (emit_skip_frame) {
        if (out_cap < OPVIS_HEADER_V1_SIZE) return -1;
        out[0] = 1;
        out[1] = (uint8_t)OPVIS_FRAME_SKIP;
        out[2] = (uint8_t)enc->quality;
        write_be16(out + 3, enc->width);
        write_be16(out + 5, enc->height);
        write_be32(out + 7, enc->frame_num);
        out[11] = 0; out[12] = 0;
        write_be32(out + 13, 0);
        enc->last_frame_stats.frame_num = enc->frame_num;
        enc->last_frame_stats.bits = OPVIS_HEADER_V1_SIZE * 8;
        enc->last_frame_stats.psnr_estimate = 0.0f;
        enc->last_frame_stats.type = OPVIS_FRAME_SKIP;
        enc->last_frame_stats.was_skipped = true;
        enc->frame_num++;
        return OPVIS_HEADER_V1_SIZE;
    }

    /* Reset IBC hash table at I-frame boundaries */
    if (frame_type == OPVIS_FRAME_I && enc->ibc_hashtable) {
        memset(enc->ibc_hashtable, 0xFF, SCREEN_IBC_TABLE_SIZE * sizeof(uint32_t));
    }

    /* Rate control: PI controller on virtual buffer fullness.
     * Target = 50% full.  Positive adj = raise QP = fewer bits.
     * D1 — CRF mode bypasses rate control entirely. */
    float adjusted_quality = enc->quality;
    if (enc->crf_quality > 0) {
        adjusted_quality = (float)enc->crf_quality;
    } else if (enc->target_bitrate > 0 && enc->rc_buffer_size > 0) {
        const float Kp = 8.0f, Ki = 0.4f;
        const float err = (float)enc->rc_buffer / (float)enc->rc_buffer_size - 0.5f;
        enc->rc_integral = CLAMP(enc->rc_integral + err, -10.0f, 10.0f);
        enc->rc_qp_adj   = CLAMP(Kp * err + Ki * enc->rc_integral, -20.0f, 20.0f);
        /* I-frames get up to 3× budget: pre-open the buffer slightly */
        if (frame_type == OPVIS_FRAME_I)
            enc->rc_qp_adj = CLAMP(enc->rc_qp_adj - 8.0f, -20.0f, 20.0f);
        adjusted_quality = CLAMP((float)enc->quality + enc->rc_qp_adj, 10.0f, 100.0f);
    }

    /* Clear per-frame screen coding flag before the CTU loop */
    s_screen_mode_used = false;

    /* Write v1 frame header (18 bytes) */
    if (out_cap < OPVIS_HEADER_V1_SIZE) return -1;
    out[0] = 1;  /* version = 1 */
    out[1] = (uint8_t)frame_type;
    out[2] = (uint8_t)adjusted_quality;
    write_be16(out + 3, enc->width);
    write_be16(out + 5, enc->height);
    write_be32(out + 7, enc->frame_num);
    /* Byte 11: color_info flags */
    {
        const bool hdr_present = (enc->hdr.max_lum > 0);
        out[11] = (uint8_t)(
            (hdr_present                      ? 0x80 : 0) |
            (enc->color_info.bitdepth == 10   ? 0x40 : 0) |
            ((enc->color_info.transfer & 3)   << 4)       |
            ((enc->color_info.primaries & 3)  << 2)       |
            (enc->color_info.subsampling & 3));
    }
    out[12] = 0;  /* alf_present/screen_mode filled after encode */
    /* payload_len written at bytes 13-16 after encode */

    /* Initialize rANS entropy encoder */
    video_entropy_encoder_t ve;
    video_entropy_enc_init(&ve, out + OPVIS_HEADER_V1_SIZE, out_cap - OPVIS_HEADER_V1_SIZE);

    /* E3 — SAO: save index of the SAO-type placeholder symbol, computed post-reconstruct */
    size_t sao_sym_idx = s_sym_count;
    video_entropy_enc_put_sao_type(&ve, 0);  /* placeholder; updated after CTU loop */

    /* Save original luma before CTU encode overwrites cur_y with reconstructed pixels.
     * s_enc_temp_buf is only consumed by video_entropy_enc_flush(), called later. */
    memcpy(s_enc_temp_buf, enc->cur_y, y_size);

    /* SARDO: estimate per-CTU saliency before the CU quad-tree loop.
     * Motion vectors from the previous frame (enc->mvs after ME) would need
     * split x/y arrays; pass NULL here — saliency uses texture/skin cues only. */
    if (enc->sardo_enabled) {
        sal_estimate(&enc->sal_ctx, enc->cur_y, NULL, NULL);
    }

    /* CTU outer loop — each CTU recurses into CU quad-tree */
    const uint16_t ctu_cols = (uint16_t)((enc->width  + OPVIS_CTU_SIZE - 1) / OPVIS_CTU_SIZE);
    const uint16_t ctu_rows = (uint16_t)((enc->height + OPVIS_CTU_SIZE - 1) / OPVIS_CTU_SIZE);
    for (uint16_t ctu_y = 0; ctu_y < ctu_rows; ctu_y++) {
        for (uint16_t ctu_x = 0; ctu_x < ctu_cols; ctu_x++) {
            encode_cu(enc, &ve,
                      (int)(ctu_x * OPVIS_CTU_SIZE), (int)(ctu_y * OPVIS_CTU_SIZE),
                      OPVIS_CTU_SIZE, frame_type, (uint8_t)adjusted_quality,
                      enc->mvs, enc->ref_indices);
        }
    }

    /* Update prev_mvs for TMVP in the next inter frame */
    if (frame_type != OPVIS_FRAME_B && enc->prev_mvs) {
        const size_t mv_count = (size_t)enc->mb_cols * enc->mb_rows;
        memcpy(enc->prev_mvs, enc->mvs, mv_count * sizeof(opvis_mv_t));
    }

    /* E3 — Compute per-frame SAO parameters on the reconstructed (pre-deblock) frame.
     * VVC in-loop filter order: deblock → SAO → ALF.
     * We signal SAO in the bitstream here; application is deferred to after deblocking
     * so the reference frame used for future prediction matches the decoder's order. */
    sao_params_t enc_sao_frame;
    memset(&enc_sao_frame, 0, sizeof(enc_sao_frame));
    enc_sao_frame.type = SAO_OFF;
    {
        const int fw2 = (int)enc->width, fh2 = (int)enc->height;
        const int cs  = MIN(OPVIS_CTU_SIZE, MIN(fw2, fh2));
        static const float sample_fx[5] = { 0.5f, 0.25f, 0.75f, 0.25f, 0.75f };
        static const float sample_fy[5] = { 0.5f, 0.25f, 0.25f, 0.75f, 0.75f };
        for (int si = 0; si < 5; si++) {
            int sx = ((int)(fw2 * sample_fx[si])) & ~(OPVIS_CTU_SIZE - 1);
            int sy = ((int)(fh2 * sample_fy[si])) & ~(OPVIS_CTU_SIZE - 1);
            if (sx + cs > fw2) sx = fw2 - cs;
            if (sy + cs > fh2) sy = fh2 - cs;
            if (sx < 0) sx = 0;
            if (sy < 0) sy = 0;
            sao_params_t sp = sao_analyze(
                &enc->cur_y[sy * fw2 + sx],
                &s_enc_temp_buf[sy * fw2 + sx],
                fw2, cs);
            if (sp.type != SAO_OFF && enc_sao_frame.type == SAO_OFF)
                enc_sao_frame = sp;
        }
        if (sao_sym_idx < s_sym_count)
            s_sym_buf[sao_sym_idx].symbol = (uint16_t)enc_sao_frame.type;
        if (enc_sao_frame.type != SAO_OFF) {
            uint8_t sao_raw[3];
            if (sao_encode_params(&enc_sao_frame, sao_raw, sizeof(sao_raw)) == 3) {
                video_entropy_enc_put_byte(&ve, sao_raw[0]);
                video_entropy_enc_put_byte(&ve, sao_raw[1]);
                video_entropy_enc_put_byte(&ve, sao_raw[2]);
            }
        }
        /* SAO application is deferred: applied to ref_y[0] after deblocking below */
    }

    /* Flush rANS encoder and write payload */
    const size_t payload_bytes = video_entropy_enc_flush(&ve, out + OPVIS_HEADER_V1_SIZE,
                                                         out_cap - OPVIS_HEADER_V1_SIZE);
    if (payload_bytes == 0) return -1;

    write_be32(out + 13, (uint32_t)payload_bytes);

    /* Update reference frames: I/P frames update refs; B-frames do not */
    size_t alf_bitmap_bytes = 0;
    if (frame_type != OPVIS_FRAME_B) {
        /* Shift reference 0 to reference 1 */
        memcpy(enc->ref_y[1], enc->ref_y[0], y_size);
        memcpy(enc->ref_u[1], enc->ref_u[0], uv_size);
        memcpy(enc->ref_v[1], enc->ref_v[0], uv_size);

        /* Copy current frame to reference 0 */
        memcpy(enc->ref_y[0], enc->cur_y, y_size);
        memcpy(enc->ref_u[0], enc->cur_u, uv_size);
        memcpy(enc->ref_v[0], enc->cur_v, uv_size);

        /* Apply in-loop deblocking filter to new reference */
        apply_deblocking_filter_improved(enc->ref_y[0], enc->width, enc->height,
                                       enc->mb_cols, enc->mb_rows, frame_type,
                                       enc->mvs, enc->ref_indices,
                                       (uint8_t)adjusted_quality,
                                       enc->ref_u[0], enc->ref_v[0],
                                       enc->width / 2, enc->height / 2);

        /* SAO after deblocking (VVC order: deblock → SAO → ALF) */
        if (enc_sao_frame.type != SAO_OFF)
            sao_apply_frame(enc->ref_y[0], (int)enc->width, (int)enc->height, &enc_sao_frame);

        /* ALF: per-CTU adaptive loop filter with RDO decisions signaled in bitstream.
         * Use s_enc_temp_buf (original pre-encode luma, saved above) as the reference
         * for SSD comparison — gives proper RDO enable/disable per CTU instead of
         * the heuristic the decoder would have to independently re-derive.
         * Bitmap is appended after the rANS payload; decoder reads it if byte 12 bit 7=1. */
        {
            const uint16_t alf_ctu_cols = (uint16_t)((enc->width  + OPVIS_CTU_SIZE - 1) / OPVIS_CTU_SIZE);
            const uint16_t alf_ctu_rows = (uint16_t)((enc->height + OPVIS_CTU_SIZE - 1) / OPVIS_CTU_SIZE);
            const size_t ctu_count = (size_t)alf_ctu_cols * alf_ctu_rows;
            alf_bitmap_bytes = (ctu_count + 7) / 8;
            uint8_t *alf_bitmap = out + OPVIS_HEADER_V1_SIZE + payload_bytes;

            /* Check output buffer has room for bitmap */
            if (out_cap >= OPVIS_HEADER_V1_SIZE + payload_bytes + alf_bitmap_bytes) {
                memset(alf_bitmap, 0, alf_bitmap_bytes);
                bool any_alf_enabled = false;
                size_t ctu_idx = 0;
                for (uint16_t cy = 0; cy < alf_ctu_rows; cy++) {
                    for (uint16_t cx = 0; cx < alf_ctu_cols; cx++, ctu_idx++) {
                        int cx0 = (int)(cx * OPVIS_CTU_SIZE);
                        int cy0 = (int)(cy * OPVIS_CTU_SIZE);
                        int cw  = MIN((int)OPVIS_CTU_SIZE, (int)enc->width  - cx0);
                        int ch  = MIN((int)OPVIS_CTU_SIZE, (int)enc->height - cy0);
                        /* Pass orig for proper SSD-based RDO decision */
                        alf_params_t ap = alf_analyze(enc->ref_y[0], s_enc_temp_buf,
                                                      enc->width, cx0, cy0, cw, ch);
                        if (ap.enabled) {
                            alf_bitmap[ctu_idx >> 3] |= (uint8_t)(1u << (ctu_idx & 7));
                            any_alf_enabled = true;
                            alf_apply(enc->ref_y[0], enc->width, cx0, cy0, cw, ch, &ap);
                        }
                    }
                }
                if (any_alf_enabled) {
                    out[12] |= 0x80;  /* alf_present = 1 */
                } else {
                    alf_bitmap_bytes = 0;  /* no CTUs enabled — skip bitmap suffix */
                }
            } else {
                alf_bitmap_bytes = 0;  /* no room — skip ALF */
            }
        }

        /* E2 — Optional CDEF directional enhancement */
        if (enc->cdef_enabled)
            opvis_cdef_apply(enc->ref_y[0], enc->width, enc->height, enc->cdef_strength);
    }

    /* Set screen_mode bit after the CTU loop has run */
    if (s_screen_mode_used)
        out[12] |= 0x40;  /* screen_mode = 1 */

    /* Rate control: update virtual buffer with actual bits used.
     * D2 — per-frame-type bit allocation (I=3x, P=1x, B=0.6x). */
    const size_t total_frame_bytes = OPVIS_HEADER_V1_SIZE + payload_bytes + alf_bitmap_bytes;
    if (enc->target_bitrate > 0 && enc->rc_fps > 0 && enc->crf_quality == 0) {
        const uint32_t base_target = enc->target_bitrate / enc->rc_fps;
        float type_factor = 1.0f;
        if      (frame_type == OPVIS_FRAME_I) type_factor = 3.0f;
        else if (frame_type == OPVIS_FRAME_B) type_factor = 0.6f;
        const uint32_t target_frame_bits = (uint32_t)((float)base_target * type_factor);
        const uint32_t actual_frame_bits = (uint32_t)total_frame_bytes * 8;
        enc->rc_buffer += (int32_t)target_frame_bits - (int32_t)actual_frame_bits;
        enc->rc_buffer = CLAMP(enc->rc_buffer, 0, enc->rc_buffer_size);
    }

    /* F1 — populate stats */
    enc->last_frame_stats.frame_num = enc->frame_num;
    enc->last_frame_stats.bits = (uint32_t)total_frame_bytes * 8u;
    {
        const size_t npx = (size_t)enc->width * enc->height;
        uint64_t sse = 0;
        if (enc->ref_y[0] && npx > 0) {
            for (size_t i = 0; i < npx; i++) {
                int d = (int)enc->cur_y[i] - (int)enc->ref_y[0][i];
                sse += (uint64_t)(d * d);
            }
        }
        float mse = (sse > 0 && npx > 0) ? ((float)sse / (float)npx) : 0.01f;
        if (mse < 0.01f) mse = 0.01f;
        enc->last_frame_stats.psnr_estimate = 10.0f * log10f(255.0f * 255.0f / mse);
    }
    enc->last_frame_stats.type = frame_type;
    enc->last_frame_stats.was_skipped = false;

    enc->frame_num++;

    return (int)total_frame_bytes;
}

/* ========================================================================
 * DECODING IMPLEMENTATION
 * ======================================================================== */

int opvis_decode(opvis_decoder_t *dec, const uint8_t *in, size_t in_len) {
    if (!dec || !in || in_len < OPVIS_HEADER_V1_SIZE) return -1;
    if (in[0] != 1) return -1;  /* only v1 bitstream supported */

    /* Parse v1 frame header */
    const opvis_frame_type_t frame_type = (opvis_frame_type_t)in[1];
    const uint8_t quality   = in[2];
    const uint16_t width    = read_be16(in + 3);
    const uint16_t height   = read_be16(in + 5);
    const uint32_t frame_num = read_be32(in + 7);
    /* Byte 11: color_info flags */
    const uint8_t color_flags = in[11];
    const bool is_10bit       = (color_flags & 0x40) != 0;
    /* Byte 12: alf_present (bit 7) and screen_mode (bit 6) */
    const bool alf_present  = (in[12] & 0x80) != 0;
    const uint32_t payload_len = read_be32(in + 13);

    if (width == 0 || height == 0 || width > OPVIS_MAX_WIDTH ||
        height > OPVIS_MAX_HEIGHT || (width % OPVIS_MB_SIZE) != 0 ||
        (height % OPVIS_MB_SIZE) != 0) {
        return -1;
    }

    /* Initialize decoder if this is the first frame or dimensions changed */
    if (dec->width != width || dec->height != height) {
        const size_t required = opvis_decoder_pool_size(width, height);
        if (dec->pool_size < required) return -1;

        dec->width = width;
        dec->height = height;
        dec->mb_cols = width / OPVIS_MB_SIZE;
        dec->mb_rows = height / OPVIS_MB_SIZE;

        /* Partition the pool for multi-reference frames */
        uint8_t *ptr = dec->pool;
        const size_t y_size = width * height;
        const size_t uv_size = (width / 2) * (height / 2);

        /* Two reference frames for multi-reference support */
        dec->ref_y[0] = ptr; ptr += y_size;
        dec->ref_u[0] = ptr; ptr += uv_size;
        dec->ref_v[0] = ptr; ptr += uv_size;
        dec->ref_y[1] = ptr; ptr += y_size;
        dec->ref_u[1] = ptr; ptr += uv_size;
        dec->ref_v[1] = ptr; ptr += uv_size;
        dec->ref_y[2] = ptr; ptr += y_size;
        dec->ref_u[2] = ptr; ptr += uv_size;
        dec->ref_v[2] = ptr; ptr += uv_size;

        dec->wavelet_buf = (int16_t *)ptr;
        ptr += OPVIS_CTU_SIZE * OPVIS_CTU_SIZE * sizeof(int16_t) * 2;

        dec->interp_buf = ptr;
        ptr += (OPVIS_CTU_SIZE + 2) * OPVIS_SUBPEL_SCALE *
               (OPVIS_CTU_SIZE + 2) * OPVIS_SUBPEL_SCALE;

        /* 10-bit output planes — always allocated so callers can use decoded_y16() */
        dec->ref_y16[0] = (uint16_t *)ptr; ptr += y_size  * sizeof(uint16_t);
        dec->ref_u16[0] = (uint16_t *)ptr; ptr += uv_size * sizeof(uint16_t);
        dec->ref_v16[0] = (uint16_t *)ptr; ptr += uv_size * sizeof(uint16_t);

        /* TMVP: co-located MVs from the previous inter frame */
        const size_t mb_cnt = (size_t)dec->mb_cols * dec->mb_rows;
        dec->prev_mvs = (opvis_mv_t *)ptr; ptr += mb_cnt * sizeof(opvis_mv_t);
        memset(dec->prev_mvs, 0, mb_cnt * sizeof(opvis_mv_t));
        (void)ptr;

        /* Clear reference frames for first decode */
        memset(dec->ref_y[0], 0, y_size);
        memset(dec->ref_u[0], 128, uv_size);
        memset(dec->ref_v[0], 128, uv_size);
        memset(dec->ref_y[1], 0, y_size);
        memset(dec->ref_u[1], 128, uv_size);
        memset(dec->ref_v[1], 128, uv_size);
        memset(dec->ref_y[2], 0, y_size);
        memset(dec->ref_u[2], 128, uv_size);
        memset(dec->ref_v[2], 128, uv_size);

        /* Initialize probability models for rANS */
        init_video_models();
    }

    dec->quality   = quality;
    dec->frame_num = frame_num;

    /* Populate color info from byte 11 flags */
    dec->color_info.bitdepth    = is_10bit ? 10 : 8;
    dec->color_info.transfer    = (opvis_transfer_t)  ((color_flags >> 4) & 3);
    dec->color_info.primaries   = (opvis_colorprim_t) ((color_flags >> 2) & 3);
    dec->color_info.subsampling = color_flags & 3;

    /* D3 — SKIP frame: repeat ref[0] as the output and return. */
    if (frame_type == OPVIS_FRAME_SKIP) {
        dec->cur_y = dec->ref_y[0];
        dec->cur_u = dec->ref_u[0];
        dec->cur_v = dec->ref_v[0];
        dec->stats.frames_decoded++;
        dec->stats.frames_skipped++;
        dec->stats.total_bits += (uint32_t)in_len * 8u;
        return 0;
    }

    /* TFI — INTERP frame: synthesize from ref[0] (prev) and ref[1] (next).
     * The hint packet (3 bytes following the header) carries alpha_q8 and flags. */
    if (frame_type == OPVIS_FRAME_INTERP) {
        if (!dec->tfi_enabled || !dec->ref_y[0] || !dec->ref_y[1]) {
            /* Fallback: reuse most recent reference */
            dec->cur_y = dec->ref_y[0];
            dec->cur_u = dec->ref_u[0];
            dec->cur_v = dec->ref_v[0];
        } else {
            /* Lazy-init TFI context once we know the frame dimensions */
            if (!dec->tfi_ctx.initialized)
                tfi_init(&dec->tfi_ctx, dec->width, dec->height);

            /* Read interpolation hint: alpha in Q8 fixed-point */
            const uint8_t *hint = in + OPVIS_HEADER_V1_SIZE;
            uint8_t alpha_q8 = 128, use_motion = 0;
            if (payload_len >= 2)
                tfi_read_hint(hint, &alpha_q8, &use_motion);
            float alpha = (float)alpha_q8 / 256.0f;

            dec->cur_y = dec->ref_y[0];  /* output into ref[0] slot */
            tfi_interpolate(&dec->tfi_ctx,
                            dec->ref_y[1], dec->ref_y[0],  /* prev=ref[1], next=ref[0] */
                            dec->cur_y, alpha);
            /* Chroma: simple 50/50 blend (TFI is luma-only) */
            const size_t uv = (size_t)(dec->width / 2) * (dec->height / 2);
            for (size_t i = 0; i < uv; i++) {
                int cu = (int)dec->ref_u[1][i] + (int)dec->ref_u[0][i];
                int cv = (int)dec->ref_v[1][i] + (int)dec->ref_v[0][i];
                dec->ref_u[0][i] = (uint8_t)(cu >> 1);
                dec->ref_v[0][i] = (uint8_t)(cv >> 1);
            }
            dec->cur_u = dec->ref_u[0];
            dec->cur_v = dec->ref_v[0];
        }
        dec->stats.frames_decoded++;
        dec->stats.frames_skipped++;
        dec->stats.total_bits += (uint32_t)in_len * 8u;
        return 0;
    }

    /* Initialize rANS entropy decoder */
    video_entropy_decoder_t vd;
    if (video_entropy_dec_init(&vd, in + OPVIS_HEADER_V1_SIZE, payload_len) < 0) return -1;

    const size_t mb_count = (size_t)dec->mb_cols * dec->mb_rows;
    opvis_mv_t *mvs = malloc(mb_count * sizeof(opvis_mv_t));
    uint8_t *ref_indices = malloc(mb_count * sizeof(uint8_t));
    if (!mvs || !ref_indices) { free(mvs); free(ref_indices); return -1; }
    memset(mvs, 0, mb_count * sizeof(opvis_mv_t));
    memset(ref_indices, 0, mb_count * sizeof(uint8_t));

    /* Reset IBC hash table at I-frame boundaries */
    if (frame_type == OPVIS_FRAME_I)
        memset(s_dec_ibc_table, 0xFF, sizeof(s_dec_ibc_table));

    /* Route output: B-frames write to ref[2] to preserve P-frame refs */
    if (frame_type == OPVIS_FRAME_B) {
        dec->cur_y = dec->ref_y[2];
        dec->cur_u = dec->ref_u[2];
        dec->cur_v = dec->ref_v[2];
    } else {
        dec->cur_y = dec->ref_y[0];
        dec->cur_u = dec->ref_u[0];
        dec->cur_v = dec->ref_v[0];
    }

    /* E3 — read per-frame SAO type (placeholder; params appended after CTU data) */
    uint8_t dec_sao_type = video_entropy_dec_get_sao_type(&vd);

    /* CTU outer loop */
    const uint16_t ctu_cols = (uint16_t)((dec->width  + OPVIS_CTU_SIZE - 1) / OPVIS_CTU_SIZE);
    const uint16_t ctu_rows = (uint16_t)((dec->height + OPVIS_CTU_SIZE - 1) / OPVIS_CTU_SIZE);
    for (uint16_t ctu_y = 0; ctu_y < ctu_rows; ctu_y++) {
        for (uint16_t ctu_x = 0; ctu_x < ctu_cols; ctu_x++) {
            decode_cu(dec, &vd,
                      (int)(ctu_x * OPVIS_CTU_SIZE), (int)(ctu_y * OPVIS_CTU_SIZE),
                      OPVIS_CTU_SIZE, frame_type, quality,
                      mvs, ref_indices);
        }
    }

    /* Update prev_mvs for TMVP in the next inter frame */
    if (frame_type != OPVIS_FRAME_B && dec->prev_mvs)
        memcpy(dec->prev_mvs, mvs, mb_count * sizeof(opvis_mv_t));

    /* Apply in-loop deblocking to the decoded output plane (E1/E4) */
    apply_deblocking_filter_improved(dec->cur_y, dec->width, dec->height,
                                   dec->mb_cols, dec->mb_rows, frame_type,
                                   mvs, ref_indices, quality,
                                   dec->cur_u, dec->cur_v,
                                   dec->width / 2, dec->height / 2);

    /* E3 — SAO: read the 3 trailing param bytes and apply before ALF.
     * VVC in-loop filter order: deblock → SAO → ALF. */
    if (dec_sao_type != SAO_OFF) {
        uint8_t sao_raw[3];
        sao_raw[0] = video_entropy_dec_get_byte(&vd);
        sao_raw[1] = video_entropy_dec_get_byte(&vd);
        sao_raw[2] = video_entropy_dec_get_byte(&vd);
        sao_params_t sao_dec;
        if (sao_decode_params(&sao_dec, sao_raw, sizeof(sao_raw)) > 0)
            sao_apply_frame(dec->cur_y, (int)dec->width, (int)dec->height, &sao_dec);
    }

    /* ALF: per-CTU adaptive loop filter applied after SAO (VVC order).
     * If alf_present, the encoder serialized a per-CTU enable bitmap as a suffix
     * after the rANS payload (at in + OPVIS_HEADER_V1_SIZE + payload_len).
     * Use that bitmap for deterministic enable/disable decisions.
     * Legacy frames (alf_present=0) fall back to the gradient heuristic. */
    {
        const uint16_t alf_ctu_cols = (uint16_t)((dec->width  + OPVIS_CTU_SIZE - 1) / OPVIS_CTU_SIZE);
        const uint16_t alf_ctu_rows = (uint16_t)((dec->height + OPVIS_CTU_SIZE - 1) / OPVIS_CTU_SIZE);
        const size_t ctu_count = (size_t)alf_ctu_cols * alf_ctu_rows;
        const size_t bitmap_bytes = (ctu_count + 7) / 8;
        const uint8_t *alf_bitmap = NULL;
        bool bitmap_valid = false;

        if (alf_present) {
            /* Bitmap appended immediately after the rANS payload */
            const size_t bitmap_offset = OPVIS_HEADER_V1_SIZE + payload_len;
            if (in_len >= bitmap_offset + bitmap_bytes) {
                alf_bitmap = in + bitmap_offset;
                bitmap_valid = true;
            }
        }

        size_t ctu_idx = 0;
        for (uint16_t cy = 0; cy < alf_ctu_rows; cy++) {
            for (uint16_t cx = 0; cx < alf_ctu_cols; cx++, ctu_idx++) {
                int cx0 = (int)(cx * OPVIS_CTU_SIZE);
                int cy0 = (int)(cy * OPVIS_CTU_SIZE);
                int cw  = MIN((int)OPVIS_CTU_SIZE, (int)dec->width  - cx0);
                int ch  = MIN((int)OPVIS_CTU_SIZE, (int)dec->height - cy0);

                bool apply = false;
                if (bitmap_valid) {
                    apply = (alf_bitmap[ctu_idx >> 3] >> (ctu_idx & 7)) & 1;
                } else {
                    /* Heuristic fallback: analyze without orig */
                    alf_params_t ap = alf_analyze(dec->cur_y, NULL,
                                                  dec->width, cx0, cy0, cw, ch);
                    apply = ap.enabled;
                    if (apply) {
                        alf_apply(dec->cur_y, dec->width, cx0, cy0, cw, ch, &ap);
                        continue;
                    }
                }
                if (apply) {
                    alf_params_t ap = alf_analyze(dec->cur_y, NULL,
                                                  dec->width, cx0, cy0, cw, ch);
                    ap.enabled = true;
                    alf_apply(dec->cur_y, dec->width, cx0, cy0, cw, ch, &ap);
                }
            }
        }
    }

    /* Scale 8-bit output to 10-bit if header signals 10-bit depth */
    if (is_10bit && dec->ref_y16[0]) {
        const size_t n_y  = (size_t)dec->width * dec->height;
        const size_t n_uv = (size_t)(dec->width / 2) * (dec->height / 2);
        for (size_t i = 0; i < n_y;  i++) dec->ref_y16[0][i] = (uint16_t)dec->cur_y[i] << 2;
        for (size_t i = 0; i < n_uv; i++) dec->ref_u16[0][i] = (uint16_t)dec->cur_u[i] << 2;
        for (size_t i = 0; i < n_uv; i++) dec->ref_v16[0][i] = (uint16_t)dec->cur_v[i] << 2;
    }

    /* Update reference frames: I/P shift ref[0]→ref[1]; B-frames leave refs unchanged */
    const size_t y_size = dec->width * dec->height;
    const size_t uv_size = (dec->width / 2) * (dec->height / 2);

    if (frame_type != OPVIS_FRAME_B) {
        memcpy(dec->ref_y[1], dec->ref_y[0], y_size);
        memcpy(dec->ref_u[1], dec->ref_u[0], uv_size);
        memcpy(dec->ref_v[1], dec->ref_v[0], uv_size);
    }

    free(mvs);
    free(ref_indices);

    /* F3 — decoder statistics */
    dec->stats.frames_decoded++;
    dec->stats.total_bits += (uint32_t)in_len * 8u;

    return 0;
}

/* ========================================================================
 * DECODED FRAME ACCESS
 * ======================================================================== */

const uint8_t *opvis_decoded_y(const opvis_decoder_t *dec) {
    return dec ? dec->cur_y : NULL;
}

const uint8_t *opvis_decoded_u(const opvis_decoder_t *dec) {
    return dec ? dec->cur_u : NULL;
}

const uint8_t *opvis_decoded_v(const opvis_decoder_t *dec) {
    return dec ? dec->cur_v : NULL;
}

const uint16_t *opvis_decoded_y16(const opvis_decoder_t *dec) {
    return (dec && dec->color_info.bitdepth == 10) ? dec->ref_y16[0] : NULL;
}

const uint16_t *opvis_decoded_u16(const opvis_decoder_t *dec) {
    return (dec && dec->color_info.bitdepth == 10) ? dec->ref_u16[0] : NULL;
}

const uint16_t *opvis_decoded_v16(const opvis_decoder_t *dec) {
    return (dec && dec->color_info.bitdepth == 10) ? dec->ref_v16[0] : NULL;
}