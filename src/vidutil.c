/* opcodec/vidutil.c — Video utility functions implementation
 *
 * Weighted Prediction and Sample Adaptive Offset (SAO) for visual quality.
 *
 * - Weighted Prediction: handles fades and brightness changes
 * - SAO: reduces banding artifacts and ringing (HEVC-style in-loop filter)
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/vidutil.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

/* ========================================================================
 * UTILITY MACROS
 * ======================================================================== */

#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define ABS(x) ((x) < 0 ? -(x) : (x))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* SAO band width (pixel range / 32 bands = 256 / 32 = 8) */
#define SAO_BAND_WIDTH 8
#define SAO_NUM_BANDS  32
#define SAO_BAND_GROUP 4  /* Process 4 consecutive bands */

/* ========================================================================
 * WEIGHTED PREDICTION IMPLEMENTATION
 * ======================================================================== */

wp_params_t wp_detect(const uint8_t *cur_luma, const uint8_t *ref_luma,
                      int width, int height)
{
    wp_params_t params = {
        .enabled = false,
        .weight = 64,      /* 1.0 in 6-bit fixed point */
        .offset = 0,
        .log2_denom = 6
    };

    if (!cur_luma || !ref_luma || width <= 0 || height <= 0) {
        return params;
    }

    /* Compute average luma (subsample every 4th pixel for speed) */
    int64_t sum_cur = 0, sum_ref = 0;
    int count = 0;

    for (int y = 0; y < height; y += 4) {
        for (int x = 0; x < width; x += 4) {
            sum_cur += cur_luma[y * width + x];
            sum_ref += ref_luma[y * width + x];
            count++;
        }
    }

    if (count == 0) {
        return params;
    }

    float avg_cur = (float)sum_cur / count;
    float avg_ref = (float)sum_ref / count;

    /* Enable weighted prediction if brightness changed significantly */
    if (fabsf(avg_cur - avg_ref) > 8.0f && avg_ref > 0.01f) {
        float ratio = avg_cur / avg_ref;

        /* Compute weight and offset */
        int weight_fp = (int)roundf(ratio * (1 << params.log2_denom));
        int offset_fp = (int)roundf(avg_cur - ratio * avg_ref);

        params.weight = (int8_t)CLAMP(weight_fp, -128, 127);
        params.offset = (int8_t)CLAMP(offset_fp, -128, 127);
        params.enabled = true;
    }

    return params;
}

void wp_apply(uint8_t *ref_block, int stride, int block_size,
              const wp_params_t *params)
{
    if (!params || !params->enabled || !ref_block) {
        return;
    }

    const int round_offset = 1 << (params->log2_denom - 1);

    for (int y = 0; y < block_size; y++) {
        for (int x = 0; x < block_size; x++) {
            int val = params->weight * ref_block[y * stride + x] +
                      params->offset * (1 << params->log2_denom) +
                      round_offset;
            val >>= params->log2_denom;
            ref_block[y * stride + x] = (uint8_t)CLAMP(val, 0, 255);
        }
    }
}

int wp_encode_params(const wp_params_t *params, uint8_t *out, size_t out_cap)
{
    if (!params || !out) {
        return -1;
    }

    if (!params->enabled) {
        if (out_cap < 1) return -1;
        out[0] = 0;  /* disabled flag */
        return 1;
    }

    if (out_cap < 4) return -1;

    out[0] = 1;  /* enabled flag */
    out[1] = (uint8_t)params->weight;
    out[2] = (uint8_t)params->offset;
    out[3] = params->log2_denom;

    return 4;
}

int wp_decode_params(wp_params_t *params, const uint8_t *in, size_t in_len)
{
    if (!params || !in) {
        return -1;
    }

    if (in_len < 1) return -1;

    params->enabled = (in[0] != 0);

    if (!params->enabled) {
        params->weight = 64;
        params->offset = 0;
        params->log2_denom = 6;
        return 1;
    }

    if (in_len < 4) return -1;

    params->weight = (int8_t)in[1];
    params->offset = (int8_t)in[2];
    params->log2_denom = in[3];

    return 4;
}

/* ========================================================================
 * SAMPLE ADAPTIVE OFFSET (SAO) IMPLEMENTATION
 * ======================================================================== */

/* Edge offset category classification */
static int sao_classify_edge(int center, int neighbor1, int neighbor2)
{
    int sign1 = (center > neighbor1) - (center < neighbor1);
    int sign2 = (center > neighbor2) - (center < neighbor2);

    if (sign1 == -1 && sign2 == -1) return 0;      /* valley */
    else if (sign1 == -1 && sign2 == 0) return 1;  /* concave */
    else if (sign1 == 0 && sign2 == 1) return 2;   /* convex */
    else if (sign1 == 1 && sign2 == 1) return 3;   /* peak */

    return -1;  /* no classification */
}

/* Analyze edge offset for specific direction */
static int sao_analyze_edge_dir(const uint8_t *recon, const uint8_t *orig,
                                int stride, int block_size, sao_eo_class_t eo_class,
                                int8_t *out_offsets)
{
    int cat_sum[4] = {0, 0, 0, 0};
    int cat_count[4] = {0, 0, 0, 0};

    for (int y = 1; y < block_size - 1; y++) {
        for (int x = 1; x < block_size - 1; x++) {
            int center = recon[y * stride + x];
            int n1, n2;
            int category;

            /* Select neighbors based on edge direction */
            switch (eo_class) {
                case SAO_EO_HORIZ:
                    n1 = recon[y * stride + x - 1];
                    n2 = recon[y * stride + x + 1];
                    break;
                case SAO_EO_VERT:
                    n1 = recon[(y - 1) * stride + x];
                    n2 = recon[(y + 1) * stride + x];
                    break;
                case SAO_EO_45:
                    n1 = recon[(y - 1) * stride + x - 1];
                    n2 = recon[(y + 1) * stride + x + 1];
                    break;
                case SAO_EO_135:
                    n1 = recon[(y - 1) * stride + x + 1];
                    n2 = recon[(y + 1) * stride + x - 1];
                    break;
                default:
                    continue;
            }

            category = sao_classify_edge(center, n1, n2);
            if (category >= 0) {
                cat_sum[category] += orig[y * stride + x] - center;
                cat_count[category]++;
            }
        }
    }

    /* Compute optimal offset per category */
    int total_distortion = 0;
    for (int c = 0; c < 4; c++) {
        if (cat_count[c] > 0) {
            out_offsets[c] = (int8_t)CLAMP(cat_sum[c] / cat_count[c], -7, 7);
            total_distortion += ABS(cat_sum[c]);
        } else {
            out_offsets[c] = 0;
        }
    }

    return total_distortion;
}

/* Analyze band offset */
static int sao_analyze_band(const uint8_t *recon, const uint8_t *orig,
                           int stride, int block_size,
                           uint8_t *out_start_band, int8_t *out_offsets)
{
    int band_sum[SAO_NUM_BANDS] = {0};
    int band_count[SAO_NUM_BANDS] = {0};

    /* Accumulate error per band */
    for (int y = 0; y < block_size; y++) {
        for (int x = 0; x < block_size; x++) {
            int recon_val = recon[y * stride + x];
            int orig_val = orig[y * stride + x];
            int band = recon_val / SAO_BAND_WIDTH;

            band = CLAMP(band, 0, SAO_NUM_BANDS - 1);

            band_sum[band] += orig_val - recon_val;
            band_count[band]++;
        }
    }

    /* Find the group of 4 consecutive bands with most pixels */
    int best_start = 0;
    int best_count = 0;

    for (int start = 0; start <= SAO_NUM_BANDS - SAO_BAND_GROUP; start++) {
        int group_count = 0;
        for (int i = 0; i < SAO_BAND_GROUP; i++) {
            group_count += band_count[start + i];
        }

        if (group_count > best_count) {
            best_count = group_count;
            best_start = start;
        }
    }

    *out_start_band = (uint8_t)best_start;

    /* Compute offsets for the selected band group */
    int total_distortion = 0;
    for (int i = 0; i < SAO_BAND_GROUP; i++) {
        int band = best_start + i;
        if (band_count[band] > 0) {
            out_offsets[i] = (int8_t)CLAMP(band_sum[band] / band_count[band], -7, 7);
            total_distortion += ABS(band_sum[band]);
        } else {
            out_offsets[i] = 0;
        }
    }

    return total_distortion;
}

sao_params_t sao_analyze(const uint8_t *recon, const uint8_t *orig,
                         int stride, int block_size)
{
    sao_params_t params = { .type = SAO_OFF };

    if (!recon || !orig || block_size <= 0) {
        return params;
    }

    /* Test edge offset for all directions */
    int best_distortion = INT_MAX;
    sao_params_t best_params = { .type = SAO_OFF };

    for (int eo_class = 0; eo_class < 4; eo_class++) {
        int8_t offsets[4];
        int distortion = sao_analyze_edge_dir(recon, orig, stride, block_size,
                                              (sao_eo_class_t)eo_class, offsets);

        if (distortion < best_distortion && distortion > 0) {
            best_distortion = distortion;
            best_params.type = SAO_EDGE;
            best_params.edge.eo_class = (sao_eo_class_t)eo_class;
            memcpy(best_params.edge.offsets, offsets, sizeof(offsets));
        }
    }

    /* Test band offset */
    uint8_t start_band;
    int8_t band_offsets[4];
    int band_distortion = sao_analyze_band(recon, orig, stride, block_size,
                                           &start_band, band_offsets);

    if (band_distortion < best_distortion && band_distortion > 0) {
        best_params.type = SAO_BAND;
        best_params.band.start_band = start_band;
        memcpy(best_params.band.offsets, band_offsets, sizeof(band_offsets));
    }

    return best_params;
}

void sao_apply(uint8_t *recon, int stride, int block_size,
               const sao_params_t *params)
{
    if (!recon || !params || params->type == SAO_OFF) {
        return;
    }

    if (params->type == SAO_EDGE) {
        for (int y = 1; y < block_size - 1; y++) {
            for (int x = 1; x < block_size - 1; x++) {
                int center = recon[y * stride + x];
                int n1, n2;

                /* Select neighbors based on edge direction */
                switch (params->edge.eo_class) {
                    case SAO_EO_HORIZ:
                        n1 = recon[y * stride + x - 1];
                        n2 = recon[y * stride + x + 1];
                        break;
                    case SAO_EO_VERT:
                        n1 = recon[(y - 1) * stride + x];
                        n2 = recon[(y + 1) * stride + x];
                        break;
                    case SAO_EO_45:
                        n1 = recon[(y - 1) * stride + x - 1];
                        n2 = recon[(y + 1) * stride + x + 1];
                        break;
                    case SAO_EO_135:
                        n1 = recon[(y - 1) * stride + x + 1];
                        n2 = recon[(y + 1) * stride + x - 1];
                        break;
                    default:
                        continue;
                }

                int category = sao_classify_edge(center, n1, n2);
                if (category >= 0) {
                    int offset = params->edge.offsets[category];
                    int new_val = center + offset;
                    recon[y * stride + x] = (uint8_t)CLAMP(new_val, 0, 255);
                }
            }
        }
    }
    else if (params->type == SAO_BAND) {
        for (int y = 0; y < block_size; y++) {
            for (int x = 0; x < block_size; x++) {
                int pixel_val = recon[y * stride + x];
                int band = pixel_val / SAO_BAND_WIDTH;

                if (band >= params->band.start_band &&
                    band < params->band.start_band + SAO_BAND_GROUP) {
                    int offset_idx = band - params->band.start_band;
                    int offset = params->band.offsets[offset_idx];
                    int new_val = pixel_val + offset;
                    recon[y * stride + x] = (uint8_t)CLAMP(new_val, 0, 255);
                }
            }
        }
    }
}

void sao_apply_frame(uint8_t *frame, int width, int height, const sao_params_t *params)
{
    if (!frame || !params || params->type == SAO_OFF) return;

    if (params->type == SAO_EDGE) {
        for (int y = 1; y < height - 1; y++) {
            for (int x = 1; x < width - 1; x++) {
                int center = frame[y * width + x];
                int n1, n2;
                switch (params->edge.eo_class) {
                    case SAO_EO_HORIZ:
                        n1 = frame[y * width + x - 1];
                        n2 = frame[y * width + x + 1];
                        break;
                    case SAO_EO_VERT:
                        n1 = frame[(y - 1) * width + x];
                        n2 = frame[(y + 1) * width + x];
                        break;
                    case SAO_EO_45:
                        n1 = frame[(y - 1) * width + x - 1];
                        n2 = frame[(y + 1) * width + x + 1];
                        break;
                    case SAO_EO_135:
                        n1 = frame[(y - 1) * width + x + 1];
                        n2 = frame[(y + 1) * width + x - 1];
                        break;
                    default:
                        continue;
                }
                int category = sao_classify_edge(center, n1, n2);
                if (category >= 0) {
                    int new_val = center + params->edge.offsets[category];
                    frame[y * width + x] = (uint8_t)CLAMP(new_val, 0, 255);
                }
            }
        }
    } else if (params->type == SAO_BAND) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int pixel_val = frame[y * width + x];
                int band = pixel_val / SAO_BAND_WIDTH;
                if (band >= params->band.start_band &&
                    band < params->band.start_band + SAO_BAND_GROUP) {
                    int offset_idx = band - params->band.start_band;
                    int new_val = pixel_val + params->band.offsets[offset_idx];
                    frame[y * width + x] = (uint8_t)CLAMP(new_val, 0, 255);
                }
            }
        }
    }
}

int sao_encode_params(const sao_params_t *params, uint8_t *out, size_t out_cap)
{
    if (!params || !out) {
        return -1;
    }

    if (params->type == SAO_OFF) {
        if (out_cap < 1) return -1;
        out[0] = 0;  /* type = SAO_OFF */
        return 1;
    }

    if (params->type == SAO_EDGE) {
        if (out_cap < 3) return -1;
        out[0] = (2 << 6) | (params->edge.eo_class << 4) |
                 ((params->edge.offsets[0] + 7) & 0xF);
        out[1] = ((params->edge.offsets[1] + 7) << 4) |
                 ((params->edge.offsets[2] + 7) & 0xF);
        out[2] = (params->edge.offsets[3] + 7) & 0xF;
        return 3;
    }

    if (params->type == SAO_BAND) {
        if (out_cap < 3) return -1;
        out[0] = (1 << 6) | (params->band.start_band << 1) |
                 ((params->band.offsets[0] + 7) >> 3);
        out[1] = (((params->band.offsets[0] + 7) & 0x7) << 5) |
                 ((params->band.offsets[1] + 7) << 1) |
                 ((params->band.offsets[2] + 7) >> 3);
        out[2] = (((params->band.offsets[2] + 7) & 0x7) << 5) |
                 ((params->band.offsets[3] + 7) << 1);
        return 3;
    }

    return -1;
}

int sao_decode_params(sao_params_t *params, const uint8_t *in, size_t in_len)
{
    if (!params || !in || in_len < 1) {
        return -1;
    }

    int type = (in[0] >> 6) & 0x3;

    if (type == 0) {
        params->type = SAO_OFF;
        return 1;
    }

    if (type == 2) {  /* SAO_EDGE */
        if (in_len < 3) return -1;

        params->type = SAO_EDGE;
        params->edge.eo_class = (sao_eo_class_t)((in[0] >> 4) & 0x3);
        params->edge.offsets[0] = (int8_t)((in[0] & 0xF) - 7);
        params->edge.offsets[1] = (int8_t)(((in[1] >> 4) & 0xF) - 7);
        params->edge.offsets[2] = (int8_t)((in[1] & 0xF) - 7);
        params->edge.offsets[3] = (int8_t)((in[2] & 0xF) - 7);
        return 3;
    }

    if (type == 1) {  /* SAO_BAND */
        if (in_len < 3) return -1;

        params->type = SAO_BAND;
        params->band.start_band = (uint8_t)((in[0] >> 1) & 0x1F);
        params->band.offsets[0] = (int8_t)((((in[0] & 0x1) << 3) |
                                           ((in[1] >> 5) & 0x7)) - 7);
        params->band.offsets[1] = (int8_t)(((in[1] >> 1) & 0xF) - 7);
        params->band.offsets[2] = (int8_t)((((in[1] & 0x1) << 3) |
                                           ((in[2] >> 5) & 0x7)) - 7);
        params->band.offsets[3] = (int8_t)(((in[2] >> 1) & 0xF) - 7);
        return 3;
    }

    return -1;  /* unknown type */
}

/* ========================================================================
 * ADAPTIVE LOOP FILTER (ALF)
 * ======================================================================== */

/*
 * 13-tap diamond tap offsets (index 6 = center pixel):
 *
 *         (0,-2)
 *    (-1,-1)(0,-1)(1,-1)
 * (-2,0)(-1,0)(0,0)(1,0)(2,0)
 *    (-1,1)(0,1)(1,1)
 *         (0,2)
 */
static const int kAlfDx[13] = { 0,-1, 0, 1,-2,-1, 0, 1, 2,-1, 0, 1, 0 };
static const int kAlfDy[13] = {-2,-1,-1,-1, 0, 0, 0, 0, 0, 1, 1, 1, 2 };

/*
 * Fixed per-class filter coefficients (sum = 64 → right-shift 6 to normalise).
 * Class 0=smooth, 1=low, 2=medium, 3=high/edges.
 * Tap order matches kAlfDx/kAlfDy; index 6 is the center tap.
 */
static const int8_t kAlfCoeffs[ALF_NUM_CLASSES][ALF_NUM_COEFFS] = {
    /* class 0 — smooth: strong 2-D low-pass */
    { 0, 2, 6, 2, 0, 6, 32, 6, 0, 2, 6, 2, 0 },
    /* class 1 — low activity: moderate low-pass */
    { 0, 1, 3, 1, 0, 3, 48, 3, 0, 1, 3, 1, 0 },
    /* class 2 — medium activity: gentle filter */
    { 0, 0, 2, 0, 0, 2, 56, 2, 0, 0, 2, 0, 0 },
    /* class 3 — high activity / edges: identity */
    { 0, 0, 0, 0, 0, 0, 64, 0, 0, 0, 0, 0, 0 },
};

/* Classify a single 4×4 block by its mean gradient magnitude. */
static uint8_t alf_classify_block(const uint8_t *frame, int fw, int fh,
                                  int bx, int by)
{
    long grad = 0;
    int  n    = 0;
    for (int r = 0; r < 4; r++) {
        int y = by + r;
        if (y < 1 || y >= fh - 1) continue;
        for (int c = 0; c < 4; c++) {
            int x = bx + c;
            if (x < 1 || x >= fw - 1) continue;
            int p   = frame[y * fw + x];
            int gh  = abs(frame[y * fw + x + 1]        - frame[y * fw + x - 1]);
            int gv  = abs(frame[(y+1) * fw + x]        - frame[(y-1) * fw + x]);
            int gd1 = abs(frame[(y+1) * fw + x + 1]    - frame[(y-1) * fw + x - 1]);
            int gd2 = abs(frame[(y+1) * fw + x - 1]    - frame[(y-1) * fw + x + 1]);
            (void)p;
            grad += gh + gv + gd1 + gd2;
            n++;
        }
    }
    int avg = n > 0 ? (int)(grad / n) : 0;
    return (uint8_t)((avg < 8) ? 0 : (avg < 20) ? 1 : (avg < 40) ? 2 : 3);
}

alf_params_t alf_analyze(const uint8_t *recon, const uint8_t *orig,
                         int frame_width,
                         int ctu_x, int ctu_y, int ctu_w, int ctu_h)
{
    alf_params_t p;
    memset(&p, 0, sizeof(p));

    if (!recon || ctu_w <= 0 || ctu_h <= 0) return p;

    /* Infer frame_height conservatively from CTU bottom boundary */
    const int fh = ctu_y + ctu_h + 2;  /* safe upper bound for border checks */
    const int fw = frame_width;

    /* Step 1: classify each 4×4 block and assign fixed-bank coefficients */
    int block_rows = (ctu_h + 3) / 4;
    int block_cols = (ctu_w + 3) / 4;
    if (block_rows > 16) block_rows = 16;
    if (block_cols > 16) block_cols = 16;

    for (int by = 0; by < block_rows; by++) {
        for (int bx = 0; bx < block_cols; bx++) {
            uint8_t cls = alf_classify_block(recon, fw, fh,
                                             ctu_x + bx * 4, ctu_y + by * 4);
            p.class_map[by][bx] = cls;
        }
    }

    /* Copy fixed coefficients for each class */
    for (int c = 0; c < ALF_NUM_CLASSES; c++)
        for (int i = 0; i < ALF_NUM_COEFFS; i++)
            p.coeffs[c][i] = kAlfCoeffs[c][i];

    /* Step 2: decide whether to enable ALF for this CTU.
     * If orig is available, enable only when the filter reduces SSD.
     * Without orig, enable whenever the block mix is not all-edges. */
    if (orig) {
        long ssd_before = 0, ssd_after = 0;
        for (int row = 0; row < ctu_h; row++) {
            int py = ctu_y + row;
            if (py < 0 || py >= fh) continue;
            for (int col = 0; col < ctu_w; col++) {
                int px = ctu_x + col;
                if (px < 0 || px >= fw) continue;
                int r   = recon[py * fw + px];
                int o   = orig[py * fw + px];
                int e   = o - r;
                ssd_before += (long)e * e;

                /* Quick filter estimate using the class */
                int by = MIN(row / 4, 15), bx = MIN(col / 4, 15);
                const int8_t *cf = p.coeffs[p.class_map[by][bx]];
                int acc = 0;
                for (int t = 0; t < 13; t++) {
                    int nx = px + kAlfDx[t], ny = py + kAlfDy[t];
                    int nb = (nx >= 0 && ny >= 0 && nx < fw && ny < fh) ?
                             recon[ny * fw + nx] : r;
                    acc += (int)cf[t] * nb;
                }
                int filt = CLAMP((acc + 32) >> 6, 0, 255);
                int ef   = o - filt;
                ssd_after += (long)ef * ef;
            }
        }
        p.enabled = (ssd_after < ssd_before);
    } else {
        /* Heuristic: enable if not all blocks are class 3 (edges-only) */
        p.enabled = false;
        for (int by = 0; by < block_rows && !p.enabled; by++)
            for (int bx = 0; bx < block_cols && !p.enabled; bx++)
                if (p.class_map[by][bx] < 3)
                    p.enabled = true;
    }

    return p;
}

void alf_apply(uint8_t *recon, int frame_width,
               int ctu_x, int ctu_y, int ctu_w, int ctu_h,
               const alf_params_t *params)
{
    if (!recon || !params || !params->enabled) return;

    const int fw = frame_width;
    const int fh = ctu_y + ctu_h + 2;

    /* Filter into a temporary row buffer to avoid using filtered pixels as input */
    static uint8_t tmp[64 * 64];
    const int area = ctu_w * ctu_h;
    if (area > 64 * 64) return;

    for (int row = 0; row < ctu_h; row++) {
        int py = ctu_y + row;
        for (int col = 0; col < ctu_w; col++) {
            int px = ctu_x + col;
            int by = MIN(row / 4, 15), bx = MIN(col / 4, 15);
            const int8_t *cf = params->coeffs[params->class_map[by][bx]];
            int acc = 0;
            for (int t = 0; t < 13; t++) {
                int nx = px + kAlfDx[t], ny = py + kAlfDy[t];
                int nb = (nx >= 0 && ny >= 0 && nx < fw && ny < fh) ?
                         recon[ny * fw + nx] : recon[py * fw + px];
                acc += (int)cf[t] * nb;
            }
            tmp[row * ctu_w + col] = (uint8_t)CLAMP((acc + 32) >> 6, 0, 255);
        }
    }

    for (int row = 0; row < ctu_h; row++) {
        int py = ctu_y + row;
        for (int col = 0; col < ctu_w; col++)
            recon[py * fw + ctu_x + col] = tmp[row * ctu_w + col];
    }
}

int alf_encode_params(const alf_params_t *params, uint8_t *out, size_t out_cap)
{
    if (!params || !out) return -1;

    /* Byte 0: enabled flag */
    if (out_cap < 1) return -1;
    out[0] = params->enabled ? 1 : 0;
    if (!params->enabled) return 1;

    /* Bytes 1–208: class_map (16×16 = 256 entries, packed 4 per byte = 64 bytes) */
    if (out_cap < 1 + 64 + ALF_NUM_CLASSES * ALF_NUM_COEFFS) return -1;
    for (int i = 0; i < 64; i++) {
        int row0 = (i * 4) / 16, col0 = (i * 4) % 16;
        int row1 = ((i * 4) + 1) / 16, col1 = ((i * 4) + 1) % 16;
        int row2 = ((i * 4) + 2) / 16, col2 = ((i * 4) + 2) % 16;
        int row3 = ((i * 4) + 3) / 16, col3 = ((i * 4) + 3) % 16;
        out[1 + i] = (uint8_t)(
            ((params->class_map[row0][col0] & 3) << 6) |
            ((params->class_map[row1][col1] & 3) << 4) |
            ((params->class_map[row2][col2] & 3) << 2) |
            ((params->class_map[row3][col3] & 3) << 0));
    }

    /* Remaining bytes: coefficients */
    uint8_t *p = out + 65;
    for (int c = 0; c < ALF_NUM_CLASSES; c++)
        for (int i = 0; i < ALF_NUM_COEFFS; i++)
            *p++ = (uint8_t)params->coeffs[c][i];

    return (int)(p - out);
}

int alf_decode_params(alf_params_t *params, const uint8_t *in, size_t in_len)
{
    if (!params || !in || in_len < 1) return -1;
    memset(params, 0, sizeof(*params));

    params->enabled = (in[0] & 1) != 0;
    if (!params->enabled) return 1;

    const size_t needed = 1 + 64 + ALF_NUM_CLASSES * ALF_NUM_COEFFS;
    if (in_len < needed) return -1;

    for (int i = 0; i < 64; i++) {
        int row0 = (i * 4) / 16, col0 = (i * 4) % 16;
        int row1 = ((i * 4) + 1) / 16, col1 = ((i * 4) + 1) % 16;
        int row2 = ((i * 4) + 2) / 16, col2 = ((i * 4) + 2) % 16;
        int row3 = ((i * 4) + 3) / 16, col3 = ((i * 4) + 3) % 16;
        params->class_map[row0][col0] = (in[1 + i] >> 6) & 3;
        params->class_map[row1][col1] = (in[1 + i] >> 4) & 3;
        params->class_map[row2][col2] = (in[1 + i] >> 2) & 3;
        params->class_map[row3][col3] = (in[1 + i] >> 0) & 3;
    }

    const uint8_t *p = in + 65;
    for (int c = 0; c < ALF_NUM_CLASSES; c++)
        for (int i = 0; i < ALF_NUM_COEFFS; i++)
            params->coeffs[c][i] = (int8_t)*p++;

    return (int)(p - in);
}