/* opcodec/vidutil.h — Video utility functions for OPVIS codec
 *
 * - Weighted Prediction: handles fades and brightness changes
 * - SAO: reduces banding artifacts and ringing (HEVC-style in-loop filter)
 * - ALF: Adaptive Loop Filter (VVC-style, 4-class, 7-tap diamond)
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_VIDUTIL_H
#define OPCODEC_VIDUTIL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- Weighted Prediction ---- */

typedef struct {
    bool    enabled;
    int8_t  weight;
    int8_t  offset;
    uint8_t log2_denom;
} wp_params_t;

wp_params_t wp_detect(const uint8_t *cur_luma, const uint8_t *ref_luma,
                      int width, int height);

void wp_apply(uint8_t *ref_block, int stride, int block_size,
              const wp_params_t *params);

int wp_encode_params(const wp_params_t *params, uint8_t *out, size_t out_cap);
int wp_decode_params(wp_params_t *params, const uint8_t *in, size_t in_len);

/* ---- Sample Adaptive Offset (SAO) ---- */

typedef enum {
    SAO_OFF  = 0,
    SAO_BAND = 1,
    SAO_EDGE = 2
} sao_type_t;

typedef enum {
    SAO_EO_HORIZ = 0,
    SAO_EO_VERT  = 1,
    SAO_EO_45    = 2,
    SAO_EO_135   = 3
} sao_eo_class_t;

typedef struct {
    sao_type_t type;
    union {
        struct {
            int8_t        offsets[4];
            sao_eo_class_t eo_class;
        } edge;
        struct {
            int8_t   offsets[4];
            uint8_t  start_band;
        } band;
    };
} sao_params_t;

sao_params_t sao_analyze(const uint8_t *recon, const uint8_t *orig,
                         int stride, int block_size);

void sao_apply(uint8_t *recon, int stride, int block_size,
               const sao_params_t *params);

/* Apply SAO over an arbitrarily-sized (non-square) luma plane. */
void sao_apply_frame(uint8_t *frame, int width, int height, const sao_params_t *params);

int sao_encode_params(const sao_params_t *params, uint8_t *out, size_t out_cap);
int sao_decode_params(sao_params_t *params, const uint8_t *in, size_t in_len);

/* ---- Adaptive Loop Filter (ALF) ---- */

/* ALF uses gradient-based classification to assign each 4×4 luma block to
 * one of 4 activity classes (smooth / low / medium / high).  A separate
 * 7-tap diamond-shaped FIR filter is applied per class.
 *
 * Diamond tap offsets (relative to center pixel):
 *   (0,-2), (-1,-1),(0,-1),(1,-1),
 *   (-2,0), (-1, 0),(0, 0),(1, 0),(2,0),
 *           (-1, 1),(0, 1),(1, 1),
 *                   (0, 2)
 * = 13 unique tap positions, 13 coefficients per class.
 *
 * Applied per-CTU after deblocking → SAO → ALF in the in-loop pipeline.
 */

#define ALF_NUM_CLASSES  4    /* smooth, low, medium, high activity */
#define ALF_NUM_COEFFS   13   /* 7-tap diamond has 13 unique positions */

/* ALF per-CTU parameters (64×64 CTU = 16×16 grid of 4×4 blocks) */
typedef struct {
    bool    enabled;
    uint8_t class_map[16][16];  /* activity class per 4×4 block */
    int8_t  coeffs[ALF_NUM_CLASSES][ALF_NUM_COEFFS];
} alf_params_t;

/*
 * Analyze a CTU to derive optimal ALF parameters.
 *
 * recon       — reconstructed luma plane (full frame, not just CTU)
 * orig        — original luma plane
 * frame_width — frame width in pixels
 * ctu_x, ctu_y — CTU top-left in pixels
 * ctu_w, ctu_h — CTU dimensions (clipped to frame boundary)
 *
 * Returns filled alf_params_t.  enabled=false if ALF not beneficial for CTU.
 */
alf_params_t alf_analyze(const uint8_t *recon, const uint8_t *orig,
                         int frame_width,
                         int ctu_x, int ctu_y, int ctu_w, int ctu_h);

/*
 * Apply ALF to a CTU in-place.
 *
 * recon       — reconstructed luma plane (full frame)
 * frame_width — frame width
 * ctu_x, ctu_y, ctu_w, ctu_h — CTU region
 * params      — ALF parameters from alf_analyze() or decoded from bitstream
 */
void alf_apply(uint8_t *recon, int frame_width,
               int ctu_x, int ctu_y, int ctu_w, int ctu_h,
               const alf_params_t *params);

/* Encode ALF parameters for one CTU (2–20 bytes depending on enabled/coeffs) */
int alf_encode_params(const alf_params_t *params, uint8_t *out, size_t out_cap);

/* Decode ALF parameters for one CTU */
int alf_decode_params(alf_params_t *params, const uint8_t *in, size_t in_len);

#endif /* OPCODEC_VIDUTIL_H */
