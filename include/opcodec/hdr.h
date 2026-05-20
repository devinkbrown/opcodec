/* opcodec/hdr.h — HDR / color-science utilities for OPVIS codec
 *
 * Pure signal-processing functions with no codec dependencies:
 *   - SMPTE ST 2084 PQ EOTF/OETF  (Dolby Vision / HDR10)
 *   - BT.2100 HLG OETF             (BBC/NHK broadcast HDR)
 *   - BT.2020 ↔ BT.709 color matrix
 *   - P010 / YUV420P10LE packing helpers
 *
 * All transfer functions operate on normalized [0, 1] signals unless
 * otherwise noted.  Luminance values are in nits (cd/m²).
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_HDR_H
#define OPCODEC_HDR_H

#include <stdint.h>
#include <stddef.h>

/* ---- SMPTE ST 2084 — Perceptual Quantizer (PQ) ---- */

/* EOTF: PQ signal [0,1] → linear light (nits, 0–10000) */
float pq_eotf(float signal);

/* OETF (inverse EOTF): linear light (nits) → PQ signal [0,1] */
float pq_oetf(float nits);

/* ---- BT.2100 — Hybrid Log-Gamma (HLG) ---- */

/* OETF: scene-linear light [0,1] → HLG signal [0,1] */
float hlg_oetf(float scene_linear);

/* EOTF: HLG signal [0,1] → display linear light [0,1] */
float hlg_eotf(float signal);

/* ---- Color matrix conversions ---- */

/* BT.2020 RGB → BT.709 RGB (for display or downscale).
 * Inputs and outputs are linear light [0,1].
 * Out-of-gamut values are soft-clipped.
 */
void bt2020_to_bt709(float r2020, float g2020, float b2020,
                     float *r709, float *g709, float *b709);

/* BT.709 RGB → BT.2020 RGB (for HDR upconversion). */
void bt709_to_bt2020(float r709, float g709, float b709,
                     float *r2020, float *g2020, float *b2020);

/* ---- YCbCr ↔ RGB conversions ---- */

/* BT.709 limited-range YCbCr → RGB [0,1] */
void yuv_bt709_to_rgb(uint8_t y, uint8_t cb, uint8_t cr,
                      float *r, float *g, float *b);

/* BT.2020 limited-range YCbCr → RGB [0,1] */
void yuv_bt2020_to_rgb(uint8_t y, uint8_t cb, uint8_t cr,
                       float *r, float *g, float *b);

/* RGB [0,1] → BT.2020 limited-range YCbCr */
void rgb_to_yuv_bt2020(float r, float g, float b,
                       uint8_t *y, uint8_t *cb, uint8_t *cr);

/* ---- 10-bit pixel format helpers ---- */

/* P010 layout: uint16_t with value in top 10 bits (bits 15-6), bottom 6 = 0.
 * These pack/unpack between [0,1023] pixel values and P010 uint16_t words.
 */
static inline uint16_t p010_pack(uint16_t v10) { return (uint16_t)(v10 << 6); }
static inline uint16_t p010_unpack(uint16_t w)  { return (uint16_t)(w >> 6); }

/*
 * Convert P010 (packed LE) Y-plane to YUV420P10LE planar.
 *
 * p010_y     — source P010 Y plane (width * height uint16_t values)
 * p010_uv    — source P010 interleaved UV (width/2 * height/2 * 2 uint16_t)
 * dst_y      — output Y plane  (width * height uint16_t)
 * dst_u      — output U plane  (width/2 * height/2 uint16_t)
 * dst_v      — output V plane  (width/2 * height/2 uint16_t)
 * width, height — frame dimensions (must be even)
 */
void p010_to_yuv420p10(const uint16_t *p010_y, const uint16_t *p010_uv,
                       uint16_t *dst_y, uint16_t *dst_u, uint16_t *dst_v,
                       int width, int height);

/* Inverse: YUV420P10LE → P010 */
void yuv420p10_to_p010(const uint16_t *src_y, const uint16_t *src_u,
                       const uint16_t *src_v,
                       uint16_t *p010_y, uint16_t *p010_uv,
                       int width, int height);

/* ---- Tone mapping ---- */

/* Simple knee-point tone mapper for HDR → SDR.
 * knee_point in [0,1], gain_above_knee in [1,4].
 * Input/output are normalized linear light [0,1] relative to peak.
 */
float tone_map_knee(float x, float knee_point, float gain_above_knee);

#endif /* OPCODEC_HDR_H */
