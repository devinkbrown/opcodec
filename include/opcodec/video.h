/* opcodec/video.h — Ophion custom video codec (OPVIS)
 *
 * Integer wavelet video codec with motion compensation.
 * Designed for real-time IRC video communication.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_VIDEO_H
#define OPCODEC_VIDEO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "saliency.h"
#include "tfi.h"

/* ---- Dimensional constants ---- */

#define OPVIS_MAX_WIDTH    1920
#define OPVIS_MAX_HEIGHT   1080
#define OPVIS_MB_SIZE      16
#define OPVIS_CTU_SIZE     64   /* coding tree unit — largest block size */
#define OPVIS_MAX_MBS      ((OPVIS_MAX_WIDTH / OPVIS_MB_SIZE) * (OPVIS_MAX_HEIGHT / OPVIS_MB_SIZE))
#define OPVIS_MV_RANGE     16
#define OPVIS_CHROMA_SIZE  8    /* 8x8 chroma blocks for 4:2:0 */
#define OPVIS_MAX_ENCODED  (OPVIS_MAX_WIDTH * OPVIS_MAX_HEIGHT * 2)
#define OPVIS_SUBPEL_BITS  2    /* quarter-pixel precision */
#define OPVIS_SUBPEL_SCALE 4
#define OPVIS_MAX_REFS     3    /* maximum reference frame slots */

/* ---- Bitstream header sizes ---- */

/* Legacy v0 header (14 bytes):
 *   byte 0:    frame type (I=0, P=1)    [also serves as version: 0 = legacy]
 *   byte 1:    quality (0-100)
 *   bytes 2-3: width (BE)
 *   bytes 4-5: height (BE)
 *   bytes 6-9: frame_num (BE u32)
 *   bytes 10-13: payload_len (BE u32)
 */
#define OPVIS_HEADER_SIZE    14

/* V1 header (18 bytes):
 *   byte 0:    version = 1
 *   byte 1:    frame type (I=0, P=1, B=2)
 *   byte 2:    quality (0-100)
 *   bytes 3-4: width (BE)
 *   bytes 5-6: height (BE)
 *   bytes 7-10: frame_num (BE u32)
 *   byte 11:   log2_ctu=6 | color_info flags
 *              bit 7: hdr_present
 *              bit 6: 10bit
 *              bits 5-4: transfer (00=SDR,01=PQ,10=HLG,11=LINEAR)
 *              bits 3-2: primaries (00=BT709,01=BT2020,10=P3D65,11=SRGB)
 *              bits 1-0: subsampling (00=4:2:0,01=4:2:2,10=4:4:4,11=rsvd)
 *   byte 12:   flags: bit7=alf_present, bit6=screen_mode, bits4-2=b_ref_dist, rest=rsvd
 *   bytes 13-16: payload_len (BE u32)
 *   [if hdr_present, 6 more bytes]: max_lum_BE16, min_lum_BE16, knee_u8, knee_gain_u8
 *
 * Decoder checks byte 0: if 0 → legacy v0 path; if 1 → v1 path.
 */
#define OPVIS_HEADER_V1_SIZE 18
#define OPVIS_HEADER_V1_HDR_EXT 6  /* extra bytes when hdr_present */

/* ---- Frame type ---- */

typedef enum {
    OPVIS_FRAME_I      = 0,
    OPVIS_FRAME_P      = 1,
    OPVIS_FRAME_B      = 2,  /* low-delay B: both refs are past frames */
    OPVIS_FRAME_SKIP   = 3,  /* repeat previous frame (D3) */
    OPVIS_FRAME_INTERP = 4,  /* TFI: synthesized from adjacent refs (not transmitted) */
} opvis_frame_type_t;

/* Encoder quality preset (D5) */
typedef enum {
    OPVIS_PRESET_FAST   = 0,
    OPVIS_PRESET_MEDIUM = 1,
    OPVIS_PRESET_SLOW   = 2,
} opvis_preset_t;

/* Per-frame encoder statistics (F1) */
typedef struct {
    uint32_t frame_num;
    uint32_t bits;
    float    psnr_estimate;
    opvis_frame_type_t type;
    bool     was_skipped;
} opvis_frame_stats_t;

/* Decoder statistics (F3) */
typedef struct {
    uint32_t frames_decoded;
    uint32_t frames_skipped;
    uint32_t total_bits;
} opvis_decoder_stats_t;

/* ---- Pixel formats ---- */

typedef enum {
    OPVIS_FMT_YUV420P     = 0,  /* 8-bit planar 4:2:0 */
    OPVIS_FMT_RGB24       = 1,  /* 8-bit packed RGB */
    OPVIS_FMT_P010        = 2,  /* 10-bit 4:2:0 packed LE (top 10 bits of uint16_t) */
    OPVIS_FMT_YUV420P10LE = 3,  /* 10-bit planar 4:2:0 (uint16_t planes) */
} opvis_pixel_fmt_t;

/* ---- HDR / color metadata ---- */

typedef enum {
    OPVIS_TF_SDR    = 0,  /* BT.709 / BT.601 gamma */
    OPVIS_TF_PQ     = 1,  /* SMPTE ST 2084 (Dolby Vision / HDR10) */
    OPVIS_TF_HLG    = 2,  /* BT.2100 Hybrid Log-Gamma (BBC/NHK) */
    OPVIS_TF_LINEAR = 3,  /* scene-linear (HDR compositing) */
} opvis_transfer_t;

typedef enum {
    OPVIS_CP_BT709  = 0,  /* HD standard (BT.709) */
    OPVIS_CP_BT2020 = 1,  /* UHD / 4K HDR wide gamut (BT.2020) */
    OPVIS_CP_P3D65  = 2,  /* DCI-P3 D65 white point (Cinema/Apple Display) */
    OPVIS_CP_SRGB   = 3,  /* sRGB primaries */
} opvis_colorprim_t;

/* Dynamic HDR metadata (per-frame, Dolby Vision style) */
typedef struct {
    opvis_transfer_t  transfer;
    opvis_colorprim_t primaries;
    uint16_t max_lum;    /* peak white in nits (100–10000) */
    uint16_t min_lum;    /* black level in 0.0001 nit units */
    uint8_t  knee;       /* tone-map knee point (0=0.0 … 255=1.0) */
    uint8_t  knee_gain;  /* gain above knee (0=1.0x … 255=4.0x) */
} opvis_hdr_meta_t;

/* Combined color information for encoder/decoder configuration */
typedef struct {
    uint8_t           bitdepth;    /* 8 or 10 */
    opvis_transfer_t  transfer;
    opvis_colorprim_t primaries;
    uint8_t           subsampling; /* 0=4:2:0, 1=4:2:2, 2=4:4:4 */
} opvis_color_info_t;

/* ---- Motion vector ---- */

typedef struct {
    int16_t x;  /* quarter-pixel units */
    int16_t y;
} opvis_mv_t;

/* ---- Encoder ---- */

typedef struct opvis_encoder {
    uint16_t width;
    uint16_t height;
    uint16_t mb_cols;
    uint16_t mb_rows;
    uint8_t  quality;      /* 0-100 */
    uint16_t gop_size;
    uint32_t frame_num;
    opvis_pixel_fmt_t input_fmt;

    /* Color / HDR configuration */
    opvis_color_info_t color_info;
    opvis_hdr_meta_t   hdr;

    /* 8-bit reference and current planes (3 slots: [0]=recent, [1]=prev, [2]=older) */
    uint8_t *ref_y[OPVIS_MAX_REFS];
    uint8_t *ref_u[OPVIS_MAX_REFS];
    uint8_t *ref_v[OPVIS_MAX_REFS];
    uint8_t *cur_y;
    uint8_t *cur_u;
    uint8_t *cur_v;

    /* 10-bit reference and current planes (populated only in 10-bit mode) */
    uint16_t *ref_y16[OPVIS_MAX_REFS];
    uint16_t *ref_u16[OPVIS_MAX_REFS];
    uint16_t *ref_v16[OPVIS_MAX_REFS];
    uint16_t *cur_y16;
    uint16_t *cur_u16;
    uint16_t *cur_v16;

    /* Wavelet work buffer (sized for largest block: 64×64) */
    int16_t *wavelet_buf;

    /* Sub-pixel interpolation buffer (CTU-level, HEVC 8-tap) */
    uint8_t *interp_buf;

    /* Motion vectors: per-4×4 block grid */
    opvis_mv_t *mvs;

    /* Reference indices (0 or 1) for each CU */
    uint8_t *ref_indices;

    /* ALF coefficients: 4 classes × 13 int16_t coefficients */
    int16_t *alf_coeffs;

    /* IBC hash table: 64K entries for screen-content Intra Block Copy */
    uint32_t *ibc_hashtable;

    /* Palette cache: per-CTU palette for screen-content coding */
    uint8_t *palette_cache;

    /* Temporal Noise Reduction (pre-encode luma filter) */
    bool    tnr_enabled;
    float   tnr_alpha;         /* temporal blend weight for static pixels (0.4–0.6) */
    uint8_t tnr_motion_thresh; /* per-pixel difference that triggers motion (12–24) */

    /* Rate control */
    uint32_t target_bitrate;
    uint32_t rc_fps;          /* frames per second (stored from set_rate_control) */
    int32_t  rc_buffer;
    int32_t  rc_buffer_size;
    float    rc_qp_adj;
    float    rc_integral;     /* PI controller integral accumulator */
    uint8_t  crf_quality;     /* D1: if >0, bypass virtual buffer rate control */

    /* Quality preset (D5) */
    bool    fast_intra_enabled;
    uint8_t me_range;         /* motion estimation search range */

    /* Previous frame's MV grid for TMVP (B3) */
    opvis_mv_t *prev_mvs;

    /* Stats / CDEF (F1, E2) */
    opvis_frame_stats_t last_frame_stats;
    bool    cdef_enabled;
    uint8_t cdef_strength;

    /* Saliency-Aware RDO (SARDO) — per-CTU importance weighting */
    sal_ctx_t sal_ctx;
    bool      sardo_enabled;

    /* Temporal Frame Interpolation — synthesize skipped frames */
    tfi_ctx_t tfi_ctx;
    bool      tfi_enabled;
    uint32_t  tfi_skip_interval; /* skip every Nth frame if quality permits */

    /* Internal allocation (single block) */
    uint8_t *pool;
    size_t pool_size;
} opvis_encoder_t;

/* ---- Decoder ---- */

typedef struct opvis_decoder {
    uint16_t width;
    uint16_t height;
    uint16_t mb_cols;
    uint16_t mb_rows;
    uint8_t  quality;
    uint32_t frame_num;

    /* Color / HDR configuration (populated from bitstream header) */
    opvis_color_info_t color_info;
    opvis_hdr_meta_t   hdr;

    /* 8-bit reference planes (3 slots) */
    uint8_t *ref_y[OPVIS_MAX_REFS];
    uint8_t *ref_u[OPVIS_MAX_REFS];
    uint8_t *ref_v[OPVIS_MAX_REFS];

    /* 10-bit reference planes (populated only in 10-bit mode) */
    uint16_t *ref_y16[OPVIS_MAX_REFS];
    uint16_t *ref_u16[OPVIS_MAX_REFS];
    uint16_t *ref_v16[OPVIS_MAX_REFS];

    /* Wavelet work buffer */
    int16_t *wavelet_buf;

    /* Sub-pixel interpolation buffer */
    uint8_t *interp_buf;

    /* Output routing: ref_y[0] for I/P frames, ref_y[2] for B-frames */
    uint8_t *cur_y;
    uint8_t *cur_u;
    uint8_t *cur_v;

    /* TMVP: co-located MVs from the previous inter frame */
    opvis_mv_t *prev_mvs;

    /* Decoder statistics (F3) */
    opvis_decoder_stats_t stats;

    /* Temporal Frame Interpolation (decoder side) */
    tfi_ctx_t tfi_ctx;
    bool      tfi_enabled;

    /* Internal allocation */
    uint8_t *pool;
    size_t pool_size;
} opvis_decoder_t;

/* ---- Pool size calculation ---- */

/* v0 API — 8-bit only, 2 reference frames (backward compatible) */
size_t opvis_encoder_pool_size(uint16_t width, uint16_t height);
size_t opvis_decoder_pool_size(uint16_t width, uint16_t height);

/* v1 API — accounts for 10-bit planes, ALF coeffs, IBC table, palette cache */
size_t opvis_encoder_pool_size_v1(uint16_t width, uint16_t height,
                                  const opvis_color_info_t *color_info);
size_t opvis_decoder_pool_size_v1(uint16_t width, uint16_t height,
                                  const opvis_color_info_t *color_info);

/* ---- Encoder API ---- */

int opvis_encoder_init(opvis_encoder_t *enc, uint16_t width, uint16_t height,
                       uint8_t quality, uint16_t gop_size,
                       opvis_pixel_fmt_t input_fmt,
                       uint8_t *pool, size_t pool_size);

void opvis_encoder_set_rate_control(opvis_encoder_t *enc,
                                    uint32_t target_bitrate_bps, uint32_t fps);

/* Enable temporal noise reduction (applied before each encode). */
void opvis_encoder_set_tnr(opvis_encoder_t *enc, bool enabled,
                           float alpha, uint8_t motion_thresh);

/* Set HDR / color configuration (call after init, before first encode) */
void opvis_encoder_set_color_info(opvis_encoder_t *enc,
                                  const opvis_color_info_t *ci,
                                  const opvis_hdr_meta_t *hdr);

int opvis_encode(opvis_encoder_t *enc,
                 const uint8_t *input, size_t input_len,
                 uint8_t *out, size_t out_cap);

/* ---- Decoder API ---- */

int opvis_decoder_init(opvis_decoder_t *dec, uint8_t *pool, size_t pool_size);

int opvis_decode(opvis_decoder_t *dec,
                 const uint8_t *in, size_t in_len);

const uint8_t *opvis_decoded_y(const opvis_decoder_t *dec);
const uint8_t *opvis_decoded_u(const opvis_decoder_t *dec);
const uint8_t *opvis_decoded_v(const opvis_decoder_t *dec);

/* 10-bit output planes (NULL when not in 10-bit mode) */
const uint16_t *opvis_decoded_y16(const opvis_decoder_t *dec);
const uint16_t *opvis_decoded_u16(const opvis_decoder_t *dec);
const uint16_t *opvis_decoded_v16(const opvis_decoder_t *dec);

/* ---- New API additions ---- */

/* D1: CRF mode — quality 0 (off) or 1-100 (constant rate factor). */
void opvis_encoder_set_crf(opvis_encoder_t *enc, uint8_t quality);

/* D5: Quality preset. Sets me_range and fast_intra. */
void opvis_encoder_set_preset(opvis_encoder_t *enc, opvis_preset_t preset);

/* F1: Last frame statistics. */
opvis_frame_stats_t opvis_encoder_get_stats(const opvis_encoder_t *enc);

/* F2: Flush — force an I-frame regardless of GOP position. */
int opvis_encoder_flush(opvis_encoder_t *enc, uint8_t *out, size_t out_cap);

/* F3: Decoder statistics. */
opvis_decoder_stats_t opvis_decoder_get_stats(const opvis_decoder_t *dec);

#endif /* OPCODEC_VIDEO_H */
