/*
 * OPFIELD — OPcodec spatial sound FIELD
 *
 * Clean-room spatial audio for opcodec. Supports object-based binaural HRTF
 * rendering, Higher-Order Ambisonics (HOA) up to 3rd order, and VBAP for
 * loudspeaker arrays.
 *
 * Novel algorithms:
 *   OPFIELD-PRISM — Phase-Resolved Interaural Spectral Morphing.
 *     Splits the signal at 700 Hz and applies Duplex-theory-aware weights:
 *       Low band (<700 Hz): full ITD, minimal ILD (phase dominates)
 *       High band (>700 Hz): partial ITD, full ILD (amplitude dominates)
 *     More accurate than single-band HRTF approximations.
 *
 *   OPFIELD-HVSS — HRTF Velocity Smooth Scan.
 *     Lerps ILD filter coefficients toward new targets on every update,
 *     eliminating comb-filtering artefacts when objects move. Smoothing
 *     speed adapts to object velocity: fast = responsive, slow = stable.
 *
 *   OPFIELD-PERQ — Perceptual JND-Zone Quantization.
 *     JND-adaptive azimuth/elevation metadata quantization based on Mills
 *     (1958) and Blauert (1997) data: 1°/zone in front, 3° at side, 5° rear.
 *     Saves ~33% metadata bandwidth vs fixed 0.01° resolution.
 *
 * Mathematical foundations (clean-room from published theory):
 *   ITD  — Woodworth-Schlosberg sphere model (1937/1954)
 *   ILD  — Brown-Duda first-order shelving approximation (1998)
 *   Pinna — Raykar et al. (2004) N1 notch measurements
 *   HOA  — AmbiX ACN/SN3D convention (AES-X274)
 *   VBAP — Pulkki vector-base amplitude panning (JAES 1997)
 *
 * OOB wire format: [n:1][per-object × n: type(1)+az_i16_be(2)+el_i16_be(2)
 *                  +dist_u16_be(2)+gain_u8(1)+spread_u8(1)]  = 1+9n bytes
 * PERQ wire:       [n:1][per-object × n: 6 bytes]  (JND-zone compressed)
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_OPFIELD_H
#define OPCODEC_OPFIELD_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Constants ---- */

#define OPFIELD_MAX_OBJECTS    32
#define OPFIELD_MAX_SPEAKERS   24
#define OPFIELD_HOA_CHANNELS   16    /* (N+1)^2 for order N=3 */
#define OPFIELD_DELAY_MAX      64    /* max ITD delay samples (covers >655 µs @ 48 kHz) */
#define OPFIELD_VBAP_MAX_TRIPS 256   /* max precomputed VBAP triplets */
#define OPFIELD_OBJ_BYTES      9     /* standard OOB bytes per object */

/* Physical constants */
#define OPFIELD_HEAD_RADIUS    0.0875f              /* m — standard head radius */
#define OPFIELD_SPEED_OF_SOUND 343.0f               /* m/s */
#define OPFIELD_EAR_OMEGA      (343.0f / 0.0875f)   /* ≈ 3920 rad/s, Brown-Duda ωe */

/* PRISM crossover frequency */
#define OPFIELD_PRISM_CROSS_HZ 700.0f

/* ---- Enumerations ---- */

typedef enum {
    OPFIELD_RENDER_BINAURAL = 0,  /* 2-ch headphone HRTF */
    OPFIELD_RENDER_HOA1     = 1,  /* 1st-order HOA, 4 channels */
    OPFIELD_RENDER_HOA3     = 2,  /* 3rd-order HOA, 16 channels */
    OPFIELD_RENDER_VBAP     = 3,  /* loudspeaker VBAP */
} opfield_render_mode_t;

typedef enum {
    OPFIELD_OBJ_POINT = 0,  /* spatialized point source */
    OPFIELD_OBJ_BED   = 1,  /* fixed-layout bed channel */
    OPFIELD_OBJ_HOA   = 2,  /* pre-encoded HOA stream */
} opfield_obj_type_t;

/* ---- 3D position ---- */

typedef struct {
    float az;    /* azimuth degrees: 0=front, +90=left, -90=right, ±180=rear */
    float el;    /* elevation degrees: 0=horizon, +90=zenith, -90=nadir */
    float dist;  /* distance in metres */
} opfield_pos_t;

/* ---- Audio object ---- */

typedef struct {
    opfield_obj_type_t type;
    opfield_pos_t      pos;
    float              gain;    /* linear amplitude [0..4] */
    float              spread;  /* 0=point source, 1=omnidirectional */
    bool               active;
} opfield_object_t;

/* ---- Scene ---- */

typedef struct {
    opfield_object_t      objects[OPFIELD_MAX_OBJECTS];
    uint8_t               n_objects;
    uint32_t              sample_rate;
    opfield_render_mode_t render_mode;
} opfield_scene_t;

/* ======================================================================
 * Per-object binaural HRTF state (PRISM + HVSS integrated)
 *
 * Signal flow per object per sample:
 *   x → [PRISM crossover LP] → lo_in, hi_in = x - lo_in
 *   lo_in → [delay ring lo: taps at lo_dly_L, lo_dly_R] → lo_L, lo_R
 *   hi_in → [delay ring hi: taps at hi_dly_L, hi_dly_R] → hi_L, hi_R
 *   lo_L,lo_R → [ILD low-band shelving: α×LO_ILD_W] → lo_L', lo_R'
 *   hi_L,hi_R → [ILD high-band shelving: α×1.0] → hi_L', hi_R'
 *   hi_L',hi_R' → [pinna notch EQ] → hi_L'', hi_R''
 *   out_L += lo_L' + hi_L'';  out_R += lo_R' + hi_R''
 *
 * HVSS: hi_ild and lo_ild coefficients lerp toward tgt_* on each update.
 * ====================================================================== */

typedef struct {
    /* ---- PRISM crossover (LP filter, mono) ---- */
    float cx_a, cx_b;           /* 1st-order LP Butterworth coefficients */
    float cx_xp,  cx_yp;       /* filter state: previous input, previous output */

    /* ---- Delay rings (post-crossover, mono) ---- */
    float dly_lo[OPFIELD_DELAY_MAX];   /* low-band delay ring */
    float dly_hi[OPFIELD_DELAY_MAX];   /* high-band delay ring */
    uint8_t dly_pos;                    /* current write position (shared) */

    uint8_t lo_dly_L, lo_dly_R;  /* full ITD delays: left and right ears */
    uint8_t hi_dly_L, hi_dly_R;  /* PRISM_HI_ITD_W-scaled delays */

    /* ---- ILD: high-band shelving (Brown-Duda, full weight) ---- */
    float hi_ild_b0[2], hi_ild_b1[2], hi_ild_a1[2];  /* [0]=L, [1]=R */
    float hi_ild_xp[2], hi_ild_yp[2];

    /* ---- ILD: low-band shelving (Brown-Duda, PRISM_LO_ILD_W scaled) ---- */
    float lo_ild_b0[2], lo_ild_b1[2], lo_ild_a1[2];
    float lo_ild_xp[2], lo_ild_yp[2];

    /* ---- Pinna notch (2nd-order peaking EQ, high band only) ---- */
    float pn_b0[2], pn_b1[2], pn_b2[2];   /* numerator per ear */
    float pn_a1[2], pn_a2[2];              /* denominator (a0 = 1, normalized) */
    float pn_x1[2], pn_x2[2];             /* input history */
    float pn_y1[2], pn_y2[2];             /* output history */

    /* ---- OPFIELD-HVSS: smooth transition targets ---- */
    float tgt_hi_b0[2], tgt_hi_b1[2], tgt_hi_a1[2];  /* hi-band ILD targets */
    float tgt_lo_b0[2], tgt_lo_b1[2], tgt_lo_a1[2];  /* lo-band ILD targets */
    uint8_t tgt_lo_dly_L, tgt_lo_dly_R;
    uint8_t tgt_hi_dly_L, tgt_hi_dly_R;
    float   hvss_alpha;   /* lerp speed: larger = faster transitions */

    /* ---- Source properties (set by opfield_binaural_update) ---- */
    float dist_m;     /* distance in metres; gain scales as 1/max(1,dist) */
    float spread;     /* [0..1]: 0=directional HRTF, 1=omnidirectional dry */
    float gain;       /* linear amplitude multiplier */
    bool  initialized; /* true after first successful update */
} opfield_hrtf_state_t;

/* ---- Binaural renderer ---- */

typedef struct {
    opfield_hrtf_state_t obj[OPFIELD_MAX_OBJECTS];
    uint32_t             sample_rate;
} opfield_binaural_t;

/* ---- VBAP ---- */

typedef struct {
    uint8_t spk[3];       /* speaker indices in the triplet */
    float   inv_L[3][3];  /* precomputed L^{-1} for this triplet */
} opfield_vbap_triplet_t;

/* 2D pair used when all speakers are coplanar (horizontal-only array) */
typedef struct {
    uint8_t spk[2];       /* speaker indices */
    float   inv_L2[2][2]; /* precomputed 2×2 L^{-1} (XY projection) */
} opfield_vbap_pair_t;

typedef struct {
    float                  spk_xyz[OPFIELD_MAX_SPEAKERS][3]; /* unit vectors */
    uint8_t                n_speakers;
    opfield_vbap_triplet_t trips[OPFIELD_VBAP_MAX_TRIPS];
    uint16_t               n_trips;
    /* 2D fallback for coplanar (horizontal) arrays */
    opfield_vbap_pair_t    pairs[OPFIELD_MAX_SPEAKERS];
    uint8_t                n_pairs;
    bool                   is_2d;
} opfield_vbap_t;

/* ======================================================================
 * Scene API
 * ====================================================================== */

void opfield_scene_init(opfield_scene_t *s, uint32_t sample_rate,
                        opfield_render_mode_t mode);

int opfield_scene_add_object(opfield_scene_t *s, const opfield_object_t *obj);

/* Standard OOB encode: returns bytes written or -1 on error. */
int opfield_scene_encode(const opfield_scene_t *s, uint8_t *out, size_t out_cap);

/* Standard OOB decode: returns bytes consumed or -1 on error. */
int opfield_scene_decode(opfield_scene_t *s, const uint8_t *data, size_t len);

/* OPFIELD-PERQ: JND-zone compressed encode (6 bytes/object vs 9).
 * Returns bytes written or -1. */
int opfield_scene_encode_perq(const opfield_scene_t *s,
                               uint8_t *out, size_t out_cap);

/* OPFIELD-PERQ decode. Returns bytes consumed or -1. */
int opfield_scene_decode_perq(opfield_scene_t *s,
                               const uint8_t *data, size_t len);

/* ======================================================================
 * Binaural HRTF rendering (PRISM + HVSS)
 * ====================================================================== */

/* Initialize renderer; zeros all state. */
void opfield_binaural_init(opfield_binaural_t *r, uint32_t sample_rate);

/* Recompute HRTF parameters for object obj_idx.
 * dist_m: source distance in metres (1/r attenuation applied for dist > 1m).
 * spread: 0=directional HRTF, 1=omnidirectional dry blend.
 * gain:   linear amplitude multiplier.
 * HVSS smoothing is applied on subsequent render calls. */
void opfield_binaural_update(opfield_binaural_t *r, uint8_t obj_idx,
                             float az_deg, float el_deg,
                             float dist_m, float spread, float gain);

/* Render n_samples of mono 'in' for object obj_idx.
 * ACCUMULATES into out_l and out_r — caller must zero them before the first object. */
void opfield_binaural_render(opfield_binaural_t *r, uint8_t obj_idx,
                             const float *in, float *out_l, float *out_r,
                             uint32_t n_samples);

/* ======================================================================
 * HOA — AmbiX ACN/SN3D spherical harmonics
 *
 * Encoding formula (unit vector x,y,z from az/el):
 *   x = cos(el)·cos(az),  y = cos(el)·sin(az),  z = sin(el)
 * See opfield_hoa.c for the explicit 16-channel SN3D formulas.
 * ====================================================================== */

/* Evaluate ACN/SN3D SH basis up to 'order' at (az_rad, el_rad).
 * out_sh capacity: (order+1)^2.  order ∈ {0,1,2,3}. */
void opfield_sh_eval(float az_rad, float el_rad, uint8_t order, float *out_sh);

/* HOA-encode one block: accumulate gain × sh[k] × in[n] into out_hoa.
 * out_hoa layout: channel-major, out_hoa[ch * n_samples + n].
 * Caller zeros out_hoa before first object. */
void opfield_hoa_encode(float az_deg, float el_deg, float gain,
                        const float *in, float *out_hoa,
                        uint32_t n_samples, uint8_t order);

/* ======================================================================
 * VBAP — Vector Base Amplitude Panning (Pulkki 1997)
 * ====================================================================== */

/* Build VBAP from speaker positions. Returns 0 on success, -1 on failure. */
int opfield_vbap_init(opfield_vbap_t *v, const opfield_pos_t *speakers,
                      uint8_t n_speakers);

/* Compute per-speaker gains for source at (az_deg, el_deg).
 * gains[] capacity: v->n_speakers.  ‖gains‖₂ = 1 (energy normalization). */
void opfield_vbap_gains(const opfield_vbap_t *v, float az_deg, float el_deg,
                        float *gains);

/* ======================================================================
 * Scene-level convenience renders
 *
 * in_bufs[i] is the mono PCM source for scene object i; NULL entries are
 * skipped.  Caller zeros the output buffer(s) before the first call.
 * ====================================================================== */

/* Render all active POINT objects through PRISM+HVSS binaural. */
void opfield_scene_render_binaural(const opfield_scene_t *s,
                                   opfield_binaural_t *r,
                                   const float *const *in_bufs,
                                   float *out_l, float *out_r,
                                   uint32_t n_samples);

/* HOA-encode all active POINT objects, accumulating into out_hoa.
 * out_hoa layout: channel-major, out_hoa[ch * n_samples + n]. */
void opfield_scene_render_hoa(const opfield_scene_t *s,
                              const float *const *in_bufs,
                              float *out_hoa,
                              uint32_t n_samples, uint8_t order);

/* VBAP pan all active POINT objects into per-speaker output streams.
 * out_spk layout: out_spk[spk * n_samples + n]. */
void opfield_scene_render_vbap(const opfield_scene_t *s,
                               const opfield_vbap_t *v,
                               const float *const *in_bufs,
                               float *out_spk,
                               uint32_t n_samples);

/* ======================================================================
 * Utility
 * ====================================================================== */

static inline void opfield_pos_to_xyz(float az_deg, float el_deg,
                                      float *x, float *y, float *z)
{
    float az = az_deg * ((float)M_PI / 180.0f);
    float el = el_deg * ((float)M_PI / 180.0f);
    *x = cosf(el) * cosf(az);
    *y = cosf(el) * sinf(az);
    *z = sinf(el);
}

#endif /* OPCODEC_OPFIELD_H */
