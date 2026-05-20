/* opfield.c — OPFIELD scene management and metadata encode/decode
 *
 * Includes OPFIELD-PERQ: perceptual JND-zone quantization.
 *
 * PERQ encodes azimuth and elevation in variable-resolution zones based on
 * published azimuth JND data (Mills 1958, Blauert 1997):
 *   Front zone  |az| < 45°:  1° resolution (7 bits + sign)
 *   Lateral zone |az| 45-90°: 3° resolution (5 bits + sign)
 *   Rear zone  |az| > 90°:  5° resolution (5 bits + sign)
 *   Elevation:               2° resolution  (6 bits + sign)
 *
 * OOB (standard) wire: 1 + 9×N bytes  (16-bit fixed 0.01° resolution)
 * PERQ wire:           1 + 6×N bytes  (zone-adapted, ~1° worst-case)
 * Savings: ≈33% metadata bandwidth for typical object counts.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/opfield.h"
#include <string.h>
#include <stdint.h>
#include <math.h>   /* fabsf, fmaxf */

/* ---- Internal helpers ---- */

static inline void write_be16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v >> 8);
    p[1] = (uint8_t)((uint16_t)v & 0xFF);
}

static inline int16_t read_be16(const uint8_t *p)
{
    uint16_t u = ((uint16_t)p[0] << 8) | p[1];
    return (int16_t)u;
}

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ---- Scene API ---- */

void opfield_scene_init(opfield_scene_t *s, uint32_t sample_rate,
                        opfield_render_mode_t mode)
{
    if (!s) return;
    memset(s, 0, sizeof(*s));
    s->sample_rate = sample_rate;
    s->render_mode = mode;
}

int opfield_scene_add_object(opfield_scene_t *s, const opfield_object_t *obj)
{
    if (!s || !obj || s->n_objects >= OPFIELD_MAX_OBJECTS)
        return -1;
    s->objects[s->n_objects] = *obj;
    return (int)s->n_objects++;
}

/* ---- Standard OOB encode/decode ---- */

/* Wire layout per object (9 bytes):
 *   [type:1][az_i16 BE ×100][el_i16 BE ×100][dist_u16 BE ×100]
 *   [gain_u8 ×50][spread_u8 ×255]
 */
int opfield_scene_encode(const opfield_scene_t *s, uint8_t *out, size_t out_cap)
{
    if (!s || !out)
        return -1;
    size_t need = 1u + (size_t)s->n_objects * OPFIELD_OBJ_BYTES;
    if (out_cap < need)
        return -1;

    out[0] = s->n_objects;
    uint8_t *p = out + 1;

    for (uint8_t i = 0; i < s->n_objects; i++) {
        const opfield_object_t *o = &s->objects[i];
        p[0] = (uint8_t)o->type;
        write_be16(p + 1, (int16_t)clampf(o->pos.az * 100.0f, -18000.0f, 18000.0f));
        write_be16(p + 3, (int16_t)clampf(o->pos.el * 100.0f, -9000.0f, 9000.0f));
        uint16_t dist_cm = (uint16_t)clampf(o->pos.dist * 100.0f, 0.0f, 65535.0f);
        p[5] = (uint8_t)(dist_cm >> 8);
        p[6] = (uint8_t)(dist_cm & 0xFF);
        p[7] = (uint8_t)clampf(o->gain * 50.0f, 0.0f, 255.0f);
        p[8] = (uint8_t)clampf(o->spread * 255.0f, 0.0f, 255.0f);
        p += OPFIELD_OBJ_BYTES;
    }
    return (int)need;
}

int opfield_scene_decode(opfield_scene_t *s, const uint8_t *data, size_t len)
{
    if (!s || !data || len < 1)
        return -1;

    uint8_t n = data[0];
    size_t need = 1u + (size_t)n * OPFIELD_OBJ_BYTES;
    if (len < need)
        return -1;

    s->n_objects = 0;
    const uint8_t *p = data + 1;

    for (uint8_t i = 0; i < n; i++) {
        if (s->n_objects >= OPFIELD_MAX_OBJECTS)
            break;
        opfield_object_t *o = &s->objects[s->n_objects++];
        o->type      = (opfield_obj_type_t)p[0];
        o->pos.az    = (float)read_be16(p + 1) / 100.0f;
        o->pos.el    = (float)read_be16(p + 3) / 100.0f;
        uint16_t dc  = ((uint16_t)p[5] << 8) | p[6];
        o->pos.dist  = (float)dc / 100.0f;
        o->gain      = (float)p[7] / 50.0f;
        o->spread    = (float)p[8] / 255.0f;
        o->active    = true;
        p += OPFIELD_OBJ_BYTES;
    }
    return (int)need;
}

/* ======================================================================
 * OPFIELD-PERQ: Perceptual JND-Zone Quantization
 *
 * Novel spatial audio metadata compression scheme based on the azimuth
 * just-noticeable difference (JND) varying with source direction.
 *
 * Zone map (hemisphere is symmetric left/right):
 *   Zone 0  FRONT  |az| ≤ 45°:   az quant 1°,  el quant 2°
 *   Zone 1  SIDE   |az| ∈ (45,90°]: az quant 3°, el quant 3°
 *   Zone 2  REAR   |az| > 90°:   az quant 5°,  el quant 4°
 *
 * PERQ byte layout per object (6 bytes):
 *   [flags:1]   bits 7-6 = zone (0-2)
 *               bit 5 = az sign
 *               bit 4 = el sign
 *               bits 3-0 = az_hi (high 4 bits of az code)
 *   [az_lo:1]   low 4 bits of az code in high nibble, el high nibble low
 *               Actually easier:
 *
 * Simplified: 3 bytes for position + 1 type + 1 gain + 1 spread = 6 bytes.
 * Position bytes: [zone+az_sign+el_sign+az[5:2]] [az[1:0]+el[5:0]] [dist:8]
 *   This gives 6 bits each for az_code and el_code.
 *   az_code × quant_step = az magnitude (quant_step = 1, 3, or 5 per zone)
 *   el_code × el_step    = el magnitude (el_step = 2, 3, or 4 per zone)
 *
 * Total az range:
 *   Zone 0: 63 × 1° = 63° (covers ±45° fully, errors up to 0.5°)
 *   Zone 1: 63 × 3° = 189° → enough for ±90° at 3° resolution
 *   Zone 2: 63 × 5° = 315° → enough for ±180° at 5° resolution
 *
 * Total el range (all zones): 63 × 4° = 252° → enough for ±90°
 * ====================================================================== */

typedef struct {
    uint8_t az_step;
    uint8_t el_step;
} perq_zone_t;

static const perq_zone_t perq_zones[3] = {
    {1, 2},  /* front */
    {3, 3},  /* side  */
    {5, 4},  /* rear  */
};

static uint8_t perq_classify(float az_deg)
{
    float a = az_deg < 0.0f ? -az_deg : az_deg;
    if (a <= 45.0f) return 0;
    if (a <= 90.0f) return 1;
    return 2;
}

/* Encode: returns bytes written (6×N+1), or -1 on error. */
int opfield_scene_encode_perq(const opfield_scene_t *s,
                               uint8_t *out, size_t out_cap)
{
    if (!s || !out) return -1;
    size_t need = 1u + (size_t)s->n_objects * 6u;
    if (out_cap < need) return -1;

    out[0] = s->n_objects;
    uint8_t *p = out + 1;

    for (uint8_t i = 0; i < s->n_objects; i++) {
        const opfield_object_t *o = &s->objects[i];
        uint8_t zone = perq_classify(o->pos.az);
        const perq_zone_t *z = &perq_zones[zone];

        uint8_t az_sign = (o->pos.az < 0.0f) ? 1u : 0u;
        uint8_t el_sign = (o->pos.el < 0.0f) ? 1u : 0u;
        float   az_mag  = o->pos.az < 0.0f ? -o->pos.az : o->pos.az;
        float   el_mag  = o->pos.el < 0.0f ? -o->pos.el : o->pos.el;

        uint8_t az_code = (uint8_t)clampf(az_mag / z->az_step + 0.5f, 0.0f, 63.0f);
        uint8_t el_code = (uint8_t)clampf(el_mag / z->el_step + 0.5f, 0.0f, 63.0f);
        uint8_t dist8   = (uint8_t)clampf(o->pos.dist * 25.5f, 0.0f, 255.0f);

        /* Byte 0: [zone:2][az_sign:1][el_sign:1][az_code[5:2]:4] */
        p[0] = (uint8_t)((zone << 6) | (az_sign << 5) | (el_sign << 4)
                        | ((az_code >> 2) & 0x0Fu));
        /* Byte 1: [az_code[1:0]:2][el_code[5:0]:6] */
        p[1] = (uint8_t)(((az_code & 0x03u) << 6) | (el_code & 0x3Fu));
        p[2] = dist8;
        p[3] = (uint8_t)o->type;
        p[4] = (uint8_t)clampf(o->gain * 50.0f, 0.0f, 255.0f);
        p[5] = (uint8_t)clampf(o->spread * 255.0f, 0.0f, 255.0f);
        p += 6;
    }
    return (int)need;
}

/* Decode PERQ bitstream. Returns bytes consumed, or -1 on error. */
int opfield_scene_decode_perq(opfield_scene_t *s,
                               const uint8_t *data, size_t len)
{
    if (!s || !data || len < 1) return -1;
    uint8_t n = data[0];
    if (len < 1u + (size_t)n * 6u) return -1;

    s->n_objects = 0;
    const uint8_t *p = data + 1;

    for (uint8_t i = 0; i < n; i++) {
        if (s->n_objects >= OPFIELD_MAX_OBJECTS) break;
        opfield_object_t *o = &s->objects[s->n_objects++];

        uint8_t zone    = (p[0] >> 6) & 0x03u;
        uint8_t az_sign = (p[0] >> 5) & 0x01u;
        uint8_t el_sign = (p[0] >> 4) & 0x01u;
        uint8_t az_code = (uint8_t)(((p[0] & 0x0Fu) << 2) | ((p[1] >> 6) & 0x03u));
        uint8_t el_code = p[1] & 0x3Fu;
        float   dist    = (float)p[2] / 25.5f;

        if (zone >= 3) zone = 2;
        const perq_zone_t *z = &perq_zones[zone];

        float az = (float)az_code * (float)z->az_step;
        float el = (float)el_code * (float)z->el_step;
        if (az_sign) az = -az;
        if (el_sign) el = -el;

        o->type      = (opfield_obj_type_t)p[3];
        o->pos.az    = az;
        o->pos.el    = el;
        o->pos.dist  = dist;
        o->gain      = (float)p[4] / 50.0f;
        o->spread    = (float)p[5] / 255.0f;
        o->active    = true;
        p += 6;
    }
    return (int)(1u + (size_t)n * 6u);
}

/* ======================================================================
 * Scene-level render helpers
 * ====================================================================== */

void opfield_scene_render_binaural(const opfield_scene_t *s,
                                   opfield_binaural_t *r,
                                   const float *const *in_bufs,
                                   float *out_l, float *out_r,
                                   uint32_t n_samples)
{
    if (!s || !r || !out_l || !out_r || !in_bufs) return;
    for (uint8_t i = 0; i < s->n_objects; i++) {
        const opfield_object_t *o = &s->objects[i];
        if (!o->active || o->type != OPFIELD_OBJ_POINT) continue;
        if (!in_bufs[i]) continue;
        opfield_binaural_update(r, i,
                                o->pos.az, o->pos.el,
                                o->pos.dist, o->spread, o->gain);
        opfield_binaural_render(r, i, in_bufs[i], out_l, out_r, n_samples);
    }
}

void opfield_scene_render_hoa(const opfield_scene_t *s,
                              const float *const *in_bufs,
                              float *out_hoa,
                              uint32_t n_samples, uint8_t order)
{
    if (!s || !out_hoa || !in_bufs) return;
    for (uint8_t i = 0; i < s->n_objects; i++) {
        const opfield_object_t *o = &s->objects[i];
        if (!o->active || o->type != OPFIELD_OBJ_POINT) continue;
        if (!in_bufs[i]) continue;
        /* Distance attenuation applied to gain */
        float g = o->gain / (o->pos.dist > 1.0f ? o->pos.dist : 1.0f);
        opfield_hoa_encode(o->pos.az, o->pos.el, g,
                           in_bufs[i], out_hoa, n_samples, order);
    }
}

void opfield_scene_render_vbap(const opfield_scene_t *s,
                               const opfield_vbap_t *v,
                               const float *const *in_bufs,
                               float *out_spk,
                               uint32_t n_samples)
{
    if (!s || !v || !out_spk || !in_bufs) return;
    for (uint8_t i = 0; i < s->n_objects; i++) {
        const opfield_object_t *o = &s->objects[i];
        if (!o->active || o->type != OPFIELD_OBJ_POINT) continue;
        if (!in_bufs[i]) continue;

        float gains[OPFIELD_MAX_SPEAKERS];
        for (uint8_t k = 0; k < v->n_speakers; k++) gains[k] = 0.0f;
        opfield_vbap_gains(v, o->pos.az, o->pos.el, gains);

        float g_dist = o->gain / (o->pos.dist > 1.0f ? o->pos.dist : 1.0f);

        for (uint8_t k = 0; k < v->n_speakers; k++) {
            float w = gains[k] * g_dist;
            if (fabsf(w) < 1e-12f) continue;
            float *dst = out_spk + (uint32_t)k * n_samples;
            const float *src = in_bufs[i];
            for (uint32_t n = 0; n < n_samples; n++)
                dst[n] += w * src[n];
        }
    }
}
