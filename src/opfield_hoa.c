/* opfield_hoa.c — OPFIELD Higher-Order Ambisonics and VBAP
 *
 * HOA encoding: AmbiX ACN/SN3D convention (AES-X274)
 *   All 16 channels (orders 0-3) are derived from first principles using the
 *   associated Legendre polynomials with SN3D normalization:
 *     N_n^m = sqrt((2 - δ_{m,0}) · (n-|m|)! / (n+|m|)!)
 *   Expressed in Cartesian form: x=cos(el)cos(az), y=cos(el)sin(az), z=sin(el).
 *
 * VBAP: Pulkki (1997) algorithm
 *   1. Triangulate speaker directions on the sphere (convex hull approach).
 *   2. Precompute inverse L^{-1} for each triplet L = [l_i | l_j | l_k].
 *   3. For source p: g_raw = L^{-1}·p; valid triplet has all g_raw_i ≥ 0.
 *   4. Normalize: g = g_raw / ‖g_raw‖₂ (equal-energy panning).
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/opfield.h"
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* ---- Utility ---- */

/* ======================================================================
 * HOA — ACN/SN3D spherical harmonics, orders 0-3
 *
 * Explicit SN3D Cartesian formulas (unit vector x,y,z):
 *
 * Order 0 (1 channel):
 *   W  = 1
 *
 * Order 1 (3 channels, SN3D factor = 1 for all):
 *   Y  = y      (sin(az)·cos(el), left/right)
 *   Z  = z      (sin(el), up/down)
 *   X  = x      (cos(az)·cos(el), front/back)
 *
 * Order 2 (5 channels):
 *   V  = √3·x·y
 *   T  = √3·y·z
 *   R  = (3z²-1)/2
 *   S  = √3·x·z
 *   U  = (√3/2)·(x²-y²)
 *
 * Order 3 (7 channels — derived from P_n^m with SN3D normalization):
 *   Q  = √(5/8)·y·(3x²-y²)     [ACN 9,  derived: N·P_3^3·sin(3az)]
 *   O  = √15·x·y·z              [ACN 10, derived: N·P_3^2·sin(2az)·sin(el)]
 *   M  = (√6/4)·y·(5z²-1)      [ACN 11, derived: N·P_3^1·sin(az)]
 *   K  = (5z³-3z)/2             [ACN 12, P_3^0]
 *   L  = (√6/4)·x·(5z²-1)      [ACN 13, derived: N·P_3^1·cos(az)]
 *   N  = (√15/2)·z·(x²-y²)     [ACN 14, derived: N·P_3^2·cos(2az)·sin(el)]
 *   P  = √(5/8)·x·(x²-3y²)     [ACN 15, derived: N·P_3^3·cos(3az)]
 *
 * Derivation for order 3 example (ACN 9, n=3, m=-3):
 *   N_3^3 = sqrt(2·0!/6!) = sqrt(1/360) = 1/(6√10)
 *   P_3^3(sin el) = 15·cos³(el)
 *   sin(3az)·cos³(el) = y·(3x²-y²) [triple-angle identity in Cartesian]
 *   Y_3^{-3} = N_3^3 · 15·cos³(el) · sin(3az)
 *             = (15/(6√10)) · y·(3x²-y²) = √(5/8) · y·(3x²-y²)  ✓
 * ====================================================================== */

#define SQRT3    1.7320508075688772f
#define SQRT5_8  0.7905694150420949f   /* sqrt(5/8) */
#define SQRT6    2.449489742783178f
#define SQRT15   3.872983346207417f

void opfield_sh_eval(float az_rad, float el_rad, uint8_t order, float *out_sh)
{
    if (!out_sh) return;

    /* Cartesian unit vector components */
    float ce = cosf(el_rad);
    float se = sinf(el_rad);
    float ca = cosf(az_rad);
    float sa = sinf(az_rad);
    float x  = ce * ca;
    float y  = ce * sa;
    float z  = se;

    /* Order 0 */
    out_sh[0] = 1.0f;

    if (order < 1) return;

    /* Order 1 */
    out_sh[1] = y;
    out_sh[2] = z;
    out_sh[3] = x;

    if (order < 2) return;

    /* Order 2 */
    float x2 = x * x, y2 = y * y, z2 = z * z;
    out_sh[4] = SQRT3   * x * y;
    out_sh[5] = SQRT3   * y * z;
    out_sh[6] = 0.5f    * (3.0f * z2 - 1.0f);
    out_sh[7] = SQRT3   * x * z;
    out_sh[8] = (SQRT3 * 0.5f) * (x2 - y2);

    if (order < 3) return;

    /* Order 3 */
    float z3 = z2 * z;
    out_sh[9]  = SQRT5_8       * y * (3.0f * x2 - y2);
    out_sh[10] = SQRT15        * x * y * z;
    out_sh[11] = (SQRT6 / 4.0f) * y * (5.0f * z2 - 1.0f);
    out_sh[12] = 0.5f          * (5.0f * z3 - 3.0f * z);
    out_sh[13] = (SQRT6 / 4.0f) * x * (5.0f * z2 - 1.0f);
    out_sh[14] = (SQRT15 / 2.0f) * z * (x2 - y2);
    out_sh[15] = SQRT5_8       * x * (x2 - 3.0f * y2);
}

void opfield_hoa_encode(float az_deg, float el_deg, float gain,
                        const float *in, float *out_hoa,
                        uint32_t n_samples, uint8_t order)
{
    if (!in || !out_hoa || order > 3) return;

    float az = az_deg * ((float)M_PI / 180.0f);
    float el = el_deg * ((float)M_PI / 180.0f);

    float sh[OPFIELD_HOA_CHANNELS];
    opfield_sh_eval(az, el, order, sh);

    uint8_t n_ch = (uint8_t)((order + 1u) * (order + 1u));

    /* Multiply by gain once */
    for (uint8_t ch = 0; ch < n_ch; ch++)
        sh[ch] *= gain;

    /* Accumulate: out_hoa[ch * n_samples + n] += sh[ch] * in[n] */
    for (uint8_t ch = 0; ch < n_ch; ch++) {
        float w   = sh[ch];
        float *dst = out_hoa + (uint32_t)ch * n_samples;
        for (uint32_t n = 0; n < n_samples; n++)
            dst[n] += w * in[n];
    }
}

/* ======================================================================
 * VBAP — Vector Base Amplitude Panning
 *
 * Pulkki (1997): gains for a source in direction p using speaker triplet
 * (i,j,k) with L=[l_i|l_j|l_k]:
 *   g_raw = L^{-1} · p
 *   if all g_raw_i ≥ 0: valid triplet.
 *   g = g_raw / ‖g_raw‖₂   (energy normalization, ‖g‖₂ = 1)
 * ====================================================================== */

/* 3×3 determinant */
static float det3(const float m[3][3])
{
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
         - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
         + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

/* 3×3 matrix inverse (column-major columns = speaker vectors).
 * M stored as M[row][col].  Returns false if nearly singular. */
static bool inv3(const float m[3][3], float inv[3][3])
{
    float d = det3(m);
    if (fabsf(d) < 1e-6f) return false;
    float di = 1.0f / d;

    inv[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * di;
    inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * di;
    inv[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * di;
    inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * di;
    inv[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * di;
    inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * di;
    inv[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * di;
    inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * di;
    inv[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * di;
    return true;
}

/* Matvec: y = M·v  (3×3 times 3-vector) */
static void matvec3(const float m[3][3], const float v[3], float y[3])
{
    for (int i = 0; i < 3; i++)
        y[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
}

int opfield_vbap_init(opfield_vbap_t *v, const opfield_pos_t *speakers,
                      uint8_t n_speakers)
{
    if (!v || !speakers || n_speakers < 3
            || n_speakers > OPFIELD_MAX_SPEAKERS) return -1;

    v->n_speakers = n_speakers;
    v->n_trips    = 0;

    /* Convert speaker positions to unit Cartesian vectors */
    for (uint8_t k = 0; k < n_speakers; k++) {
        float az = speakers[k].az * ((float)M_PI / 180.0f);
        float el = speakers[k].el * ((float)M_PI / 180.0f);
        v->spk_xyz[k][0] = cosf(el) * cosf(az);  /* x */
        v->spk_xyz[k][1] = cosf(el) * sinf(az);  /* y */
        v->spk_xyz[k][2] = sinf(el);              /* z */
    }

    /* Generate all C(n,3) triplets; keep those that form a valid outward-facing
     * triangle (determinant > 0 = vectors form a right-handed basis covering
     * that part of the sphere, so all sources in that octant will yield positive
     * gains for this triplet).
     *
     * Triangle validity: det(L) > threshold.  A negative det means the triplet
     * wraps the "wrong way" around the sphere. */
    for (uint8_t i = 0; i < n_speakers && v->n_trips < OPFIELD_VBAP_MAX_TRIPS; i++) {
        for (uint8_t j = i + 1u; j < n_speakers && v->n_trips < OPFIELD_VBAP_MAX_TRIPS; j++) {
            for (uint8_t k = j + 1u; k < n_speakers && v->n_trips < OPFIELD_VBAP_MAX_TRIPS; k++) {
                /* L = [l_i | l_j | l_k] in column-major (L[row][col]) */
                float L[3][3];
                for (int r = 0; r < 3; r++) {
                    L[r][0] = v->spk_xyz[i][r];
                    L[r][1] = v->spk_xyz[j][r];
                    L[r][2] = v->spk_xyz[k][r];
                }

                float d = det3(L);
                if (d < 1e-3f) continue;  /* invalid or wrong-facing */

                float invL[3][3];
                if (!inv3(L, invL)) continue;

                opfield_vbap_triplet_t *t = &v->trips[v->n_trips++];
                t->spk[0] = i; t->spk[1] = j; t->spk[2] = k;
                memcpy(t->inv_L, invL, sizeof(invL));
            }
        }
    }

    /* ---- 2D fallback: detect coplanar (horizontal) speaker arrays ---- *
     * When all speakers are within ~3° of the equatorial plane the z-rows
     * of every 3×3 L matrix are near-zero, giving det≈0 for all triplets.
     * Fall back to Pulkki 2D arc-pair panning using projected XY vectors.  */
    float max_z = 0.0f;
    for (uint8_t k = 0; k < n_speakers; k++) {
        float z = fabsf(v->spk_xyz[k][2]);
        if (z > max_z) max_z = z;
    }
    v->is_2d = (max_z < 0.06f);   /* sin(3.5°) ≈ 0.061 */
    v->n_pairs = 0;

    if (v->is_2d) {
        /* Sort speaker indices by azimuth angle (insertion sort) */
        uint8_t order[OPFIELD_MAX_SPEAKERS];
        float   az[OPFIELD_MAX_SPEAKERS];
        for (uint8_t k = 0; k < n_speakers; k++) {
            order[k] = k;
            az[k]    = atan2f(v->spk_xyz[k][1], v->spk_xyz[k][0]);
        }
        for (uint8_t i = 1; i < n_speakers; i++) {
            uint8_t oi = order[i]; float ai = az[i];
            uint8_t j  = i;
            while (j > 0 && az[j - 1] > ai) {
                az[j] = az[j - 1]; order[j] = order[j - 1]; j--;
            }
            az[j] = ai; order[j] = oi;
        }

        /* Build one arc pair per adjacent speaker (circular) */
        for (uint8_t k = 0; k < n_speakers; k++) {
            uint8_t si = order[k];
            uint8_t sj = order[(k + 1u) % n_speakers];
            float xi = v->spk_xyz[si][0], yi = v->spk_xyz[si][1];
            float xj = v->spk_xyz[sj][0], yj = v->spk_xyz[sj][1];
            float d  = xi * yj - xj * yi;
            if (fabsf(d) < 1e-6f) continue;  /* collinear, skip */
            float di = 1.0f / d;
            opfield_vbap_pair_t *pr = &v->pairs[v->n_pairs++];
            pr->spk[0] = si; pr->spk[1] = sj;
            pr->inv_L2[0][0] =  yj * di; pr->inv_L2[0][1] = -xj * di;
            pr->inv_L2[1][0] = -yi * di; pr->inv_L2[1][1] =  xi * di;
        }
    }

    return (v->n_trips > 0 || v->n_pairs > 0) ? 0 : -1;
}

void opfield_vbap_gains(const opfield_vbap_t *v, float az_deg, float el_deg,
                        float *gains)
{
    if (!v || !gains) return;

    /* Zero all speaker gains */
    for (uint8_t k = 0; k < v->n_speakers; k++)
        gains[k] = 0.0f;

    /* Source direction unit vector */
    float p[3];
    {
        float az = az_deg * ((float)M_PI / 180.0f);
        float el = el_deg * ((float)M_PI / 180.0f);
        p[0] = cosf(el) * cosf(az);
        p[1] = cosf(el) * sinf(az);
        p[2] = sinf(el);
    }

    /* Find valid triplet: g_raw = L^{-1}·p, all components ≥ -ε */
    const float eps = -1e-4f;
    for (uint16_t t = 0; t < v->n_trips; t++) {
        const opfield_vbap_triplet_t *tr = &v->trips[t];
        float g[3];
        matvec3(tr->inv_L, p, g);

        if (g[0] < eps || g[1] < eps || g[2] < eps) continue;

        /* Clamp small negatives caused by floating-point to zero */
        if (g[0] < 0.0f) g[0] = 0.0f;
        if (g[1] < 0.0f) g[1] = 0.0f;
        if (g[2] < 0.0f) g[2] = 0.0f;

        /* Energy normalization: ‖g‖₂ = 1 */
        float norm = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
        if (norm < 1e-10f) norm = 1e-10f;
        float inv_norm = 1.0f / norm;

        gains[tr->spk[0]] = g[0] * inv_norm;
        gains[tr->spk[1]] = g[1] * inv_norm;
        gains[tr->spk[2]] = g[2] * inv_norm;
        return;  /* done — first valid triplet wins */
    }

    /* 2D fallback: arc-pair panning for coplanar (horizontal) arrays */
    if (v->is_2d && v->n_pairs > 0) {
        for (uint8_t t = 0; t < v->n_pairs; t++) {
            const opfield_vbap_pair_t *pr = &v->pairs[t];
            float g[2];
            g[0] = pr->inv_L2[0][0] * p[0] + pr->inv_L2[0][1] * p[1];
            g[1] = pr->inv_L2[1][0] * p[0] + pr->inv_L2[1][1] * p[1];

            if (g[0] < eps || g[1] < eps) continue;

            if (g[0] < 0.0f) g[0] = 0.0f;
            if (g[1] < 0.0f) g[1] = 0.0f;

            float norm = sqrtf(g[0] * g[0] + g[1] * g[1]);
            if (norm < 1e-10f) norm = 1e-10f;

            gains[pr->spk[0]] = g[0] / norm;
            gains[pr->spk[1]] = g[1] / norm;
            return;
        }
    }

    /* Last resort: nearest speaker */
    float best_dot = -2.0f;
    uint8_t best   = 0;
    for (uint8_t k = 0; k < v->n_speakers; k++) {
        float dot = v->spk_xyz[k][0] * p[0]
                  + v->spk_xyz[k][1] * p[1]
                  + v->spk_xyz[k][2] * p[2];
        if (dot > best_dot) { best_dot = dot; best = k; }
    }
    gains[best] = 1.0f;
}
