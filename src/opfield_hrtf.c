/* opfield_hrtf.c — OPFIELD binaural HRTF rendering (PRISM + HVSS)
 *
 * OPFIELD-PRISM (Phase-Resolved Interaural Spectral Morphing):
 *   Duplex theory (Lord Rayleigh 1907) shows that below ~700 Hz sound
 *   localization depends on interaural PHASE (ITD), and above ~1500 Hz on
 *   interaural LEVEL (ILD). Single-band simplified HRTF ignores this.
 *
 *   PRISM splits each source signal at 700 Hz via a 1st-order LP crossover:
 *     Low band  (<700 Hz): full ITD  (weight 1.0),  minimal ILD (weight 0.15)
 *     High band (>700 Hz): scaled ITD (weight 0.25), full ILD   (weight 1.0)
 *   The two bands are processed independently then summed.
 *
 * OPFIELD-HVSS (HRTF Velocity Smooth Scan):
 *   ILD filter coefficients lerp from current toward target values each update.
 *   Smoothing speed α adapts: large change → α=0.15 (fast), no change → α=0.03.
 *   Prevents comb-filter artefacts when objects move rapidly.
 *
 * Models (clean-room from published theory):
 *   ITD  — Woodworth (1937): τ = (r/c)(θ + sin θ),  θ = arcsin(cos(el)·sin(az))
 *   ILD  — Brown-Duda (1998): H(s) = (α·s + ωe)/(s + ωe)
 *            α = 1 + cos(θ_ear)/2,  ωe = c/r ≈ 3920 rad/s
 *            Bilinear coeffs (k = 2·fs):
 *              b0 = (α·k + ωe)/(k + ωe)
 *              b1 = (ωe - α·k)/(k + ωe)
 *              a1 = (ωe - k)/(k + ωe)
 *   Pinna — Peaking EQ (Raykar et al. 2004):
 *            f_notch = 8000 + 40·el_deg Hz,  Q = 2.5,  gain = −12 dB
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/opfield.h"
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/* PRISM weights */
#define PRISM_LO_ILD_W   0.15f   /* low-band ILD factor [0..1] */
#define PRISM_HI_ITD_W   0.25f   /* high-band ITD fraction of full delay */

/* HVSS alpha range */
#define HVSS_ALPHA_FAST  0.15f
#define HVSS_ALPHA_SLOW  0.03f

/* Pinna notch parameters */
#define PINNA_F0_BASE    8000.0f  /* Hz at elevation 0° */
#define PINNA_F0_SLOPE   40.0f   /* Hz per degree elevation */
#define PINNA_F0_MIN     5000.0f
#define PINNA_F0_MAX     14000.0f
#define PINNA_Q          2.5f
#define PINNA_GAIN_DB    (-12.0f)

/* ---- Helpers ---- */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Woodworth (1937) ITD in seconds.
 * Positive result: source on LEFT, right ear is the far ear. */
static float compute_itd_s(float az_deg, float el_deg)
{
    float az = az_deg * ((float)M_PI / 180.0f);
    float el = el_deg * ((float)M_PI / 180.0f);
    /* Effective azimuth: project source onto the interaural axis */
    float theta = asinf(clampf(cosf(el) * sinf(az), -1.0f, 1.0f));
    return (OPFIELD_HEAD_RADIUS / OPFIELD_SPEED_OF_SOUND) * (theta + sinf(theta));
}

/* Brown-Duda (1998) ILD shelving coefficients.
 * H(s) = (α·s + ωe)/(s + ωe),  α = 1 + cos(θ_ear)/2
 * cos(θ_ear_L) =  cos(el)·sin(az)  [left ear axis at +90°]
 * cos(θ_ear_R) = -cos(el)·sin(az)  [right ear axis at -90°]
 *
 * ild_weight ∈ [0,1]: 0 → unity (no ILD), 1 → full Brown-Duda shadow.
 * ild_weight < 1 blends α toward 1.0 (unity gain).
 */
static void compute_ild(float az_deg, float el_deg, uint32_t fs,
                         float ild_weight,
                         float b0[2], float b1[2], float a1[2])
{
    float az = az_deg * ((float)M_PI / 180.0f);
    float el = el_deg * ((float)M_PI / 180.0f);
    float ce_sa = cosf(el) * sinf(az);  /* cos(el)·sin(az) */

    float k  = 2.0f * (float)fs;
    float we = OPFIELD_EAR_OMEGA;

    /* cos of ear angle: +cos_ear = near ear, -cos_ear = far ear */
    float cos_ear[2] = { ce_sa, -ce_sa };

    for (int e = 0; e < 2; e++) {
        float alpha_full = 1.0f + cos_ear[e] * 0.5f;   /* Brown-Duda α */
        float alpha = 1.0f + (alpha_full - 1.0f) * ild_weight;  /* blended */

        float denom = k + we;
        b0[e] = (alpha * k + we) / denom;
        b1[e] = (we - alpha * k) / denom;
        a1[e] = (we - k)         / denom;
    }
}

/* Pinna elevation notch: 2nd-order peaking EQ (RBJ Audio EQ Cookbook).
 * Applied to high-band only. Same coefficients for both ears (symmetric model). */
static void compute_pinna(float el_deg, uint32_t fs,
                           float b0[2], float b1[2], float b2[2],
                           float a1[2], float a2[2])
{
    float f0 = clampf(PINNA_F0_BASE + PINNA_F0_SLOPE * el_deg,
                      PINNA_F0_MIN, PINNA_F0_MAX);
    float w0    = 2.0f * (float)M_PI * f0 / (float)fs;
    float A     = powf(10.0f, PINNA_GAIN_DB / 40.0f);  /* amplitude ratio */
    float alpha = sinf(w0) / (2.0f * PINNA_Q);
    float cw    = cosf(w0);

    float a0_r = 1.0f / (1.0f + alpha / A);

    for (int e = 0; e < 2; e++) {
        b0[e] = (1.0f + alpha * A) * a0_r;
        b1[e] = -2.0f * cw         * a0_r;
        b2[e] = (1.0f - alpha * A) * a0_r;
        a1[e] = -2.0f * cw         * a0_r;
        a2[e] = (1.0f - alpha / A) * a0_r;
    }
}

/* 1st-order Butterworth LP crossover via bilinear transform.
 * y[n] = a·(x[n] + x[n-1]) + b·y[n-1]
 * HP[n] = x[n] - LP[n]  (perfect reconstruction).
 * a = ωc/(ωc+k),  b = (k−ωc)/(k+ωc),  k = 2·fs. */
static void compute_crossover(float f_hz, uint32_t fs, float *out_a, float *out_b)
{
    float wc = 2.0f * (float)M_PI * f_hz;
    float k  = 2.0f * (float)fs;
    *out_a = wc / (wc + k);
    *out_b = (k - wc) / (k + wc);
}

/* ---- Public API ---- */

void opfield_binaural_init(opfield_binaural_t *r, uint32_t sample_rate)
{
    if (!r) return;
    memset(r, 0, sizeof(*r));
    r->sample_rate = sample_rate ? sample_rate : 48000u;
}

void opfield_binaural_update(opfield_binaural_t *r, uint8_t obj_idx,
                             float az_deg, float el_deg,
                             float dist_m, float spread, float gain)
{
    if (!r || obj_idx >= OPFIELD_MAX_OBJECTS) return;
    opfield_hrtf_state_t *st = &r->obj[obj_idx];
    uint32_t fs = r->sample_rate;
    if (!fs) fs = 48000;

    /* Store source properties for use in render */
    st->dist_m = dist_m  > 0.0f ? dist_m  : 1.0f;
    st->spread = spread  < 0.0f ? 0.0f : (spread  > 1.0f ? 1.0f : spread);
    st->gain   = gain    > 0.0f ? gain    : 0.0f;

    /* Woodworth ITD → integer sample delay per ear */
    float itd_s   = compute_itd_s(az_deg, el_deg);
    float itd_abs = fabsf(itd_s) * (float)fs;
    uint8_t itd   = (uint8_t)clampf(itd_abs + 0.5f, 0.0f,
                                     (float)(OPFIELD_DELAY_MAX - 1));

    /* Positive ITD = source on left: left ear is near (0 delay), right delayed */
    uint8_t lo_L = (itd_s >= 0.0f) ? 0u : itd;
    uint8_t lo_R = (itd_s >= 0.0f) ? itd : 0u;
    uint8_t hi_L = (uint8_t)(lo_L * PRISM_HI_ITD_W + 0.5f);
    uint8_t hi_R = (uint8_t)(lo_R * PRISM_HI_ITD_W + 0.5f);

    /* HVSS: compare new command against the PREVIOUS COMMAND (targets), not the
     * committed delay.  Committed delays only advance during render, so
     * consecutive identical updates without rendering would always see large
     * delta from stale committed state — incorrectly triggering fast alpha. */
    int delta = abs((int)lo_L - (int)st->tgt_lo_dly_L)
              + abs((int)lo_R - (int)st->tgt_lo_dly_R);
    st->hvss_alpha = (delta > 2) ? HVSS_ALPHA_FAST : HVSS_ALPHA_SLOW;

    /* Compute target ILD coefficients */
    compute_ild(az_deg, el_deg, fs, 1.0f,
                st->tgt_hi_b0, st->tgt_hi_b1, st->tgt_hi_a1);
    compute_ild(az_deg, el_deg, fs, PRISM_LO_ILD_W,
                st->tgt_lo_b0, st->tgt_lo_b1, st->tgt_lo_a1);

    /* Store targets for delay as well */
    st->tgt_lo_dly_L = lo_L;  st->tgt_lo_dly_R = lo_R;
    st->tgt_hi_dly_L = hi_L;  st->tgt_hi_dly_R = hi_R;

    /* Pinna notch (immediately applied — no HVSS needed, less perceptible) */
    compute_pinna(el_deg, fs,
                  st->pn_b0, st->pn_b1, st->pn_b2,
                  st->pn_a1, st->pn_a2);

    /* Crossover coefficients */
    compute_crossover(OPFIELD_PRISM_CROSS_HZ, fs, &st->cx_a, &st->cx_b);

    /* First update: commit all state immediately so render never sees zero
     * coefficients.  Use an explicit flag — the delay-zero heuristic breaks
     * for legitimate on-axis sources where all delays are zero. */
    if (!st->initialized) {
        st->initialized = true;
        st->lo_dly_L = lo_L;  st->lo_dly_R = lo_R;
        st->hi_dly_L = hi_L;  st->hi_dly_R = hi_R;
        for (int e = 0; e < 2; e++) {
            st->hi_ild_b0[e] = st->tgt_hi_b0[e];
            st->hi_ild_b1[e] = st->tgt_hi_b1[e];
            st->hi_ild_a1[e] = st->tgt_hi_a1[e];
            st->lo_ild_b0[e] = st->tgt_lo_b0[e];
            st->lo_ild_b1[e] = st->tgt_lo_b1[e];
            st->lo_ild_a1[e] = st->tgt_lo_a1[e];
        }
    }
}

void opfield_binaural_render(opfield_binaural_t *r, uint8_t obj_idx,
                             const float *in, float *out_l, float *out_r,
                             uint32_t n_samples)
{
    if (!r || !in || !out_l || !out_r || obj_idx >= OPFIELD_MAX_OBJECTS)
        return;

    opfield_hrtf_state_t *st = &r->obj[obj_idx];

    /* Distance attenuation (1/r law, clamped: no boost below 1 m) and spread */
    float g_total = st->gain / (st->dist_m > 1.0f ? st->dist_m : 1.0f);
    float sp = st->spread;   /* 0=full HRTF, 1=omnidirectional dry */

    /* Cache filter state locally */
    float cx_a = st->cx_a, cx_b = st->cx_b;
    float cx_xp = st->cx_xp, cx_yp = st->cx_yp;
    uint8_t pos  = st->dly_pos;
    float   alpha = st->hvss_alpha;

    for (uint32_t n = 0; n < n_samples; n++) {
        float x = in[n];

        /* PRISM crossover: mono LP filter → lo_in; hi_in = x - lo_in */
        float lp = cx_a * (x + cx_xp) + cx_b * cx_yp;
        cx_xp = x;
        cx_yp = lp;
        float lo_in = lp;
        float hi_in = x - lp;

        /* Write both bands to their delay rings */
        st->dly_lo[pos] = lo_in;
        st->dly_hi[pos] = hi_in;

        /* HVSS: lerp current ILD and delays toward targets each sample.
         * Doing it per-sample (not per-block) gives perfectly smooth transitions. */
        for (int e = 0; e < 2; e++) {
            st->hi_ild_b0[e] += alpha * (st->tgt_hi_b0[e] - st->hi_ild_b0[e]);
            st->hi_ild_b1[e] += alpha * (st->tgt_hi_b1[e] - st->hi_ild_b1[e]);
            st->hi_ild_a1[e] += alpha * (st->tgt_hi_a1[e] - st->hi_ild_a1[e]);
            st->lo_ild_b0[e] += alpha * (st->tgt_lo_b0[e] - st->lo_ild_b0[e]);
            st->lo_ild_b1[e] += alpha * (st->tgt_lo_b1[e] - st->lo_ild_b1[e]);
            st->lo_ild_a1[e] += alpha * (st->tgt_lo_a1[e] - st->lo_ild_a1[e]);
        }
        /* Step delays (integer, committed from targets) */
        st->lo_dly_L = st->tgt_lo_dly_L;
        st->lo_dly_R = st->tgt_lo_dly_R;
        st->hi_dly_L = st->tgt_hi_dly_L;
        st->hi_dly_R = st->tgt_hi_dly_R;

        /* Read low band: full ITD taps */
        uint8_t mask = OPFIELD_DELAY_MAX - 1u;  /* DELAY_MAX must be power of 2 */
        float lo_L_del = st->dly_lo[(pos - st->lo_dly_L) & mask];
        float lo_R_del = st->dly_lo[(pos - st->lo_dly_R) & mask];

        /* Read high band: PRISM_HI_ITD_W-scaled ITD taps */
        float hi_L_del = st->dly_hi[(pos - st->hi_dly_L) & mask];
        float hi_R_del = st->dly_hi[(pos - st->hi_dly_R) & mask];

        /* Advance write position */
        pos = (uint8_t)((pos + 1u) & mask);

        /* Apply low-band ILD (low weight, phase-dominant band) */
        float lo_L = st->lo_ild_b0[0] * lo_L_del
                   + st->lo_ild_b1[0] * st->lo_ild_xp[0]
                   - st->lo_ild_a1[0] * st->lo_ild_yp[0];
        st->lo_ild_xp[0] = lo_L_del; st->lo_ild_yp[0] = lo_L;

        float lo_R = st->lo_ild_b0[1] * lo_R_del
                   + st->lo_ild_b1[1] * st->lo_ild_xp[1]
                   - st->lo_ild_a1[1] * st->lo_ild_yp[1];
        st->lo_ild_xp[1] = lo_R_del; st->lo_ild_yp[1] = lo_R;

        /* Apply high-band ILD (full weight, amplitude-dominant band) */
        float hi_L = st->hi_ild_b0[0] * hi_L_del
                   + st->hi_ild_b1[0] * st->hi_ild_xp[0]
                   - st->hi_ild_a1[0] * st->hi_ild_yp[0];
        st->hi_ild_xp[0] = hi_L_del; st->hi_ild_yp[0] = hi_L;

        float hi_R = st->hi_ild_b0[1] * hi_R_del
                   + st->hi_ild_b1[1] * st->hi_ild_xp[1]
                   - st->hi_ild_a1[1] * st->hi_ild_yp[1];
        st->hi_ild_xp[1] = hi_R_del; st->hi_ild_yp[1] = hi_R;

        /* Pinna elevation notch on high band (2nd-order biquad, Direct Form I) */
        float pn_L = st->pn_b0[0] * hi_L + st->pn_b1[0] * st->pn_x1[0]
                   + st->pn_b2[0] * st->pn_x2[0]
                   - st->pn_a1[0] * st->pn_y1[0] - st->pn_a2[0] * st->pn_y2[0];
        st->pn_x2[0] = st->pn_x1[0]; st->pn_x1[0] = hi_L;
        st->pn_y2[0] = st->pn_y1[0]; st->pn_y1[0] = pn_L;

        float pn_R = st->pn_b0[1] * hi_R + st->pn_b1[1] * st->pn_x1[1]
                   + st->pn_b2[1] * st->pn_x2[1]
                   - st->pn_a1[1] * st->pn_y1[1] - st->pn_a2[1] * st->pn_y2[1];
        st->pn_x2[1] = st->pn_x1[1]; st->pn_x1[1] = hi_R;
        st->pn_y2[1] = st->pn_y1[1]; st->pn_y1[1] = pn_R;

        /* PRISM-SPREAD-DIST: blend directional HRTF with omnidirectional dry,
         * then apply 1/r distance attenuation and object gain. */
        float dry = x * 0.7071f;  /* 1/√2: equal power on both ears */
        float wet_L = lo_L + pn_L;
        float wet_R = lo_R + pn_R;
        out_l[n] += g_total * ((1.0f - sp) * wet_L + sp * dry);
        out_r[n] += g_total * ((1.0f - sp) * wet_R + sp * dry);
    }

    /* Write back state that was cached locally */
    st->cx_xp  = cx_xp;
    st->cx_yp  = cx_yp;
    st->dly_pos = pos;
}
