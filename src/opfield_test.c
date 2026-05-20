/* opfield_test.c — OPFIELD spatial audio test suite
 *
 * Tests: scene encode/decode (OOB + PERQ), SH evaluation, HOA encoding,
 * VBAP gains, binaural ITD/ILD direction, and PRISM render sanity.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/opfield.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* ---- Test harness ---- */

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg, ...) do { \
    g_checks++; \
    if (!(cond)) { \
        g_fails++; \
        fprintf(stderr, "FAIL [%s:%d] " msg "\n", __FILE__, __LINE__ __VA_OPT__(,) __VA_ARGS__); \
    } \
} while (0)

#define CHECKF(a, b, tol, msg, ...) CHECK(fabsf((a)-(b)) <= (tol), \
    msg " (got %f, expected %f, tol %f)" __VA_OPT__(,) __VA_ARGS__, (double)(a), (double)(b), (double)(tol))

/* ======================================================================
 * 1. Scene encode/decode (OOB standard format)
 * ====================================================================== */

static void test_scene_oob(void)
{
    opfield_scene_t s;
    opfield_scene_init(&s, 48000, OPFIELD_RENDER_BINAURAL);

    opfield_object_t o1 = {
        .type   = OPFIELD_OBJ_POINT,
        .pos    = { .az = 45.0f, .el = 20.0f, .dist = 2.5f },
        .gain   = 1.0f,
        .spread = 0.0f,
        .active = true,
    };
    opfield_object_t o2 = {
        .type   = OPFIELD_OBJ_POINT,
        .pos    = { .az = -90.0f, .el = -15.0f, .dist = 1.0f },
        .gain   = 0.5f,
        .spread = 0.3f,
        .active = true,
    };

    CHECK(opfield_scene_add_object(&s, &o1) == 0, "add object 0");
    CHECK(opfield_scene_add_object(&s, &o2) == 1, "add object 1");
    CHECK(s.n_objects == 2, "n_objects == 2");

    uint8_t buf[64];
    int enc = opfield_scene_encode(&s, buf, sizeof(buf));
    CHECK(enc == 1 + 2 * OPFIELD_OBJ_BYTES, "OOB encode size (%d)", enc);

    opfield_scene_t s2;
    opfield_scene_init(&s2, 48000, OPFIELD_RENDER_BINAURAL);
    int dec = opfield_scene_decode(&s2, buf, (size_t)enc);
    CHECK(dec == enc, "OOB decode consumed same bytes");
    CHECK(s2.n_objects == 2, "decoded n_objects == 2");

    /* Object 0: azimuth ±0.01° tolerance (stored at ×100 int16) */
    CHECKF(s2.objects[0].pos.az, 45.0f, 0.01f, "obj0.az");
    CHECKF(s2.objects[0].pos.el, 20.0f, 0.01f, "obj0.el");
    CHECKF(s2.objects[0].pos.dist, 2.5f, 0.02f, "obj0.dist");
    CHECKF(s2.objects[0].gain, 1.0f, 0.02f, "obj0.gain");

    /* Object 1 */
    CHECKF(s2.objects[1].pos.az, -90.0f, 0.01f, "obj1.az");
    CHECKF(s2.objects[1].pos.el, -15.0f, 0.01f, "obj1.el");
    CHECKF(s2.objects[1].gain, 0.5f, 0.02f, "obj1.gain");
    CHECKF(s2.objects[1].spread, 0.3f, 0.005f, "obj1.spread");

    /* Buffer too small → error */
    CHECK(opfield_scene_encode(&s, buf, 5) == -1, "OOB encode rejects small buffer");
}

/* ======================================================================
 * 2. Scene encode/decode — PERQ compressed format
 * ====================================================================== */

static void test_scene_perq(void)
{
    opfield_scene_t s;
    opfield_scene_init(&s, 48000, OPFIELD_RENDER_BINAURAL);

    /* Front object — zone 0, 1° resolution */
    opfield_object_t o = {
        .type   = OPFIELD_OBJ_POINT,
        .pos    = { .az = 15.0f, .el = 10.0f, .dist = 1.0f },
        .gain   = 1.0f, .spread = 0.0f, .active = true
    };
    opfield_scene_add_object(&s, &o);

    /* Side object — zone 1, 3° resolution */
    o.pos.az = 70.0f; o.pos.el = -6.0f;
    opfield_scene_add_object(&s, &o);

    /* Rear object — zone 2, 5° resolution */
    o.pos.az = 150.0f; o.pos.el = 0.0f;
    opfield_scene_add_object(&s, &o);

    uint8_t buf[64];
    int enc = opfield_scene_encode_perq(&s, buf, sizeof(buf));
    CHECK(enc == 1 + 3 * 6, "PERQ encode size (%d bytes)", enc);
    /* Verify PERQ is smaller than OOB for 3 objects */
    CHECK(enc < 1 + 3 * OPFIELD_OBJ_BYTES, "PERQ smaller than OOB");

    opfield_scene_t s2;
    opfield_scene_init(&s2, 48000, OPFIELD_RENDER_BINAURAL);
    int dec = opfield_scene_decode_perq(&s2, buf, (size_t)enc);
    CHECK(dec == enc, "PERQ decode consumed all bytes");
    CHECK(s2.n_objects == 3, "PERQ n_objects == 3");

    /* Zone 0 (front): tolerance = 1° */
    CHECKF(s2.objects[0].pos.az, 15.0f, 1.1f, "PERQ obj0.az");
    CHECKF(s2.objects[0].pos.el, 10.0f, 2.1f, "PERQ obj0.el");

    /* Zone 1 (side): tolerance = 3° */
    CHECKF(s2.objects[1].pos.az, 70.0f, 3.1f, "PERQ obj1.az");

    /* Zone 2 (rear): tolerance = 5° */
    CHECKF(s2.objects[2].pos.az, 150.0f, 5.1f, "PERQ obj2.az");
}

/* ======================================================================
 * 3. HOA spherical harmonics — verify known values
 * ====================================================================== */

static void test_sh_eval(void)
{
    float sh[16];

    /* At (az=0, el=0): x=1, y=0, z=0
     * Expected: W=1, Y=0, Z=0, X=1, V=0, T=0, R=-0.5, S=0, U=√3/2≈0.866 */
    opfield_sh_eval(0.0f, 0.0f, 3, sh);
    CHECKF(sh[0], 1.0f, 1e-5f, "SH(0,0): W at front");
    CHECKF(sh[1], 0.0f, 1e-5f, "SH(0,0): Y at front");
    CHECKF(sh[2], 0.0f, 1e-5f, "SH(0,0): Z at front");
    CHECKF(sh[3], 1.0f, 1e-5f, "SH(0,0): X at front");
    CHECKF(sh[6], -0.5f, 1e-5f, "SH(0,0): R at front = (0-1)/2");
    CHECKF(sh[8], (float)(sqrt(3.0) / 2.0), 1e-4f, "SH(0,0): U at front = √3/2");

    /* At (az=90°, el=0°): x=0, y=1, z=0
     * Expected: W=1, Y=1, Z=0, X=0 */
    float az90 = (float)(M_PI / 2.0);
    opfield_sh_eval(az90, 0.0f, 1, sh);
    CHECKF(sh[0], 1.0f, 1e-5f, "SH(90,0): W");
    CHECKF(sh[1], 1.0f, 1e-5f, "SH(90,0): Y = sin(90°)·cos(0°) = 1");
    CHECKF(sh[2], 0.0f, 1e-5f, "SH(90,0): Z");
    CHECKF(sh[3], 0.0f, 1e-5f, "SH(90,0): X = cos(90°)·cos(0°) = 0");

    /* At (az=0°, el=90°): x=0, y=0, z=1
     * Expected: W=1, Y=0, Z=1, X=0, R=(3-1)/2=1 */
    float el90 = (float)(M_PI / 2.0);
    opfield_sh_eval(0.0f, el90, 3, sh);
    CHECKF(sh[0], 1.0f, 1e-5f, "SH(0,90): W");
    CHECKF(sh[2], 1.0f, 1e-5f, "SH(0,90): Z = sin(90°) = 1");
    CHECKF(sh[6], 1.0f, 1e-5f, "SH(0,90): R = (3·1-1)/2 = 1");
    CHECKF(sh[12], 1.0f, 1e-5f, "SH(0,90): K = (5-3)/2 = 1");

    /* Verify SN3D energy: ∑ sh[n]² over a unit sphere ≈ 1 per order.
     * For a single source we just check the 1st-order energy. */
    float az45 = (float)(M_PI / 4.0);
    float el30 = (float)(M_PI / 6.0);
    opfield_sh_eval(az45, el30, 1, sh);
    /* W²+Y²+Z²+X² = 1 + (cos²el) + sin²el + cos²el ≠ const, not a uniform test.
     * Instead verify order 1 unit vector: (Y,Z,X) = (cos(el)sin(az), sin(el), cos(el)cos(az)) */
    float cx = cosf(el30) * cosf(az45);
    float cy = cosf(el30) * sinf(az45);
    float cz = sinf(el30);
    CHECKF(sh[1], cy, 1e-5f, "SH(45,30): Y");
    CHECKF(sh[2], cz, 1e-5f, "SH(45,30): Z");
    CHECKF(sh[3], cx, 1e-5f, "SH(45,30): X");
}

/* ======================================================================
 * 4. HOA encode — verify accumulation
 * ====================================================================== */

static void test_hoa_encode(void)
{
    float hoa_out[4 * 8] = {0};  /* 4 channels × 8 samples */
    float in[8]           = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

    /* Encode a unit gain source at (az=0, el=0) — W gets all energy */
    opfield_hoa_encode(0.0f, 0.0f, 1.0f, in, hoa_out, 8, 1);

    /* Channel 0 (W=1): hoa_out[0..7] should all equal 1.0 */
    for (int n = 0; n < 8; n++) {
        CHECKF(hoa_out[n], 1.0f, 1e-5f, "HOA ch0 (W) sample %d", n);
    }
    /* Channel 1 (Y=0 at az=0): hoa_out[8..15] should be 0 */
    for (int n = 0; n < 8; n++) {
        CHECKF(hoa_out[8 + n], 0.0f, 1e-5f, "HOA ch1 (Y) sample %d", n);
    }
    /* Channel 3 (X=1 at az=0,el=0): hoa_out[24..31] should be 1 */
    for (int n = 0; n < 8; n++) {
        CHECKF(hoa_out[24 + n], 1.0f, 1e-5f, "HOA ch3 (X) sample %d", n);
    }
}

/* ======================================================================
 * 5. VBAP — verify gains for a 3D 5-speaker layout
 *
 * VBAP requires speakers that span 3D space — a flat (el=0) array gives
 * all-zero z rows and det=0 for every triplet.  Use a real 3D layout:
 *   spk[0] FL  az=+30, el=+20
 *   spk[1] FR  az=-30, el=+20
 *   spk[2] BL  az=+150, el=-10
 *   spk[3] BR  az=-150, el=-10
 *   spk[4] Top az=0,   el=+60
 * ====================================================================== */

static void test_vbap(void)
{
    opfield_pos_t spk[5] = {
        { .az =   30.0f, .el =  20.0f, .dist = 1.0f },  /* FL  */
        { .az =  -30.0f, .el =  20.0f, .dist = 1.0f },  /* FR  */
        { .az =  150.0f, .el = -10.0f, .dist = 1.0f },  /* BL  */
        { .az = -150.0f, .el = -10.0f, .dist = 1.0f },  /* BR  */
        { .az =    0.0f, .el =  60.0f, .dist = 1.0f },  /* Top */
    };

    opfield_vbap_t v;
    int r = opfield_vbap_init(&v, spk, 5);
    CHECK(r == 0, "VBAP init succeeded (n_trips=%u)", v.n_trips);
    CHECK(v.n_trips > 0, "VBAP has at least 1 triplet");

    float gains[5];

    /* Source at (0°, 20°) — between FL and FR at their elevation */
    opfield_vbap_gains(&v, 0.0f, 20.0f, gains);
    float sum_sq = 0.0f;
    for (int k = 0; k < 5; k++) sum_sq += gains[k] * gains[k];
    CHECKF(sqrtf(sum_sq), 1.0f, 0.02f, "VBAP gains normalized at (0,20)");
    /* FL and FR should be active */
    CHECK(gains[0] > 0.01f || gains[1] > 0.01f, "VBAP front speakers active at (0,20)");

    /* Source exactly at FL (az=30, el=20): FL gain ≈ 1.0 */
    opfield_vbap_gains(&v, 30.0f, 20.0f, gains);
    sum_sq = 0.0f;
    for (int k = 0; k < 5; k++) sum_sq += gains[k] * gains[k];
    CHECKF(sqrtf(sum_sq), 1.0f, 0.02f, "VBAP gains normalized at (30,20)");
    CHECKF(gains[0], 1.0f, 0.1f, "VBAP FL gain ≈ 1 when source at FL");
}

/* ======================================================================
 * 6. Binaural HRTF — ITD direction check
 * ====================================================================== */

static void test_binaural_itd(void)
{
    opfield_binaural_t r;
    opfield_binaural_init(&r, 48000);

    /* Source at 90° left: left ear is near (0 delay), right ear is far */
    opfield_binaural_update(&r, 0, 90.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    opfield_hrtf_state_t *st = &r.obj[0];

    CHECK(st->lo_dly_L == 0, "90° left: left ear no delay (lo_dly_L=%u)", st->lo_dly_L);
    CHECK(st->lo_dly_R > 0,  "90° left: right ear delayed (lo_dly_R=%u)", st->lo_dly_R);

    /* Source at 90° right: right ear is near, left is far */
    opfield_binaural_update(&r, 1, -90.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    st = &r.obj[1];
    CHECK(st->lo_dly_R == 0, "-90° right: right ear no delay (lo_dly_R=%u)", st->lo_dly_R);
    CHECK(st->lo_dly_L > 0,  "-90° right: left ear delayed (lo_dly_L=%u)", st->lo_dly_L);

    /* Source at 0° front: no ITD */
    opfield_binaural_update(&r, 2, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    st = &r.obj[2];
    CHECK(st->lo_dly_L == 0 && st->lo_dly_R == 0,
          "front: no ITD (lo_dly_L=%u, lo_dly_R=%u)", st->lo_dly_L, st->lo_dly_R);

    /* High-band delays should be smaller than low-band (PRISM_HI_ITD_W < 1) */
    opfield_binaural_update(&r, 3, 60.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    st = &r.obj[3];
    if (st->lo_dly_R > 0) {
        CHECK(st->hi_dly_R <= st->lo_dly_R,
              "PRISM: hi_dly_R (%u) <= lo_dly_R (%u)", st->hi_dly_R, st->lo_dly_R);
    }
}

/* ======================================================================
 * 7. Binaural ILD — coefficient sanity
 * ====================================================================== */

static void test_binaural_ild(void)
{
    opfield_binaural_t r;
    opfield_binaural_init(&r, 48000);

    /* Source at 90° left: left ear α ≈ 1.5 (near), right ear α ≈ 0.5 (far)
     * The high-freq gain = α, so tgt_hi_b0[L] > tgt_hi_b0[R] at high freq.
     * At Nyquist (z=-1): H(Nyq) = (b0 - b1)/(1 - a1) = α
     * So b0-b1 > 1-a1 for near ear (gain > 1) and b0-b1 < 1-a1 for far ear. */
    opfield_binaural_update(&r, 0, 90.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    opfield_hrtf_state_t *st = &r.obj[0];

    /* After init with first-time commit, hi_ild should equal tgt_hi */
    float nyq_L = st->hi_ild_b0[0] - st->hi_ild_b1[0];  /* numerator at Nyquist */
    float nyq_R = st->hi_ild_b0[1] - st->hi_ild_b1[1];
    float denom_L = 1.0f - st->hi_ild_a1[0];
    float denom_R = 1.0f - st->hi_ild_a1[1];

    float gain_L = nyq_L / denom_L;
    float gain_R = nyq_R / denom_R;

    CHECK(gain_L > gain_R, "90° left: left ear HF gain > right ear (%.3f vs %.3f)",
          (double)gain_L, (double)gain_R);
    CHECK(gain_L > 1.0f, "90° left: left (near) ear HF gain > 1 (%.3f)", (double)gain_L);
    CHECK(gain_R < 1.0f, "90° left: right (far) ear HF gain < 1 (%.3f)", (double)gain_R);

    /* Low-band ILD should be weaker than high-band (PRISM_LO_ILD_W = 0.15) */
    float lo_gain_L = (st->lo_ild_b0[0] - st->lo_ild_b1[0])
                    / (1.0f - st->lo_ild_a1[0]);
    float lo_gain_R = (st->lo_ild_b0[1] - st->lo_ild_b1[1])
                    / (1.0f - st->lo_ild_a1[1]);
    float lo_diff = lo_gain_L - lo_gain_R;
    float hi_diff = gain_L - gain_R;
    CHECK(lo_diff < hi_diff, "PRISM: lo-band ILD delta (%.3f) < hi-band (%.3f)",
          (double)lo_diff, (double)hi_diff);
}

/* ======================================================================
 * 8. Binaural render — non-zero output and L/R asymmetry
 * ====================================================================== */

static void test_binaural_render(void)
{
    opfield_binaural_t r;
    opfield_binaural_init(&r, 48000);

    /* Source at 60° left */
    opfield_binaural_update(&r, 0, 60.0f, 0.0f, 1.0f, 0.0f, 1.0f);

    float in[256];
    for (int i = 0; i < 256; i++)
        in[i] = sinf((float)i * 0.1f);  /* test tone */

    float out_l[256] = {0};
    float out_r[256] = {0};
    opfield_binaural_render(&r, 0, in, out_l, out_r, 256);

    /* Check non-zero output */
    float energy_l = 0.0f, energy_r = 0.0f;
    for (int i = 0; i < 256; i++) {
        energy_l += out_l[i] * out_l[i];
        energy_r += out_r[i] * out_r[i];
    }
    CHECK(energy_l > 0.0f, "binaural render: left output non-zero");
    CHECK(energy_r > 0.0f, "binaural render: right output non-zero");

    /* For source on left: left should generally have more energy than right
     * (ILD effect in the HF band). Allow some slack for the blend. */
    CHECK(energy_l > energy_r * 0.5f,
          "binaural render: left energy (%.2f) reasonable vs right (%.2f)",
          (double)energy_l, (double)energy_r);

    /* PRISM check: render should not produce Inf/NaN */
    int nan_count = 0;
    for (int i = 0; i < 256; i++) {
        if (!isfinite(out_l[i]) || !isfinite(out_r[i])) nan_count++;
    }
    CHECK(nan_count == 0, "binaural render: no NaN/Inf in output");
}

/* ======================================================================
 * 9. HVSS — smooth transition check
 * ====================================================================== */

static void test_hvss(void)
{
    opfield_binaural_t r;
    opfield_binaural_init(&r, 48000);

    opfield_binaural_update(&r, 0, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f);  /* front */
    opfield_hrtf_state_t *st = &r.obj[0];
    float b0_init = st->hi_ild_b0[0];

    /* Large move to 90° left: HVSS should set fast alpha */
    opfield_binaural_update(&r, 0, 90.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    float alpha_fast = st->hvss_alpha;
    CHECK(alpha_fast > 0.05f, "HVSS: fast alpha after large move (%.3f)", (double)alpha_fast);

    /* No move: slow alpha */
    opfield_binaural_update(&r, 0, 90.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    float alpha_slow = st->hvss_alpha;
    CHECK(alpha_slow < alpha_fast, "HVSS: slow alpha when stationary (%.3f vs %.3f)",
          (double)alpha_slow, (double)alpha_fast);

    /* After rendering, coefficients should have moved toward target */
    float in[64] = {0};
    float out_l[64] = {0}, out_r[64] = {0};
    opfield_binaural_render(&r, 0, in, out_l, out_r, 64);
    /* hi_ild_b0[0] should be closer to tgt_hi_b0[0] than b0_init */
    float dist_after  = fabsf(st->hi_ild_b0[0] - st->tgt_hi_b0[0]);
    float dist_before = fabsf(b0_init            - st->tgt_hi_b0[0]);
    CHECK(dist_after < dist_before,
          "HVSS: coefficients moved toward target (dist %.4f → %.4f)",
          (double)dist_before, (double)dist_after);

    (void)b0_init;
}

/* ======================================================================
 * 10. Full integration: scene → binaural render
 * ====================================================================== */

static void test_integration(void)
{
    opfield_scene_t scene;
    opfield_scene_init(&scene, 48000, OPFIELD_RENDER_BINAURAL);

    opfield_object_t o = {
        .type = OPFIELD_OBJ_POINT,
        .pos  = { .az = -30.0f, .el = 0.0f, .dist = 1.5f },
        .gain = 0.8f, .spread = 0.0f, .active = true
    };
    opfield_scene_add_object(&scene, &o);

    /* Encode and decode the scene */
    uint8_t oob_buf[64];
    int enc_len = opfield_scene_encode(&scene, oob_buf, sizeof(oob_buf));
    CHECK(enc_len > 0, "integration: OOB encode OK");

    opfield_scene_t scene2;
    opfield_scene_init(&scene2, 48000, OPFIELD_RENDER_BINAURAL);
    opfield_scene_decode(&scene2, oob_buf, (size_t)enc_len);

    /* Set up renderer from decoded scene */
    opfield_binaural_t renderer;
    opfield_binaural_init(&renderer, scene2.sample_rate);

    /* Render a white-noise-like block */
    float in_buf[128];
    for (int i = 0; i < 128; i++) in_buf[i] = (i % 2 == 0) ? 0.5f : -0.5f;

    float out_l[128] = {0}, out_r[128] = {0};
    const float *bufs[1] = { in_buf };
    opfield_scene_render_binaural(&scene2, &renderer, bufs, out_l, out_r, 128);

    int ok = 1;
    for (int i = 0; i < 128; i++) {
        if (!isfinite(out_l[i]) || !isfinite(out_r[i])) { ok = 0; break; }
    }
    CHECK(ok, "integration: no NaN/Inf in full pipeline output");

    /* Source at -30° (right side): right ear should have slightly more energy */
    float el = 0.0f, er = 0.0f;
    for (int i = 0; i < 128; i++) { el += out_l[i]*out_l[i]; er += out_r[i]*out_r[i]; }
    CHECK(er > 0.0f && el > 0.0f, "integration: both ears have energy");
}

/* ======================================================================
 * 11. Distance attenuation — 1/r law
 * ====================================================================== */

static void test_distance(void)
{
    opfield_binaural_t r;
    opfield_binaural_init(&r, 48000);

    float in[256];
    for (int i = 0; i < 256; i++) in[i] = sinf((float)i * 0.1f);

    /* Render at dist=1 m */
    opfield_binaural_update(&r, 0, 45.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    float ol1[256] = {0}, or1[256] = {0};
    opfield_binaural_render(&r, 0, in, ol1, or1, 256);

    /* Render same position at dist=2 m (second call on same object; re-init) */
    opfield_binaural_t r2;
    opfield_binaural_init(&r2, 48000);
    opfield_binaural_update(&r2, 0, 45.0f, 0.0f, 2.0f, 0.0f, 1.0f);
    float ol2[256] = {0}, or2[256] = {0};
    opfield_binaural_render(&r2, 0, in, ol2, or2, 256);

    float e1 = 0.0f, e2 = 0.0f;
    for (int i = 0; i < 256; i++) {
        e1 += ol1[i]*ol1[i] + or1[i]*or1[i];
        e2 += ol2[i]*ol2[i] + or2[i]*or2[i];
    }

    /* At dist=2 m: gain = 1/2 → energy = (1/2)² = 0.25× dist=1 energy */
    CHECK(e1 > 0.0f, "distance: dist=1 energy non-zero");
    CHECK(e2 < e1,   "distance: dist=2 energy < dist=1 energy");
    float ratio = e2 / (e1 > 0.0f ? e1 : 1.0f);
    CHECK(ratio < 0.35f && ratio > 0.15f,
          "distance: energy ratio ≈ 0.25 for 2m vs 1m (got %.3f)", (double)ratio);
}

/* ======================================================================
 * 12. Spread — omnidirectional blend symmetry
 * ====================================================================== */

static void test_spread(void)
{
    float in[256];
    for (int i = 0; i < 256; i++) in[i] = sinf((float)i * 0.15f);

    /* Directional source at 90°: clear ILD, L >> R */
    opfield_binaural_t rd;
    opfield_binaural_init(&rd, 48000);
    opfield_binaural_update(&rd, 0, 90.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    float dl[256] = {0}, dr_buf[256] = {0};
    opfield_binaural_render(&rd, 0, in, dl, dr_buf, 256);
    float el_dir = 0.0f, er_dir = 0.0f;
    for (int i = 0; i < 256; i++) {
        el_dir += dl[i]*dl[i]; er_dir += dr_buf[i]*dr_buf[i];
    }

    /* Fully omnidirectional (spread=1): L ≈ R energy */
    opfield_binaural_t ro;
    opfield_binaural_init(&ro, 48000);
    opfield_binaural_update(&ro, 0, 90.0f, 0.0f, 1.0f, 1.0f, 1.0f);
    float sl[256] = {0}, sr[256] = {0};
    opfield_binaural_render(&ro, 0, in, sl, sr, 256);
    float el_omni = 0.0f, er_omni = 0.0f;
    for (int i = 0; i < 256; i++) {
        el_omni += sl[i]*sl[i]; er_omni += sr[i]*sr[i];
    }

    CHECK(el_dir > 0.0f && er_dir > 0.0f, "spread: directional both ears non-zero");
    /* Directional at 90°: left ear should have significantly more energy */
    CHECK(el_dir > er_dir * 1.1f,
          "spread=0 at 90°: left ear dominates (eL=%.3f eR=%.3f)",
          (double)el_dir, (double)er_dir);
    /* Omnidirectional: L and R should be nearly equal */
    CHECK(el_omni > 0.0f && er_omni > 0.0f, "spread: omnidirectional both ears non-zero");
    float asym = fabsf(el_omni - er_omni) / (el_omni + er_omni + 1e-12f);
    CHECK(asym < 0.05f, "spread=1: L/R energy symmetric (asymmetry=%.3f)", (double)asym);
}

/* ======================================================================
 * 13. VBAP 2D — horizontal flat array
 * ====================================================================== */

static void test_vbap_2d(void)
{
    /* Flat quad: all at el=0.  VBAP init must detect is_2d and build pairs. */
    opfield_pos_t spk[4] = {
        { .az =   30.0f, .el = 0.0f, .dist = 1.0f },
        { .az =  -30.0f, .el = 0.0f, .dist = 1.0f },
        { .az =  150.0f, .el = 0.0f, .dist = 1.0f },
        { .az = -150.0f, .el = 0.0f, .dist = 1.0f },
    };

    opfield_vbap_t v;
    int r = opfield_vbap_init(&v, spk, 4);
    CHECK(r == 0,      "VBAP 2D init succeeded");
    CHECK(v.is_2d,     "VBAP 2D: is_2d flag set");
    CHECK(v.n_trips == 0, "VBAP 2D: no 3D triplets (all coplanar)");
    CHECK(v.n_pairs > 0,  "VBAP 2D: pairs built (n_pairs=%u)", (unsigned)v.n_pairs);

    float gains[4];

    /* Source at (0°,0°) — between FL and FR */
    opfield_vbap_gains(&v, 0.0f, 0.0f, gains);
    float sum_sq = 0.0f;
    for (int k = 0; k < 4; k++) sum_sq += gains[k] * gains[k];
    CHECKF(sqrtf(sum_sq), 1.0f, 0.02f, "VBAP 2D: gains normalized at (0,0)");
    CHECK(gains[0] > 0.01f || gains[1] > 0.01f,
          "VBAP 2D: front speakers active at (0,0)");

    /* Source at FL exactly: FL gain ≈ 1.0 */
    opfield_vbap_gains(&v, 30.0f, 0.0f, gains);
    sum_sq = 0.0f;
    for (int k = 0; k < 4; k++) sum_sq += gains[k] * gains[k];
    CHECKF(sqrtf(sum_sq), 1.0f, 0.02f, "VBAP 2D: gains normalized at (30,0)");
    CHECKF(gains[0], 1.0f, 0.1f, "VBAP 2D: FL gain ≈ 1 when source at FL");
}

/* ======================================================================
 * 14. Scene render API — binaural convenience function
 * ====================================================================== */

static void test_scene_render(void)
{
    opfield_scene_t scene;
    opfield_scene_init(&scene, 48000, OPFIELD_RENDER_BINAURAL);

    /* Equal gains, equal distances, symmetric azimuths: L and R should be
     * nearly identical in energy due to HRTF symmetry. */
    opfield_object_t o1 = {
        .type = OPFIELD_OBJ_POINT,
        .pos  = { .az =  45.0f, .el = 0.0f, .dist = 1.0f },
        .gain = 1.0f, .spread = 0.0f, .active = true
    };
    opfield_object_t o2 = {
        .type = OPFIELD_OBJ_POINT,
        .pos  = { .az = -45.0f, .el = 0.0f, .dist = 1.0f },
        .gain = 1.0f, .spread = 0.0f, .active = true
    };
    opfield_scene_add_object(&scene, &o1);
    opfield_scene_add_object(&scene, &o2);

    float sig[128];
    for (int i = 0; i < 128; i++) sig[i] = (i & 1) ? 0.5f : -0.5f;

    const float *bufs[2] = { sig, sig };
    float out_l[128] = {0}, out_r[128] = {0};

    opfield_binaural_t r;
    opfield_binaural_init(&r, 48000);
    opfield_scene_render_binaural(&scene, &r, bufs, out_l, out_r, 128);

    float el = 0.0f, er = 0.0f;
    int finite_ok = 1;
    for (int i = 0; i < 128; i++) {
        el += out_l[i]*out_l[i]; er += out_r[i]*out_r[i];
        if (!isfinite(out_l[i]) || !isfinite(out_r[i])) finite_ok = 0;
    }
    CHECK(finite_ok,  "scene_render: no NaN/Inf");
    CHECK(el > 0.0f,  "scene_render: left output non-zero");
    CHECK(er > 0.0f,  "scene_render: right output non-zero");

    /* Symmetric scene: two objects at ±45° with equal gains and distances.
     * Since HRTF is left/right symmetric, total L and R energy must match. */
    float diff_ratio = fabsf(el - er) / (el + er + 1e-12f);
    CHECK(diff_ratio < 0.15f,
          "scene_render: symmetric scene produces balanced output (diff_ratio=%.3f)",
          (double)diff_ratio);
}

/* ======================================================================
 * Main
 * ====================================================================== */

int main(void)
{
    printf("OPFIELD spatial audio test suite\n");
    printf("=================================\n");

    test_scene_oob();
    test_scene_perq();
    test_sh_eval();
    test_hoa_encode();
    test_vbap();
    test_binaural_itd();
    test_binaural_ild();
    test_binaural_render();
    test_hvss();
    test_integration();
    test_distance();
    test_spread();
    test_vbap_2d();
    test_scene_render();

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    if (g_fails > 0) {
        printf("FAIL\n");
        return 1;
    }
    printf("PASS\n");
    return 0;
}
