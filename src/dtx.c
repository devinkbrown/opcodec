/*
 * opcodec/dtx.c — DTX / CNG implementation
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/dtx.h"
#include <string.h>
#include <math.h>

/* LCG noise generator */
static inline float lcg_noise(uint32_t *s)
{
    *s = *s * 1664525u + 1013904223u;
    return ((float)(int32_t)(*s >> 1)) / (float)(1u << 30);  /* [-1, 1] */
}

/* ── Encoder ─────────────────────────────────────────────────────────── */

void dtx_enc_init(dtx_enc_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->noise_level_db = -60.0f;
    ctx->spectral_tilt  = 0.0f;
    ctx->initialized    = true;
}

bool dtx_enc_update(dtx_enc_t *ctx, bool is_speech,
                    const float *noise_psd, float noise_db, uint32_t now_ms)
{
    if (!ctx || !ctx->initialized) return true;

    /* Update noise model */
    if (noise_psd) {
        for (int b = 0; b < DTX_SID_BANDS; b++) {
            ctx->noise_psd[b] = 0.9f * ctx->noise_psd[b] + 0.1f * noise_psd[b];
        }
    }
    ctx->noise_level_db = noise_db;

    if (is_speech) {
        /* Speech frame — reset hangover, ensure DTX is not active */
        ctx->hangover_count = DTX_HANGOVER_FRAMES;
        ctx->silent_count   = 0;
        ctx->dtx_active     = false;
        ctx->vad_state      = 0;
        return true;  /* transmit */
    }

    /* Silence frame */
    ctx->silent_count++;

    if (ctx->hangover_count > 0) {
        ctx->hangover_count--;
        return true;  /* transmit during hangover */
    }

    /* Activate DTX */
    ctx->dtx_active = true;
    ctx->vad_state  = 1;
    (void)now_ms;
    return false;  /* suppress transmission */
}

int dtx_enc_build_sid(dtx_enc_t *ctx, uint8_t *out, uint32_t now_ms)
{
    if (!ctx || !out || !ctx->dtx_active) return 0;

    /* Rate limit SID to DTX_SID_INTERVAL_MS */
    if (ctx->last_sid_ms != 0 &&
        (now_ms - ctx->last_sid_ms) < (uint32_t)DTX_SID_INTERVAL_MS)
        return 0;

    /* Build SID: pack 8 band energies as 4-bit codes into 4 bytes */
    /* Each byte holds two 4-bit band energy codes */
    for (int b = 0; b < DTX_SID_BANDS / 2; b++) {
        float e0 = ctx->noise_psd[b * 2];
        float e1 = ctx->noise_psd[b * 2 + 1];
        /* Map to 4-bit log code (0=very quiet, 15=loud) */
        float db0 = (e0 > 1e-20f) ? 10.0f * log10f(e0) : -60.0f;
        float db1 = (e1 > 1e-20f) ? 10.0f * log10f(e1) : -60.0f;
        int c0 = (int)((db0 + 60.0f) * 15.0f / 60.0f + 0.5f);
        int c1 = (int)((db1 + 60.0f) * 15.0f / 60.0f + 0.5f);
        if (c0 < 0)  c0 = 0;
        if (c0 > 15) c0 = 15;
        if (c1 < 0)  c1 = 0;
        if (c1 > 15) c1 = 15;
        out[b] = (uint8_t)((c1 << 4) | c0);
    }

    /* Level + flags byte */
    int level_code = (int)(-ctx->noise_level_db);  /* positive dB below FS */
    if (level_code < 0)  level_code = 0;
    if (level_code > 63) level_code = 63;
    out[DTX_SID_BANDS / 2] = (uint8_t)((level_code << 2) | 0x01u);  /* flag bit */

    /* Spectral tilt */
    int tilt_code = (int)(ctx->spectral_tilt * 255.0f + 0.5f);
    if (tilt_code < 0)   tilt_code = 0;
    if (tilt_code > 255) tilt_code = 255;
    out[DTX_SID_BYTES - 1] = (uint8_t)tilt_code;

    ctx->last_sid_ms = now_ms;
    return DTX_SID_BYTES;
}

void dtx_sid_write(const dtx_sid_t *sid, uint8_t *out)
{
    if (!sid || !out) return;
    memcpy(out, sid->band_energy, DTX_SID_BANDS / 2);
    out[DTX_SID_BANDS / 2] = sid->level_flags;
    out[DTX_SID_BYTES - 1] = sid->tilt;
}

void dtx_sid_read(dtx_sid_t *sid, const uint8_t *in)
{
    if (!sid || !in) return;
    memcpy(sid->band_energy, in, DTX_SID_BANDS / 2);
    sid->level_flags = in[DTX_SID_BANDS / 2];
    sid->tilt        = in[DTX_SID_BYTES - 1];
}

/* ── Decoder / CNG ────────────────────────────────────────────────────── */

void dtx_dec_init(dtx_dec_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->noise_level = 0.001f;  /* -60 dBFS default */
    ctx->fade_gain   = 0.0f;
    ctx->fade_step   = 1.0f / (float)DTX_FADE_SAMPLES;
    ctx->rng         = 0xABCD1234u;
    ctx->initialized = true;
}

void dtx_dec_set_sid(dtx_dec_t *ctx, const uint8_t *sid_packet, int len)
{
    if (!ctx || !sid_packet || len < DTX_SID_BYTES) return;

    /* Unpack band energies */
    for (int b = 0; b < DTX_SID_BANDS / 2; b++) {
        uint8_t byte = sid_packet[b];
        int c0 = byte & 0x0F, c1 = byte >> 4;
        float db0 = (float)c0 * 60.0f / 15.0f - 60.0f;
        float db1 = (float)c1 * 60.0f / 15.0f - 60.0f;
        ctx->noise_psd[b * 2]     = powf(10.0f, db0 / 10.0f);
        ctx->noise_psd[b * 2 + 1] = powf(10.0f, db1 / 10.0f);
    }

    /* Level */
    int level_code = (sid_packet[DTX_SID_BANDS / 2] >> 2) & 0x3F;
    float db = -(float)level_code;
    ctx->noise_level = powf(10.0f, db / 20.0f);

    /* Spectral tilt */
    ctx->spectral_tilt = (float)sid_packet[DTX_SID_BYTES - 1] / 255.0f;

    /* Start fading in */
    ctx->cng_active = true;
    ctx->fade_in    = true;
    ctx->fade_step  = 1.0f / (float)DTX_FADE_SAMPLES;
}

void dtx_dec_voice_onset(dtx_dec_t *ctx)
{
    if (!ctx) return;
    ctx->cng_active = false;
    ctx->fade_in    = false;
    ctx->fade_step  = 1.0f / (float)DTX_FADE_SAMPLES;
}

void dtx_dec_generate(dtx_dec_t *ctx, float *out, int n_samples)
{
    if (!ctx || !ctx->initialized || !out || n_samples <= 0) return;

    /* If neither CNG active nor fading out, output silence */
    if (!ctx->cng_active && ctx->fade_gain <= 0.001f) {
        for (int i = 0; i < n_samples; i++) out[i] = 0.0f;
        return;
    }

    /* Spectral tilt filter state (AR(1) pre-emphasis shaping) */
    float tilt = ctx->spectral_tilt * 0.8f;  /* scale: 0=white, 0.8=pink-like */
    float prev = 0.0f;

    for (int i = 0; i < n_samples; i++) {
        /* White noise */
        float noise = lcg_noise(&ctx->rng) * ctx->noise_level;

        /* Apply spectral tilt (simple 1-pole filter: y[n] = x[n] - tilt*x[n-1]) */
        float shaped = noise - tilt * prev;
        prev = noise;

        /* Fade gain */
        if (ctx->fade_in) {
            if (ctx->fade_gain < 1.0f) ctx->fade_gain += ctx->fade_step;
            if (ctx->fade_gain > 1.0f) ctx->fade_gain = 1.0f;
        } else {
            if (ctx->fade_gain > 0.0f) ctx->fade_gain -= ctx->fade_step;
            if (ctx->fade_gain < 0.0f) ctx->fade_gain = 0.0f;
        }

        float v = shaped * ctx->fade_gain;
        if (v >  1.0f) v =  1.0f;
        if (v < -1.0f) v = -1.0f;
        out[i] = v;
    }
}
