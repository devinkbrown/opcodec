/*
 * opcodec/epsc.c — Emotional Prosody Side Channel
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/epsc.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* MIDI note 0 = C-1 = 8.176 Hz; 12 semitones per octave */
static const float MIDI_BASE_HZ = 8.17579891564371f;

float epsc_f0_to_hz(uint8_t code)
{
    if (code == 0) return 0.0f;
    /* MIDI note: code maps to notes 33–127 shifted down so code 1 = note 33 */
    float note = (float)(code + 32);
    return MIDI_BASE_HZ * powf(2.0f, note / 12.0f);
}

uint8_t epsc_hz_to_f0(float f0_hz)
{
    if (f0_hz <= 0.0f || f0_hz > 3500.0f) return EPSC_F0_UNVOICED;
    float note = 12.0f * log2f(f0_hz / MIDI_BASE_HZ);
    int code = (int)(note - 32.0f + 0.5f);
    if (code < 1)   code = 1;
    if (code > 127) code = 127;
    return (uint8_t)code;
}

void epsc_init(epsc_ctx_t *ctx, uint32_t sample_rate)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->sample_rate = sample_rate;
    ctx->prev_energy = 0.0f;
    ctx->prev_voiced = false;
    ctx->prev_f0 = EPSC_F0_UNVOICED;
}

epsc_frame_t epsc_extract(epsc_ctx_t *ctx,
                           const float *pcm, int frame_size,
                           float pitch_hz)
{
    epsc_frame_t f = {0};

    /* ── F0 ── */
    f.f0_code = epsc_hz_to_f0(pitch_hz);
    f.voiced  = (f.f0_code != EPSC_F0_UNVOICED);

    /* ── Energy ── */
    float energy = 0.0f;
    for (int i = 0; i < frame_size; i++)
        energy += pcm[i] * pcm[i];
    energy = (frame_size > 0) ? energy / (float)frame_size : 0.0f;

    /* Convert RMS to dB SPL (0 dBFS → 94 dB SPL assumed) */
    float rms = sqrtf(energy + 1e-12f);
    float db  = 94.0f + 20.0f * log10f(rms + 1e-12f);
    if (db < 0.0f)  db = 0.0f;
    if (db > 93.0f) db = 93.0f;
    /* 5-bit code: 0–31 maps to 0–93 dB in 3 dB steps */
    f.energy_db = (uint8_t)((int)(db / 3.0f));
    if (f.energy_db > 31) f.energy_db = 31;

    /* ── Speaking rate ── */
    /* Detect voiced onset (unvoiced→voiced transition) */
    if (f.voiced && !ctx->prev_voiced) {
        ctx->onset_times[ctx->onset_head] = (float)frame_size / (float)ctx->sample_rate;
        ctx->onset_head = (ctx->onset_head + 1) % EPSC_HISTORY_FRAMES;
        if (ctx->onset_count < EPSC_HISTORY_FRAMES) ctx->onset_count++;
    }
    /* Estimate syllable rate from onset intervals */
    float rate = 3.0f;  /* neutral: ~3 syllables/sec */
    if (ctx->onset_count >= 2) {
        /* Average inter-onset interval over history */
        float sum = 0.0f;
        int n = (ctx->onset_count < 4) ? ctx->onset_count : 4;
        for (int i = 0; i < n; i++) {
            int idx = ((ctx->onset_head - 1 - i) + EPSC_HISTORY_FRAMES)
                      % EPSC_HISTORY_FRAMES;
            sum += ctx->onset_times[idx];
        }
        if (sum > 0.0f) rate = (float)n / sum;
    }
    /* Quantize: 0 ≤ 1 syl/s, 7 ≥ 8 syl/s */
    int rate_code = (int)((rate - 1.0f) * 7.0f / 7.0f + 0.5f);
    if (rate_code < 0) rate_code = 0;
    if (rate_code > 7) rate_code = 7;
    f.rate_code = (uint8_t)rate_code;

    /* Update previous-frame state */
    ctx->prev_f0     = f.f0_code;
    ctx->prev_energy = energy;
    ctx->prev_voiced = f.voiced;

    return f;
}

int epsc_encode(const epsc_frame_t *f, uint8_t *out)
{
    /*
     * Bit layout (16 bits, big-endian):
     *   bits 15-9 : f0_code (7 bits)
     *   bits  8-4 : energy_db (5 bits)
     *   bit   3   : voiced
     *   bits  2-0 : rate_code (3 bits)
     */
    uint16_t word = (uint16_t)(
        ((uint16_t)(f->f0_code  & 0x7Fu) << 9) |
        ((uint16_t)(f->energy_db & 0x1Fu) << 4) |
        ((uint16_t)(f->voiced   ? 1u : 0u) << 3) |
        ((uint16_t)(f->rate_code & 0x07u))
    );
    out[0] = (uint8_t)(word >> 8);
    out[1] = (uint8_t)(word & 0xFFu);
    return EPSC_PACKET_BYTES;
}

int epsc_decode(const uint8_t *in, epsc_frame_t *f)
{
    uint16_t word = ((uint16_t)in[0] << 8) | (uint16_t)in[1];
    f->f0_code   = (uint8_t)((word >> 9) & 0x7Fu);
    f->energy_db = (uint8_t)((word >> 4) & 0x1Fu);
    f->voiced    = (bool)  ((word >> 3) & 0x01u);
    f->rate_code = (uint8_t)(word        & 0x07u);
    return EPSC_PACKET_BYTES;
}

void epsc_apply(float *pcm, int frame_size,
                const epsc_frame_t *target, const epsc_frame_t *actual,
                uint32_t sample_rate)
{
    if (!pcm || frame_size <= 0 || !target) return;

    /* ── Energy normalization ── */
    if (actual) {
        float target_rms = powf(10.0f,
            ((float)target->energy_db * 3.0f - 94.0f) / 20.0f);
        float actual_rms = powf(10.0f,
            ((float)actual->energy_db * 3.0f - 94.0f) / 20.0f);
        float gain = (actual_rms > 1e-6f) ? (target_rms / actual_rms) : 1.0f;
        /* Soft-limit the gain to ±6 dB to avoid clipping artefacts */
        if (gain > 2.0f)  gain = 2.0f;
        if (gain < 0.5f)  gain = 0.5f;
        for (int i = 0; i < frame_size; i++) {
            float s = pcm[i] * gain;
            if (s >  1.0f) s =  1.0f;
            if (s < -1.0f) s = -1.0f;
            pcm[i] = s;
        }
    }

    /* ── Pitch resynthesis (lightweight WSOLA-style pitch shift) ── */
    if (target->voiced && actual && actual->voiced) {
        float f0_target = epsc_f0_to_hz(target->f0_code);
        float f0_actual = epsc_f0_to_hz(actual->f0_code);
        float ratio = (f0_actual > 0.0f) ? (f0_target / f0_actual) : 1.0f;

        /* Only adjust if ratio is outside ±1 semitone */
        if (ratio < 0.94f || ratio > 1.06f) {
            /* Resample via linear interpolation (basic pitch shift) */
            float tmp[4096];
            if (frame_size > 4096) frame_size = 4096;
            for (int i = 0; i < frame_size; i++) {
                float src_pos = (float)i * ratio;
                int   j0 = (int)src_pos;
                float frac = src_pos - (float)j0;
                if (j0 >= frame_size - 1) j0 = frame_size - 2;
                if (j0 < 0) j0 = 0;
                tmp[i] = pcm[j0] + frac * (pcm[j0 + 1] - pcm[j0]);
            }
            memcpy(pcm, tmp, (size_t)frame_size * sizeof(float));
        }
    }

    (void)sample_rate;  /* reserved for future PSOLA implementation */
}
