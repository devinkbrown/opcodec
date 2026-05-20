/*
 * opcodec/dtx.h — Discontinuous Transmission (DTX) + Comfort Noise Generation (CNG)
 *
 * During silence or background-noise-only periods, DTX suppresses audio packet
 * transmission entirely. The receiver generates perceptually matched comfort
 * noise (CNG) to avoid the unnatural "dead air" effect that occurs when all
 * audio is simply muted.
 *
 * Protocol design:
 *   1. Encoder-side VAD (integrated with vmd.c): when the VAD classifies N
 *      consecutive frames as silence/noise-only, DTX activates.
 *   2. On DTX activation, a SID (Silence Indicator) packet is sent:
 *      - 2 bytes: spectral noise descriptor (8 Mel-band log-energy codes,
 *        4 bits each = 32 bits = 4 bytes, or 8 bands × 4 bits).
 *      - 1 byte: spectral tilt factor (pre-emphasis coefficient, 0.0–1.0).
 *      - 1 byte: noise level in dBFS + flags.
 *      Total: DTX_SID_BYTES = 4 bytes.
 *   3. SID packets are repeated every DTX_SID_INTERVAL_MS during sustained
 *      silence to allow the receiver to track slowly changing background noise.
 *   4. On voice onset, DTX deactivates immediately and normal audio resumes.
 *
 * Receiver CNG:
 *   - Generates shaped white noise matching the transmitted spectrum.
 *   - White noise → 8-band filter bank → per-band amplitude from SID → IFFT.
 *   - Gain ramps in/out (16ms fade) to avoid clicks on DTX transitions.
 *
 * Bitrate savings: during silence (~40–60% of a typical call), DTX reduces
 * audio bitrate from ~6–128 kbps to 8 bps × (1000/DTX_SID_INTERVAL_MS).
 * At SID every 200ms: 4 bytes / 0.2s = 160 bps, effectively ~97% reduction.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_DTX_H
#define OPCODEC_DTX_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define DTX_SID_BYTES            4    /* bytes per SID packet */
#define DTX_SID_BANDS            8    /* spectral bands in SID */
#define DTX_HANGOVER_FRAMES      5    /* frames of audio after VAD drops (avoids clipping) */
#define DTX_SID_INTERVAL_MS    200    /* resend SID every 200ms during DTX */
#define DTX_FADE_SAMPLES        256   /* CNG fade in/out ramp length */

/* SID packet structure (4 bytes) */
typedef struct {
    uint8_t band_energy[DTX_SID_BANDS / 2];  /* packed 4-bit log-energy codes, 4 bytes */
    uint8_t level_flags; /* bits 7-2: dBFS (6-bit, −63–0), bits 1-0: flags */
    uint8_t tilt;        /* spectral tilt: 0=flat, 255=max pre-emphasis */
} dtx_sid_t;

/* DTX encoder context */
typedef struct {
    float   noise_psd[DTX_SID_BANDS];  /* per-band noise power (smoothed) */
    float   spectral_tilt;             /* AR(1) noise model coefficient */
    float   noise_level_db;            /* overall noise level dBFS */

    int     vad_state;       /* 0=speech, 1=silence (after hangover) */
    int     hangover_count;  /* countdown from DTX_HANGOVER_FRAMES */
    int     silent_count;    /* frames of consecutive silence */
    uint32_t last_sid_ms;    /* timestamp of last SID packet */
    bool    dtx_active;
    bool    initialized;
} dtx_enc_t;

/* DTX decoder / CNG context */
typedef struct {
    float   noise_psd[DTX_SID_BANDS];  /* per-band target noise power */
    float   spectral_tilt;
    float   noise_level;               /* linear amplitude */

    float   fade_gain;     /* current fade gain [0,1] */
    float   fade_step;     /* per-sample gain step */
    bool    fade_in;       /* true = fading in, false = fading out */
    bool    cng_active;    /* whether to generate comfort noise */

    uint32_t rng;          /* noise generator state */
    bool     initialized;
} dtx_dec_t;

/* ── Encoder API ── */

void dtx_enc_init(dtx_enc_t *ctx);

/*
 * Feed VAD classification to DTX controller.
 * is_speech:  true = this frame contains speech, false = silence/noise
 * noise_psd:  per-band noise PSD estimate from ns2 (DTX_SID_BANDS floats), or NULL
 * now_ms:     current timestamp (for SID rate limiting)
 *
 * Returns true if this frame should be transmitted normally.
 * Returns false if DTX suppresses this frame (caller skips encoding).
 */
bool dtx_enc_update(dtx_enc_t *ctx, bool is_speech,
                    const float *noise_psd, float noise_db, uint32_t now_ms);

/*
 * Produce a SID packet (called when dtx_enc_update returns false and a SID
 * is due). Returns DTX_SID_BYTES, or 0 if no SID needed yet.
 */
int dtx_enc_build_sid(dtx_enc_t *ctx, uint8_t *out, uint32_t now_ms);

/*
 * Serialize a dtx_sid_t to DTX_SID_BYTES.
 */
void dtx_sid_write(const dtx_sid_t *sid, uint8_t *out);
void dtx_sid_read(dtx_sid_t *sid, const uint8_t *in);

/* ── Decoder / CNG API ── */

void dtx_dec_init(dtx_dec_t *ctx);

/*
 * Notify decoder that a SID packet was received (DTX silence period).
 * Updates CNG parameters and starts comfort noise generation.
 */
void dtx_dec_set_sid(dtx_dec_t *ctx, const uint8_t *sid_packet, int len);

/*
 * Notify decoder that normal audio has resumed (DTX deactivated).
 * Starts fade-out of CNG.
 */
void dtx_dec_voice_onset(dtx_dec_t *ctx);

/*
 * Generate comfort noise into output buffer.
 * Call this instead of audio decode when DTX is active (cng_active == true).
 * n_samples: number of samples to generate
 */
void dtx_dec_generate(dtx_dec_t *ctx, float *out, int n_samples);

static inline bool dtx_dec_is_active(const dtx_dec_t *ctx) {
    return ctx->cng_active || (ctx->fade_gain > 0.01f);
}

#endif /* OPCODEC_DTX_H */
