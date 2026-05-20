/*
 * opcodec/aec2.h — Acoustic Environment Codec v2 (AEC2)
 *
 * Transmits the acoustic environment of a speaker as a compact Room Impulse
 * Response (RIR) descriptor. The receiver applies the RIR as a convolution
 * reverb on the synthesized audio, giving the listener a perceptual sense of
 * the remote room — office, hallway, bathroom, outdoors, etc.
 *
 * Codec design:
 *   1. RIR estimation: frequency-domain Wiener deconvolution of recorded vs
 *      clean speech produces a 512-tap FIR (32ms at 16 kHz).
 *   2. Compression: the 512 complex STFT coefficients are grouped into
 *      AEC2_BANDS frequency bands and each band's energy is encoded as a
 *      6-bit log value. Early reflections (first 10 taps at 16 kHz = 0.625ms)
 *      are coded as delta values (4 bits each). Total: ≤ 256 bytes/update.
 *   3. Update rate: every 2 seconds (the room changes slowly). If the room
 *      is silent or the delta between consecutive frames is < AEC2_UPDATE_THRESH
 *      no packet is sent.
 *   4. Receiver: apply the decoded RIR as a circular-buffer FIR convolution
 *      (O(N) per sample, N=512). Can be disabled/faded out when not needed.
 *
 * Bitstream layout (≤ 256 bytes):
 *   byte 0:     version = 2
 *   byte 1:     n_bands (actual number of coded bands, ≤ AEC2_MAX_BANDS)
 *   byte 2:     n_early (number of coded early reflection taps, ≤ 16)
 *   byte 3:     flags: bit0=wet_gain_present, bit1=dry_gain_present
 *   bytes 4-N:  band energies: ceil(n_bands * 6 / 8) bytes (packed 6-bit codes)
 *   bytes N+1..: early reflection deltas: ceil(n_early * 4 / 8) bytes
 *   [if wet_gain_present]: 1 byte (0-255 maps 0-1.0)
 *   [if dry_gain_present]: 1 byte (0-255 maps 0-1.0)
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_AEC2_H
#define OPCODEC_AEC2_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define AEC2_RIR_TAPS        512    /* FIR filter length (32ms at 16 kHz) */
#define AEC2_MAX_BANDS        32    /* frequency band partitions */
#define AEC2_MAX_EARLY        16    /* early reflection taps transmitted */
#define AEC2_PACKET_MAX      256    /* max bytes per AEC2 update packet */
#define AEC2_UPDATE_INTERVAL_MS 2000 /* send at most one update per 2 seconds */
#define AEC2_UPDATE_THRESH   0.05f  /* minimum RIR change to trigger send */
#define AEC2_WET_DEFAULT     0.25f  /* default reverb wet mix (25%) */
#define AEC2_DRY_DEFAULT     0.80f  /* default dry mix */

/* Encoded AEC2 environment descriptor */
typedef struct {
    uint8_t n_bands;        /* number of valid band entries */
    uint8_t n_early;        /* number of early reflection taps */
    uint8_t band_energy[AEC2_MAX_BANDS];  /* 6-bit log-energy codes (stored as uint8_t) */
    int8_t  early_tap[AEC2_MAX_EARLY];   /* delta-coded early taps (4-bit signed, stored as int8_t) */
    float   wet_gain;       /* reverb wet mix 0.0–1.0 */
    float   dry_gain;       /* dry mix 0.0–1.0 */
} aec2_desc_t;

/* AEC2 encoder context */
typedef struct {
    float    rir[AEC2_RIR_TAPS];       /* current room impulse response */
    float    prev_rir[AEC2_RIR_TAPS];  /* previous for delta comparison */
    float    wet_gain;
    float    dry_gain;
    uint32_t last_update_ms;
    bool     has_rir;
} aec2_enc_t;

/* AEC2 decoder / convolution context */
typedef struct {
    float    rir[AEC2_RIR_TAPS];     /* decoded room impulse response */
    float    conv_buf[AEC2_RIR_TAPS]; /* circular convolution buffer */
    int      conv_head;              /* circular buffer write position */
    float    wet_gain;
    float    dry_gain;
    bool     active;                 /* whether convolution is applied */
} aec2_dec_t;

/* ── Encoder API ── */

void aec2_enc_init(aec2_enc_t *ctx);

/*
 * Feed a new estimated RIR (AEC2_RIR_TAPS float samples).
 * Call from a higher-level acoustic echo canceller or room estimator.
 */
void aec2_enc_set_rir(aec2_enc_t *ctx, const float *rir, float wet, float dry);

/*
 * Produce an AEC2 packet into 'out' (max AEC2_PACKET_MAX bytes).
 * Returns bytes written, or 0 if no update needed (no change / too soon).
 * now_ms: current time for rate-limiting.
 */
int aec2_encode(aec2_enc_t *ctx, uint8_t *out, int out_cap, uint32_t now_ms);

/* ── Decoder API ── */

void aec2_dec_init(aec2_dec_t *ctx);

/*
 * Decode an AEC2 packet received from the remote peer.
 * Returns 0 on success, -1 on error.
 */
int aec2_decode(aec2_dec_t *ctx, const uint8_t *in, int in_len);

/*
 * Apply AEC2 room convolution to one frame of PCM samples (in-place).
 * Mixes wet (reverbed) and dry (direct) signal according to gain settings.
 * sample_rate: used to verify tap count; if mismatched, convolution is skipped.
 */
void aec2_apply(aec2_dec_t *ctx, float *pcm, int n_samples);

/*
 * Disable AEC2 convolution (fade to dry immediately).
 */
static inline void aec2_dec_disable(aec2_dec_t *ctx) { ctx->active = false; }
static inline void aec2_dec_enable(aec2_dec_t *ctx)  { ctx->active = true;  }

#endif /* OPCODEC_AEC2_H */
