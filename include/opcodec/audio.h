/*
 * opcodec/audio.h — Ophion custom audio codec (OPVOX)
 *
 * Sub-band MDCT audio codec with full frequency range support.
 * Covers narrowband voice through fullband music.
 *
 * Supported configurations:
 *   8000 Hz mono    — narrowband voice        (~6-16 kbps)
 *   16000 Hz mono   — wideband voice          (~12-32 kbps)
 *   32000 Hz mono   — super-wideband voice    (~24-64 kbps)
 *   48000 Hz mono   — fullband music          (~32-128 kbps)
 *   48000 Hz stereo — fullband stereo          (~64-512 kbps)
 *
 * Algorithm:
 *   1. Pre-emphasis + windowing
 *   2. MDCT (Modified Discrete Cosine Transform)
 *   3. Psychoacoustic-guided bit allocation across bands
 *   4. Scalar quantization of MDCT coefficients
 *   5. Rice/Golomb entropy coding
 *
 * Frame structure:
 *   [header:2] [band_alloc:N] [quantized_coeffs:variable]
 *
 * Header byte 0: flags
 *   bit 7:    stereo (1) / mono (0)
 *   bit 6-5:  sample rate (00=8k, 01=16k, 10=48k, 11=32k)
 *   bit 4-3:  quality (00=low, 01=normal, 10=high, 11=ultra)
 *   bit 2:    silence frame (comfort noise)
 *   bit 1:    short blocks for transients (1) / long blocks (0)
 *   bit 0:    joint stereo (1) / independent stereo (0)
 *
 * Header byte 1: frame length (encoded data bytes following header)
 *
 * Pure C, no platform dependencies, compiles to native + WASM.
 *
 * Copyright (c) 2026 Ophion Development Team.  GPL v2.
 */

#ifndef OPCODEC_AUDIO_H
#define OPCODEC_AUDIO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "opcodec/pitch.h"
#include "opcodec/bwe.h"
#include "opcodec/tns.h"
#include "opcodec/energy.h"
#include "opcodec/pns.h"
#include "opcodec/vmd.h"
#include "opcodec/psych.h"

/* Supported sample rates */
#define OPVOX_RATE_8K          8000
#define OPVOX_RATE_16K         16000
#define OPVOX_RATE_32K         32000
#define OPVOX_RATE_48K         48000

/* Frame duration: 20ms at all sample rates */
#define OPVOX_FRAME_MS         20
#define OPVOX_FRAME_8K         160      /* 20ms at 8kHz */
#define OPVOX_FRAME_16K        320      /* 20ms at 16kHz */
#define OPVOX_FRAME_32K        640      /* 20ms at 32kHz */
#define OPVOX_FRAME_48K        960      /* 20ms at 48kHz */
#define OPVOX_MAX_FRAME        960      /* largest frame */

/* MDCT uses 50% overlap — transform size = 2 * frame_samples */
#define OPVOX_MDCT_MAX         1920     /* 2 * 960 */

/* Short block mode for transients */
#define OPVOX_SHORT_BLOCKS     8        /* number of short blocks per frame */

/* Sub-bands for bit allocation */
#define OPVOX_BANDS_8K         8
#define OPVOX_BANDS_16K        16
#define OPVOX_BANDS_32K        24
#define OPVOX_BANDS_48K        32
#define OPVOX_MAX_BANDS        32

/* Maximum encoded frame size */
#define OPVOX_MAX_ENCODED      512

typedef enum {
	OPVOX_QUALITY_LOW    = 0,    /* voice-optimized, minimum bitrate */
	OPVOX_QUALITY_NORMAL = 1,    /* balanced voice/music */
	OPVOX_QUALITY_HIGH   = 2,    /* high quality */
	OPVOX_QUALITY_ULTRA  = 3,    /* maximum quality */
} opvox_quality_t;

typedef enum {
	OPVOX_WINDOW_SINE = 0,       /* sine window (default for low quality) */
	OPVOX_WINDOW_KBD  = 1,       /* Kaiser-Bessel derived window (better frequency selectivity) */
} opvox_window_t;

/* Header flags */
#define OPVOX_FLAG_STEREO      0x80
#define OPVOX_FLAG_RATE_MASK   0x60
#define OPVOX_FLAG_RATE_8K     0x00
#define OPVOX_FLAG_RATE_16K    0x20
#define OPVOX_FLAG_RATE_48K    0x40
#define OPVOX_FLAG_RATE_32K    0x60
#define OPVOX_FLAG_QUAL_MASK   0x18
#define OPVOX_FLAG_QUAL_SHIFT  3
#define OPVOX_FLAG_SILENCE     0x04
#define OPVOX_FLAG_SHORT_BLOCKS 0x02
#define OPVOX_FLAG_JOINT_STEREO 0x01

typedef struct opvox_encoder {
	uint32_t sample_rate;
	uint16_t frame_samples;
	uint16_t mdct_size;
	uint8_t  n_bands;
	uint8_t  channels;
	opvox_quality_t quality;
	opvox_window_t  window_type;

	/* MDCT overlap buffer (previous frame's second half) */
	float overlap[OPVOX_MAX_FRAME];
	float overlap_r[OPVOX_MAX_FRAME];   /* right channel */

	/* MDCT window (precomputed - type depends on window_type) */
	float window[OPVOX_MDCT_MAX];

	/* Pre-emphasis state */
	float pre_emph[2];   /* [left, right] */

	/* VAD */
	float energy_avg;
	float silence_threshold;

	/* Band energy tracking for bit allocation */
	float band_energy[OPVOX_MAX_BANDS];
	float band_energy_smooth[OPVOX_MAX_BANDS];

	/* Noise shaping feedback */
	float noise_feedback[OPVOX_MAX_FRAME];

	/* Transient detection state */
	float prev_quarter_energy[4];

	/* Pre-echo attack gain limiting — separate state per channel */
	float prev_frame_max_energy;
	float gain_limit;
	float prev_frame_max_energy_r;
	float gain_limit_r;

	/* Short block state for transients */
	bool prev_transient;
	float short_overlap[OPVOX_SHORT_BLOCKS][OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS];
	float short_overlap_r[OPVOX_SHORT_BLOCKS][OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS];

	/* Absolute hearing threshold per band */
	float hearing_threshold[OPVOX_MAX_BANDS];

	/* Bit budget per quality level (bits per frame) */
	uint16_t target_bits;

	/* Pitch detection and filtering */
	pitch_detector_t  pitch_det;
	pitch_filter_t    pitch_filt;    /* L channel pre-filter */
	pitch_filter_t    pitch_filt_r;  /* R channel pre-filter (stereo) */

	/* SNS state */
	sns_ctx_t         sns;

	/* BWE state */
	bwe_ctx_t         bwe;

	/* TNS state */
	tns_ctx_t         tns;

	/* Energy quantization state */
	energy_ctx_t      energy;
	energy_ctx_t      energy_r;   /* right channel (stereo only) */

	/* PNS: perceptual noise substitution */
	pns_ctx_t         pns;

	/* VMD: voice/music/content detection */
	vmd_ctx_t         vmd;

	/* Psychoacoustic masking model (Johnston spreading function) */
	psych_ctx_t       psych;
} opvox_encoder_t;

typedef struct opvox_decoder {
	uint32_t sample_rate;
	uint16_t frame_samples;
	uint16_t mdct_size;
	uint8_t  n_bands;
	uint8_t  channels;
	opvox_quality_t quality;
	opvox_window_t  window_type;

	/* MDCT overlap-add buffer */
	float overlap[OPVOX_MAX_FRAME];
	float overlap_r[OPVOX_MAX_FRAME];

	/* MDCT window (precomputed - type depends on window_type) */
	float window[OPVOX_MDCT_MAX];

	/* De-emphasis state */
	float de_emph[2];

	/* PLC state */
	float prev_mdct[OPVOX_MAX_FRAME];
	float prev_mdct_r[OPVOX_MAX_FRAME];
	uint8_t plc_count;

	/* Band energy storage for dequantization */
	float band_energy[OPVOX_MAX_BANDS];

	/* Absolute hearing threshold per band */
	float hearing_threshold[OPVOX_MAX_BANDS];

	/* Comfort noise RNG */
	uint32_t rng;

	/* Short block state for transients */
	bool prev_was_short;
	float short_overlap[OPVOX_SHORT_BLOCKS][OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS];
	float short_overlap_r[OPVOX_SHORT_BLOCKS][OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS];

	/* Pitch post-filter */
	pitch_filter_t    pitch_filt;    /* L channel post-filter */
	pitch_filter_t    pitch_filt_r;  /* R channel post-filter (stereo) */

	/* SNS state */
	sns_ctx_t         sns;

	/* BWE state */
	bwe_ctx_t         bwe;

	/* TNS state */
	tns_ctx_t         tns;

	/* Energy quantization state */
	energy_ctx_t      energy;
	energy_ctx_t      energy_r;   /* right channel (stereo only) */

	/* Band energy storage for right channel (stereo) */
	float band_energy_r[OPVOX_MAX_BANDS];

	/* PNS: perceptual noise substitution */
	pns_ctx_t         pns;
} opvox_decoder_t;

/*
 * Initialize encoder.
 * sample_rate: 8000, 16000, 32000, or 48000
 * channels: 1 (mono) or 2 (stereo)
 */
int opvox_encoder_init(opvox_encoder_t *enc, uint32_t sample_rate,
                       uint8_t channels, opvox_quality_t quality);

/*
 * Initialize decoder.  Parameters must match the encoder.
 * sample_rate: 8000, 16000, 32000, or 48000
 */
int opvox_decoder_init(opvox_decoder_t *dec, uint32_t sample_rate,
                       uint8_t channels, opvox_quality_t quality);

/*
 * Encode one frame of interleaved PCM samples (16-bit signed).
 * For mono: frame_samples samples.
 * For stereo: frame_samples * 2 samples (interleaved L R L R...).
 * Returns bytes written to `out`, or -1 on error.
 * Returns 0 for silence frames.
 */
int opvox_encode(opvox_encoder_t *enc,
                 const int16_t *pcm,
                 uint8_t *out, size_t out_cap);

/*
 * Decode one frame.  Writes interleaved PCM to `pcm`.
 * If `in` is NULL or `in_len` is 0, generates comfort noise.
 * Returns 0 on success, -1 on error.
 */
int opvox_decode(opvox_decoder_t *dec,
                 const uint8_t *in, size_t in_len,
                 int16_t *pcm);

/*
 * Packet loss concealment.
 */
int opvox_decode_plc(opvox_decoder_t *dec, int16_t *pcm);

/*
 * Query frame size for a given sample rate.
 */
static inline uint16_t
opvox_frame_samples(uint32_t rate)
{
	switch (rate) {
	case OPVOX_RATE_8K:  return OPVOX_FRAME_8K;
	case OPVOX_RATE_16K: return OPVOX_FRAME_16K;
	case OPVOX_RATE_32K: return OPVOX_FRAME_32K;
	case OPVOX_RATE_48K: return OPVOX_FRAME_48K;
	default:             return 0;
	}
}

#endif /* OPCODEC_AUDIO_H */
