/* opcodec/avsession.h — Audio/Video session layer
 *
 * Integrates OPVOX audio and OPVIS video codecs with jitter buffering,
 * A/V synchronization, and a unified packet format for real-time calls
 * and streaming over IRC DCC or similar transports.
 *
 * Packet wire format (each UDP/DCC payload):
 *   byte 0:    type (OPAV_PKT_AUDIO=0, OPAV_PKT_VIDEO=1, OPAV_PKT_KEYFRAME=2, OPAV_PKT_SKIP=3)
 *   bytes 1-4: sequence number (BE uint32)
 *   bytes 5-8: timestamp (BE uint32) — audio: sample count, video: frame number × video_ts_scale
 *   bytes 9-10: payload_len (BE uint16)
 *   bytes 11+: payload
 *
 * A/V sync:
 *   Audio and video timestamps are correlated via a common clock.
 *   The session tracks audio_ts (in samples) and video_ts (in frames × video_ts_scale).
 *   video_ts_scale = audio_rate / video_fps so both clocks tick at the same real-time rate.
 *   Lip sync offset = audio_ts - video_ts * video_ts_scale.
 *   If offset > sync_threshold_samples, the decoder should hold video or skip audio.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_AVSESSION_H
#define OPCODEC_AVSESSION_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "opcodec/audio.h"
#include "opcodec/video.h"
#include "opcodec/jitter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Packet type codes */
#define OPAV_PKT_AUDIO      0
#define OPAV_PKT_VIDEO      1
#define OPAV_PKT_KEYFRAME   2   /* I-frame video */
#define OPAV_PKT_SKIP       3   /* skipped video frame */

/* Packet header size */
#define OPAV_HEADER_SIZE    11

/* Maximum single-packet payload (audio frame or single video slice) */
#define OPAV_MAX_PAYLOAD    (OPVIS_MAX_ENCODED / 2)   /* ~4KB per packet */

/* Lip sync: how many samples of audio/video drift before correction */
#define OPAV_SYNC_THRESHOLD_SAMPLES  1920   /* 40ms at 48kHz */

/* video_ts_scale: video frames are stamped in units of (audio_rate / fps)
 * so a 30fps / 48kHz session uses scale = 1600 samples/frame */
static inline uint32_t opav_video_ts_scale(uint32_t audio_rate, uint32_t fps) {
    return (fps > 0) ? (audio_rate / fps) : 1600;
}

/* ---- Encoder-side session ---- */

typedef struct {
    opvox_encoder_t  audio;          /* OPVOX audio encoder */
    uint32_t         audio_rate;     /* audio sample rate (Hz) */
    uint16_t         audio_frame_samples;
    uint32_t         audio_ts;       /* running audio timestamp (samples sent) */
    uint32_t         audio_seq;      /* audio packet sequence */

    /* Video encoder is caller-owned (needs large pool) */
    opvis_encoder_t *video;          /* OPVIS video encoder (caller provides) */
    uint32_t         video_ts;       /* running video timestamp (frames) */
    uint32_t         video_seq;      /* video packet sequence */
    uint32_t         video_ts_scale; /* samples per frame = audio_rate / fps */
    uint32_t         video_fps;

    /* FEC (optional) */
    bool     fec_enabled;
    uint8_t  fec_n;    /* total packets in FEC group */
    uint8_t  fec_k;    /* data packets in group */
} opav_encoder_t;

/* ---- Decoder-side session ---- */

typedef struct {
    opvox_decoder_t  audio;          /* OPVOX audio decoder */
    uint32_t         audio_rate;
    uint16_t         audio_frame_samples;
    uint8_t          audio_channels;
    opjit_buffer_t   audio_jit;      /* jitter buffer for audio */

    /* Video decoder is caller-owned (needs large pool) */
    opvis_decoder_t *video;          /* OPVIS video decoder (caller provides) */
    opjit_buffer_t   video_jit;      /* jitter buffer for video */

    /* A/V sync state */
    int64_t          audio_ts;       /* latest audio timestamp decoded */
    int64_t          video_ts;       /* latest video timestamp decoded */
    uint32_t         video_ts_scale; /* samples per frame */
    uint32_t         video_fps;
    int32_t          av_offset;      /* measured A/V offset (samples, + = audio ahead) */

    /* Lip sync correction */
    bool             sync_enabled;
    uint8_t          video_hold;     /* frames to hold video (when audio is behind) */

    /* Last-frame availability flag (set when opav_pull_video successfully decodes) */
    bool             video_frame_ready;
} opav_decoder_t;

/* ---- Encoder API ---- */

/*
 * Initialize the encoder session.
 * video_enc: caller-allocated and initialized opvis_encoder_t
 * audio_rate: 8000/16000/32000/48000
 * audio_channels: 1 or 2
 * audio_quality: OPVOX_QUALITY_*
 * video_fps: frames per second (e.g., 30)
 * Returns 0 on success.
 */
int opav_encoder_init(opav_encoder_t *sess,
                      opvis_encoder_t *video_enc,
                      uint32_t audio_rate, uint8_t audio_channels,
                      opvox_quality_t audio_quality, uint32_t video_fps);

/*
 * Enable FEC for audio (XOR-based n-of-k).
 * n: total packets per group (e.g., 5), k: data packets (e.g., 4)
 */
void opav_encoder_set_fec(opav_encoder_t *sess, bool enabled, uint8_t n, uint8_t k);

/*
 * Encode one audio frame (frame_samples PCM samples).
 * Writes an OPAV packet to `out`. Returns bytes written or -1.
 */
int opav_encode_audio(opav_encoder_t *sess,
                      const int16_t *pcm,
                      uint8_t *out, size_t out_cap);

/*
 * Encode one video frame.
 * input: raw frame pixels (format set on video_enc)
 * input_len: byte length of input
 * Writes one or more OPAV packets to `out`. Returns bytes written or -1.
 * For large frames, the caller may need out_cap >= OPVIS_MAX_ENCODED + OPAV_HEADER_SIZE.
 */
int opav_encode_video(opav_encoder_t *sess,
                      const uint8_t *input, size_t input_len,
                      uint8_t *out, size_t out_cap);

/* ---- Decoder API ---- */

/*
 * Initialize the decoder session.
 * video_dec: caller-allocated and initialized opvis_decoder_t
 * audio_rate/channels/quality: must match encoder
 * video_fps: must match encoder
 * audio_jit_min_ms / audio_jit_max_ms: jitter buffer range for audio
 * video_jit_min_ms / video_jit_max_ms: jitter buffer range for video
 */
int opav_decoder_init(opav_decoder_t *sess,
                      opvis_decoder_t *video_dec,
                      uint32_t audio_rate, uint8_t audio_channels,
                      opvox_quality_t audio_quality, uint32_t video_fps,
                      uint16_t audio_jit_min_ms, uint16_t audio_jit_max_ms,
                      uint16_t video_jit_min_ms, uint16_t video_jit_max_ms);

/*
 * Enable/disable lip sync correction.
 */
void opav_decoder_set_sync(opav_decoder_t *sess, bool enabled);

/*
 * Push a received OPAV packet into the appropriate jitter buffer.
 * arrival_ms: current wall clock in ms (for jitter calculation)
 * Returns 0 on success, -1 on malformed packet.
 */
int opav_push_packet(opav_decoder_t *sess,
                     const uint8_t *pkt, size_t pkt_len,
                     uint32_t arrival_ms);

/*
 * Pull decoded audio for one frame interval.
 * pcm: output buffer (frame_samples * channels * sizeof(int16_t) bytes)
 * frame_samples: samples per frame (from opvox_frame_samples())
 * Returns 0 on success (pcm filled), 1 on PLC (packet lost, concealment applied),
 * -1 on error.
 */
int opav_pull_audio(opav_decoder_t *sess, int16_t *pcm, uint16_t frame_samples);

/*
 * Pull decoded video for one frame.
 * Returns 0 if a new frame is available (access via opvis_decoded_y/u/v()),
 * 1 if the previous frame should be repeated (hold),
 * -1 on error.
 */
int opav_pull_video(opav_decoder_t *sess);

/*
 * Get current A/V offset in milliseconds (positive = audio ahead of video).
 * Values outside ±100ms indicate sync problems.
 */
int32_t opav_av_offset_ms(const opav_decoder_t *sess);

/*
 * Get decoder statistics string (audio jitter, video jitter, A/V offset).
 * Writes into buf[buf_len]. Returns buf.
 */
char *opav_stats_str(const opav_decoder_t *sess, char *buf, size_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* OPCODEC_AVSESSION_H */
