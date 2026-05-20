/* opcodec/avsession.c - Audio/Video session implementation
 *
 * See avsession.h for protocol details.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/avsession.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ---- Packet header helpers ---- */

static void write_opav_header(uint8_t *hdr, uint8_t type,
                              uint32_t seq, uint32_t ts, uint16_t payload_len) {
    hdr[0]  = type;
    hdr[1]  = (uint8_t)((seq >> 24) & 0xFF);
    hdr[2]  = (uint8_t)((seq >> 16) & 0xFF);
    hdr[3]  = (uint8_t)((seq >>  8) & 0xFF);
    hdr[4]  = (uint8_t)( seq        & 0xFF);
    hdr[5]  = (uint8_t)((ts  >> 24) & 0xFF);
    hdr[6]  = (uint8_t)((ts  >> 16) & 0xFF);
    hdr[7]  = (uint8_t)((ts  >>  8) & 0xFF);
    hdr[8]  = (uint8_t)( ts         & 0xFF);
    hdr[9]  = (uint8_t)((payload_len >> 8) & 0xFF);
    hdr[10] = (uint8_t)( payload_len       & 0xFF);
}

static int read_opav_header(const uint8_t *hdr, size_t hdr_avail,
                            uint8_t *type, uint32_t *seq,
                            uint32_t *ts, uint16_t *payload_len) {
    if (hdr_avail < OPAV_HEADER_SIZE) return -1;
    *type = hdr[0];
    *seq  = ((uint32_t)hdr[1] << 24) | ((uint32_t)hdr[2] << 16) |
            ((uint32_t)hdr[3] <<  8) |  (uint32_t)hdr[4];
    *ts   = ((uint32_t)hdr[5] << 24) | ((uint32_t)hdr[6] << 16) |
            ((uint32_t)hdr[7] <<  8) |  (uint32_t)hdr[8];
    *payload_len = (uint16_t)(((uint16_t)hdr[9] << 8) | (uint16_t)hdr[10]);
    return 0;
}

/* ---- Encoder ---- */

int opav_encoder_init(opav_encoder_t *sess,
                      opvis_encoder_t *video_enc,
                      uint32_t audio_rate, uint8_t audio_channels,
                      opvox_quality_t audio_quality, uint32_t video_fps) {
    if (!sess || !video_enc) return -1;
    if (video_fps == 0) return -1;

    memset(sess, 0, sizeof(*sess));

    if (opvox_encoder_init(&sess->audio, audio_rate, audio_channels, audio_quality) != 0)
        return -1;

    sess->audio_rate          = audio_rate;
    sess->audio_frame_samples = opvox_frame_samples(audio_rate);
    if (sess->audio_frame_samples == 0) return -1;

    sess->video           = video_enc;
    sess->video_fps       = video_fps;
    sess->video_ts_scale  = opav_video_ts_scale(audio_rate, video_fps);
    sess->audio_ts        = 0;
    sess->audio_seq       = 0;
    sess->video_ts        = 0;
    sess->video_seq       = 0;

    sess->fec_enabled = false;
    sess->fec_n = 0;
    sess->fec_k = 0;
    return 0;
}

void opav_encoder_set_fec(opav_encoder_t *sess, bool enabled, uint8_t n, uint8_t k) {
    if (!sess) return;
    sess->fec_enabled = enabled && n > 0 && k > 0 && k <= n;
    sess->fec_n = n;
    sess->fec_k = k;
}

int opav_encode_audio(opav_encoder_t *sess,
                      const int16_t *pcm,
                      uint8_t *out, size_t out_cap) {
    if (!sess || !pcm || !out) return -1;
    if (out_cap < OPAV_HEADER_SIZE + 2) return -1;

    /* Reserve room for header; encode into payload slot */
    uint8_t *payload = out + OPAV_HEADER_SIZE;
    size_t   payload_cap = out_cap - OPAV_HEADER_SIZE;

    int n = opvox_encode(&sess->audio, pcm, payload, payload_cap);
    if (n < 0) return -1;
    /* n == 0 means a silence/comfort-noise frame - still emit a zero-length packet */
    if ((size_t)n > 0xFFFFu) return -1;

    write_opav_header(out, OPAV_PKT_AUDIO,
                      sess->audio_seq, sess->audio_ts,
                      (uint16_t)n);

    sess->audio_seq++;
    sess->audio_ts += sess->audio_frame_samples;

    return (int)(OPAV_HEADER_SIZE + (size_t)n);
}

int opav_encode_video(opav_encoder_t *sess,
                      const uint8_t *input, size_t input_len,
                      uint8_t *out, size_t out_cap) {
    if (!sess || !input || !out || !sess->video) return -1;
    if (out_cap < OPAV_HEADER_SIZE + OPVIS_HEADER_V1_SIZE) return -1;

    uint8_t *payload = out + OPAV_HEADER_SIZE;
    size_t   payload_cap = out_cap - OPAV_HEADER_SIZE;

    int n = opvis_encode(sess->video, input, input_len, payload, payload_cap);
    if (n < 0) return -1;
    if ((size_t)n > 0xFFFFu) return -1;

    opvis_frame_stats_t st = opvis_encoder_get_stats(sess->video);
    uint8_t pkt_type;
    if (st.was_skipped || st.type == OPVIS_FRAME_SKIP)
        pkt_type = OPAV_PKT_SKIP;
    else if (st.type == OPVIS_FRAME_I)
        pkt_type = OPAV_PKT_KEYFRAME;
    else
        pkt_type = OPAV_PKT_VIDEO;

    uint32_t ts = sess->video_ts * sess->video_ts_scale;
    write_opav_header(out, pkt_type, sess->video_seq, ts, (uint16_t)n);

    sess->video_seq++;
    sess->video_ts++;

    return (int)(OPAV_HEADER_SIZE + (size_t)n);
}

/* ---- Decoder ---- */

int opav_decoder_init(opav_decoder_t *sess,
                      opvis_decoder_t *video_dec,
                      uint32_t audio_rate, uint8_t audio_channels,
                      opvox_quality_t audio_quality, uint32_t video_fps,
                      uint16_t audio_jit_min_ms, uint16_t audio_jit_max_ms,
                      uint16_t video_jit_min_ms, uint16_t video_jit_max_ms) {
    if (!sess || !video_dec) return -1;
    if (video_fps == 0) return -1;

    memset(sess, 0, sizeof(*sess));

    if (opvox_decoder_init(&sess->audio, audio_rate, audio_channels, audio_quality) != 0)
        return -1;

    sess->audio_rate          = audio_rate;
    sess->audio_channels      = audio_channels;
    sess->audio_frame_samples = opvox_frame_samples(audio_rate);
    if (sess->audio_frame_samples == 0) return -1;

    sess->video          = video_dec;
    sess->video_fps      = video_fps;
    sess->video_ts_scale = opav_video_ts_scale(audio_rate, video_fps);

    /* Audio jitter buffer - frame duration = 20ms (OPVOX_FRAME_MS) */
    opjit_init(&sess->audio_jit, OPVOX_FRAME_MS,
               audio_jit_min_ms, audio_jit_max_ms);

    /* Video jitter buffer - frame duration depends on fps */
    uint16_t video_frame_ms = (uint16_t)(1000u / video_fps);
    if (video_frame_ms == 0) video_frame_ms = 1;
    opjit_init(&sess->video_jit, video_frame_ms,
               video_jit_min_ms, video_jit_max_ms);

    sess->audio_ts = 0;
    sess->video_ts = 0;
    sess->av_offset = 0;
    sess->sync_enabled = true;
    sess->video_hold = 0;
    sess->video_frame_ready = false;
    return 0;
}

void opav_decoder_set_sync(opav_decoder_t *sess, bool enabled) {
    if (!sess) return;
    sess->sync_enabled = enabled;
}

int opav_push_packet(opav_decoder_t *sess,
                     const uint8_t *pkt, size_t pkt_len,
                     uint32_t arrival_ms) {
    if (!sess || !pkt) return -1;
    if (pkt_len < OPAV_HEADER_SIZE) return -1;

    uint8_t  type;
    uint32_t seq, ts;
    uint16_t payload_len;
    if (read_opav_header(pkt, pkt_len, &type, &seq, &ts, &payload_len) != 0)
        return -1;
    if (pkt_len < (size_t)OPAV_HEADER_SIZE + (size_t)payload_len) return -1;
    if (payload_len > OPJIT_MAX_PAYLOAD) return -1;

    const uint8_t *payload = pkt + OPAV_HEADER_SIZE;

    opjit_buffer_t *jb;
    switch (type) {
        case OPAV_PKT_AUDIO:
            jb = &sess->audio_jit;
            break;
        case OPAV_PKT_VIDEO:
        case OPAV_PKT_KEYFRAME:
        case OPAV_PKT_SKIP:
            jb = &sess->video_jit;
            break;
        default:
            return -1;
    }

    opjit_push(jb, seq, ts, payload, payload_len, arrival_ms);
    return 0;
}

int opav_pull_audio(opav_decoder_t *sess, int16_t *pcm, uint16_t frame_samples) {
    if (!sess || !pcm) return -1;

    uint8_t  buf[OPJIT_MAX_PAYLOAD];
    uint16_t len = 0;
    opjit_status_t st = opjit_pull(&sess->audio_jit, buf, &len);

    int ret = 0;
    switch (st) {
        case OPJIT_OK:
            if (opvox_decode(&sess->audio, buf, len, pcm) != 0) {
                /* fall back to PLC on decode failure */
                opvox_decode_plc(&sess->audio, pcm);
                ret = 1;
            } else {
                ret = 0;
            }
            break;
        case OPJIT_LOST:
        case OPJIT_EMPTY:
            opvox_decode_plc(&sess->audio, pcm);
            ret = 1;
            break;
        case OPJIT_LATE:
        case OPJIT_FULL:
        default:
            /* Treat as loss */
            opvox_decode_plc(&sess->audio, pcm);
            ret = 1;
            break;
    }

    sess->audio_ts += (int64_t)frame_samples;

    /* Recompute A/V offset: + = audio ahead of video */
    int64_t video_samples = sess->video_ts * (int64_t)sess->video_ts_scale;
    int64_t off = sess->audio_ts - video_samples;
    if (off >  (int64_t)INT32_MAX) off =  (int64_t)INT32_MAX;
    if (off < -(int64_t)INT32_MAX) off = -(int64_t)INT32_MAX;
    sess->av_offset = (int32_t)off;

    return ret;
}

int opav_pull_video(opav_decoder_t *sess) {
    if (!sess || !sess->video) return -1;

    /* Hold logic: when audio is significantly behind video, we previously
     * decided to repeat the last decoded frame for `video_hold` ticks. */
    if (sess->video_hold > 0) {
        sess->video_hold--;
        return 1;
    }

    if (sess->sync_enabled) {
        /* If audio is far behind video (av_offset very negative), hold the
         * current video frame so audio can catch up. */
        if (sess->av_offset < -(int32_t)OPAV_SYNC_THRESHOLD_SAMPLES) {
            sess->video_hold = 1;
            return 1;
        }
    }

    uint8_t  buf[OPJIT_MAX_PAYLOAD];
    uint16_t len = 0;
    opjit_status_t st = opjit_pull(&sess->video_jit, buf, &len);

    switch (st) {
        case OPJIT_OK:
            if (opvis_decode(sess->video, buf, len) != 0) {
                /* decode failure - hold previous frame */
                return 1;
            }
            sess->video_ts++;
            sess->video_frame_ready = true;
            return 0;
        case OPJIT_LOST:
        case OPJIT_EMPTY:
        case OPJIT_LATE:
        case OPJIT_FULL:
        default:
            /* Hold previous frame */
            return 1;
    }
}

int32_t opav_av_offset_ms(const opav_decoder_t *sess) {
    if (!sess || sess->audio_rate == 0) return 0;
    /* offset_samples * 1000 / rate */
    int64_t scaled = (int64_t)sess->av_offset * 1000;
    return (int32_t)(scaled / (int64_t)sess->audio_rate);
}

char *opav_stats_str(const opav_decoder_t *sess, char *buf, size_t buf_len) {
    if (!buf || buf_len == 0) return buf;
    if (!sess) {
        if (buf_len > 0) buf[0] = '\0';
        return buf;
    }
    int32_t av_ms = opav_av_offset_ms(sess);
    uint16_t a_jit = opjit_jitter_ms(&sess->audio_jit);
    uint16_t v_jit = opjit_jitter_ms(&sess->video_jit);
    snprintf(buf, buf_len,
             "a_jit=%ums v_jit=%ums av=%+dms a_lost=%u v_lost=%u",
             (unsigned)a_jit, (unsigned)v_jit, (int)av_ms,
             (unsigned)sess->audio_jit.stat_lost,
             (unsigned)sess->video_jit.stat_lost);
    return buf;
}
