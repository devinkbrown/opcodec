/*
 * opcodec/wasm_api.c — Browser WASM exported API
 *
 * Thin C wrapper exposing encode AND decode functions to JavaScript via
 * Emscripten cwrap/ccall.  Only the codec paths needed by browser clients
 * are included — no opssl, no IRC-server-only paths.
 *
 * Exported functions:
 *
 *   Heap helpers:
 *     opcodec_alloc_f32(n) → float*
 *     opcodec_alloc_i16(n) → int16_t*
 *     opcodec_alloc_u8(n)  → uint8_t*
 *     opcodec_free(ptr)
 *
 *   Noise suppression:
 *     ns2_wasm_create()                        → handle
 *     ns2_wasm_process(handle, pcm_f32, n)
 *     ns2_wasm_get_noise_db(handle)             → float
 *     ns2_wasm_destroy(handle)
 *
 *   DTX decoder (comfort noise):
 *     dtx_wasm_dec_create()                    → handle
 *     dtx_wasm_set_sid(handle, sid_ptr, len)
 *     dtx_wasm_voice_onset(handle)
 *     dtx_wasm_generate(handle, out_f32, n)
 *     dtx_wasm_destroy(handle)
 *
 *   Audio encoder (opvox):
 *     opvox_wasm_enc_create(sample_rate, quality) → handle
 *       quality: 0=LOW, 1=NORMAL, 2=HIGH, 3=ULTRA
 *     opvox_wasm_encode(handle, pcm_i16, n_samples, out_u8, out_cap) → int (encoded bytes)
 *     opvox_wasm_enc_destroy(handle)
 *
 *   Audio decoder (opvox):
 *     opvox_wasm_dec_create(sample_rate, quality) → handle
 *     opvox_wasm_decode(handle, in_u8, in_len, out_i16, n_samples)   → int (0=ok)
 *     opvox_wasm_dec_destroy(handle)
 *
 *   Video encoder (opvis):
 *     opvis_wasm_enc_create(width, height, quality) → handle
 *       quality: 0–100 (higher = better)
 *     opvis_wasm_enc_encode(handle, y, u, v, out_u8, out_cap, force_key) → int (encoded bytes)
 *     opvis_wasm_enc_flush(handle, out_u8, out_cap)                       → int (flushed bytes)
 *     opvis_wasm_enc_set_quality(handle, quality)
 *     opvis_wasm_enc_destroy(handle)
 *
 *   Video decoder (opvis):
 *     opvis_wasm_dec_create(width, height)                            → handle
 *     opvis_wasm_dec_decode(handle, in_u8, in_len, out_y, out_u, out_v) → int (0=ok)
 *     opvis_wasm_dec_destroy(handle)
 *
 * JavaScript usage:
 *
 *   const M = await createOpcodec();
 *
 *   // Encode a 960-sample 48kHz audio frame
 *   const enc  = M.ccall('opvox_wasm_enc_create', 'number', ['number','number'], [48000, 2]);
 *   const pcm  = M.ccall('opcodec_alloc_i16', 'number', ['number'], [960]);
 *   M.HEAP16.set(samples, pcm >> 1);
 *   const out  = M.ccall('opcodec_alloc_u8', 'number', ['number'], [512]);
 *   const len  = M.ccall('opvox_wasm_encode', 'number',
 *                        ['number','number','number','number','number'],
 *                        [enc, pcm, 960, out, 512]);
 *   const frame = M.HEAPU8.slice(out, out + len);
 *   M.ccall('opcodec_free', null, ['number'], [pcm]);
 *   M.ccall('opcodec_free', null, ['number'], [out]);
 *
 * Copyright (c) 2026 Ophion Development Team.  GPL v2.
 */

/* Suppress empty-translation-unit warning when not building for WASM */
typedef int opcodec_wasm_unused_t;

#ifdef OPCODEC_WASM

#include "opcodec/audio.h"
#include "opcodec/ns2.h"
#include "opcodec/dtx.h"
#include "opcodec/video.h"
#include <stdlib.h>
#include <string.h>
#include <emscripten.h>

/* ── Heap helpers ──────────────────────────────────────────────────────── */

EMSCRIPTEN_KEEPALIVE
float *opcodec_alloc_f32(int n)
{
    return (n > 0) ? (float *)calloc((size_t)n, sizeof(float)) : NULL;
}

EMSCRIPTEN_KEEPALIVE
int16_t *opcodec_alloc_i16(int n)
{
    return (n > 0) ? (int16_t *)calloc((size_t)n, sizeof(int16_t)) : NULL;
}

EMSCRIPTEN_KEEPALIVE
uint8_t *opcodec_alloc_u8(int n)
{
    return (n > 0) ? (uint8_t *)calloc((size_t)n, 1) : NULL;
}

EMSCRIPTEN_KEEPALIVE
void opcodec_free(void *p)
{
    free(p);
}

/* ── NS2 noise suppression ─────────────────────────────────────────────── */

EMSCRIPTEN_KEEPALIVE
void *ns2_wasm_create(void)
{
    ns2_ctx_t *ctx = (ns2_ctx_t *)calloc(1, sizeof(ns2_ctx_t));
    if (ctx) ns2_init(ctx, 48000, NS2_MODE_MODERATE);
    return ctx;
}

EMSCRIPTEN_KEEPALIVE
void ns2_wasm_process(void *handle, float *pcm, int n)
{
    if (!handle || !pcm || n <= 0) return;
    ns2_process((ns2_ctx_t *)handle, pcm, n);
}

EMSCRIPTEN_KEEPALIVE
float ns2_wasm_get_noise_db(void *handle)
{
    if (!handle) return -60.0f;
    return ns2_get_noise_db((ns2_ctx_t *)handle);
}

EMSCRIPTEN_KEEPALIVE
void ns2_wasm_destroy(void *handle) { free(handle); }

/* ── DTX decoder ───────────────────────────────────────────────────────── */

EMSCRIPTEN_KEEPALIVE
void *dtx_wasm_dec_create(void)
{
    dtx_dec_t *ctx = (dtx_dec_t *)calloc(1, sizeof(dtx_dec_t));
    if (ctx) dtx_dec_init(ctx);
    return ctx;
}

EMSCRIPTEN_KEEPALIVE
void dtx_wasm_set_sid(void *handle, const uint8_t *sid_packet, int len)
{
    if (handle) dtx_dec_set_sid((dtx_dec_t *)handle, sid_packet, len);
}

EMSCRIPTEN_KEEPALIVE
void dtx_wasm_voice_onset(void *handle)
{
    if (handle) dtx_dec_voice_onset((dtx_dec_t *)handle);
}

EMSCRIPTEN_KEEPALIVE
void dtx_wasm_generate(void *handle, float *out, int n)
{
    if (handle && out && n > 0) dtx_dec_generate((dtx_dec_t *)handle, out, n);
}

EMSCRIPTEN_KEEPALIVE
void dtx_wasm_destroy(void *handle) { free(handle); }

/* ── OPVOX audio encoder ───────────────────────────────────────────────── */

typedef struct {
    opvox_encoder_t enc;
    uint32_t        sample_rate;
} wasm_aud_enc_t;

EMSCRIPTEN_KEEPALIVE
void *opvox_wasm_enc_create(int sample_rate, int quality)
{
    if (sample_rate != 8000 && sample_rate != 16000 &&
        sample_rate != 32000 && sample_rate != 48000)
        return NULL;
    if (quality < 0 || quality > 3) quality = 2;

    wasm_aud_enc_t *w = (wasm_aud_enc_t *)calloc(1, sizeof(wasm_aud_enc_t));
    if (!w) return NULL;
    w->sample_rate = (uint32_t)sample_rate;

    if (opvox_encoder_init(&w->enc, (uint32_t)sample_rate, 1,
                           (opvox_quality_t)quality) != 0) {
        free(w); return NULL;
    }
    return w;
}

/*
 * Encode one 20ms audio frame.
 * pcm_i16: mono int16 PCM, exactly frame_samples samples (960 at 48kHz).
 * out_u8:  caller-allocated buffer (256 bytes is enough).
 * Returns encoded byte count, or -1 on error.
 */
EMSCRIPTEN_KEEPALIVE
int opvox_wasm_encode(void *handle,
                      const int16_t *pcm_i16, int n_samples,
                      uint8_t *out_u8, int out_cap)
{
    if (!handle || !pcm_i16 || n_samples <= 0 || !out_u8 || out_cap <= 0)
        return -1;
    wasm_aud_enc_t *w = (wasm_aud_enc_t *)handle;
    return opvox_encode(&w->enc, pcm_i16, out_u8, (size_t)out_cap);
}

EMSCRIPTEN_KEEPALIVE
void opvox_wasm_enc_destroy(void *handle) { free(handle); }

/* ── OPVOX audio decoder ───────────────────────────────────────────────── */

typedef struct {
    opvox_decoder_t dec;
} wasm_aud_dec_t;

EMSCRIPTEN_KEEPALIVE
void *opvox_wasm_dec_create(int sample_rate, int quality)
{
    if (sample_rate != 8000 && sample_rate != 16000 &&
        sample_rate != 32000 && sample_rate != 48000)
        return NULL;
    if (quality < 0 || quality > 3) quality = 2;

    wasm_aud_dec_t *w = (wasm_aud_dec_t *)calloc(1, sizeof(wasm_aud_dec_t));
    if (!w) return NULL;
    if (opvox_decoder_init(&w->dec, (uint32_t)sample_rate, 1,
                           (opvox_quality_t)quality) != 0) {
        free(w); return NULL;
    }
    return w;
}

/*
 * Decode one opvox frame.
 * in_u8:    encoded frame bytes (may be NULL for comfort noise / PLC).
 * out_i16:  output PCM buffer (caller-allocated, must hold n_samples samples).
 * Returns 0 on success, -1 on error.
 */
EMSCRIPTEN_KEEPALIVE
int opvox_wasm_decode(void *handle,
                      const uint8_t *in_u8, int in_len,
                      int16_t *out_i16, int n_samples)
{
    if (!handle || !out_i16 || n_samples <= 0) return -1;
    wasm_aud_dec_t *w = (wasm_aud_dec_t *)handle;
    return opvox_decode(&w->dec, in_u8, (size_t)(in_len > 0 ? in_len : 0),
                        out_i16, n_samples);
}

EMSCRIPTEN_KEEPALIVE
void opvox_wasm_dec_destroy(void *handle) { free(handle); }

/* ── OPVIS video encoder ───────────────────────────────────────────────── */

#define WASM_VID_OUT_CAP   (1024 * 1024)   /* 1MB output scratch per encoder */

typedef struct {
    opvis_encoder_t enc;
    int             pool_size;
    uint8_t        *pool;
    int             width;
    int             height;
    int             frame_num;
} wasm_vid_enc_t;

EMSCRIPTEN_KEEPALIVE
void *opvis_wasm_enc_create(int width, int height, int quality)
{
    if (width <= 0 || height <= 0 || width > 3840 || height > 2160) return NULL;
    if (quality < 0)   quality = 0;
    if (quality > 100) quality = 100;

    wasm_vid_enc_t *w = (wasm_vid_enc_t *)calloc(1, sizeof(wasm_vid_enc_t));
    if (!w) return NULL;
    w->width  = width;
    w->height = height;

    w->pool_size = opvis_encoder_pool_size((uint16_t)width, (uint16_t)height);
    w->pool      = (uint8_t *)calloc(1, (size_t)w->pool_size);
    if (!w->pool) { free(w); return NULL; }

    if (opvis_encoder_init(&w->enc, (uint16_t)width, (uint16_t)height,
                           (uint8_t)quality, w->pool, w->pool_size) != 0) {
        free(w->pool); free(w); return NULL;
    }
    return w;
}

/*
 * Encode one video frame.
 * y, u, v:   YUV420P planes in WASM heap (sizes: width*height, w/2*h/2 each).
 * out_u8:    caller-allocated output buffer (must be ≥ width*height*3/2 bytes).
 * out_cap:   byte capacity of out_u8.
 * force_key: non-zero = force I-frame regardless of GOP schedule.
 * Returns encoded byte count, or -1 on error.
 */
EMSCRIPTEN_KEEPALIVE
int opvis_wasm_enc_encode(void *handle,
                          const uint8_t *y, const uint8_t *u, const uint8_t *v,
                          uint8_t *out_u8, int out_cap, int force_key)
{
    if (!handle || !y || !u || !v || !out_u8 || out_cap <= 0) return -1;
    wasm_vid_enc_t *w = (wasm_vid_enc_t *)handle;

    opvis_frame_type_t ftype = (w->frame_num == 0 || force_key)
                               ? OPVIS_FRAME_I : OPVIS_FRAME_P;
    w->frame_num++;

    return opvis_encode(&w->enc, y, u, v,
                        (uint32_t)w->width, (uint32_t)w->height,
                        ftype, out_u8, (size_t)out_cap);
}

EMSCRIPTEN_KEEPALIVE
int opvis_wasm_enc_flush(void *handle, uint8_t *out_u8, int out_cap)
{
    if (!handle || !out_u8 || out_cap <= 0) return 0;
    wasm_vid_enc_t *w = (wasm_vid_enc_t *)handle;
    return opvis_encoder_flush(&w->enc, out_u8, (size_t)out_cap);
}

EMSCRIPTEN_KEEPALIVE
void opvis_wasm_enc_set_quality(void *handle, int quality)
{
    if (!handle) return;
    wasm_vid_enc_t *w = (wasm_vid_enc_t *)handle;
    if (quality < 0) quality = 0;
    if (quality > 100) quality = 100;
    opvis_encoder_set_crf(&w->enc, (uint8_t)quality);
}

EMSCRIPTEN_KEEPALIVE
void opvis_wasm_enc_destroy(void *handle)
{
    if (!handle) return;
    wasm_vid_enc_t *w = (wasm_vid_enc_t *)handle;
    free(w->pool);
    free(w);
}

/* ── OPVIS video decoder ───────────────────────────────────────────────── */

typedef struct {
    opvis_decoder_t dec;
    int             pool_size;
    uint8_t        *pool;
} wasm_vid_dec_t;

EMSCRIPTEN_KEEPALIVE
void *opvis_wasm_dec_create(int width, int height)
{
    if (width <= 0 || height <= 0 || width > 3840 || height > 2160) return NULL;

    wasm_vid_dec_t *w = (wasm_vid_dec_t *)calloc(1, sizeof(wasm_vid_dec_t));
    if (!w) return NULL;

    w->pool_size = opvis_decoder_pool_size(width, height);
    w->pool      = (uint8_t *)calloc(1, (size_t)w->pool_size);
    if (!w->pool) { free(w); return NULL; }

    if (opvis_decoder_init(&w->dec, width, height, w->pool, w->pool_size) != 0) {
        free(w->pool); free(w); return NULL;
    }
    return w;
}

EMSCRIPTEN_KEEPALIVE
int opvis_wasm_dec_decode(void *handle,
                          const uint8_t *in, int in_len,
                          uint8_t *out_y, uint8_t *out_u, uint8_t *out_v)
{
    if (!handle || !in || in_len <= 0) return -1;
    wasm_vid_dec_t *w = (wasm_vid_dec_t *)handle;
    return opvis_decode(&w->dec, in, (uint32_t)in_len, out_y, out_u, out_v);
}

EMSCRIPTEN_KEEPALIVE
void opvis_wasm_dec_destroy(void *handle)
{
    if (!handle) return;
    wasm_vid_dec_t *w = (wasm_vid_dec_t *)handle;
    free(w->pool);
    free(w);
}

#endif /* OPCODEC_WASM */
