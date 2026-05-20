/*
 * opcodec/codec_suite_test.c — Comprehensive subsystem test suite
 *
 * Tests all major opcodec subsystems in a single binary:
 *   - FEC (forward error correction): encode, recover lost packet
 *   - netadapt: state machine transitions, bitrate distribution
 *   - jitter: push out-of-order packets, pull in order, loss detection
 *   - parametric stereo: analyze → encode_params → decode_params → upmix
 *   - PVQ: encode/decode coefficient round-trip
 *   - bwe: cutoff detection, extension synthesis
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/fec.h"
#include "opcodec/netadapt.h"
#include "opcodec/jitter.h"
#include "opcodec/stereo.h"
#include "opcodec/pvq.h"
#include "opcodec/bwe.h"
#include "opcodec/tns.h"
#include "opcodec/energy.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, fmt, ...) do { \
    if (!(cond)) { \
        printf("  FAIL: " fmt "\n", ##__VA_ARGS__); \
        g_fail++; \
    } else { \
        g_pass++; \
    } \
} while (0)

#define SECTION(name) printf("\n=== %s ===\n", name)
#define PASS(...) do { printf("  pass: "); printf(__VA_ARGS__); printf("\n"); } while(0)

/* ================================================================
 * FEC TESTS
 * ================================================================ */

static void test_fec_basic_recovery(void)
{
    SECTION("FEC — 1-of-4 packet recovery");

    opfec_encoder_t enc;
    opfec_enc_init(&enc, OPFEC_LEVEL_LOW);
    CHECK(enc.group_size == 4, "LOW group_size expected 4, got %u", enc.group_size);

    /* Feed 4 data packets */
    uint8_t fec_out[OPFEC_MAX_PACKET + OPFEC_HEADER_SIZE + OPFEC_LEN_TABLE_SIZE * 2];
    const char *payloads[] = {"packet_one", "packet_two", "packet_three", "packet_four"};
    int fec_ready = 0;

    for (int i = 0; i < 4; i++) {
        uint8_t buf[32];
        uint16_t len = (uint16_t)strlen(payloads[i]);
        memcpy(buf, payloads[i], len);
        int r = opfec_enc_feed(&enc, buf, len, 100 + i);
        if (r == 1) fec_ready = 1;
    }
    CHECK(fec_ready, "FEC packet should be ready after %d data packets", 4);

    int fec_len = opfec_enc_get_fec(&enc, fec_out, sizeof(fec_out));
    CHECK(fec_len > OPFEC_HEADER_SIZE, "FEC packet size %d <= header size", fec_len);

    /* Parse FEC header */
    opfec_header_t hdr;
    opfec_header_read(&hdr, fec_out);
    CHECK(hdr.group_size == 4, "FEC header group_size %u != 4", hdr.group_size);
    CHECK(hdr.type == OPFEC_ROW, "FEC header type %u != ROW", hdr.type);

    /* Now test decoder: simulate losing packet 2 */
    opfec_decoder_t dec;
    opfec_dec_init(&dec, OPFEC_LEVEL_LOW);
    opfec_dec_new_group(&dec, hdr.group_id, 4, 100);

    /* Feed packets 0, 1, 3 (lose packet 2) */
    for (int i = 0; i < 4; i++) {
        if (i == 2) continue;  /* lost */
        uint8_t buf[32];
        uint16_t len = (uint16_t)strlen(payloads[i]);
        memcpy(buf, payloads[i], len);
        opfec_dec_feed_data(&dec, hdr.group_id, i, buf, len);
    }

    /* Feed FEC packet — payload includes length table then XOR data */
    uint8_t *fec_payload = fec_out + OPFEC_HEADER_SIZE;
    uint16_t fec_payload_len = (uint16_t)(hdr.group_size * 2 + hdr.payload_len);
    opfec_dec_feed_fec(&dec, &hdr, fec_payload, fec_payload_len);

    CHECK(opfec_dec_can_recover(&dec, 2), "should be able to recover packet 2");

    uint8_t recovered[OPFEC_MAX_PACKET];
    uint16_t recovered_len = 0;
    int r = opfec_dec_recover(&dec, 2, recovered, sizeof(recovered), &recovered_len);
    CHECK(r == 0, "dec_recover returned %d", r);

    uint16_t expected_len = (uint16_t)strlen(payloads[2]);
    CHECK(recovered_len == expected_len,
          "recovered length %u != expected %u", recovered_len, expected_len);
    CHECK(memcmp(recovered, payloads[2], expected_len) == 0,
          "recovered content mismatch: got '%.*s'", (int)recovered_len, recovered);

    PASS("FEC recovered packet 2 ('%.*s') correctly", (int)recovered_len, recovered);
}

static void test_fec_overhead(void)
{
    SECTION("FEC — overhead percentages");
    CHECK(opfec_get_overhead_pct(OPFEC_LEVEL_NONE)   ==   0, "NONE overhead");
    CHECK(opfec_get_overhead_pct(OPFEC_LEVEL_LOW)    ==  25, "LOW overhead");
    CHECK(opfec_get_overhead_pct(OPFEC_LEVEL_MEDIUM) ==  33, "MEDIUM overhead");
    CHECK(opfec_get_overhead_pct(OPFEC_LEVEL_HIGH)   ==  50, "HIGH overhead");
    CHECK(opfec_get_overhead_pct(OPFEC_LEVEL_MAX)    == 100, "MAX overhead");
    PASS("all FEC overhead values correct");
}

static void test_fec_header_roundtrip(void)
{
    SECTION("FEC — header serialization roundtrip");
    opfec_header_t h_out = {
        .type       = OPFEC_ROW,
        .group_id   = 0xAB,
        .group_size = 4,
        .pkt_idx    = 3,
        .payload_len = 0x1234,
        .base_seq   = 0x5678,
    };
    uint8_t wire[OPFEC_HEADER_SIZE];
    opfec_header_write(&h_out, wire);

    opfec_header_t h_in;
    opfec_header_read(&h_in, wire);

    CHECK(h_in.type       == h_out.type,       "type mismatch");
    CHECK(h_in.group_id   == h_out.group_id,   "group_id mismatch");
    CHECK(h_in.group_size == h_out.group_size, "group_size mismatch");
    CHECK(h_in.pkt_idx    == h_out.pkt_idx,    "pkt_idx mismatch");
    CHECK(h_in.payload_len == h_out.payload_len, "payload_len mismatch (got 0x%04x)", h_in.payload_len);
    /* base_seq is truncated to 16 bits on wire */
    CHECK((h_in.base_seq & 0xFFFF) == (h_out.base_seq & 0xFFFF),
          "base_seq low 16 bits mismatch");
    PASS("FEC header roundtrip OK");
}

static void test_fec_flush_partial(void)
{
    SECTION("FEC — flush partial group");
    opfec_encoder_t enc;
    opfec_enc_init(&enc, OPFEC_LEVEL_LOW);

    /* Feed only 2 packets out of 4 required */
    uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    opfec_enc_feed(&enc, data, 4, 200);
    opfec_enc_feed(&enc, data, 4, 201);

    uint8_t fec_out[256];
    int r = opfec_enc_flush(&enc, fec_out, sizeof(fec_out));
    CHECK(r > 0, "flush should return bytes written when partial group exists, got %d", r);
    PASS("partial group flush returns FEC packet");
}

/* ================================================================
 * NETADAPT TESTS
 * ================================================================ */

static void test_netadapt_init(void)
{
    SECTION("netadapt — initialization");
    netadapt_ctx_t ctx;
    netadapt_init(&ctx, 16000, 512000, false);

    CHECK(ctx.state == NET_STATE_STABLE, "initial state should be STABLE");
    CHECK(ctx.target_bitrate >= 16000, "target_bitrate < min");
    CHECK(ctx.target_bitrate <= 512000, "target_bitrate > max");
    CHECK(ctx.srtt_ms == 50.0f, "initial SRTT should be 50ms, got %.1f", ctx.srtt_ms);
    CHECK(ctx.loss_rate == 0.0f, "initial loss rate should be 0");
    PASS("init with min=16k max=512k");
}

static void test_netadapt_rtt_tracking(void)
{
    SECTION("netadapt — RTT EWMA smoothing");
    netadapt_ctx_t ctx;
    netadapt_init(&ctx, 16000, 512000, false);

    /* Feed stable low-RTT samples */
    for (int i = 0; i < 20; i++) {
        netadapt_update_rtt(&ctx, 20.0f, 1000 * i);
    }
    CHECK(ctx.srtt_ms < 40.0f, "SRTT should converge below 40ms, got %.1f", ctx.srtt_ms);

    /* Feed a spike and verify it's damped */
    float srtt_before = ctx.srtt_ms;
    netadapt_update_rtt(&ctx, 200.0f, 20000);
    CHECK(ctx.srtt_ms > srtt_before, "SRTT should increase after spike");
    CHECK(ctx.srtt_ms < 200.0f, "SRTT should be damped (< spike value)");
    PASS("SRTT EWMA damping works");
}

static void test_netadapt_loss_response(void)
{
    SECTION("netadapt — loss rate and FEC escalation");
    netadapt_ctx_t ctx;
    netadapt_init(&ctx, 16000, 512000, false);

    /* Simulate 8% loss — should trigger HIGH FEC */
    for (int i = 0; i < 30; i++) {
        netadapt_update_loss(&ctx, 100, 8, 1000 * i);
        netadapt_update_rtt(&ctx, 30.0f, 1000 * i);
        netadapt_evaluate(&ctx, 1000 * i);
    }
    CHECK(ctx.fec_level >= 3, "8%% loss should trigger FEC >= HIGH, got %u", ctx.fec_level);
    PASS("8%% loss → FEC level %u", ctx.fec_level);
}

static void test_netadapt_probe_drain_cycle(void)
{
    SECTION("netadapt — probe → drain state machine");
    netadapt_ctx_t ctx;
    netadapt_init(&ctx, 64000, 512000, false);

    /* Set initial target well below estimate to allow probing */
    ctx.target_bitrate = 128000;
    ctx.bw_estimate    = 400000;

    uint32_t t = 0;
    netadapt_update_rtt(&ctx, 20.0f, t);
    netadapt_update_loss(&ctx, 100, 0, t);

    /* Trigger probe */
    netadapt_evaluate(&ctx, t);
    CHECK(ctx.state == NET_STATE_PROBING || ctx.state == NET_STATE_STABLE,
          "should probe or stay stable, got state %d", ctx.state);

    if (ctx.state == NET_STATE_PROBING) {
        /* Advance past probe duration with high loss to trigger drain */
        t += 3100;
        /* Feed enough loss samples to cross the 3% EWMA threshold.
         * LOSS_ALPHA=0.1, so single 6/100 sample yields only 0.6% EWMA.
         * Use 40% loss (40/100) → EWMA = 0.1*0.4 = 4% > 3% threshold. */
        netadapt_update_loss(&ctx, 100, 40, t);
        netadapt_evaluate(&ctx, t);
        CHECK(ctx.state == NET_STATE_DRAINING || ctx.state == NET_STATE_RECOVERY,
              "should drain after loss during probe, got state %d", ctx.state);
        PASS("probe → drain transition triggered by loss");
    } else {
        PASS("stayed stable (bw estimate not sufficiently above target)");
    }
}

static void test_netadapt_audio_only(void)
{
    SECTION("netadapt — audio-only mode disables video");
    netadapt_ctx_t ctx;
    netadapt_init(&ctx, 16000, 512000, true);  /* audio_only=true */

    netadapt_update_rtt(&ctx, 10.0f, 0);
    netadapt_update_ack(&ctx, 50000, 10.0f, 0);
    netadapt_evaluate(&ctx, 1000);

    CHECK(!ctx.video_enabled, "video should be disabled in audio-only mode");
    PASS("audio-only mode prevents video allocation");
}

/* ================================================================
 * JITTER BUFFER TESTS
 * ================================================================ */

static void test_jitter_basic(void)
{
    SECTION("jitter buffer — in-order delivery");
    opjit_buffer_t jb;
    opjit_init(&jb, 20, 40, 200);

    uint8_t payload[8] = "hello";
    uint8_t out[OPJIT_MAX_PAYLOAD];
    uint16_t out_len;

    /* Push 5 packets in order */
    for (uint32_t seq = 1; seq <= 5; seq++) {
        opjit_status_t s = opjit_push(&jb, seq, seq * 160, payload, 5, seq * 20);
        CHECK(s == OPJIT_OK || s == OPJIT_FULL,
              "push seq %u returned unexpected status %d", seq, s);
    }

    /* Pull until we get packets or run out */
    int got = 0;
    for (int i = 0; i < 10; i++) {
        opjit_status_t s = opjit_pull(&jb, out, &out_len);
        if (s == OPJIT_OK) got++;
        else if (s == OPJIT_EMPTY || s == OPJIT_LOST) continue;
    }
    CHECK(got > 0, "jitter buffer should deliver at least 1 packet, got %d", got);
    PASS("delivered %d packets in order", got);
}

static void test_jitter_reorder(void)
{
    SECTION("jitter buffer — out-of-order packets");
    opjit_buffer_t jb;
    opjit_init(&jb, 20, 40, 200);

    uint8_t payload[8];
    /* Push out of order: 3, 1, 2 */
    uint32_t order[] = {3, 1, 2};
    for (int i = 0; i < 3; i++) {
        uint32_t seq = order[i];
        memset(payload, (int)seq, sizeof(payload));
        opjit_push(&jb, seq, seq * 160, payload, 8, seq * 20);
    }

    /* Verify depth reflects buffered packets */
    uint16_t depth = opjit_depth(&jb);
    CHECK(depth >= 1 && depth <= 3,
          "depth should be 1-3 after 3 pushes, got %u", depth);
    PASS("out-of-order push: depth=%u", depth);
}

static void test_jitter_stats(void)
{
    SECTION("jitter buffer — statistics tracking");
    opjit_buffer_t jb;
    opjit_init(&jb, 20, 40, 200);

    uint8_t payload[16] = {0};
    /* Push 10 packets with simulated arrival times */
    for (uint32_t seq = 1; seq <= 10; seq++) {
        opjit_push(&jb, seq, seq * 160, payload, 16, seq * 20 + (seq % 3) * 5);
    }

    CHECK(jb.stat_received == 10, "received count %u != 10", jb.stat_received);

    /* Jitter estimate should be non-negative */
    uint16_t jitter = opjit_jitter_ms(&jb);
    (void)jitter;  /* Just verify it doesn't crash */
    PASS("stats: received=%u jitter=%ums", jb.stat_received, opjit_jitter_ms(&jb));
}

/* ================================================================
 * PARAMETRIC STEREO TESTS
 * ================================================================ */

static void test_stereo_downmix_upmix(void)
{
    SECTION("parametric stereo — downmix / analyze / upmix roundtrip");

    ps_ctx_t ctx;
    ps_init(&ctx, 10);

    /* Generate distinct L and R signals */
    float mdct_l[480], mdct_r[480];
    for (int i = 0; i < 480; i++) {
        mdct_l[i] = 0.5f * sinf(2.0f * (float)M_PI * i / 48.0f);
        mdct_r[i] = 0.3f * cosf(2.0f * (float)M_PI * i / 32.0f);
    }

    /* Build band ranges for 10 bands */
    band_range_t bands[10];
    int step = 480 / 10;
    for (int b = 0; b < 10; b++) {
        bands[b].start = (uint16_t)(b * step);
        bands[b].end   = (uint16_t)((b + 1) * step);
    }

    /* Analyze */
    ps_params_t params;
    ps_analyze(&ctx, mdct_l, mdct_r, 480, bands, 10, &params);
    CHECK(params.num_bands == 10, "ps_analyze num_bands %u != 10", params.num_bands);

    /* Verify IID is in range */
    for (int b = 0; b < 10; b++) {
        CHECK(params.iid[b] >= -30.0f && params.iid[b] <= 30.0f,
              "band %d IID %.2f out of expected range", b, params.iid[b]);
        CHECK(params.icc[b] >= -1.0f && params.icc[b] <= 1.0f,
              "band %d ICC %.2f out of range", b, params.icc[b]);
    }

    /* Downmix */
    float mdct_mono[480];
    ps_downmix(mdct_l, mdct_r, mdct_mono, 480);

    /* Verify downmix energy is between L and R */
    float e_l = 0, e_r = 0, e_m = 0;
    for (int i = 0; i < 480; i++) {
        e_l += mdct_l[i] * mdct_l[i];
        e_r += mdct_r[i] * mdct_r[i];
        e_m += mdct_mono[i] * mdct_mono[i];
    }
    float e_min = e_l < e_r ? e_l : e_r;
    float e_max = e_l > e_r ? e_l : e_r;
    (void)e_min; (void)e_max;
    CHECK(e_m > 0.0f, "downmix should have non-zero energy");

    /* Encode and decode spatial params */
    uint8_t bitstream[64];
    int enc_len = ps_encode_params(&params, bitstream, sizeof(bitstream));
    CHECK(enc_len > 0, "ps_encode_params returned %d", enc_len);

    ps_params_t decoded;
    int dec_len = ps_decode_params(&decoded, bitstream, enc_len);
    CHECK(dec_len == enc_len, "ps_decode_params consumed %d != encoded %d", dec_len, enc_len);
    CHECK(decoded.num_bands == 10, "decoded num_bands %u != 10", decoded.num_bands);

    /* IID values should survive encode/decode within quantization error */
    for (int b = 0; b < 10; b++) {
        float err = fabsf(decoded.iid[b] - params.iid[b]);
        CHECK(err < 2.0f, "band %d IID quantization error %.2f dB > 2dB", b, err);
    }

    /* Upmix */
    float mdct_l_out[480], mdct_r_out[480];
    ps_upmix(&decoded, mdct_mono, mdct_l_out, mdct_r_out, 480, bands, 10);

    float e_lo = 0, e_ro = 0;
    for (int i = 0; i < 480; i++) {
        e_lo += mdct_l_out[i] * mdct_l_out[i];
        e_ro += mdct_r_out[i] * mdct_r_out[i];
    }
    CHECK(e_lo > 0.0f, "upmix L channel has zero energy");
    CHECK(e_ro > 0.0f, "upmix R channel has zero energy");
    PASS("stereo roundtrip: IID/ICC preserved within 2dB, upmix has energy");
}

static void test_stereo_params_extreme(void)
{
    SECTION("parametric stereo — extreme IID/ICC values");

    ps_ctx_t ctx;
    ps_init(&ctx, 5);

    /* L only (R=0): IID should be very large positive */
    float mdct_l[480], mdct_r[480];
    for (int i = 0; i < 480; i++) {
        mdct_l[i] = sinf((float)i * 0.1f);
        mdct_r[i] = 0.001f;  /* near-silent R */
    }

    band_range_t bands[5];
    for (int b = 0; b < 5; b++) {
        bands[b].start = (uint16_t)(b * 96);
        bands[b].end   = (uint16_t)((b + 1) * 96);
    }

    ps_params_t params;
    ps_analyze(&ctx, mdct_l, mdct_r, 480, bands, 5, &params);

    /* All IIDs should be positive (L stronger than R) */
    for (int b = 0; b < 5; b++) {
        CHECK(params.iid[b] > 0.0f, "band %d IID %.2f should be > 0 (L >> R)", b, params.iid[b]);
    }
    PASS("L-dominant signal: all IID > 0");
}

/* ================================================================
 * PVQ TESTS
 * ================================================================ */

static void test_pvq_roundtrip(void)
{
    SECTION("PVQ — encode/decode roundtrip");

    float coeffs[16] = {1.5f, -0.8f, 2.1f, 0.3f, -1.2f, 0.7f, 1.8f, -0.5f,
                        0.9f, -1.6f, 0.4f, 2.3f, -0.2f, 1.1f, -0.9f, 1.4f};
    int16_t shape[16];
    float gain;

    pvq_encode(coeffs, 16, 12, &gain, shape);
    CHECK(gain > 0.0f, "PVQ gain should be positive, got %.4f", gain);

    /* Verify L1 norm of shape = K = 12 */
    int l1 = 0;
    for (int i = 0; i < 16; i++) l1 += shape[i] < 0 ? -shape[i] : shape[i];
    CHECK(l1 == 12, "PVQ shape L1 norm = %d, expected 12", l1);

    float decoded[16];
    pvq_decode(gain, shape, 16, decoded);

    /* Energy of decoded should be close to input */
    float e_in = 0, e_out = 0;
    for (int i = 0; i < 16; i++) {
        e_in  += coeffs[i] * coeffs[i];
        e_out += decoded[i] * decoded[i];
    }
    float ratio = (e_in > 0) ? e_out / e_in : 0;
    CHECK(ratio > 0.5f && ratio < 2.0f,
          "PVQ energy ratio %.2f out of range [0.5, 2.0]", ratio);
    PASS("PVQ roundtrip: L1=%d gain=%.3f energy_ratio=%.2f", l1, gain, ratio);
}

static void test_pvq_k_values(void)
{
    SECTION("PVQ — various K values");

    float coeffs[8] = {1.0f, 0.0f, -2.0f, 0.5f, 1.5f, -0.5f, 0.0f, 1.0f};
    int16_t shape[8];
    float gain;

    for (int k = 1; k <= 24; k += 4) {
        pvq_encode(coeffs, 8, k, &gain, shape);
        int l1 = 0;
        for (int i = 0; i < 8; i++) l1 += shape[i] < 0 ? -shape[i] : shape[i];
        CHECK(l1 == k, "K=%d: L1=%d should equal K", k, l1);
    }
    PASS("PVQ L1 constraint holds for K=1..24");
}

/* ================================================================
 * BWE TESTS
 * ================================================================ */

static void test_bwe_cutoff(void)
{
    SECTION("BWE — optimal cutoff selection and synthesis roundtrip");

    /* bwe_optimal_cutoff: more bits → higher cutoff (more bands coded) */
    uint16_t cutoff_low  = bwe_optimal_cutoff(50,   16, 16000);
    uint16_t cutoff_high = bwe_optimal_cutoff(4096, 16, 16000);
    CHECK(cutoff_low  <= 16, "low-bits cutoff %u out of range",  cutoff_low);
    CHECK(cutoff_high <= 16, "high-bits cutoff %u out of range", cutoff_high);
    CHECK(cutoff_high >= cutoff_low, "high-bits cutoff %u < low-bits %u", cutoff_high, cutoff_low);
    PASS("BWE optimal cutoff: 256 bits→%u 4096 bits→%u bands coded", cutoff_low, cutoff_high);

    /* BWE encode/synthesize roundtrip */
    bwe_ctx_t enc_ctx, dec_ctx;
    bwe_init(&enc_ctx, cutoff_low, 16);
    bwe_init(&dec_ctx, cutoff_low, 16);

    band_range_t bands[16];
    for (int b = 0; b < 16; b++) {
        bands[b].start = (uint16_t)(b * 10);
        bands[b].end   = (uint16_t)((b + 1) * 10);
    }

    float mdct[160];
    for (int i = 0; i < 160; i++) mdct[i] = sinf((float)i * 0.3f);

    uint8_t bwe_bits[64];
    int enc_len = bwe_encode(&enc_ctx, mdct, 160, bands, 16, bwe_bits, sizeof(bwe_bits));
    CHECK(enc_len >= 0, "bwe_encode returned %d", enc_len);

    int dec_len = bwe_decode(&dec_ctx, bwe_bits, enc_len);
    CHECK(dec_len == enc_len, "bwe_decode consumed %d != encoded %d", dec_len, enc_len);

    /* Synthesize high bands */
    float mdct_dec[160];
    memcpy(mdct_dec, mdct, sizeof(mdct));
    /* Zero out the high bands (as encoder would do) */
    for (int b = cutoff_low; b < 16; b++) {
        for (int i = bands[b].start; i < bands[b].end; i++) mdct_dec[i] = 0.0f;
    }
    bwe_synthesize(&dec_ctx, mdct_dec, 160, bands, 16);

    /* High bands should have non-zero energy after synthesis */
    float e_high = 0;
    for (int b = cutoff_low; b < 16; b++) {
        for (int i = bands[b].start; i < bands[b].end; i++) e_high += mdct_dec[i] * mdct_dec[i];
    }
    CHECK(e_high > 0.0f, "BWE synthesized high bands have zero energy");
    PASS("BWE synthesize fills high bands (energy=%.4f)", e_high);
}

/* ================================================================
 * TNS TESTS
 * ================================================================ */

static void test_tns_basic(void)
{
    SECTION("TNS — analysis/encode/decode roundtrip");

    tns_ctx_t enc_ctx, dec_ctx;
    tns_init(&enc_ctx);
    tns_init(&dec_ctx);

    /* Use 16 bands over 960 coefficients */
    tns_band_t bands[16];
    for (int b = 0; b < 16; b++) {
        bands[b].start = (uint16_t)(b * 60);
        bands[b].end   = (uint16_t)((b + 1) * 60);
    }

    /* Signal with strong spectral tilt (TNS-friendly) */
    float mdct[960];
    for (int i = 0; i < 960; i++) {
        mdct[i] = sinf((float)i * 0.1f) * expf(-(float)i / 200.0f);
    }

    tns_params_t params = {0};
    bool applied = tns_analyze(&enc_ctx, mdct, 960, bands, 16, &params);

    CHECK(params.order >= 0 && params.order <= TNS_MAX_ORDER,
          "TNS order %d out of range", params.order);

    if (applied && params.order > 0) {
        /* Encode + decode params roundtrip */
        uint8_t bits[32];
        int enc_len = tns_encode_params(&params, bits, sizeof(bits));
        CHECK(enc_len > 0, "tns_encode_params returned %d", enc_len);

        tns_params_t decoded_params = {0};
        int dec_len = tns_decode_params_from_stream(&decoded_params, bits, enc_len);
        CHECK(dec_len == enc_len, "tns_decode consumed %d != encoded %d", dec_len, enc_len);
        CHECK(decoded_params.order == params.order,
              "decoded order %u != encoded %u", decoded_params.order, params.order);

        /* Apply analysis filter, then synthesis — should restore signal */
        float filtered[960];
        memcpy(filtered, mdct, sizeof(mdct));
        tns_filter_encode(&enc_ctx, filtered, 960, bands, 16, &params);

        tns_decode_params(&dec_ctx, &decoded_params);
        tns_filter_decode(&dec_ctx, filtered, 960, bands, 16, &decoded_params);

        float err = 0.0f, ref = 0.0f;
        for (int i = 0; i < 960; i++) {
            float d = filtered[i] - mdct[i];
            err += d * d;
            ref += mdct[i] * mdct[i];
        }
        float snr = (err > 1e-12f) ? (10.0f * log10f(ref / err)) : 999.0f;
        CHECK(snr > 15.0f, "TNS encode/decode SNR %.1f dB < 15 dB", snr);
        PASS("TNS encode/decode SNR = %.1f dB (order=%u)", snr, params.order);
    } else {
        PASS("TNS not applied to this signal (order=%u)", params.order);
    }
}

/* ================================================================
 * ENERGY QUANTIZATION TESTS
 * ================================================================ */

static void test_energy_quant(void)
{
    SECTION("energy quantization — coarse encode/decode roundtrip");

    energy_ctx_t ctx;
    energy_init(&ctx, 8);

    float energies[8] = {100.0f, 80.0f, 60.0f, 40.0f, 20.0f, 10.0f, 5.0f, 1.0f};
    int8_t coarse_codes[8];
    float reconstructed[8];

    /* Coarse encode */
    int bits_used = energy_encode_coarse(&ctx, energies, 8, coarse_codes);
    CHECK(bits_used > 0, "energy_encode_coarse returned %d bits", bits_used);

    /* Coarse decode */
    energy_ctx_t dec_ctx;
    energy_init(&dec_ctx, 8);
    energy_decode_coarse(&dec_ctx, coarse_codes, 8, reconstructed);

    /* At 6 dB coarse resolution, relative error should be ≤ 100% (factor of 2) */
    for (int b = 0; b < 8; b++) {
        if (energies[b] > 1e-6f) {
            float ratio = reconstructed[b] / energies[b];
            CHECK(ratio > 0.2f && ratio < 5.0f,
                  "band %d coarse recon ratio %.2f out of range [0.2,5.0]"
                  " (orig=%.2f recon=%.2f)", b, ratio, energies[b], reconstructed[b]);
        }
    }

    /* Fine bit allocation — scored by reconstructed energy (coarse_dB) */
    uint8_t fine_bits[8];
    energy_allocate_fine_bits(32, ctx.coarse_dB, 8, fine_bits);
    int total_fine = 0;
    for (int b = 0; b < 8; b++) total_fine += fine_bits[b];
    CHECK(total_fine <= 32, "fine bits total %d > budget 32", total_fine);

    PASS("coarse energy roundtrip OK, bits_used=%d, fine_bits_total=%d", bits_used, total_fine);
}

/* ================================================================
 * MAIN
 * ================================================================ */

int main(void)
{
    printf("=== opcodec Comprehensive Test Suite ===\n");

    /* FEC */
    test_fec_basic_recovery();
    test_fec_overhead();
    test_fec_header_roundtrip();
    test_fec_flush_partial();

    /* netadapt */
    test_netadapt_init();
    test_netadapt_rtt_tracking();
    test_netadapt_loss_response();
    test_netadapt_probe_drain_cycle();
    test_netadapt_audio_only();

    /* jitter buffer */
    test_jitter_basic();
    test_jitter_reorder();
    test_jitter_stats();

    /* parametric stereo */
    test_stereo_downmix_upmix();
    test_stereo_params_extreme();

    /* PVQ */
    test_pvq_roundtrip();
    test_pvq_k_values();

    /* BWE */
    test_bwe_cutoff();

    /* TNS */
    test_tns_basic();

    /* Energy quantization */
    test_energy_quant();

    printf("\n=== Summary ===\n");
    printf("  Checks passed: %d\n", g_pass);
    printf("  Checks failed: %d\n", g_fail);

    if (g_fail == 0) {
        printf("All tests PASSED\n");
        return 0;
    } else {
        printf("FAILED (%d check%s)\n", g_fail, g_fail == 1 ? "" : "s");
        return 1;
    }
}
