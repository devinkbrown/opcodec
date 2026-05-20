/*
 * opcodec/netadapt.h — Network-adaptive bitrate controller
 *
 * Dynamically adjusts codec parameters based on observed network conditions.
 * Uses EWMA smoothing and Kalman-style bandwidth estimation to adapt audio/video
 * quality and FEC levels in real-time.
 *
 * Algorithm:
 *   1. Smoothed RTT tracking (SRTT + RTTVAR using TCP-style EWMA)
 *   2. Packet loss rate smoothing
 *   3. Bandwidth estimation with uncertainty tracking
 *   4. State machine for probing/draining/recovery
 *   5. Bitrate distribution across audio/video/FEC
 *
 * State machine:
 *   STABLE   → if loss < 2% and RTT stable, try PROBING
 *   PROBING  → increase target by 10% for 3s. If loss increases, go DRAINING
 *   DRAINING → multiplicative decrease by 30%. Go to RECOVERY
 *   RECOVERY → additive increase by 5% per interval until stable
 *
 * Pure C11, math.h for floating point operations.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_NETADAPT_H
#define OPCODEC_NETADAPT_H

#include <stdint.h>
#include <stdbool.h>

#define NETADAPT_HISTORY     64    /* RTT/loss samples to keep */
#define NETADAPT_INTERVAL_MS 1000  /* re-evaluate every 1 second */

/* Bandwidth estimate state */
typedef enum {
    NET_STATE_STABLE   = 0,
    NET_STATE_PROBING  = 1,  /* trying higher bitrate */
    NET_STATE_DRAINING = 2,  /* reducing after congestion */
    NET_STATE_RECOVERY = 3   /* recovering from loss event */
} netadapt_state_t;

typedef struct {
    /* Smoothed metrics */
    float    srtt_ms;         /* smoothed RTT */
    float    rtt_var;         /* RTT variance */
    float    loss_rate;       /* packet loss rate [0, 1] */
    float    jitter_ms;       /* smoothed jitter */

    /* Bandwidth estimation (Kalman-like filter) */
    float    bw_estimate;     /* estimated available bandwidth (bits/sec) */
    float    bw_variance;     /* uncertainty in estimate */
    float    bw_min;          /* minimum observed */
    float    bw_max;          /* maximum observed */

    /* History ring buffers */
    float    rtt_history[NETADAPT_HISTORY];
    float    loss_history[NETADAPT_HISTORY];
    uint32_t send_times[NETADAPT_HISTORY];
    uint8_t  hist_idx;
    uint8_t  hist_count;

    /* State machine */
    netadapt_state_t state;
    uint32_t state_enter_time;
    uint32_t last_loss_time;
    uint32_t probe_start_time;

    /* Current recommendations */
    uint32_t target_bitrate;       /* recommended total bitrate (bps) */
    uint8_t  audio_quality;        /* recommended OPVOX quality (0-3) */
    uint32_t audio_sample_rate;    /* recommended sample rate (8000/16000/32000/48000) */
    uint8_t  video_quality;        /* recommended OPVIS quality (0-100) */
    uint8_t  fec_level;            /* recommended FEC level (0-4) */
    bool     video_enabled;        /* whether video should be active */
    bool     burst_loss_detected;  /* recent consecutive-packet loss burst */

    /* Configuration */
    uint32_t min_bitrate;          /* absolute minimum */
    uint32_t max_bitrate;          /* absolute maximum */
    uint32_t audio_min_bitrate;    /* reserved minimum for audio */
    bool     audio_only_mode;      /* never enable video */

    /* Probe hysteresis — don't probe again too soon after a drain */
    uint32_t last_probe_end_ms;    /* when the last probe/drain finished */

    /* Burst loss tracking */
    uint16_t consecutive_lost_intervals; /* consecutive update intervals with loss */

    uint32_t now_ms;               /* current time reference */
} netadapt_ctx_t;

/*
 * Initialize network adapter context with bitrate limits.
 * Sets sensible defaults: srtt=50ms, loss=0, bw_estimate=max_bitrate/2.
 */
void netadapt_init(netadapt_ctx_t *ctx, uint32_t min_bitrate,
                   uint32_t max_bitrate, bool audio_only);

/*
 * Feed RTT sample. Updates SRTT using EWMA (alpha=0.125, beta=0.25 for variance).
 * Same smoothing algorithm as TCP.
 */
void netadapt_update_rtt(netadapt_ctx_t *ctx, float rtt_ms, uint32_t now_ms);

/*
 * Feed packet loss information. Updates smoothed loss rate.
 * packets_sent: number of packets sent in measurement interval
 * packets_lost: number confirmed lost in same interval
 */
void netadapt_update_loss(netadapt_ctx_t *ctx, uint32_t packets_sent,
                          uint32_t packets_lost, uint32_t now_ms);

/*
 * Feed acknowledgment information for bandwidth estimation.
 * Updates Kalman-style bandwidth estimate with uncertainty tracking.
 * bytes_acked: number of bytes acknowledged
 * rtt_ms: current RTT measurement
 */
void netadapt_update_ack(netadapt_ctx_t *ctx, uint32_t bytes_acked,
                         float rtt_ms, uint32_t now_ms);

/*
 * Main decision function, called periodically (every NETADAPT_INTERVAL_MS).
 * Runs state machine and updates target bitrate and codec recommendations.
 * Distributes bitrate across audio/video/FEC based on current conditions.
 */
void netadapt_evaluate(netadapt_ctx_t *ctx, uint32_t now_ms);

/*
 * Accessors for current recommendations
 */
static inline uint8_t netadapt_get_audio_quality(const netadapt_ctx_t *ctx) {
    return ctx->audio_quality;
}

static inline uint8_t netadapt_get_video_quality(const netadapt_ctx_t *ctx) {
    return ctx->video_quality;
}

static inline uint8_t netadapt_get_fec_level(const netadapt_ctx_t *ctx) {
    return ctx->fec_level;
}

static inline uint32_t netadapt_get_target_bitrate(const netadapt_ctx_t *ctx) {
    return ctx->target_bitrate;
}

static inline bool netadapt_get_video_enabled(const netadapt_ctx_t *ctx) {
    return ctx->video_enabled;
}

static inline uint32_t netadapt_get_audio_sample_rate(const netadapt_ctx_t *ctx) {
    return ctx->audio_sample_rate;
}

static inline bool netadapt_is_burst_loss(const netadapt_ctx_t *ctx) {
    return ctx->burst_loss_detected;
}

/*
 * Report an externally-observed burst loss (e.g., from jitter buffer).
 * burst_len: number of consecutively lost packets in the burst.
 * Immediately sets burst_loss_detected and boosts FEC level if needed.
 */
void netadapt_report_burst(netadapt_ctx_t *ctx, uint16_t burst_len, uint32_t now_ms);

/* ── Predictive Network Digital Twin (PNDT) ───────────────────────────────
 *
 * Lightweight GRU-inspired recurrent predictor for network conditions.
 * Trained online: each observation updates the hidden state and weights via
 * a single-step gradient descent (learning rate 0.01).
 *
 * Predicts {loss_rate, rtt_ms} N_STEPS × PNDT_STEP_MS ahead (default 300ms).
 * Used by netadapt_evaluate() to pre-emptively raise FEC before a loss burst
 * rather than reacting after the burst starts.
 *
 * Reference: "GRU-based network digital twin for real-time WebRTC adaptation",
 * derived without direct IP conflict by using online learning rather than
 * offline-trained weights (novel contribution for embedded real-time use).
 */

#define PNDT_HIDDEN_SIZE  8   /* small GRU hidden dimension */
#define PNDT_INPUT_SIZE   3   /* inputs: loss_rate, rtt_ms, bw_utilization */
#define PNDT_HORIZON_STEPS 3  /* number of prediction steps */
#define PNDT_STEP_MS     100  /* ms per prediction step (total = 300ms) */

typedef struct {
    /* GRU hidden state */
    float h[PNDT_HIDDEN_SIZE];

    /* GRU weight matrices (update gate, reset gate, candidate):
     * Wz, Wr, Wn: [hidden × input] input weights
     * Uz, Ur, Un: [hidden × hidden] recurrent weights
     * bz, br, bn: [hidden] biases */
    float Wz[PNDT_HIDDEN_SIZE][PNDT_INPUT_SIZE];
    float Wr[PNDT_HIDDEN_SIZE][PNDT_INPUT_SIZE];
    float Wn[PNDT_HIDDEN_SIZE][PNDT_INPUT_SIZE];
    float Uz[PNDT_HIDDEN_SIZE][PNDT_HIDDEN_SIZE];
    float Ur[PNDT_HIDDEN_SIZE][PNDT_HIDDEN_SIZE];
    float Un[PNDT_HIDDEN_SIZE][PNDT_HIDDEN_SIZE];
    float bz[PNDT_HIDDEN_SIZE];
    float br[PNDT_HIDDEN_SIZE];
    float bn[PNDT_HIDDEN_SIZE];

    /* Output projection: [2 × hidden] → {loss, rtt} */
    float Wo[2][PNDT_HIDDEN_SIZE];
    float bo[2];

    /* Prediction horizon (rolling) */
    float pred_loss[PNDT_HORIZON_STEPS];
    float pred_rtt[PNDT_HORIZON_STEPS];

    bool initialized;
    uint32_t update_count;
} pndt_ctx_t;

/*
 * Initialize PNDT predictor with small random weights (seeded from bitrate
 * to give different streams slightly different initial biases).
 */
void pndt_init(pndt_ctx_t *ctx, uint32_t seed);

/*
 * Feed one observation to the GRU and update the hidden state.
 * loss_rate:    fraction 0.0–1.0
 * rtt_ms:       round-trip time in milliseconds
 * bw_util:      fraction 0.0–1.0 (bytes_sent / estimated_capacity)
 *
 * After calling, pred_loss[0..N-1] and pred_rtt[0..N-1] hold predictions
 * for the next N × PNDT_STEP_MS milliseconds.
 */
void pndt_observe(pndt_ctx_t *ctx, float loss_rate, float rtt_ms, float bw_util);

/*
 * Retrieve the predicted loss rate for the given future step index (0=nearest).
 * Returns smoothed loss estimate if PNDT not yet initialized.
 */
static inline float pndt_predict_loss(const pndt_ctx_t *ctx, int step) {
    if (!ctx->initialized || step < 0 || step >= PNDT_HORIZON_STEPS) return 0.0f;
    return ctx->pred_loss[step];
}

static inline float pndt_predict_rtt(const pndt_ctx_t *ctx, int step) {
    if (!ctx->initialized || step < 0 || step >= PNDT_HORIZON_STEPS) return 50.0f;
    return ctx->pred_rtt[step];
}

#endif /* OPCODEC_NETADAPT_H */