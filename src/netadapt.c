/*
 * opcodec/netadapt.c — Network-adaptive bitrate controller implementation
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/netadapt.h"
#include <math.h>
#include <string.h>

/* TCP-style EWMA smoothing constants */
#define SRTT_ALPHA    0.125f  /* smoothing factor for SRTT */
#define RTTVAR_BETA   0.25f   /* smoothing factor for RTT variance */

/* Loss smoothing */
#define LOSS_ALPHA    0.1f    /* smoothing factor for loss rate */

/* Bandwidth estimation parameters */
#define BW_ALPHA      0.2f    /* smoothing factor for bandwidth */
#define BW_VARIANCE_MIN 0.1f  /* minimum variance to maintain uncertainty */

/* State machine timing (milliseconds) */
#define PROBE_DURATION_MS    3000   /* how long to probe higher bitrate */
#define RECOVERY_MIN_MS      5000   /* minimum time in recovery before stable */
#define PROBE_HOLD_MS        8000   /* min time stable before probing again */

/* Bitrate adaptation parameters */
#define PROBE_INCREASE       1.1f   /* 10% increase when probing */
#define DRAIN_DECREASE       0.7f   /* 30% decrease when draining */
#define RECOVERY_INCREASE    1.05f  /* 5% increase during recovery */

/* Quality mapping thresholds (bits per second) */
#define AUDIO_LOW_THRESHOLD    16000   /* < 16 kbps = LOW */
#define AUDIO_NORMAL_THRESHOLD 32000   /* < 32 kbps = NORMAL */
#define AUDIO_HIGH_THRESHOLD   64000   /* < 64 kbps = HIGH, >= 64 kbps = ULTRA */

/* Video minimum bitrate threshold */
#define VIDEO_MIN_BITRATE      48000   /* minimum 48 kbps for video */

/* FEC level thresholds based on loss rate */
#define FEC_NONE_THRESHOLD     0.01f   /* < 1% loss = no FEC */
#define FEC_LOW_THRESHOLD      0.03f   /* < 3% loss = low FEC */
#define FEC_MEDIUM_THRESHOLD   0.05f   /* < 5% loss = medium FEC */
#define FEC_HIGH_THRESHOLD     0.10f   /* < 10% loss = high FEC */
/* >= 10% loss = max FEC */

/* Helper function to add sample to ring buffer */
static void add_to_history(float *buffer, uint8_t *idx, uint8_t *count,
                          uint8_t max_size, float value) {
    buffer[*idx] = value;
    *idx = (*idx + 1) % max_size;
    if (*count < max_size) {
        (*count)++;
    }
}

/* Helper function to compute variance from history */
__attribute__((unused))
static float compute_variance(const float *buffer, uint8_t count, float mean) {
    if (count < 2) return 0.0f;

    float variance = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        float diff = buffer[i] - mean;
        variance += diff * diff;
    }
    return variance / (count - 1);
}

void netadapt_init(netadapt_ctx_t *ctx, uint32_t min_bitrate,
                   uint32_t max_bitrate, bool audio_only) {
    memset(ctx, 0, sizeof(*ctx));

    /* Configuration */
    ctx->min_bitrate = min_bitrate;
    ctx->max_bitrate = max_bitrate;
    ctx->audio_min_bitrate = AUDIO_LOW_THRESHOLD;  /* Reserve minimum for audio */
    ctx->audio_only_mode = audio_only;

    /* Initial smoothed metrics */
    ctx->srtt_ms = 50.0f;      /* start with 50ms RTT assumption */
    ctx->rtt_var = 10.0f;      /* moderate initial variance */
    ctx->loss_rate = 0.0f;     /* optimistic start */
    ctx->jitter_ms = 0.0f;

    /* Initial bandwidth estimate */
    ctx->bw_estimate = max_bitrate / 2.0f;  /* conservative start */
    ctx->bw_variance = max_bitrate / 4.0f;  /* moderate uncertainty */
    ctx->bw_min = min_bitrate;
    ctx->bw_max = max_bitrate;

    /* State machine */
    ctx->state = NET_STATE_STABLE;
    ctx->state_enter_time = 0;
    ctx->last_loss_time = 0;
    ctx->probe_start_time = 0;
    ctx->last_probe_end_ms = 0;

    /* Initial recommendations */
    ctx->target_bitrate = min_bitrate;
    ctx->audio_quality = 0;         /* OPVOX_QUALITY_LOW */
    ctx->audio_sample_rate = 8000;  /* conservative start */
    ctx->video_quality = 0;
    ctx->fec_level = 0;             /* OPFEC_LEVEL_NONE */
    ctx->video_enabled = !audio_only && (max_bitrate > VIDEO_MIN_BITRATE);
    ctx->burst_loss_detected = false;
    ctx->consecutive_lost_intervals = 0;

    ctx->now_ms = 0;
}

void netadapt_update_rtt(netadapt_ctx_t *ctx, float rtt_ms, uint32_t now_ms) {
    ctx->now_ms = now_ms;

    /* Add to history */
    add_to_history(ctx->rtt_history, &ctx->hist_idx, &ctx->hist_count,
                   NETADAPT_HISTORY, rtt_ms);

    /* TCP-style SRTT and RTTVAR update */
    if (ctx->hist_count == 1) {
        /* First measurement */
        ctx->srtt_ms = rtt_ms;
        ctx->rtt_var = rtt_ms / 2.0f;
    } else {
        /* EWMA smoothing */
        float err = rtt_ms - ctx->srtt_ms;
        ctx->srtt_ms += SRTT_ALPHA * err;
        ctx->rtt_var = (1.0f - RTTVAR_BETA) * ctx->rtt_var + RTTVAR_BETA * fabsf(err);
    }

    /* Update jitter (absolute deviation from SRTT) */
    float jitter_sample = fabsf(rtt_ms - ctx->srtt_ms);
    if (ctx->hist_count == 1) {
        ctx->jitter_ms = jitter_sample;
    } else {
        ctx->jitter_ms = (1.0f - SRTT_ALPHA) * ctx->jitter_ms + SRTT_ALPHA * jitter_sample;
    }
}

void netadapt_update_loss(netadapt_ctx_t *ctx, uint32_t packets_sent,
                          uint32_t packets_lost, uint32_t now_ms) {
    ctx->now_ms = now_ms;

    if (packets_sent == 0) return;

    float current_loss = (float)packets_lost / packets_sent;

    /* Add to history */
    add_to_history(ctx->loss_history, &ctx->hist_idx, &ctx->hist_count,
                   NETADAPT_HISTORY, current_loss);

    /* EWMA smoothing */
    if (ctx->hist_count == 1) {
        ctx->loss_rate = current_loss;
    } else {
        ctx->loss_rate = (1.0f - LOSS_ALPHA) * ctx->loss_rate + LOSS_ALPHA * current_loss;
    }

    /* Track time of loss events for state machine */
    if (packets_lost > 0) {
        ctx->last_loss_time = now_ms;
        ctx->consecutive_lost_intervals++;
    } else {
        ctx->consecutive_lost_intervals = 0;
    }

    /* Burst detected when loss appears in 2+ consecutive intervals */
    ctx->burst_loss_detected = (ctx->consecutive_lost_intervals >= 2);
}

void netadapt_update_ack(netadapt_ctx_t *ctx, uint32_t bytes_acked,
                         float rtt_ms, uint32_t now_ms) {
    ctx->now_ms = now_ms;

    if (bytes_acked == 0 || rtt_ms <= 0.0f) return;

    /* Compute throughput sample (bits per second) */
    float throughput_sample = (bytes_acked * 8.0f * 1000.0f) / rtt_ms;

    /* Kalman-like bandwidth estimation */
    float prediction_error = throughput_sample - ctx->bw_estimate;
    float kalman_gain = ctx->bw_variance / (ctx->bw_variance + ctx->rtt_var * 1000.0f);

    /* Update estimate and variance */
    ctx->bw_estimate += kalman_gain * prediction_error;
    ctx->bw_variance = (1.0f - kalman_gain) * ctx->bw_variance;

    /* Maintain minimum variance to keep some uncertainty */
    if (ctx->bw_variance < BW_VARIANCE_MIN) {
        ctx->bw_variance = BW_VARIANCE_MIN;
    }

    /* Update observed min/max */
    if (throughput_sample < ctx->bw_min) {
        ctx->bw_min = throughput_sample;
    }
    if (throughput_sample > ctx->bw_max) {
        ctx->bw_max = throughput_sample;
    }

    /* Clamp estimate to reasonable bounds */
    if (ctx->bw_estimate < ctx->min_bitrate) {
        ctx->bw_estimate = ctx->min_bitrate;
    }
    if (ctx->bw_estimate > ctx->max_bitrate) {
        ctx->bw_estimate = ctx->max_bitrate;
    }
}

static void distribute_bitrate(netadapt_ctx_t *ctx) {
    uint32_t total_bitrate = ctx->target_bitrate;

    /* Always reserve minimum for audio */
    uint32_t audio_bitrate = ctx->audio_min_bitrate;
    if (audio_bitrate > total_bitrate) {
        audio_bitrate = total_bitrate;
    }

    /* Map audio bitrate to quality level and sample rate.
     * Sample rate ladder: 8k/16k/32k/48k chosen to match the bitrate tier
     * that each quality level actually uses. */
    if (audio_bitrate < AUDIO_LOW_THRESHOLD) {
        ctx->audio_quality = 0;       /* OPVOX_QUALITY_LOW */
        ctx->audio_sample_rate = 8000;
    } else if (audio_bitrate < AUDIO_NORMAL_THRESHOLD) {
        ctx->audio_quality = 1;       /* OPVOX_QUALITY_NORMAL */
        ctx->audio_sample_rate = 16000;
    } else if (audio_bitrate < AUDIO_HIGH_THRESHOLD) {
        ctx->audio_quality = 2;       /* OPVOX_QUALITY_HIGH */
        ctx->audio_sample_rate = 32000;
    } else {
        ctx->audio_quality = 3;       /* OPVOX_QUALITY_ULTRA */
        ctx->audio_sample_rate = 48000;
    }

    /* Determine FEC level based on loss rate.
     * Burst loss: bump the level up by one step so interleaved FEC has
     * enough redundancy to cover a run of consecutive losses. */
    if (ctx->loss_rate < FEC_NONE_THRESHOLD) {
        ctx->fec_level = 0;  /* OPFEC_LEVEL_NONE */
    } else if (ctx->loss_rate < FEC_LOW_THRESHOLD) {
        ctx->fec_level = 1;  /* OPFEC_LEVEL_LOW */
    } else if (ctx->loss_rate < FEC_MEDIUM_THRESHOLD) {
        ctx->fec_level = 2;  /* OPFEC_LEVEL_MEDIUM */
    } else if (ctx->loss_rate < FEC_HIGH_THRESHOLD) {
        ctx->fec_level = 3;  /* OPFEC_LEVEL_HIGH */
    } else {
        ctx->fec_level = 4;  /* OPFEC_LEVEL_MAX */
    }

    /* Burst loss needs one extra level of redundancy */
    if (ctx->burst_loss_detected && ctx->fec_level < 4) {
        ctx->fec_level++;
    }

    /* Calculate FEC overhead (approximate) */
    float fec_overhead = 0.0f;
    switch (ctx->fec_level) {
        case 1: fec_overhead = 0.25f; break;  /* 25% overhead */
        case 2: fec_overhead = 0.33f; break;  /* 33% overhead */
        case 3: fec_overhead = 0.50f; break;  /* 50% overhead */
        case 4: fec_overhead = 1.00f; break;  /* 100% overhead */
        default: fec_overhead = 0.0f; break;
    }

    /* Calculate available bitrate for video after audio and FEC */
    uint32_t remaining = total_bitrate - audio_bitrate;
    uint32_t video_bitrate = (uint32_t)(remaining / (1.0f + fec_overhead));

    /* Enable video if we have sufficient bitrate and not audio-only mode */
    ctx->video_enabled = !ctx->audio_only_mode && (video_bitrate >= VIDEO_MIN_BITRATE);

    if (ctx->video_enabled) {
        /* Map video bitrate to quality (0-100) linearly */
        if (video_bitrate >= ctx->max_bitrate / 2) {
            ctx->video_quality = 100;
        } else {
            ctx->video_quality = (uint8_t)((video_bitrate * 100) / (ctx->max_bitrate / 2));
            if (ctx->video_quality > 100) ctx->video_quality = 100;
        }
    } else {
        ctx->video_quality = 0;
    }
}

void netadapt_evaluate(netadapt_ctx_t *ctx, uint32_t now_ms) {
    ctx->now_ms = now_ms;

    /* State machine for bitrate adaptation */
    switch (ctx->state) {
        case NET_STATE_STABLE: {
            /* Check if we can try probing higher bitrate.
             * Require PROBE_HOLD_MS since the last probe/drain cycle to
             * prevent rapid thrashing after recovering from congestion. */
            uint32_t time_since_probe = now_ms - ctx->last_probe_end_ms;
            if (ctx->loss_rate < 0.02f &&           /* < 2% loss */
                ctx->rtt_var < 20.0f &&              /* RTT is stable */
                !ctx->burst_loss_detected &&         /* no burst in flight */
                time_since_probe >= PROBE_HOLD_MS && /* held off long enough */
                ctx->target_bitrate < ctx->bw_estimate * 0.8f) {  /* headroom */

                ctx->state = NET_STATE_PROBING;
                ctx->state_enter_time = now_ms;
                ctx->probe_start_time = now_ms;

                /* Increase target bitrate by 10% */
                uint32_t new_target = (uint32_t)(ctx->target_bitrate * PROBE_INCREASE);
                if (new_target > ctx->max_bitrate) {
                    new_target = ctx->max_bitrate;
                }
                ctx->target_bitrate = new_target;
            }
            break;
        }

        case NET_STATE_PROBING: {
            uint32_t probe_duration = now_ms - ctx->probe_start_time;

            if (probe_duration >= PROBE_DURATION_MS) {
                /* Probe period finished */
                if (ctx->loss_rate > 0.03f ||  /* loss increased */
                    ctx->srtt_ms > 100.0f) {   /* RTT too high */

                    /* Probe failed, drain aggressively */
                    ctx->state = NET_STATE_DRAINING;
                    ctx->state_enter_time = now_ms;
                } else {
                    /* Probe succeeded, adopt new rate */
                    ctx->state = NET_STATE_STABLE;
                    ctx->state_enter_time = now_ms;
                    ctx->last_probe_end_ms = now_ms;
                }
            } else if (ctx->loss_rate > 0.05f || ctx->burst_loss_detected) {
                /* Early abort on significant loss or burst */
                ctx->state = NET_STATE_DRAINING;
                ctx->state_enter_time = now_ms;
            }
            break;
        }

        case NET_STATE_DRAINING: {
            /* Multiplicative decrease */
            uint32_t new_target = (uint32_t)(ctx->target_bitrate * DRAIN_DECREASE);
            if (new_target < ctx->min_bitrate) {
                new_target = ctx->min_bitrate;
            }
            ctx->target_bitrate = new_target;

            /* Record when this drain completed for probe hysteresis */
            ctx->last_probe_end_ms = now_ms;

            /* Move to recovery */
            ctx->state = NET_STATE_RECOVERY;
            ctx->state_enter_time = now_ms;
            break;
        }

        case NET_STATE_RECOVERY: {
            uint32_t recovery_duration = now_ms - ctx->state_enter_time;

            if (recovery_duration >= RECOVERY_MIN_MS) {
                if (ctx->loss_rate < 0.02f) {
                    /* Recovery successful */
                    ctx->state = NET_STATE_STABLE;
                    ctx->state_enter_time = now_ms;
                } else {
                    /* Additive increase during recovery */
                    uint32_t new_target = (uint32_t)(ctx->target_bitrate * RECOVERY_INCREASE);
                    if (new_target > ctx->bw_estimate * 0.6f) {
                        new_target = (uint32_t)(ctx->bw_estimate * 0.6f);  /* conservative limit */
                    }
                    if (new_target > ctx->max_bitrate) {
                        new_target = ctx->max_bitrate;
                    }
                    ctx->target_bitrate = new_target;
                }
            }

            /* If loss appears again during recovery, go back to draining */
            if (ctx->loss_rate > 0.04f) {
                ctx->state = NET_STATE_DRAINING;
                ctx->state_enter_time = now_ms;
            }
            break;
        }
    }

    /* Clamp target bitrate to configured bounds */
    if (ctx->target_bitrate < ctx->min_bitrate) {
        ctx->target_bitrate = ctx->min_bitrate;
    }
    if (ctx->target_bitrate > ctx->max_bitrate) {
        ctx->target_bitrate = ctx->max_bitrate;
    }

    /* Distribute bitrate across audio/video/FEC */
    distribute_bitrate(ctx);
}

void netadapt_report_burst(netadapt_ctx_t *ctx, uint16_t burst_len, uint32_t now_ms)
{
    if (!ctx)
        return;

    ctx->now_ms = now_ms;
    ctx->burst_loss_detected = (burst_len >= 2);

    if (burst_len >= 2) {
        ctx->last_loss_time = now_ms;
        ctx->consecutive_lost_intervals += burst_len;

        /* Boost FEC level for burst protection, but cap at MAX */
        if (ctx->fec_level < 4) {
            ctx->fec_level++;
        }

        /* Abort probing immediately if we're mid-probe */
        if (ctx->state == NET_STATE_PROBING) {
            ctx->state = NET_STATE_DRAINING;
            ctx->state_enter_time = now_ms;
            ctx->last_probe_end_ms = now_ms;
        }
    }
}

/* ── PNDT implementation ─────────────────────────────────────────────────── */

/* Sigmoid activation */
static float pndt_sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

/* Tanh activation (uses libm) */
static float pndt_tanh(float x) {
    return tanhf(x);
}

/* LCG for weight initialization */
static float pndt_rand(uint32_t *seed) {
    *seed = *seed * 1664525u + 1013904223u;
    return ((float)(*seed >> 16) / 32768.0f) - 1.0f;  /* [-1, 1] */
}

void pndt_init(pndt_ctx_t *ctx, uint32_t seed)
{
    memset(ctx, 0, sizeof(*ctx));
    uint32_t rng = seed ^ 0xDEADBEEFu;

    /* Xavier-like initialization: scale by 1/sqrt(fan_in) */
    const float wscale = 0.3f;  /* conservative for online learning stability */

    for (int i = 0; i < PNDT_HIDDEN_SIZE; i++) {
        for (int j = 0; j < PNDT_INPUT_SIZE; j++) {
            ctx->Wz[i][j] = pndt_rand(&rng) * wscale;
            ctx->Wr[i][j] = pndt_rand(&rng) * wscale;
            ctx->Wn[i][j] = pndt_rand(&rng) * wscale;
        }
        for (int j = 0; j < PNDT_HIDDEN_SIZE; j++) {
            ctx->Uz[i][j] = pndt_rand(&rng) * wscale;
            ctx->Ur[i][j] = pndt_rand(&rng) * wscale;
            ctx->Un[i][j] = pndt_rand(&rng) * wscale;
        }
        ctx->bz[i] = 0.0f;
        ctx->br[i] = 0.0f;
        ctx->bn[i] = 0.0f;
        for (int k = 0; k < 2; k++)
            ctx->Wo[k][i] = pndt_rand(&rng) * wscale;
    }
    ctx->bo[0] = 0.0f;
    ctx->bo[1] = 50.0f;  /* bias rtt toward 50ms default */

    ctx->initialized = true;
}

void pndt_observe(pndt_ctx_t *ctx, float loss_rate, float rtt_ms, float bw_util)
{
    if (!ctx->initialized) return;

    /* Normalize inputs to ~[0,1] range */
    float x[PNDT_INPUT_SIZE];
    x[0] = loss_rate;                            /* already 0-1 */
    x[1] = rtt_ms / 500.0f;                     /* normalize: 500ms = 1.0 */
    x[2] = bw_util;                              /* already 0-1 */

    /* ── GRU forward pass ── */
    float z[PNDT_HIDDEN_SIZE], r[PNDT_HIDDEN_SIZE];
    float n[PNDT_HIDDEN_SIZE], h_new[PNDT_HIDDEN_SIZE];

    for (int i = 0; i < PNDT_HIDDEN_SIZE; i++) {
        float uz = ctx->bz[i], ur = ctx->br[i], un = ctx->bn[i];
        for (int j = 0; j < PNDT_INPUT_SIZE; j++) {
            uz += ctx->Wz[i][j] * x[j];
            ur += ctx->Wr[i][j] * x[j];
            un += ctx->Wn[i][j] * x[j];
        }
        for (int j = 0; j < PNDT_HIDDEN_SIZE; j++) {
            uz += ctx->Uz[i][j] * ctx->h[j];
            ur += ctx->Ur[i][j] * ctx->h[j];
        }
        z[i] = pndt_sigmoid(uz);
        r[i] = pndt_sigmoid(ur);

        /* Reset-gated recurrent part for candidate */
        float gated_h = 0.0f;
        for (int j = 0; j < PNDT_HIDDEN_SIZE; j++)
            gated_h += ctx->Un[i][j] * (r[j] * ctx->h[j]);
        n[i] = pndt_tanh(un + gated_h);
        h_new[i] = (1.0f - z[i]) * n[i] + z[i] * ctx->h[i];
    }

    /* Output projection: predict {loss, rtt} at step 1 */
    float out[2];
    for (int k = 0; k < 2; k++) {
        float v = ctx->bo[k];
        for (int i = 0; i < PNDT_HIDDEN_SIZE; i++)
            v += ctx->Wo[k][i] * h_new[i];
        out[k] = v;
    }
    /* Clamp predictions to valid range */
    float pred_loss = out[0] < 0.0f ? 0.0f : (out[0] > 1.0f ? 1.0f : out[0]);
    float pred_rtt  = out[1] < 0.0f ? 0.0f : (out[1] > 2000.0f ? 2000.0f : out[1]);

    /* Roll prediction horizon forward: [0]=nearest, [N-1]=furthest */
    for (int s = PNDT_HORIZON_STEPS - 1; s > 0; s--) {
        ctx->pred_loss[s] = ctx->pred_loss[s - 1];
        ctx->pred_rtt[s]  = ctx->pred_rtt[s - 1];
    }
    /* Blend new prediction with previous (EWMA smoothing, alpha=0.3) */
    ctx->pred_loss[0] = 0.7f * ctx->pred_loss[0] + 0.3f * pred_loss;
    ctx->pred_rtt[0]  = 0.7f * ctx->pred_rtt[0]  + 0.3f * pred_rtt;

    /* ── Online weight update (stochastic gradient, target = observation) ── */
    const float lr = 0.01f;
    /* Simple output-layer update: ∂L/∂Wo = 2*(pred-target)*h */
    float err_loss = pred_loss - loss_rate;
    float err_rtt  = (pred_rtt  - rtt_ms) / 500.0f;  /* normalized */
    for (int i = 0; i < PNDT_HIDDEN_SIZE; i++) {
        ctx->Wo[0][i] -= lr * 2.0f * err_loss * h_new[i];
        ctx->Wo[1][i] -= lr * 2.0f * err_rtt  * h_new[i];
    }
    ctx->bo[0] -= lr * 2.0f * err_loss;
    ctx->bo[1] -= lr * 2.0f * err_rtt;

    /* Commit hidden state */
    for (int i = 0; i < PNDT_HIDDEN_SIZE; i++)
        ctx->h[i] = h_new[i];

    ctx->update_count++;
}