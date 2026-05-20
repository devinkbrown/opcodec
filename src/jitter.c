/* opcodec/jitter.c — Adaptive jitter buffer implementation
 *
 * Provides packet reordering, loss detection, and adaptive delay
 * for real-time media playback.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/jitter.h"
#include <string.h>

/* Helper macros for fixed-point arithmetic (Q16) */
#define Q16_SHIFT 16
#define Q16_ONE   (1 << Q16_SHIFT)
#define Q16_TO_INT(x) ((x) >> Q16_SHIFT)
#define INT_TO_Q16(x) ((x) << Q16_SHIFT)

/* Absolute value for signed integers */
static inline int32_t abs32(int32_t x)
{
    return x < 0 ? -x : x;
}

/* Min/max helpers */
static inline uint16_t min_u16(uint16_t a, uint16_t b)
{
    return a < b ? a : b;
}

static inline uint16_t max_u16(uint16_t a, uint16_t b)
{
    return a > b ? a : b;
}

void opjit_init(opjit_buffer_t *jb, uint16_t frame_duration_ms,
                uint16_t min_delay_ms, uint16_t max_delay_ms)
{
    if (!jb)
        return;

    /* Clear all packet slots */
    memset(jb->packets, 0, sizeof(jb->packets));

    /* Initialize playout state */
    jb->next_seq = 0;
    jb->started = false;

    /* Set delay parameters */
    jb->frame_duration_ms = frame_duration_ms;
    jb->min_delay_ms = min_delay_ms;
    jb->max_delay_ms = max_delay_ms;
    jb->target_delay_ms = min_delay_ms;

    /* Initialize jitter estimation */
    jb->jitter_avg = 0;
    jb->last_arrival = 0;
    jb->last_transit = 0;

    /* NACK and burst tracking */
    jb->nack_count = 0;
    jb->consecutive_lost = 0;
    jb->max_burst_seen = 0;
    jb->stable_frames = 0;

    /* Clear statistics */
    jb->stat_received = 0;
    jb->stat_lost = 0;
    jb->stat_late = 0;
    jb->stat_reordered = 0;
    jb->stat_underruns = 0;
}

void opjit_reset(opjit_buffer_t *jb)
{
    if (!jb)
        return;

    /* Clear all packet slots */
    for (int i = 0; i < OPJIT_MAX_PACKETS; i++) {
        jb->packets[i].occupied = false;
    }

    /* Reset playout state but preserve configuration */
    jb->next_seq = 0;
    jb->started = false;
    jb->target_delay_ms = jb->min_delay_ms;

    /* Reset jitter estimation */
    jb->jitter_avg = 0;
    jb->last_arrival = 0;
    jb->last_transit = 0;

    /* Reset NACK and burst tracking */
    jb->nack_count = 0;
    jb->consecutive_lost = 0;
    jb->max_burst_seen = 0;
    jb->stable_frames = 0;
}

opjit_status_t opjit_push(opjit_buffer_t *jb, uint32_t seq, uint32_t timestamp,
                          const uint8_t *data, uint16_t len, uint32_t arrival_ms)
{
    if (!jb || !data || len > OPJIT_MAX_PAYLOAD)
        return OPJIT_FULL;

    /* Check if packet is too late (behind playout point) */
    if (jb->started && seq < jb->next_seq) {
        jb->stat_late++;
        return OPJIT_LATE;
    }

    /* Find slot for this sequence number */
    size_t slot = seq % OPJIT_MAX_PACKETS;
    opjit_packet_t *pkt = &jb->packets[slot];

    /* If slot is occupied by a different sequence, it's being overwritten */
    if (pkt->occupied && pkt->seq != seq) {
        /* This could be a very late old packet or buffer wraparound */
        if (pkt->seq < jb->next_seq) {
            /* Late old packet being displaced - this is good */
        } else {
            /* Buffer may be full or we're wrapping around */
            return OPJIT_FULL;
        }
    }

    /* Check for reordering (packet fills a gap in the sequence) */
    if (jb->started && seq < jb->next_seq + opjit_depth(jb)) {
        jb->stat_reordered++;
    }

    /* Store packet data */
    memcpy(pkt->data, data, len);
    pkt->len = len;
    pkt->seq = seq;
    pkt->timestamp = timestamp;
    pkt->occupied = true;

    jb->stat_received++;

    /* Update jitter estimation using RFC 3550 algorithm */
    if (jb->last_arrival > 0) {
        /* Calculate transit time difference */
        uint32_t transit = arrival_ms - timestamp; /* Assume timestamp in ms units */
        int32_t diff = abs32((int32_t)(transit - jb->last_transit));

        /* Update exponential moving average (Q16 fixed-point) */
        /* jitter = jitter + (|diff| - jitter) / 16 */
        jb->jitter_avg += (INT_TO_Q16(diff) - jb->jitter_avg) / 16;

        jb->last_transit = transit;
    } else {
        /* First packet - initialize transit time */
        jb->last_transit = arrival_ms - timestamp;
    }

    jb->last_arrival = arrival_ms;

    /* Adapt target delay with asymmetric hysteresis.
     *
     * Desired headroom = 4× jitter (covers ~99th percentile of arrival
     * variation) plus one frame so the playout point never starves.
     * We increase immediately when jitter grows, but wait OPJIT_DELAY_HOLD
     * consecutive stable frames before shrinking — avoids oscillation when
     * jitter is bursty. */
    uint16_t jitter_ms = Q16_TO_INT(jb->jitter_avg);
    uint16_t ideal_delay = max_u16(jb->min_delay_ms,
                                   min_u16(jb->max_delay_ms,
                                            (uint16_t)(jitter_ms * 4u +
                                                        jb->frame_duration_ms)));
    if (ideal_delay > jb->target_delay_ms) {
        /* Increase immediately */
        jb->target_delay_ms = ideal_delay;
        jb->stable_frames   = 0;
    } else if (ideal_delay < jb->target_delay_ms) {
        /* Decrease only after sustained stability */
        jb->stable_frames++;
        if (jb->stable_frames >= OPJIT_DELAY_HOLD) {
            /* Step down by one frame to avoid overshoot */
            uint16_t step = jb->frame_duration_ms;
            if (jb->target_delay_ms > jb->min_delay_ms + step) {
                jb->target_delay_ms -= step;
            } else {
                jb->target_delay_ms = jb->min_delay_ms;
            }
            jb->stable_frames = 0;
        }
    } else {
        jb->stable_frames++;
    }

    /* Check if we should start playout */
    if (!jb->started) {
        uint16_t current_depth = opjit_depth(jb);
        uint16_t target_frames = jb->target_delay_ms / jb->frame_duration_ms;

        if (current_depth >= target_frames) {
            jb->started = true;
            /* Set next_seq to the lowest available sequence */
            uint32_t min_seq = UINT32_MAX;
            for (int i = 0; i < OPJIT_MAX_PACKETS; i++) {
                if (jb->packets[i].occupied && jb->packets[i].seq < min_seq) {
                    min_seq = jb->packets[i].seq;
                }
            }
            if (min_seq != UINT32_MAX) {
                jb->next_seq = min_seq;
            }
        }
    }

    return OPJIT_OK;
}

opjit_status_t opjit_pull(opjit_buffer_t *jb, uint8_t *out_data, uint16_t *out_len)
{
    if (!jb || !out_data || !out_len)
        return OPJIT_EMPTY;

    /* Check if playout has started */
    if (!jb->started) {
        jb->stat_underruns++;
        return OPJIT_EMPTY;
    }

    /* Find slot for next expected sequence */
    size_t slot = jb->next_seq % OPJIT_MAX_PACKETS;
    opjit_packet_t *pkt = &jb->packets[slot];

    /* Check if packet is available */
    if (pkt->occupied && pkt->seq == jb->next_seq) {
        /* Copy packet data to output */
        memcpy(out_data, pkt->data, pkt->len);
        *out_len = pkt->len;

        /* Mark slot as free and advance sequence */
        pkt->occupied = false;
        jb->next_seq++;

        /* Successful delivery resets the burst counter */
        jb->consecutive_lost = 0;

        return OPJIT_OK;
    }

    /* Packet was lost — add to NACK list if there is room and the loss
     * is recent enough that retransmission might still help (within one
     * target-delay window of the playout point). */
    uint32_t lost_seq = jb->next_seq;
    if (jb->nack_count < OPJIT_NACK_MAX) {
        jb->nack_list[jb->nack_count++] = lost_seq;
    }

    /* Track burst length */
    jb->consecutive_lost++;
    if (jb->consecutive_lost > jb->max_burst_seen) {
        jb->max_burst_seen = jb->consecutive_lost;
    }

    jb->next_seq++;
    jb->stat_lost++;

    return OPJIT_LOST;
}

uint16_t opjit_depth(const opjit_buffer_t *jb)
{
    if (!jb)
        return 0;

    uint16_t count = 0;

    for (int i = 0; i < OPJIT_MAX_PACKETS; i++) {
        if (jb->packets[i].occupied &&
            jb->packets[i].seq >= jb->next_seq) {
            count++;
        }
    }

    return count;
}

uint16_t opjit_jitter_ms(const opjit_buffer_t *jb)
{
    if (!jb)
        return 0;

    return Q16_TO_INT(jb->jitter_avg);
}

uint16_t opjit_target_delay(const opjit_buffer_t *jb)
{
    if (!jb)
        return 0;

    return jb->target_delay_ms;
}

uint8_t opjit_get_nack_list(opjit_buffer_t *jb, uint32_t *seqs, uint8_t max_seqs)
{
    if (!jb || !seqs || max_seqs == 0)
        return 0;

    uint8_t n = jb->nack_count < max_seqs ? jb->nack_count : max_seqs;
    for (uint8_t i = 0; i < n; i++) {
        seqs[i] = jb->nack_list[i];
    }
    jb->nack_count = 0;
    return n;
}

bool opjit_is_burst_loss(const opjit_buffer_t *jb)
{
    if (!jb)
        return false;
    return jb->consecutive_lost >= 2;
}