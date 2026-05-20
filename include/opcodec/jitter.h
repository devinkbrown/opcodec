/* opcodec/jitter.h — Adaptive jitter buffer for real-time media
 *
 * Handles packet reordering, loss detection, and adaptive delay.
 * Used by both audio and video decoders.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_JITTER_H
#define OPCODEC_JITTER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Maximum packets the buffer can hold */
#define OPJIT_MAX_PACKETS    64
/* Maximum packet payload size */
#define OPJIT_MAX_PAYLOAD    2048
/* Maximum NACK entries tracked simultaneously */
#define OPJIT_NACK_MAX       16
/* Frames of low-jitter before we shrink target delay (hysteresis) */
#define OPJIT_DELAY_HOLD     12

typedef enum {
    OPJIT_OK           = 0,   /* packet available */
    OPJIT_EMPTY        = 1,   /* no packet ready yet (buffer underrun) */
    OPJIT_LOST         = 2,   /* packet was lost (gap in sequence) */
    OPJIT_LATE         = 3,   /* packet arrived too late, discarded */
    OPJIT_FULL         = 4,   /* buffer is full, packet dropped */
} opjit_status_t;

typedef struct {
    uint8_t  data[OPJIT_MAX_PAYLOAD];
    uint16_t len;
    uint32_t seq;          /* sequence number */
    uint32_t timestamp;    /* RTP-style timestamp (sample count or frame count) */
    bool     occupied;
} opjit_packet_t;

typedef struct opjit_buffer {
    opjit_packet_t packets[OPJIT_MAX_PACKETS];

    /* Playout state */
    uint32_t next_seq;           /* next expected sequence number for playout */
    bool     started;            /* have we started playout? */

    /* Adaptive delay */
    uint16_t target_delay_ms;    /* current target buffering delay */
    uint16_t min_delay_ms;       /* minimum delay (floor) */
    uint16_t max_delay_ms;       /* maximum delay (ceiling) */
    uint16_t frame_duration_ms;  /* duration of one frame in ms */

    /* Jitter estimation (exponential moving average) */
    int32_t  jitter_avg;         /* average jitter in timestamp units (Q16 fixed-point) */
    uint32_t last_arrival;       /* timestamp of last packet arrival */
    uint32_t last_transit;       /* last transit time difference */

    /* NACK — recently declared-lost sequences still worth requesting */
    uint32_t nack_list[OPJIT_NACK_MAX];
    uint8_t  nack_count;

    /* Burst loss detection */
    uint16_t consecutive_lost;   /* current run of consecutive lost sequences */
    uint16_t max_burst_seen;     /* longest burst since last reset */

    /* Delay hysteresis — increase immediately, decrease slowly */
    uint16_t stable_frames;      /* consecutive frames with acceptable jitter */

    /* Statistics */
    uint32_t stat_received;
    uint32_t stat_lost;
    uint32_t stat_late;
    uint32_t stat_reordered;
    uint32_t stat_underruns;
} opjit_buffer_t;

/*
 * Initialize jitter buffer.
 * frame_duration_ms: duration of one frame (e.g., 20 for 20ms audio frames)
 * min_delay_ms: minimum buffering delay (e.g., 40ms = 2 frames)
 * max_delay_ms: maximum buffering delay (e.g., 200ms = 10 frames)
 */
void opjit_init(opjit_buffer_t *jb, uint16_t frame_duration_ms,
                uint16_t min_delay_ms, uint16_t max_delay_ms);

/*
 * Reset buffer state (flush all packets).
 */
void opjit_reset(opjit_buffer_t *jb);

/*
 * Push a received packet into the buffer.
 * arrival_ms: current time in milliseconds (for jitter calculation)
 * Returns OPJIT_OK on success, OPJIT_FULL if buffer is full, OPJIT_LATE if too late.
 */
opjit_status_t opjit_push(opjit_buffer_t *jb, uint32_t seq, uint32_t timestamp,
                           const uint8_t *data, uint16_t len, uint32_t arrival_ms);

/*
 * Pull the next packet for playout.
 * out_data: receives packet payload (must be OPJIT_MAX_PAYLOAD bytes)
 * out_len: receives payload length
 * Returns OPJIT_OK if packet available, OPJIT_EMPTY if buffering,
 * OPJIT_LOST if packet was lost (caller should do PLC).
 */
opjit_status_t opjit_pull(opjit_buffer_t *jb, uint8_t *out_data, uint16_t *out_len);

/*
 * Get current buffer depth in packets.
 */
uint16_t opjit_depth(const opjit_buffer_t *jb);

/*
 * Get estimated jitter in milliseconds.
 */
uint16_t opjit_jitter_ms(const opjit_buffer_t *jb);

/*
 * Get current target delay in milliseconds.
 */
uint16_t opjit_target_delay(const opjit_buffer_t *jb);

/*
 * Fill seqs[] with up to max_seqs sequence numbers the caller should NACK.
 * Returns the number of entries written. Entries are consumed on each call.
 */
uint8_t opjit_get_nack_list(opjit_buffer_t *jb, uint32_t *seqs, uint8_t max_seqs);

/*
 * Returns true if the most recent loss pattern looks like a burst
 * (consecutive_lost >= 2), suggesting FEC interleaving is warranted.
 */
bool opjit_is_burst_loss(const opjit_buffer_t *jb);

#endif /* OPCODEC_JITTER_H */