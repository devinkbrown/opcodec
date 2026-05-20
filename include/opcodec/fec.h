/*
 * Forward Error Correction (FEC) for opcodec
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 *
 * XOR-based FEC with flexible protection levels for real-time media.
 * Supports 1D and 2D FEC schemes to recover lost packets without retransmission.
 */

#ifndef OPCODEC_FEC_H
#define OPCODEC_FEC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define OPFEC_MAX_GROUP     8       /* max packets per FEC group */
#define OPFEC_MAX_PACKET    4096    /* max packet payload size */
#define OPFEC_MAX_COLS      4       /* max columns for 2D FEC */
#define OPFEC_MAX_ROWS      4       /* max rows for 2D FEC */
#define OPFEC_HEADER_SIZE   8       /* size of FEC header in bytes */
#define OPFEC_LEN_TABLE_SIZE (OPFEC_MAX_GROUP * 2)  /* length table size in FEC packet */

/* FEC protection level */
typedef enum {
    OPFEC_LEVEL_NONE   = 0,    /* No FEC (0% overhead) */
    OPFEC_LEVEL_LOW    = 1,    /* 1 FEC per 4 packets (25% overhead) */
    OPFEC_LEVEL_MEDIUM = 2,    /* 1 FEC per 3 packets (33% overhead) */
    OPFEC_LEVEL_HIGH   = 3,    /* 1 FEC per 2 packets (50% overhead) */
    OPFEC_LEVEL_MAX    = 4     /* 2D FEC, rows + columns (100% overhead) */
} opfec_level_t;

/* FEC packet type */
typedef enum {
    OPFEC_DATA = 0,     /* normal data packet */
    OPFEC_ROW  = 1,     /* row FEC packet */
    OPFEC_COL  = 2      /* column FEC packet */
} opfec_type_t;

/* FEC packet header (prepended to FEC packets)
 * Wire format: [type:1][group_id:1][group_size:1][pkt_idx:1][payload_len:2 BE][base_seq:2 BE]
 * Followed by: [len_table: group_size * 2 bytes] + [xor_payload: payload_len bytes]
 */
typedef struct {
    uint8_t   type;          /* opfec_type_t */
    uint8_t   group_id;      /* FEC group identifier */
    uint8_t   group_size;    /* number of data packets in this group */
    uint8_t   pkt_idx;       /* index of this packet within the group */
    uint16_t  payload_len;   /* length of the XOR payload (not including length table) */
    uint32_t  base_seq;      /* sequence number of first packet in group (truncated to 16 bits on wire) */
} opfec_header_t;

/* Encoder: generates FEC packets from data packets */
typedef struct {
    opfec_level_t level;

    /* Current group buffer */
    uint8_t  group_buf[OPFEC_MAX_GROUP][OPFEC_MAX_PACKET];
    uint16_t group_lens[OPFEC_MAX_GROUP];
    uint8_t  group_count;       /* packets received in current group */
    uint8_t  group_size;        /* target group size based on level */
    uint8_t  group_id;          /* incrementing group ID */
    uint32_t base_seq;          /* base sequence of current group */

    /* 2D FEC state for LEVEL_MAX */
    uint8_t  col_buf[OPFEC_MAX_COLS][OPFEC_MAX_PACKET]; /* column XOR accumulators */
    uint16_t col_max_len[OPFEC_MAX_COLS];
    uint8_t  rows_completed;    /* rows completed for current 2D block */
    bool     col_fec_pending;   /* column FEC packets need to be generated */
} opfec_encoder_t;

/* Decoder: recovers lost packets using FEC */
typedef struct {
    opfec_level_t level;

    /* Received packet buffer per group */
    uint8_t  pkt_buf[OPFEC_MAX_GROUP + 1][OPFEC_MAX_PACKET]; /* +1 for FEC pkt */
    uint16_t pkt_lens[OPFEC_MAX_GROUP + 1];
    bool     pkt_received[OPFEC_MAX_GROUP + 1]; /* which packets we have */
    uint8_t  group_size;
    uint8_t  current_group_id;
    uint32_t current_base_seq;
    bool     fec_received;      /* whether FEC packet for current group arrived */

    /* Length table from FEC packet */
    uint16_t original_lens[OPFEC_MAX_GROUP];  /* original packet lengths */

    /* Recovery output buffer */
    uint8_t  recovered[OPFEC_MAX_PACKET];
    uint16_t recovered_len;
} opfec_decoder_t;

/* ---- Encoder API ---- */

/* Initialize FEC encoder */
void opfec_enc_init(opfec_encoder_t *enc, opfec_level_t level);

/* Feed a data packet to the FEC encoder.
 * Returns:
 *   0 = packet buffered, no FEC packet ready yet
 *   1 = FEC packet ready, call opfec_enc_get_fec() to retrieve it
 *  -1 = error (invalid parameters or buffer full)
 */
int opfec_enc_feed(opfec_encoder_t *enc,
                   const uint8_t *data, uint16_t len,
                   uint32_t seq);

/* Get the generated FEC packet after opfec_enc_feed returns 1.
 * Writes the FEC packet (header + XOR payload) to out.
 * Returns total bytes written, or -1 on error. */
int opfec_enc_get_fec(opfec_encoder_t *enc,
                      uint8_t *out, size_t out_cap);

/* Flush any partial group (generate FEC for fewer than group_size packets).
 * Returns 1 if FEC packet ready, 0 if nothing to flush. */
int opfec_enc_flush(opfec_encoder_t *enc,
                    uint8_t *out, size_t out_cap);

/* ---- Decoder API ---- */

/* Initialize FEC decoder */
void opfec_dec_init(opfec_decoder_t *dec, opfec_level_t level);

/* Feed a received data packet to the decoder.
 *
 * Returns:
 *   0 = packet stored
 *  -1 = error
 */
int opfec_dec_feed_data(opfec_decoder_t *dec,
                        uint8_t group_id, uint8_t pkt_idx,
                        const uint8_t *data, uint16_t len);

/* Feed a received FEC packet to the decoder.
 *
 * Returns:
 *   0 = FEC packet stored
 *  -1 = error
 */
int opfec_dec_feed_fec(opfec_decoder_t *dec,
                       const opfec_header_t *header,
                       const uint8_t *fec_payload, uint16_t fec_len);

/* Attempt to recover a lost packet in the current group.
 *
 * Parameters:
 *   lost_idx     — index of the lost packet within the group
 *   out          — buffer to write recovered packet
 *   out_cap      — capacity
 *   out_len      — actual recovered length
 *
 * Returns:
 *   0 = successfully recovered
 *  -1 = cannot recover (too many losses or no FEC)
 */
int opfec_dec_recover(opfec_decoder_t *dec,
                      uint8_t lost_idx,
                      uint8_t *out, size_t out_cap,
                      uint16_t *out_len);

/* Check if a specific packet in the current group can be recovered.
 * Returns true if recovery is possible. */
bool opfec_dec_can_recover(const opfec_decoder_t *dec, uint8_t lost_idx);

/* Reset decoder for a new group */
void opfec_dec_new_group(opfec_decoder_t *dec, uint8_t group_id,
                         uint8_t group_size, uint32_t base_seq);

/* ---- Interleaved FEC ---- *
 *
 * Stride-2 interleaving converts burst loss into distributed loss.
 *
 *   Encoder groups:  A = even-indexed frames (0, 2, 4, 6, ...)
 *                    B = odd-indexed frames  (1, 3, 5, 7, ...)
 *
 *   A burst of 2 consecutive frames (e.g., frames 2 and 3) hits each
 *   group exactly once, so both FEC packets can recover the losses.
 *
 * On the wire, stride slot is embedded in group_id: group A produces
 * group_ids 0, 2, 4 ... (even); group B produces 1, 3, 5 ... (odd).
 * The decoder maps received FEC packets to the right slot via group_id & 1.
 *
 * Added latency = 1 extra frame (stride=2, so FEC-A emits after frames
 * 0,2,4,6 are available — i.e., after the 6th frame, ~120ms at 20ms/frame).
 */

/* Maximum stride supported */
#define OPFEC_MAX_STRIDE 2

typedef struct {
    opfec_encoder_t groups[OPFEC_MAX_STRIDE]; /* one encoder per stride slot */
    uint32_t        feed_count;               /* total data packets fed */
} opfec_interleaved_enc_t;

typedef struct {
    opfec_decoder_t groups[OPFEC_MAX_STRIDE]; /* one decoder per stride slot */
} opfec_interleaved_dec_t;

/*
 * Initialize interleaved encoder. level applies to each stride group.
 * group_ids are assigned: even to slot 0, odd to slot 1.
 */
void opfec_interleaved_enc_init(opfec_interleaved_enc_t *il,
                                opfec_level_t level);

/*
 * Feed a data packet to the interleaved encoder.
 * seq determines which stride slot (seq % OPFEC_MAX_STRIDE) receives it.
 *
 * Returns:
 *   0  = buffered, no FEC ready yet
 *   1  = FEC packet ready for slot 0, call opfec_interleaved_get_fec(il, 0, ...)
 *   2  = FEC packet ready for slot 1, call opfec_interleaved_get_fec(il, 1, ...)
 *   3  = FEC packets ready for both slots (unlikely but possible on flush)
 *  -1  = error
 */
int opfec_interleaved_feed(opfec_interleaved_enc_t *il,
                           const uint8_t *data, uint16_t len, uint32_t seq);

/*
 * Retrieve the FEC packet for the given stride slot (0 or 1).
 * Returns bytes written, or -1 on error.
 */
int opfec_interleaved_get_fec(opfec_interleaved_enc_t *il, uint8_t slot,
                               uint8_t *out, size_t out_cap);

/*
 * Flush any partial groups. Returns bitmask of slots with FEC ready (see feed).
 */
int opfec_interleaved_flush(opfec_interleaved_enc_t *il,
                             uint8_t *out0, size_t cap0,
                             uint8_t *out1, size_t cap1);

/*
 * Initialize interleaved decoder.
 */
void opfec_interleaved_dec_init(opfec_interleaved_dec_t *il,
                                opfec_level_t level);

/*
 * Feed a received data packet. slot = seq % OPFEC_MAX_STRIDE.
 */
int opfec_interleaved_dec_feed_data(opfec_interleaved_dec_t *il,
                                    uint32_t seq,
                                    uint8_t group_id, uint8_t pkt_idx,
                                    const uint8_t *data, uint16_t len);

/*
 * Feed a received FEC packet. slot is derived from group_id & 1.
 */
int opfec_interleaved_dec_feed_fec(opfec_interleaved_dec_t *il,
                                   const opfec_header_t *header,
                                   const uint8_t *fec_payload, uint16_t fec_len);

/*
 * Attempt recovery of a lost packet.
 * seq: sequence number of the lost packet.
 * pkt_idx: index within the FEC group.
 */
int opfec_interleaved_recover(opfec_interleaved_dec_t *il,
                              uint32_t seq, uint8_t pkt_idx,
                              uint8_t *out, size_t out_cap, uint16_t *out_len);

/* ---- Header serialization ---- */

/* Serialize FEC header to bytes (8 bytes) */
void opfec_header_write(const opfec_header_t *h, uint8_t *out);

/* Parse FEC header from bytes */
void opfec_header_read(opfec_header_t *h, const uint8_t *in);

/* ---- Utility functions ---- */

/* Get group size for a given protection level */
uint8_t opfec_get_group_size(opfec_level_t level);

/* Calculate overhead percentage for a protection level */
uint8_t opfec_get_overhead_pct(opfec_level_t level);

#endif /* OPCODEC_FEC_H */