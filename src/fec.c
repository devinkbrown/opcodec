/*
 * Forward Error Correction (FEC) for opcodec
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 *
 * XOR-based FEC implementation with 1D and 2D protection schemes.
 * Provides packet recovery for real-time media over lossy networks.
 */

#include "opcodec/fec.h"
#include <string.h>
#include <assert.h>

/* Group sizes for each protection level */
static const uint8_t group_sizes[] = {
    0,  /* OPFEC_LEVEL_NONE - no FEC */
    4,  /* OPFEC_LEVEL_LOW - 1 FEC per 4 packets */
    3,  /* OPFEC_LEVEL_MEDIUM - 1 FEC per 3 packets */
    2,  /* OPFEC_LEVEL_HIGH - 1 FEC per 2 packets */
    2   /* OPFEC_LEVEL_MAX - 2D FEC with 2x2 grid */
};

/* ---- Utility functions ---- */

uint8_t opfec_get_group_size(opfec_level_t level)
{
    if (level < OPFEC_LEVEL_NONE || level > OPFEC_LEVEL_MAX)
        return 0;
    return group_sizes[level];
}

uint8_t opfec_get_overhead_pct(opfec_level_t level)
{
    switch (level) {
    case OPFEC_LEVEL_NONE:   return 0;
    case OPFEC_LEVEL_LOW:    return 25;
    case OPFEC_LEVEL_MEDIUM: return 33;
    case OPFEC_LEVEL_HIGH:   return 50;
    case OPFEC_LEVEL_MAX:    return 100;
    default:                 return 0;
    }
}

/* ---- Header serialization ---- */

void opfec_header_write(const opfec_header_t *h, uint8_t *out)
{
    if (!h || !out)
        return;

    out[0] = h->type;
    out[1] = h->group_id;
    out[2] = h->group_size;
    out[3] = h->pkt_idx;

    /* Network byte order for 16-bit and 32-bit fields */
    out[4] = (h->payload_len >> 8) & 0xFF;
    out[5] = h->payload_len & 0xFF;

    /* Pack 32-bit base_seq into remaining 2 bytes (truncated) */
    out[6] = (h->base_seq >> 8) & 0xFF;
    out[7] = h->base_seq & 0xFF;
}

void opfec_header_read(opfec_header_t *h, const uint8_t *in)
{
    if (!h || !in)
        return;

    h->type = in[0];
    h->group_id = in[1];
    h->group_size = in[2];
    h->pkt_idx = in[3];

    /* Network byte order for 16-bit and 32-bit fields */
    h->payload_len = ((uint16_t)in[4] << 8) | in[5];

    /* Reconstruct 32-bit base_seq from 2 bytes (may wrap) */
    h->base_seq = ((uint32_t)in[6] << 8) | in[7];
}

/* ---- Private encoder helpers ---- */

static void xor_packets(const uint8_t *src, uint8_t *dst, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        dst[i] ^= src[i];
    }
}

static uint16_t find_max_len(const uint16_t *lens, uint8_t count)
{
    uint16_t max_len = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (lens[i] > max_len)
            max_len = lens[i];
    }
    return max_len;
}

static int generate_row_fec(opfec_encoder_t *enc, uint8_t *out, size_t out_cap)
{
    if (enc->group_count == 0)
        return 0;

    /* Find maximum packet length */
    uint16_t max_len = find_max_len(enc->group_lens, enc->group_count);

    /* Calculate total FEC packet size: header + length table + XOR payload */
    uint16_t len_table_size = enc->group_count * 2;  /* 2 bytes per length */
    size_t total_size = OPFEC_HEADER_SIZE + len_table_size + max_len;

    if (out_cap < total_size)
        return -1;

    /* Create FEC payload by XORing all packets */
    uint8_t fec_payload[max_len];
    memset(fec_payload, 0, max_len);

    for (uint8_t i = 0; i < enc->group_count; i++) {
        /* XOR up to the length of this packet */
        for (uint16_t j = 0; j < enc->group_lens[i]; j++) {
            fec_payload[j] ^= enc->group_buf[i][j];
        }
    }

    /* Build FEC header */
    opfec_header_t header = {
        .type = OPFEC_ROW,
        .group_id = enc->group_id,
        .group_size = enc->group_count,
        .pkt_idx = 0,
        .payload_len = max_len,  /* XOR payload length only */
        .base_seq = enc->base_seq
    };

    /* Write header */
    opfec_header_write(&header, out);
    uint8_t *ptr = out + OPFEC_HEADER_SIZE;

    /* Write length table (network byte order) */
    for (uint8_t i = 0; i < enc->group_count; i++) {
        *ptr++ = (enc->group_lens[i] >> 8) & 0xFF;
        *ptr++ = enc->group_lens[i] & 0xFF;
    }

    /* Write XOR payload */
    memcpy(ptr, fec_payload, max_len);

    return total_size;
}

/* ---- Encoder API ---- */

void opfec_enc_init(opfec_encoder_t *enc, opfec_level_t level)
{
    memset(enc, 0, sizeof(*enc));
    enc->level = level;
    enc->group_size = opfec_get_group_size(level);
    enc->group_id = 0;
}

int opfec_enc_feed(opfec_encoder_t *enc, const uint8_t *data, uint16_t len, uint32_t seq)
{
    if (!enc || !data || len == 0 || len > OPFEC_MAX_PACKET)
        return -1;

    if (enc->level == OPFEC_LEVEL_NONE)
        return 0;

    if (enc->group_count >= OPFEC_MAX_GROUP)
        return -1;

    /* Initialize base sequence for first packet in group */
    if (enc->group_count == 0) {
        enc->base_seq = seq;
    }

    /* Copy packet data */
    memcpy(enc->group_buf[enc->group_count], data, len);
    enc->group_lens[enc->group_count] = len;
    enc->group_count++;

    /* Update column accumulators for 2D FEC */
    if (enc->level == OPFEC_LEVEL_MAX) {
        uint8_t col = (enc->group_count - 1) % OPFEC_MAX_COLS;

        /* Initialize or extend column buffer */
        if (enc->col_max_len[col] < len) {
            /* Zero-pad the existing accumulator if needed */
            memset(enc->col_buf[col] + enc->col_max_len[col], 0,
                   len - enc->col_max_len[col]);
            enc->col_max_len[col] = len;
        }

        /* XOR this packet into the column accumulator */
        xor_packets(data, enc->col_buf[col], len);
    }

    /* Check if we have a complete group */
    if (enc->group_count >= enc->group_size) {
        return 1;  /* FEC packet ready */
    }

    return 0;  /* Not ready yet */
}

int opfec_enc_get_fec(opfec_encoder_t *enc, uint8_t *out, size_t out_cap)
{
    if (!enc || !out)
        return -1;

    int bytes_written = generate_row_fec(enc, out, out_cap);
    if (bytes_written < 0)
        return -1;

    /* For 2D FEC, mark that we completed a row */
    if (enc->level == OPFEC_LEVEL_MAX) {
        enc->rows_completed++;
        if (enc->rows_completed >= OPFEC_MAX_ROWS) {
            enc->col_fec_pending = true;
        }
    }

    /* Reset group state */
    enc->group_count = 0;
    enc->group_id++;

    return bytes_written;
}

int opfec_enc_flush(opfec_encoder_t *enc, uint8_t *out, size_t out_cap)
{
    if (!enc || !out)
        return -1;

    if (enc->group_count == 0)
        return 0;  /* Nothing to flush */

    int bytes_written = generate_row_fec(enc, out, out_cap);
    if (bytes_written < 0)
        return -1;

    /* Reset group state */
    enc->group_count = 0;
    enc->group_id++;

    return bytes_written;
}

/* ---- Decoder API ---- */

void opfec_dec_init(opfec_decoder_t *dec, opfec_level_t level)
{
    memset(dec, 0, sizeof(*dec));
    dec->level = level;
    dec->current_group_id = 0xFF;  /* Invalid marker */
}

int opfec_dec_feed_data(opfec_decoder_t *dec, uint8_t group_id, uint8_t pkt_idx,
                        const uint8_t *data, uint16_t len)
{
    if (!dec || !data || len == 0 || len > OPFEC_MAX_PACKET)
        return -1;

    if (pkt_idx >= OPFEC_MAX_GROUP)
        return -1;

    /* Switch to new group if needed */
    if (dec->current_group_id != group_id) {
        memset(dec->pkt_received, 0, sizeof(dec->pkt_received));
        dec->current_group_id = group_id;
        dec->fec_received = false;
    }

    /* Store the data packet */
    memcpy(dec->pkt_buf[pkt_idx], data, len);
    dec->pkt_lens[pkt_idx] = len;
    dec->pkt_received[pkt_idx] = true;

    return 0;
}

int opfec_dec_feed_fec(opfec_decoder_t *dec, const opfec_header_t *header,
                       const uint8_t *fec_payload, uint16_t fec_len)
{
    if (!dec || !header || !fec_payload || header->group_size > OPFEC_MAX_GROUP)
        return -1;

    /* Validate FEC payload size (length table + XOR data) */
    uint16_t len_table_size = header->group_size * 2;
    if (fec_len < len_table_size || (fec_len - len_table_size) != header->payload_len)
        return -1;

    /* Switch to new group if needed */
    if (dec->current_group_id != header->group_id) {
        memset(dec->pkt_received, 0, sizeof(dec->pkt_received));
        memset(dec->original_lens, 0, sizeof(dec->original_lens));
        dec->current_group_id = header->group_id;
        dec->fec_received = false;
    }

    dec->group_size = header->group_size;
    dec->current_base_seq = header->base_seq;

    /* Parse length table from FEC payload */
    const uint8_t *ptr = fec_payload;
    for (uint8_t i = 0; i < header->group_size; i++) {
        dec->original_lens[i] = ((uint16_t)ptr[0] << 8) | ptr[1];
        ptr += 2;
    }

    /* Store XOR payload in the FEC slot */
    memcpy(dec->pkt_buf[OPFEC_MAX_GROUP], ptr, header->payload_len);
    dec->pkt_lens[OPFEC_MAX_GROUP] = header->payload_len;
    dec->fec_received = true;

    return 0;
}

bool opfec_dec_can_recover(const opfec_decoder_t *dec, uint8_t lost_idx)
{
    if (!dec || !dec->fec_received || lost_idx >= dec->group_size)
        return false;

    /* Count missing packets */
    uint8_t missing_count = 0;
    for (uint8_t i = 0; i < dec->group_size; i++) {
        if (!dec->pkt_received[i]) {
            missing_count++;
            if (missing_count > 1)
                return false;  /* Can only recover 1 lost packet */
        }
    }

    /* Must have exactly 1 missing packet and it must be the requested one */
    return (missing_count == 1 && !dec->pkt_received[lost_idx]);
}

int opfec_dec_recover(opfec_decoder_t *dec, uint8_t lost_idx,
                      uint8_t *out, size_t out_cap, uint16_t *out_len)
{
    if (!opfec_dec_can_recover(dec, lost_idx))
        return -1;

    /* Get the original length of the lost packet from the length table */
    uint16_t original_len = dec->original_lens[lost_idx];
    if (original_len > out_cap)
        return -1;

    uint16_t fec_len = dec->pkt_lens[OPFEC_MAX_GROUP];

    /* Start with FEC payload */
    memcpy(out, dec->pkt_buf[OPFEC_MAX_GROUP], fec_len);

    /* XOR with all received packets except the lost one */
    for (uint8_t i = 0; i < dec->group_size; i++) {
        if (i == lost_idx || !dec->pkt_received[i])
            continue;

        /* XOR up to the length of the received packet */
        xor_packets(dec->pkt_buf[i], out, dec->pkt_lens[i]);
    }

    /* The recovered packet length is known from the length table */
    *out_len = original_len;
    return 0;
}

void opfec_dec_new_group(opfec_decoder_t *dec, uint8_t group_id,
                         uint8_t group_size, uint32_t base_seq)
{
    if (!dec)
        return;

    memset(dec->pkt_received, 0, sizeof(dec->pkt_received));
    memset(dec->original_lens, 0, sizeof(dec->original_lens));
    dec->current_group_id = group_id;
    dec->group_size = group_size;
    dec->current_base_seq = base_seq;
    dec->fec_received = false;
}

/* ---- Interleaved FEC implementation ---- */

void opfec_interleaved_enc_init(opfec_interleaved_enc_t *il, opfec_level_t level)
{
    if (!il)
        return;

    for (int s = 0; s < OPFEC_MAX_STRIDE; s++) {
        opfec_enc_init(&il->groups[s], level);
        /* Assign disjoint group_id namespaces: slot 0 → even, slot 1 → odd */
        il->groups[s].group_id = (uint8_t)s;
    }
    il->feed_count = 0;
}

int opfec_interleaved_feed(opfec_interleaved_enc_t *il,
                           const uint8_t *data, uint16_t len, uint32_t seq)
{
    if (!il || !data)
        return -1;

    uint8_t slot = (uint8_t)(seq % OPFEC_MAX_STRIDE);
    int r = opfec_enc_feed(&il->groups[slot], data, len, seq);
    il->feed_count++;

    if (r < 0)
        return -1;
    if (r == 1) {
        /* Advance group_id by OPFEC_MAX_STRIDE to stay in the same parity lane */
        return (1 << slot);  /* bitmask: bit 0 = slot 0 ready, bit 1 = slot 1 ready */
    }
    return 0;
}

int opfec_interleaved_get_fec(opfec_interleaved_enc_t *il, uint8_t slot,
                               uint8_t *out, size_t out_cap)
{
    if (!il || !out || slot >= OPFEC_MAX_STRIDE)
        return -1;

    int n = opfec_enc_get_fec(&il->groups[slot], out, out_cap);
    if (n > 0) {
        /* Jump group_id forward by OPFEC_MAX_STRIDE to keep parity */
        il->groups[slot].group_id = (uint8_t)(il->groups[slot].group_id +
                                               OPFEC_MAX_STRIDE - 1);
    }
    return n;
}

int opfec_interleaved_flush(opfec_interleaved_enc_t *il,
                             uint8_t *out0, size_t cap0,
                             uint8_t *out1, size_t cap1)
{
    if (!il)
        return -1;

    int mask = 0;
    if (out0) {
        int n = opfec_enc_flush(&il->groups[0], out0, cap0);
        if (n > 0) mask |= 1;
    }
    if (out1) {
        int n = opfec_enc_flush(&il->groups[1], out1, cap1);
        if (n > 0) mask |= 2;
    }
    return mask;
}

void opfec_interleaved_dec_init(opfec_interleaved_dec_t *il, opfec_level_t level)
{
    if (!il)
        return;

    for (int s = 0; s < OPFEC_MAX_STRIDE; s++) {
        opfec_dec_init(&il->groups[s], level);
    }
}

int opfec_interleaved_dec_feed_data(opfec_interleaved_dec_t *il,
                                    uint32_t seq,
                                    uint8_t group_id, uint8_t pkt_idx,
                                    const uint8_t *data, uint16_t len)
{
    if (!il)
        return -1;

    uint8_t slot = (uint8_t)(seq % OPFEC_MAX_STRIDE);
    return opfec_dec_feed_data(&il->groups[slot], group_id, pkt_idx, data, len);
}

int opfec_interleaved_dec_feed_fec(opfec_interleaved_dec_t *il,
                                   const opfec_header_t *header,
                                   const uint8_t *fec_payload, uint16_t fec_len)
{
    if (!il || !header)
        return -1;

    /* Stride slot encoded in group_id parity */
    uint8_t slot = (uint8_t)(header->group_id & (OPFEC_MAX_STRIDE - 1));
    return opfec_dec_feed_fec(&il->groups[slot], header, fec_payload, fec_len);
}

int opfec_interleaved_recover(opfec_interleaved_dec_t *il,
                              uint32_t seq, uint8_t pkt_idx,
                              uint8_t *out, size_t out_cap, uint16_t *out_len)
{
    if (!il)
        return -1;

    uint8_t slot = (uint8_t)(seq % OPFEC_MAX_STRIDE);
    return opfec_dec_recover(&il->groups[slot], pkt_idx, out, out_cap, out_len);
}