/*
 * opcodec/secure.c — AEAD security layer implementation
 *
 * SFrame/DAVE-inspired per-frame ChaCha20-Poly1305 encryption with:
 * - Extended header format with sender ID and frame type
 * - Per-sender key derivation for group scenarios
 * - Salt-based XOR nonce construction
 * - Generation-based key ratcheting with forward secrecy
 * - Extended 512-packet anti-replay window
 * - Grace period key retention for out-of-order frames
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/secure.h"
#include "opssl/crypto.h"
#include <string.h>
#include <stdio.h>

/* Cipher suite information table */
static const opsec_suite_info_t suite_table[] = {
    { OPSEC_SUITE_CHACHA20_POLY1305, 32, 16, 12, "ChaCha20-Poly1305" },
    { OPSEC_SUITE_AES_128_GCM,       16, 16, 12, "AES-128-GCM" },
    { OPSEC_SUITE_AES_256_GCM,       32, 16, 12, "AES-256-GCM" },
};

/* Static function declarations */
static void opsec_write_be16(uint8_t *out, uint16_t val);
static void opsec_write_be32(uint8_t *out, uint32_t val);
static uint16_t opsec_read_be16(const uint8_t *in);
static uint32_t opsec_read_be32(const uint8_t *in);
static void opsec_construct_nonce_v1(uint8_t *nonce, uint16_t epoch, uint32_t seq);
static void opsec_construct_nonce_v2(uint8_t *nonce, const uint8_t *salt, uint32_t counter);
static void opsec_construct_aad_v1(uint8_t *aad, uint16_t epoch, uint32_t seq);
static bool opsec_replay_check(opsec_ctx_t *ctx, uint32_t seq);
static void opsec_replay_update(opsec_ctx_t *ctx, uint32_t seq);
static int opsec_derive_keys(const uint8_t *secret, size_t secret_len,
                            const char *send_info, const char *recv_info,
                            uint8_t *send_key, uint8_t *recv_key);
static int opsec_ratchet_key(const uint8_t *current_key, uint32_t generation,
                            uint8_t *new_key);
static int ct_memcmp(const void *a, const void *b, size_t n);
static bool opsec_is_v2_frame(const uint8_t *data, size_t len);
static uint32_t opsec_sender_id_to_index(uint32_t sender_id);
static int opsec_seal_v1_legacy(opsec_ctx_t *ctx,
                                const uint8_t *plaintext, size_t pt_len,
                                uint8_t *out, size_t out_cap);
static int opsec_open_v1_legacy(opsec_ctx_t *ctx,
                                const uint8_t *ciphertext, size_t ct_len,
                                uint8_t *out, size_t out_cap);
static int suite_to_opssl(opsec_suite_t suite);
static uint32_t xorshift32(uint32_t *state);

int opsec_init(opsec_ctx_t *ctx, const uint8_t *shared_secret,
               size_t secret_len, bool is_initiator)
{
    if (!ctx || !shared_secret || secret_len == 0) {
        return -1;
    }

    /* Clear the context */
    memset(ctx, 0, sizeof(*ctx));

    /* Store root secret for key ratcheting */
    if (secret_len > OPSEC_KEY_SIZE) {
        secret_len = OPSEC_KEY_SIZE;
    }
    memcpy(ctx->root_secret, shared_secret, secret_len);
    if (secret_len < OPSEC_KEY_SIZE) {
        memset(ctx->root_secret + secret_len, 0, OPSEC_KEY_SIZE - secret_len);
    }

    /* Derive initial keys based on initiator role */
    const char *send_info, *recv_info;
    if (is_initiator) {
        send_info = "opcodec-send-key";
        recv_info = "opcodec-recv-key";
    } else {
        send_info = "opcodec-recv-key";
        recv_info = "opcodec-send-key";
    }

    if (opsec_derive_keys(shared_secret, secret_len, send_info, recv_info,
                         ctx->send_key, ctx->recv_key) != 0) {
        return -1;
    }

    /* Initialize epochs and sequence numbers */
    ctx->send_epoch = 0;
    ctx->recv_epoch = 0;
    ctx->send_seq = 0;
    ctx->replay_seq_base = 0;
    memset(ctx->replay_bitmap, 0, sizeof(ctx->replay_bitmap));

    /* Initialize V2 fields */
    ctx->my_sender_id = 0;
    memset(ctx->sender_counters, 0, sizeof(ctx->sender_counters));
    ctx->send_generation = 0;
    memset(ctx->recv_generation, 0, sizeof(ctx->recv_generation));
    ctx->prev_recv_epoch = 0;
    ctx->prev_epoch_deadline = 0;
    memset(ctx->prev_recv_key, 0, sizeof(ctx->prev_recv_key));

    /* Initialize cipher suite and traffic analysis fields */
    ctx->suite = OPSEC_SUITE_CHACHA20_POLY1305;  /* Default to ChaCha20-Poly1305 */
    ctx->traffic_flags = 0;
    ctx->cbr_target_size = 0;

    ctx->v2_mode = false;  /* Default to V1 for backward compatibility */
    ctx->initialized = true;
    return 0;
}

int opsec_init_v2(opsec_ctx_t *ctx, const uint8_t *shared_secret,
                  size_t secret_len, uint32_t sender_id)
{
    if (opsec_init(ctx, shared_secret, secret_len, true) != 0) {
        return -1;
    }

    /* Enable V2 mode and set sender ID */
    ctx->v2_mode = true;
    ctx->my_sender_id = sender_id;

    /* Derive per-sender keys for all potential senders */
    for (uint32_t i = 0; i < OPSEC_MAX_SENDERS; i++) {
        if (opsec_derive_sender_key(ctx, i,
                                   ctx->sender_keys[i], OPSEC_KEY_SIZE,
                                   ctx->sender_salts[i], OPSEC_SALT_SIZE) != 0) {
            return -1;
        }
    }

    return 0;
}

int opsec_init_suite(opsec_ctx_t *ctx, const uint8_t *shared_secret,
                     size_t secret_len, bool is_initiator,
                     opsec_suite_t suite)
{
    if (suite >= OPSEC_SUITE_COUNT) {
        return -1;
    }

    if (opsec_init(ctx, shared_secret, secret_len, is_initiator) != 0) {
        return -1;
    }

    /* Set the specified cipher suite */
    ctx->suite = suite;
    return 0;
}

int opsec_derive_sender_key(opsec_ctx_t *ctx,
                            uint32_t sender_id,
                            uint8_t *key_out, size_t key_len,
                            uint8_t *salt_out, size_t salt_len)
{
    if (!ctx || !ctx->initialized || !key_out || !salt_out) {
        return -1;
    }

    if (key_len < OPSEC_KEY_SIZE || salt_len < OPSEC_SALT_SIZE) {
        return -1;
    }

    /* Construct info strings with sender ID and epoch */
    char key_info[64];
    char salt_info[64];
    snprintf(key_info, sizeof(key_info), "ophion-sender-key");
    snprintf(salt_info, sizeof(salt_info), "ophion-sender-salt");

    /* Append sender_id (4 bytes BE) and epoch (2 bytes BE) */
    uint8_t key_suffix[6], salt_suffix[6];
    opsec_write_be32(key_suffix, sender_id);
    opsec_write_be16(key_suffix + 4, ctx->send_epoch);
    opsec_write_be32(salt_suffix, sender_id);
    opsec_write_be16(salt_suffix + 4, ctx->send_epoch);

    /* Combine info string with binary suffix */
    uint8_t key_info_full[sizeof(key_info) + 6];
    uint8_t salt_info_full[sizeof(salt_info) + 6];
    memcpy(key_info_full, key_info, strlen(key_info));
    memcpy(key_info_full + strlen(key_info), key_suffix, 6);
    memcpy(salt_info_full, salt_info, strlen(salt_info));
    memcpy(salt_info_full + strlen(salt_info), salt_suffix, 6);

    /* Derive key and salt using HKDF-Expand */
    if (opssl_hkdf_expand(OPSSL_HMAC_SHA256,
                         ctx->root_secret, OPSEC_KEY_SIZE,
                         key_info_full, strlen(key_info) + 6,
                         key_out, OPSEC_KEY_SIZE) != 0) {
        return -1;
    }

    if (opssl_hkdf_expand(OPSSL_HMAC_SHA256,
                         ctx->root_secret, OPSEC_KEY_SIZE,
                         salt_info_full, strlen(salt_info) + 6,
                         salt_out, OPSEC_SALT_SIZE) != 0) {
        return -1;
    }

    return 0;
}

int opsec_seal_v2(opsec_ctx_t *ctx,
                  uint8_t frame_type, uint32_t sender_id, uint8_t codec_id,
                  const uint8_t *plaintext, size_t pt_len,
                  uint8_t *out, size_t out_cap)
{
    if (!ctx || !ctx->initialized || !plaintext || !out) {
        return -1;
    }

    if (pt_len > OPSEC_MAX_PAYLOAD) {
        return -1;
    }

    if (out_cap < pt_len + OPSEC_V2_OVERHEAD) {
        return -1;
    }

    /* Get sender index and current counter */
    uint32_t sender_idx = opsec_sender_id_to_index(sender_id);
    if (sender_idx >= OPSEC_MAX_SENDERS) {
        return -1;
    }

    uint32_t counter = ctx->sender_counters[sender_idx];
    uint32_t generation = counter >> 16;

    /* Check if we need to ratchet the key */
    if (generation > ctx->send_generation) {
        if (opsec_ratchet_key(ctx->sender_keys[sender_idx], generation,
                             ctx->sender_keys[sender_idx]) != 0) {
            return -1;
        }
        ctx->send_generation = generation;
    }

    /* Construct V2 header: version(1) + frame_type(1) + sender_id(4) +
     * counter(4) + epoch(2) + codec_id(1) + flags(1) = 14 bytes */
    uint8_t *header = out;
    header[0] = OPSEC_VERSION_V2;
    header[1] = frame_type;
    opsec_write_be32(header + 2, sender_id);
    opsec_write_be32(header + 6, counter);
    opsec_write_be16(header + 10, ctx->send_epoch);
    header[12] = codec_id;
    header[13] = 0; /* flags - unused for now */

    /* Construct salt-based XOR nonce */
    uint8_t nonce[OPSEC_NONCE_SIZE];
    opsec_construct_nonce_v2(nonce, ctx->sender_salts[sender_idx], counter);

    /* AAD is the entire cleartext header */
    const uint8_t *aad = header;
    size_t aad_len = OPSEC_V2_HEADER_SIZE;

    /* Initialize AEAD context with selected cipher suite */
    int opssl_algo = suite_to_opssl(ctx->suite);
    if (opssl_algo < 0) {
        return -1;
    }

    /* Apply traffic analysis protection if enabled */
    uint8_t padded_data[OPSEC_MAX_PAYLOAD + 256]; /* Extra space for padding */
    const uint8_t *data_to_encrypt = plaintext;
    size_t data_len = pt_len;

    if (ctx->traffic_flags & (OPSEC_TRAFFIC_CBR | OPSEC_TRAFFIC_RANDOM_PAD)) {
        if (pt_len > sizeof(padded_data) - 256) {
            return -1; /* Too large for padding */
        }

        memcpy(padded_data, plaintext, pt_len);
        data_len = pt_len;

        /* Random padding */
        if (ctx->traffic_flags & OPSEC_TRAFFIC_RANDOM_PAD) {
            uint32_t rng = counter ^ ctx->send_epoch;
            uint8_t extra = xorshift32(&rng) % 64;
            for (uint8_t i = 0; i < extra; i++) {
                padded_data[data_len++] = extra + 1; /* PKCS#7 style */
            }
        }

        /* CBR mode - pad to fixed size */
        if (ctx->traffic_flags & OPSEC_TRAFFIC_CBR) {
            uint16_t target_size = ctx->cbr_target_size;
            if (target_size > OPSEC_V2_OVERHEAD &&
                target_size <= sizeof(padded_data) + OPSEC_V2_OVERHEAD) {

                size_t target_pt_size = target_size - OPSEC_V2_OVERHEAD - 16; /* Account for tag */
                if (data_len < target_pt_size) {
                    uint8_t pad_len = target_pt_size - data_len;
                    for (size_t i = data_len; i < target_pt_size; i++) {
                        padded_data[i] = pad_len;
                    }
                    data_len = target_pt_size;
                }
            }
        }

        data_to_encrypt = padded_data;
    }

    /* Get cipher suite key size */
    const opsec_suite_info_t *suite_info = opsec_suite_info(ctx->suite);
    if (!suite_info) {
        return -1;
    }

    opssl_aead_ctx_t *aead_ctx = opssl_aead_new((opssl_aead_algo_t)opssl_algo);
    if (!aead_ctx) {
        return -1;
    }

    if (opssl_aead_set_key(aead_ctx, ctx->sender_keys[sender_idx], suite_info->key_size) != 0) {
        opssl_aead_free(aead_ctx);
        return -1;
    }

    /* Encrypt: output goes after the header */
    uint8_t *ct_out = out + OPSEC_V2_HEADER_SIZE;
    size_t ct_len;
    size_t max_ct = out_cap - OPSEC_V2_HEADER_SIZE;

    int ret = opssl_aead_seal(aead_ctx,
                             ct_out, &ct_len, max_ct,
                             nonce, OPSEC_NONCE_SIZE,
                             data_to_encrypt, data_len,
                             aad, aad_len);

    opssl_aead_free(aead_ctx);

    if (ret != 0) {
        return -1;
    }

    /* Increment counter */
    ctx->sender_counters[sender_idx]++;

    /* Return total bytes: header + ciphertext + tag */
    return OPSEC_V2_HEADER_SIZE + ct_len;
}

int opsec_open_v2(opsec_ctx_t *ctx,
                  const uint8_t *ciphertext, size_t ct_len,
                  uint8_t *out, size_t out_cap,
                  uint8_t *frame_type_out, uint32_t *sender_id_out)
{
    if (!ctx || !ctx->initialized || !ciphertext || !out) {
        return -1;
    }

    if (ct_len < OPSEC_V2_OVERHEAD) {
        return -1;
    }

    /* Parse V2 header */
    const uint8_t *header = ciphertext;
    uint8_t version = header[0];
    uint8_t frame_type = header[1];
    uint32_t sender_id = opsec_read_be32(header + 2);
    uint32_t counter = opsec_read_be32(header + 6);
    uint16_t epoch = opsec_read_be16(header + 10);
    uint8_t codec_id = header[12];
    uint8_t flags = header[13];
    (void)codec_id;  /* Reserved for future use */
    (void)flags;     /* Reserved for future use */

    if (version != OPSEC_VERSION_V2) {
        return -1;
    }

    /* Get sender index */
    uint32_t sender_idx = opsec_sender_id_to_index(sender_id);
    if (sender_idx >= OPSEC_MAX_SENDERS) {
        return -1;
    }

    /* Check epoch - allow current or previous with grace period */
    bool use_previous_key = false;
    if (epoch == ctx->recv_epoch) {
        /* Current epoch - use current key */
    } else if (epoch == ctx->prev_recv_epoch &&
               counter < ctx->prev_epoch_deadline &&
               ctx->prev_recv_epoch != 0) {
        /* Previous epoch within grace period - use previous key */
        use_previous_key = true;
    } else {
        return -1; /* Invalid epoch */
    }

    /* Anti-replay check (simplified for now - could be per-sender) */
    if (!opsec_replay_check(ctx, counter)) {
        return -1;
    }

    /* Determine which key to use */
    uint8_t *recv_key;
    if (use_previous_key) {
        recv_key = ctx->prev_recv_key;
    } else {
        uint32_t generation = counter >> 16;
        /* Check if we need to ratchet the receive key */
        if (generation > ctx->recv_generation[sender_idx]) {
            if (opsec_ratchet_key(ctx->sender_keys[sender_idx], generation,
                                 ctx->sender_keys[sender_idx]) != 0) {
                return -1;
            }
            ctx->recv_generation[sender_idx] = generation;
        }
        recv_key = ctx->sender_keys[sender_idx];
    }

    /* Construct salt-based XOR nonce */
    uint8_t nonce[OPSEC_NONCE_SIZE];
    opsec_construct_nonce_v2(nonce, ctx->sender_salts[sender_idx], counter);

    /* AAD is the entire cleartext header */
    const uint8_t *aad = header;
    size_t aad_len = OPSEC_V2_HEADER_SIZE;

    /* Initialize AEAD context with selected cipher suite */
    int opssl_algo = suite_to_opssl(ctx->suite);
    if (opssl_algo < 0) {
        return -1;
    }

    /* Get cipher suite key size */
    const opsec_suite_info_t *suite_info = opsec_suite_info(ctx->suite);
    if (!suite_info) {
        return -1;
    }

    opssl_aead_ctx_t *aead_ctx = opssl_aead_new((opssl_aead_algo_t)opssl_algo);
    if (!aead_ctx) {
        return -1;
    }

    if (opssl_aead_set_key(aead_ctx, recv_key, suite_info->key_size) != 0) {
        opssl_aead_free(aead_ctx);
        return -1;
    }

    /* Decrypt: input starts after header */
    const uint8_t *ct_in = ciphertext + OPSEC_V2_HEADER_SIZE;
    size_t ct_payload_len = ct_len - OPSEC_V2_HEADER_SIZE;
    size_t pt_len;

    int ret = opssl_aead_open(aead_ctx,
                             out, &pt_len, out_cap,
                             nonce, OPSEC_NONCE_SIZE,
                             ct_in, ct_payload_len,
                             aad, aad_len);

    opssl_aead_free(aead_ctx);

    if (ret != 0) {
        return -1;
    }

    /* Update replay window */
    opsec_replay_update(ctx, counter);

    /* Check for dummy frame */
    if (frame_type == OPSEC_FRAME_DUMMY) {
        if (frame_type_out) *frame_type_out = frame_type;
        if (sender_id_out) *sender_id_out = sender_id;
        return OPSEC_DUMMY_FRAME;
    }

    /* Return extracted header fields */
    if (frame_type_out) *frame_type_out = frame_type;
    if (sender_id_out) *sender_id_out = sender_id;

    return (int)pt_len;
}

int opsec_seal(opsec_ctx_t *ctx,
               const uint8_t *plaintext, size_t pt_len,
               uint8_t *out, size_t out_cap)
{
    if (!ctx || !ctx->v2_mode) {
        /* Legacy V1 implementation */
        return opsec_seal_v1_legacy(ctx, plaintext, pt_len, out, out_cap);
    } else {
        /* V2 mode: use default parameters */
        return opsec_seal_v2(ctx, OPSEC_FRAME_AUDIO, ctx->my_sender_id, 0,
                            plaintext, pt_len, out, out_cap);
    }
}

/* Legacy V1 seal implementation */
static int opsec_seal_v1_legacy(opsec_ctx_t *ctx,
                                const uint8_t *plaintext, size_t pt_len,
                                uint8_t *out, size_t out_cap)
{
    if (!ctx || !ctx->initialized || !plaintext || !out) {
        return -1;
    }

    if (pt_len > OPSEC_MAX_PAYLOAD) {
        return -1;
    }

    if (out_cap < pt_len + OPSEC_OVERHEAD) {
        return -1;
    }

    /* Construct nonce: epoch(2) || seq(4) || zeros(6) */
    uint8_t nonce[OPSEC_NONCE_SIZE];
    opsec_construct_nonce_v1(nonce, ctx->send_epoch, ctx->send_seq);

    /* Construct AAD: epoch(2) || seq(4) */
    uint8_t aad[OPSEC_EPOCH_SIZE + OPSEC_SEQ_SIZE];
    opsec_construct_aad_v1(aad, ctx->send_epoch, ctx->send_seq);

    /* Write epoch and seq to output */
    opsec_write_be16(out, ctx->send_epoch);
    opsec_write_be32(out + OPSEC_EPOCH_SIZE, ctx->send_seq);

    /* Initialize AEAD context */
    opssl_aead_ctx_t *aead_ctx = opssl_aead_new(OPSSL_AEAD_CHACHA20_POLY1305);
    if (!aead_ctx) {
        return -1;
    }

    if (opssl_aead_set_key(aead_ctx, ctx->send_key, OPSEC_KEY_SIZE) != 0) {
        opssl_aead_free(aead_ctx);
        return -1;
    }

    /* Encrypt: output goes after epoch+seq header */
    uint8_t *ct_out = out + OPSEC_EPOCH_SIZE + OPSEC_SEQ_SIZE;
    size_t ct_len;
    size_t max_ct = out_cap - OPSEC_EPOCH_SIZE - OPSEC_SEQ_SIZE;

    int ret = opssl_aead_seal(aead_ctx,
                             ct_out, &ct_len, max_ct,
                             nonce, OPSEC_NONCE_SIZE,
                             plaintext, pt_len,
                             aad, sizeof(aad));

    opssl_aead_free(aead_ctx);

    if (ret != 0) {
        return -1;
    }

    /* Increment send_seq; if it wraps, auto-rotate */
    ctx->send_seq++;
    if (ctx->send_seq == 0) {
        if (opsec_rotate(ctx) != 0) {
            return -1;
        }
    }

    /* Return total bytes: header + ciphertext + tag */
    return OPSEC_EPOCH_SIZE + OPSEC_SEQ_SIZE + ct_len;
}

int opsec_open(opsec_ctx_t *ctx,
               const uint8_t *ciphertext, size_t ct_len,
               uint8_t *out, size_t out_cap)
{
    if (!ctx || !ctx->initialized || !ciphertext || !out) {
        return -1;
    }

    /* Auto-detect frame format */
    if (opsec_is_v2_frame(ciphertext, ct_len)) {
        /* V2 format */
        return opsec_open_v2(ctx, ciphertext, ct_len, out, out_cap, NULL, NULL);
    } else {
        /* V1 format */
        return opsec_open_v1_legacy(ctx, ciphertext, ct_len, out, out_cap);
    }
}

/* Legacy V1 open implementation */
static int opsec_open_v1_legacy(opsec_ctx_t *ctx,
                                const uint8_t *ciphertext, size_t ct_len,
                                uint8_t *out, size_t out_cap)
{
    if (ct_len < OPSEC_OVERHEAD) {
        return -1;
    }

    /* Read epoch and seq from input */
    uint16_t epoch = opsec_read_be16(ciphertext);
    uint32_t seq = opsec_read_be32(ciphertext + OPSEC_EPOCH_SIZE);

    /* Check epoch - must match recv_epoch or recv_epoch+1 (for rotation) */
    if (epoch != ctx->recv_epoch && epoch != (ctx->recv_epoch + 1)) {
        return -1;
    }

    /* Anti-replay check */
    if (!opsec_replay_check(ctx, seq)) {
        return -1;
    }

    /* Construct nonce and AAD */
    uint8_t nonce[OPSEC_NONCE_SIZE];
    opsec_construct_nonce_v1(nonce, epoch, seq);

    uint8_t aad[OPSEC_EPOCH_SIZE + OPSEC_SEQ_SIZE];
    opsec_construct_aad_v1(aad, epoch, seq);

    /* Initialize AEAD context */
    opssl_aead_ctx_t *aead_ctx = opssl_aead_new(OPSSL_AEAD_CHACHA20_POLY1305);
    if (!aead_ctx) {
        return -1;
    }

    if (opssl_aead_set_key(aead_ctx, ctx->recv_key, OPSEC_KEY_SIZE) != 0) {
        opssl_aead_free(aead_ctx);
        return -1;
    }

    /* Decrypt: input starts after epoch+seq header */
    const uint8_t *ct_in = ciphertext + OPSEC_EPOCH_SIZE + OPSEC_SEQ_SIZE;
    size_t ct_payload_len = ct_len - OPSEC_EPOCH_SIZE - OPSEC_SEQ_SIZE;
    size_t pt_len;

    int ret = opssl_aead_open(aead_ctx,
                             out, &pt_len, out_cap,
                             nonce, OPSEC_NONCE_SIZE,
                             ct_in, ct_payload_len,
                             aad, sizeof(aad));

    opssl_aead_free(aead_ctx);

    if (ret != 0) {
        return -1;
    }

    /* Update replay window */
    opsec_replay_update(ctx, seq);

    /* If this was an epoch+1 frame, update recv_epoch */
    if (epoch == ctx->recv_epoch + 1) {
        ctx->recv_epoch = epoch;
        /* Reset replay window for new epoch */
        ctx->replay_seq_base = 0;
        memset(ctx->replay_bitmap, 0, sizeof(ctx->replay_bitmap));
        opsec_replay_update(ctx, seq);
    }

    return (int)pt_len;
}

int opsec_rotate(opsec_ctx_t *ctx)
{
    if (!ctx || !ctx->initialized) {
        return -1;
    }

    /* Save current receive key as previous for grace period */
    memcpy(ctx->prev_recv_key, ctx->recv_key, OPSEC_KEY_SIZE);
    ctx->prev_recv_epoch = ctx->recv_epoch;
    ctx->prev_epoch_deadline = ctx->send_seq + OPSEC_GRACE_PERIOD;

    /* Increment epochs */
    ctx->send_epoch++;
    ctx->recv_epoch = ctx->send_epoch;

    /* Derive new keys: HKDF-Expand(root_secret || old_key, "opcodec-ratchet", 32) */
    uint8_t new_send_key[OPSEC_KEY_SIZE];
    uint8_t new_recv_key[OPSEC_KEY_SIZE];

    /* Combine root_secret with old keys for key material */
    uint8_t send_ikm[OPSEC_KEY_SIZE * 2];
    uint8_t recv_ikm[OPSEC_KEY_SIZE * 2];

    memcpy(send_ikm, ctx->root_secret, OPSEC_KEY_SIZE);
    memcpy(send_ikm + OPSEC_KEY_SIZE, ctx->send_key, OPSEC_KEY_SIZE);

    memcpy(recv_ikm, ctx->root_secret, OPSEC_KEY_SIZE);
    memcpy(recv_ikm + OPSEC_KEY_SIZE, ctx->recv_key, OPSEC_KEY_SIZE);

    const char *ratchet_info = "opcodec-ratchet";

    if (opssl_hkdf_expand(OPSSL_HMAC_SHA256, send_ikm, sizeof(send_ikm),
                         (const uint8_t *)ratchet_info, strlen(ratchet_info),
                         new_send_key, OPSEC_KEY_SIZE) != 0) {
        return -1;
    }

    if (opssl_hkdf_expand(OPSSL_HMAC_SHA256, recv_ikm, sizeof(recv_ikm),
                         (const uint8_t *)ratchet_info, strlen(ratchet_info),
                         new_recv_key, OPSEC_KEY_SIZE) != 0) {
        return -1;
    }

    /* Update keys */
    memcpy(ctx->send_key, new_send_key, OPSEC_KEY_SIZE);
    memcpy(ctx->recv_key, new_recv_key, OPSEC_KEY_SIZE);

    /* Re-derive V2 per-sender keys if in V2 mode */
    if (ctx->v2_mode) {
        for (uint32_t i = 0; i < OPSEC_MAX_SENDERS; i++) {
            if (opsec_derive_sender_key(ctx, i,
                                       ctx->sender_keys[i], OPSEC_KEY_SIZE,
                                       ctx->sender_salts[i], OPSEC_SALT_SIZE) != 0) {
                return -1;
            }
        }
    }

    /* Clear old key material securely */
    opsec_zeroize(send_ikm, sizeof(send_ikm));
    opsec_zeroize(recv_ikm, sizeof(recv_ikm));
    opsec_zeroize(new_send_key, OPSEC_KEY_SIZE);
    opsec_zeroize(new_recv_key, OPSEC_KEY_SIZE);

    /* Reset send_seq */
    ctx->send_seq = 0;

    /* Clear replay window */
    ctx->replay_seq_base = 0;
    memset(ctx->replay_bitmap, 0, sizeof(ctx->replay_bitmap));

    return 0;
}

size_t opsec_pad(uint8_t *buf, size_t data_len, size_t buf_cap)
{
    if (!buf || data_len >= buf_cap) {
        return data_len;
    }

    /* PKCS#7-style padding to nearest OPSEC_PAD_BLOCK */
    size_t pad_to = ((data_len + OPSEC_PAD_BLOCK) / OPSEC_PAD_BLOCK) * OPSEC_PAD_BLOCK;
    size_t pad_bytes = pad_to - data_len;

    /* Always pad at least 1 byte */
    if (pad_bytes == 0) {
        pad_bytes = OPSEC_PAD_BLOCK;
        pad_to += OPSEC_PAD_BLOCK;
    }

    if (pad_to > buf_cap) {
        return data_len;  /* Cannot pad, return original length */
    }

    /* Fill padding bytes with padding length */
    for (size_t i = data_len; i < pad_to; i++) {
        buf[i] = (uint8_t)pad_bytes;
    }

    return pad_to;
}

size_t opsec_unpad(const uint8_t *buf, size_t padded_len)
{
    if (!buf || padded_len == 0) {
        return 0;
    }

    /* Read last byte to get padding length */
    uint8_t pad_len = buf[padded_len - 1];

    if (pad_len == 0 || pad_len > OPSEC_PAD_BLOCK || pad_len > padded_len) {
        return 0;  /* Invalid padding */
    }

    /* Verify all padding bytes are the same */
    for (size_t i = padded_len - pad_len; i < padded_len; i++) {
        if (buf[i] != pad_len) {
            return 0;  /* Invalid padding */
        }
    }

    return padded_len - pad_len;
}

/* ──── Static Helper Functions ───────────────────────────────────────── */

static void opsec_write_be16(uint8_t *out, uint16_t val)
{
    out[0] = (uint8_t)(val >> 8);
    out[1] = (uint8_t)(val & 0xFF);
}

static void opsec_write_be32(uint8_t *out, uint32_t val)
{
    out[0] = (uint8_t)(val >> 24);
    out[1] = (uint8_t)(val >> 16);
    out[2] = (uint8_t)(val >> 8);
    out[3] = (uint8_t)(val & 0xFF);
}

static uint16_t opsec_read_be16(const uint8_t *in)
{
    return ((uint16_t)in[0] << 8) | in[1];
}

static uint32_t opsec_read_be32(const uint8_t *in)
{
    return ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) |
           ((uint32_t)in[2] << 8)  | in[3];
}

static void opsec_construct_nonce_v1(uint8_t *nonce, uint16_t epoch, uint32_t seq)
{
    opsec_write_be16(nonce, epoch);
    opsec_write_be32(nonce + 2, seq);
    memset(nonce + 6, 0, 6);  /* Zero-fill remaining 6 bytes */
}

static void opsec_construct_nonce_v2(uint8_t *nonce, const uint8_t *salt, uint32_t counter)
{
    /* Construct 12-byte counter in big-endian format */
    uint8_t counter_12[12];
    memset(counter_12, 0, 8);  /* High 8 bytes zero */
    opsec_write_be32(counter_12 + 8, counter);  /* Low 4 bytes = counter */

    /* XOR salt with counter to form nonce */
    for (int i = 0; i < OPSEC_NONCE_SIZE; i++) {
        nonce[i] = salt[i] ^ counter_12[i];
    }
}

static void opsec_construct_aad_v1(uint8_t *aad, uint16_t epoch, uint32_t seq)
{
    opsec_write_be16(aad, epoch);
    opsec_write_be32(aad + 2, seq);
}

static bool opsec_replay_check(opsec_ctx_t *ctx, uint32_t seq)
{
    /* If seq is too old (before window), reject */
    if (seq + OPSEC_REPLAY_WINDOW <= ctx->replay_seq_base) {
        return false;
    }

    /* If seq is newer than window, it's valid (will shift window) */
    if (seq >= ctx->replay_seq_base + OPSEC_REPLAY_WINDOW) {
        return true;
    }

    /* seq is within current window, check if already received */
    uint32_t offset = seq - ctx->replay_seq_base;
    uint32_t word_idx = offset / 64;
    uint32_t bit_idx = offset % 64;

    if (word_idx >= OPSEC_REPLAY_WINDOW / 64) {
        return false;  /* Should not happen */
    }

    return (ctx->replay_bitmap[word_idx] & (1ULL << bit_idx)) == 0;
}

static void opsec_replay_update(opsec_ctx_t *ctx, uint32_t seq)
{
    /* If seq extends beyond current window, shift window */
    if (seq >= ctx->replay_seq_base + OPSEC_REPLAY_WINDOW) {
        uint32_t shift = seq - (ctx->replay_seq_base + OPSEC_REPLAY_WINDOW - 1);

        /* Shift bitmap */
        if (shift >= OPSEC_REPLAY_WINDOW) {
            /* Complete reset */
            memset(ctx->replay_bitmap, 0, sizeof(ctx->replay_bitmap));
        } else {
            /* Shift by 'shift' bits */
            uint32_t word_shift = shift / 64;
            uint32_t bit_shift = shift % 64;

            if (word_shift > 0) {
                /* Shift by whole words */
                for (uint32_t i = 0; i < (OPSEC_REPLAY_WINDOW / 64) - word_shift; i++) {
                    ctx->replay_bitmap[i] = ctx->replay_bitmap[i + word_shift];
                }
                for (uint32_t i = (OPSEC_REPLAY_WINDOW / 64) - word_shift;
                     i < OPSEC_REPLAY_WINDOW / 64; i++) {
                    ctx->replay_bitmap[i] = 0;
                }
            }

            if (bit_shift > 0) {
                /* Shift by remaining bits */
                for (int i = 0; i < (OPSEC_REPLAY_WINDOW / 64) - 1; i++) {
                    ctx->replay_bitmap[i] = (ctx->replay_bitmap[i] >> bit_shift) |
                                           (ctx->replay_bitmap[i + 1] << (64 - bit_shift));
                }
                ctx->replay_bitmap[(OPSEC_REPLAY_WINDOW / 64) - 1] >>= bit_shift;
            }
        }

        ctx->replay_seq_base = seq - OPSEC_REPLAY_WINDOW + 1;
    }

    /* Mark seq as received */
    uint32_t offset = seq - ctx->replay_seq_base;
    uint32_t word_idx = offset / 64;
    uint32_t bit_idx = offset % 64;

    if (word_idx < OPSEC_REPLAY_WINDOW / 64) {
        ctx->replay_bitmap[word_idx] |= (1ULL << bit_idx);
    }
}

static int opsec_derive_keys(const uint8_t *secret, size_t secret_len,
                            const char *send_info, const char *recv_info,
                            uint8_t *send_key, uint8_t *recv_key)
{
    if (opssl_hkdf_expand(OPSSL_HMAC_SHA256, secret, secret_len,
                         (const uint8_t *)send_info, strlen(send_info),
                         send_key, OPSEC_KEY_SIZE) != 0) {
        return -1;
    }

    if (opssl_hkdf_expand(OPSSL_HMAC_SHA256, secret, secret_len,
                         (const uint8_t *)recv_info, strlen(recv_info),
                         recv_key, OPSEC_KEY_SIZE) != 0) {
        return -1;
    }

    return 0;
}

static int opsec_ratchet_key(const uint8_t *current_key, uint32_t generation,
                            uint8_t *new_key)
{
    if (!current_key || !new_key) {
        return -1;
    }

    /* Construct ratchet info: "ophion-ratchet" || generation_be4 */
    char ratchet_info[] = "ophion-ratchet";
    uint8_t generation_bytes[4];
    opsec_write_be32(generation_bytes, generation);

    uint8_t full_info[sizeof(ratchet_info) - 1 + 4];
    memcpy(full_info, ratchet_info, sizeof(ratchet_info) - 1);
    memcpy(full_info + sizeof(ratchet_info) - 1, generation_bytes, 4);

    /* Derive new key: HKDF-Expand(current_key, ratchet_info, 32) */
    return opssl_hkdf_expand(OPSSL_HMAC_SHA256,
                            current_key, OPSEC_KEY_SIZE,
                            full_info, sizeof(full_info),
                            new_key, OPSEC_KEY_SIZE);
}

/* Constant-time memory comparison to prevent timing attacks.
 * Note: opssl_aead_open likely already uses constant-time tag comparison internally,
 * but this function is available for explicit use when needed. */
__attribute__((unused))
static int ct_memcmp(const void *a, const void *b, size_t n)
{
    const uint8_t *pa = (const uint8_t *)a;
    const uint8_t *pb = (const uint8_t *)b;
    uint8_t result = 0;

    for (size_t i = 0; i < n; i++) {
        result |= pa[i] ^ pb[i];
    }

    return result;
}

static bool opsec_is_v2_frame(const uint8_t *data, size_t len)
{
    if (!data || len < 1) {
        return false;
    }

    /* Check if first byte is V2 version */
    return data[0] == OPSEC_VERSION_V2;
}

static uint32_t opsec_sender_id_to_index(uint32_t sender_id)
{
    /* Simple mapping for small groups - could be enhanced */
    return sender_id % OPSEC_MAX_SENDERS;
}

/* Map cipher suite to opssl AEAD algorithm */
static int suite_to_opssl(opsec_suite_t suite)
{
    switch (suite) {
    case OPSEC_SUITE_CHACHA20_POLY1305: return OPSSL_AEAD_CHACHA20_POLY1305;
    case OPSEC_SUITE_AES_128_GCM:       return OPSSL_AEAD_AES_128_GCM;
    case OPSEC_SUITE_AES_256_GCM:       return OPSSL_AEAD_AES_256_GCM;
    default: return -1;
    }
}

/* Simple 32-bit xorshift PRNG for traffic analysis padding */
static uint32_t xorshift32(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

/* ──── Cipher Suite Negotiation ─────────────────────────────────────────── */

const opsec_suite_info_t *opsec_suite_info(opsec_suite_t suite)
{
    if (suite >= OPSEC_SUITE_COUNT) {
        return NULL;
    }
    return &suite_table[suite];
}

int opsec_negotiate_suite(const opsec_suite_t *local, uint8_t local_count,
                          const opsec_suite_t *remote, uint8_t remote_count)
{
    if (!local || !remote || local_count == 0 || remote_count == 0) {
        return -1;
    }

    /* Find the first local suite that also appears in remote list */
    for (uint8_t i = 0; i < local_count; i++) {
        opsec_suite_t candidate = local[i];

        /* Check if this suite is valid */
        if (candidate >= OPSEC_SUITE_COUNT) {
            continue;
        }

        /* Check if remote supports this suite */
        for (uint8_t j = 0; j < remote_count; j++) {
            if (remote[j] == candidate) {
                return (int)candidate;
            }
        }
    }

    return -1; /* No mutual support */
}

/* ──── Traffic Analysis Countermeasures ───────────────────────────────────── */

int opsec_set_traffic_protection(opsec_ctx_t *ctx, uint32_t flags, uint16_t cbr_size)
{
    if (!ctx || !ctx->initialized) {
        return -1;
    }

    ctx->traffic_flags = flags;
    ctx->cbr_target_size = cbr_size;

    /* Auto-detect CBR size if not specified */
    if ((flags & OPSEC_TRAFFIC_CBR) && cbr_size == 0) {
        ctx->cbr_target_size = OPSEC_MAX_PAYLOAD / 2; /* Default to 2048 bytes */
    }

    return 0;
}

int opsec_seal_dummy(opsec_ctx_t *ctx, uint8_t *out, size_t out_cap)
{
    if (!ctx || !ctx->initialized || !out) {
        return -1;
    }

    /* Generate pseudo-random dummy payload */
    uint8_t dummy[64];
    uint32_t rng = ctx->send_seq ^ 0xDEADBEEF; /* Seed with sequence number */

    for (size_t i = 0; i < sizeof(dummy); i++) {
        dummy[i] = (uint8_t)xorshift32(&rng);
    }

    if (ctx->v2_mode) {
        /* V2 mode: use dummy frame type */
        return opsec_seal_v2(ctx, OPSEC_FRAME_DUMMY, ctx->my_sender_id, 0,
                           dummy, sizeof(dummy), out, out_cap);
    } else {
        /* V1 mode: just seal normally (receiver has no way to detect dummy) */
        return opsec_seal(ctx, dummy, sizeof(dummy), out, out_cap);
    }
}

/* ──── Security Helpers ──────────────────────────────────────────────────── */

void opsec_zeroize(void *buf, size_t len)
{
    volatile uint8_t *p = (volatile uint8_t *)buf;
    while (len--) {
        *p++ = 0;
    }
}

