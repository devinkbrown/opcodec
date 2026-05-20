/* opcodec/secure.h — Per-frame AEAD encryption for media
 *
 * Provides:
 *   - ChaCha20-Poly1305 AEAD encryption per frame
 *   - SFrame/DAVE-inspired extended header format
 *   - Per-sender key derivation for group scenarios
 *   - Salt-based XOR nonce construction
 *   - Generation-based key ratcheting with forward secrecy
 *   - Extended anti-replay with 512-packet window
 *   - Grace period key retention for out-of-order frames
 *
 * V2 encrypted frame format:
 *   [version:1][frame_type:1][sender_id:4][counter:4][epoch:2][codec_id:1][flags:1][encrypted_payload:N][auth_tag:16]
 *
 * V1 legacy format (for backward compatibility):
 *   [epoch:2][seq:4][encrypted_payload:N][auth_tag:16]
 *
 * AAD includes entire cleartext header
 * Nonce = XOR(sender_salt, counter_as_12_byte_big_endian) for enhanced randomization
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_SECURE_H
#define OPCODEC_SECURE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define OPSEC_KEY_SIZE       32   /* ChaCha20 key */
#define OPSEC_SALT_SIZE      12   /* Salt for nonce XOR */
#define OPSEC_TAG_SIZE       16   /* Poly1305 tag */
#define OPSEC_NONCE_SIZE     12
#define OPSEC_EPOCH_SIZE     2
#define OPSEC_SEQ_SIZE       4

/* V2 Header fields */
#define OPSEC_V2_VERSION_SIZE     1
#define OPSEC_V2_FRAME_TYPE_SIZE  1
#define OPSEC_V2_SENDER_ID_SIZE   4
#define OPSEC_V2_COUNTER_SIZE     4
#define OPSEC_V2_CODEC_ID_SIZE    1
#define OPSEC_V2_FLAGS_SIZE       1
#define OPSEC_V2_HEADER_SIZE      (OPSEC_V2_VERSION_SIZE + OPSEC_V2_FRAME_TYPE_SIZE + \
                                   OPSEC_V2_SENDER_ID_SIZE + OPSEC_V2_COUNTER_SIZE + \
                                   OPSEC_EPOCH_SIZE + OPSEC_V2_CODEC_ID_SIZE + \
                                   OPSEC_V2_FLAGS_SIZE)  /* 14 bytes */

/* Legacy V1 header */
#define OPSEC_V1_HEADER_SIZE      (OPSEC_EPOCH_SIZE + OPSEC_SEQ_SIZE)  /* 6 bytes */

/* Overhead calculations */
#define OPSEC_V2_OVERHEAD         (OPSEC_V2_HEADER_SIZE + OPSEC_TAG_SIZE)  /* 30 bytes */
#define OPSEC_OVERHEAD            (OPSEC_V1_HEADER_SIZE + OPSEC_TAG_SIZE)  /* 22 bytes (legacy) */

#define OPSEC_MAX_PAYLOAD         4096
#define OPSEC_REPLAY_WINDOW       512  /* Extended sliding window for video */
#define OPSEC_PAD_BLOCK           16   /* pad to nearest 16 bytes */

/* Frame types */
#define OPSEC_FRAME_AUDIO         0x00
#define OPSEC_FRAME_VIDEO_KEY     0x01
#define OPSEC_FRAME_VIDEO_DELTA   0x02
#define OPSEC_FRAME_DUMMY         0xFF  /* Dummy frame for traffic analysis resistance */

/* Flags */
#define OPSEC_FLAG_PADDED         0x01
#define OPSEC_FLAG_REKEYING       0x02

/* Protocol version */
#define OPSEC_VERSION_V1          0x00
#define OPSEC_VERSION_V2          0x01

/* Generation ratcheting */
#define OPSEC_RATCHET_INTERVAL    65536  /* 2^16 frames */
#define OPSEC_MAX_SENDERS         4     /* Small group size */
#define OPSEC_GRACE_PERIOD        8192  /* frames to retain old epoch keys */

/* Return codes */
#define OPSEC_DUMMY_FRAME         -2    /* Returned when a dummy frame is decoded */

/* Cipher suite identifiers */
typedef enum {
    OPSEC_SUITE_CHACHA20_POLY1305  = 0,  /* default, no AES-NI needed */
    OPSEC_SUITE_AES_128_GCM        = 1,  /* for AES-NI platforms */
    OPSEC_SUITE_AES_256_GCM        = 2,  /* high-security mode */
    OPSEC_SUITE_COUNT              = 3
} opsec_suite_t;

/* Cipher suite properties */
typedef struct {
    opsec_suite_t id;
    uint8_t       key_size;     /* key size in bytes */
    uint8_t       tag_size;     /* auth tag size in bytes */
    uint8_t       nonce_size;   /* nonce size in bytes */
    const char   *name;
} opsec_suite_info_t;

/* Traffic analysis resistance flags */
#define OPSEC_TRAFFIC_CBR         (1 << 0)  /* pad all frames to constant size */
#define OPSEC_TRAFFIC_CONTINUOUS  (1 << 1)  /* send dummy frames during silence */
#define OPSEC_TRAFFIC_RANDOM_PAD  (1 << 2)  /* add random extra padding */

typedef struct opsec_ctx {
    /* Legacy V1 keys (for backward compatibility) */
    uint8_t  send_key[OPSEC_KEY_SIZE];
    uint8_t  recv_key[OPSEC_KEY_SIZE];
    uint16_t send_epoch;
    uint16_t recv_epoch;
    uint32_t send_seq;

    /* V2 per-sender keys and salts */
    uint32_t my_sender_id;                              /* Our sender ID */
    uint8_t  sender_keys[OPSEC_MAX_SENDERS][OPSEC_KEY_SIZE];  /* Per-sender keys */
    uint8_t  sender_salts[OPSEC_MAX_SENDERS][OPSEC_SALT_SIZE]; /* Per-sender salts */
    uint32_t sender_counters[OPSEC_MAX_SENDERS];        /* Per-sender frame counters */

    /* Generation-based key ratcheting */
    uint32_t send_generation;                           /* Current send generation */
    uint32_t recv_generation[OPSEC_MAX_SENDERS];       /* Per-sender recv generations */

    /* Extended anti-replay sliding window (512 packets) */
    uint32_t replay_seq_base;
    uint64_t replay_bitmap[OPSEC_REPLAY_WINDOW / 64];  /* 8 uint64_t words */

    /* Grace period key retention */
    uint8_t  prev_recv_key[OPSEC_KEY_SIZE];            /* Previous epoch key */
    uint16_t prev_recv_epoch;                          /* Previous epoch number */
    uint32_t prev_epoch_deadline;                      /* Counter deadline for old epoch */

    /* Key ratchet state */
    uint8_t  root_secret[OPSEC_KEY_SIZE];

    /* Cipher suite and traffic analysis resistance */
    opsec_suite_t suite;                               /* active cipher suite */
    uint32_t traffic_flags;                            /* OPSEC_TRAFFIC_* flags */
    uint16_t cbr_target_size;                          /* target encrypted frame size for CBR mode */

    bool initialized;
    bool v2_mode;                                       /* Use V2 format */
} opsec_ctx_t;

/* Initialize from shared secret (e.g., from VEIL handshake).
 * Derives send_key and recv_key using HKDF.
 * is_initiator determines which key is send vs recv.
 * sender_id identifies this participant in group scenarios. */
int opsec_init(opsec_ctx_t *ctx, const uint8_t *shared_secret,
               size_t secret_len, bool is_initiator);

/* Initialize with V2 mode and sender ID */
int opsec_init_v2(opsec_ctx_t *ctx, const uint8_t *shared_secret,
                  size_t secret_len, uint32_t sender_id);

/* Initialize with specific cipher suite */
int opsec_init_suite(opsec_ctx_t *ctx, const uint8_t *shared_secret,
                     size_t secret_len, bool is_initiator,
                     opsec_suite_t suite);

/* Derive per-sender key and salt for group scenarios */
int opsec_derive_sender_key(opsec_ctx_t *ctx,
                            uint32_t sender_id,
                            uint8_t *key_out, size_t key_len,
                            uint8_t *salt_out, size_t salt_len);

/* V2 encrypt with full header support */
int opsec_seal_v2(opsec_ctx_t *ctx,
                  uint8_t frame_type, uint32_t sender_id, uint8_t codec_id,
                  const uint8_t *plaintext, size_t pt_len,
                  uint8_t *out, size_t out_cap);

/* V2 decrypt with header extraction */
int opsec_open_v2(opsec_ctx_t *ctx,
                  const uint8_t *ciphertext, size_t ct_len,
                  uint8_t *out, size_t out_cap,
                  uint8_t *frame_type_out, uint32_t *sender_id_out);

/* Legacy V1 encrypt - backward compatible */
int opsec_seal(opsec_ctx_t *ctx,
               const uint8_t *plaintext, size_t pt_len,
               uint8_t *out, size_t out_cap);

/* Legacy V1 decrypt - auto-detects format */
int opsec_open(opsec_ctx_t *ctx,
               const uint8_t *ciphertext, size_t ct_len,
               uint8_t *out, size_t out_cap);

/* Rotate keys (advance to next epoch).
 * Derives new send/recv keys from root_secret + current keys.
 * Called periodically or on rekeying event. */
int opsec_rotate(opsec_ctx_t *ctx);

/* Pad plaintext to fixed block size to resist traffic analysis.
 * Returns padded length. */
size_t opsec_pad(uint8_t *buf, size_t data_len, size_t buf_cap);

/* Remove padding. Returns original data length. */
size_t opsec_unpad(const uint8_t *buf, size_t padded_len);

/* ──── Cipher Suite Negotiation ─────────────────────────────────────────── */

/* Get cipher suite properties */
const opsec_suite_info_t *opsec_suite_info(opsec_suite_t suite);

/* Negotiate cipher suite from a list of supported suites.
 * Returns the highest-priority mutually supported suite, or -1 if none. */
int opsec_negotiate_suite(const opsec_suite_t *local, uint8_t local_count,
                          const opsec_suite_t *remote, uint8_t remote_count);

/* ──── Traffic Analysis Countermeasures ───────────────────────────────────── */

/* Configure traffic analysis resistance.
 * flags: bitmask of OPSEC_TRAFFIC_* flags
 * cbr_size: target constant frame size for CBR mode (0 = auto-detect)
 * Returns 0 on success. */
int opsec_set_traffic_protection(opsec_ctx_t *ctx, uint32_t flags, uint16_t cbr_size);

/* Generate a dummy (keepalive) encrypted frame.
 * Used with OPSEC_TRAFFIC_CONTINUOUS to fill silence gaps.
 * The frame encrypts random padding and is indistinguishable from real data.
 * Returns total bytes written or -1 on error. */
int opsec_seal_dummy(opsec_ctx_t *ctx, uint8_t *out, size_t out_cap);

/* ──── Security Helpers ──────────────────────────────────────────────────── */

/* Securely erase sensitive memory (compiler-safe) */
void opsec_zeroize(void *buf, size_t len);

#endif /* OPCODEC_SECURE_H */