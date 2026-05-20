/* opcodec/screen.c — Screen content coding: palette mode + Intra Block Copy
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/screen.h"
#include <string.h>
#include <stdlib.h>

/* ---- Palette ---- */

bool screen_palette_detect(const uint8_t *block, int N,
                           uint8_t *palette_y, int *n_entries,
                           uint8_t *indices) {
    bool seen[256];
    memset(seen, 0, sizeof(seen));
    int count = 0;

    for (int i = 0; i < N * N; i++) {
        if (!seen[block[i]]) {
            seen[block[i]] = true;
            if (++count > SCREEN_PALETTE_MAX)
                return false;
        }
    }

    /* Build sorted palette (low-to-high Y ordering) */
    uint8_t lut[256];
    *n_entries = 0;
    for (int v = 0; v < 256; v++) {
        if (seen[v]) {
            lut[v] = (uint8_t)(*n_entries);
            palette_y[(*n_entries)++] = (uint8_t)v;
        }
    }

    for (int i = 0; i < N * N; i++)
        indices[i] = lut[block[i]];

    return true;
}

void screen_palette_reconstruct(const uint8_t *palette_y, const uint8_t *indices,
                                int N, uint8_t *dst) {
    for (int i = 0; i < N * N; i++)
        dst[i] = palette_y[indices[i]];
}

/* ---- IBC ---- */

uint32_t screen_ibc_hash(const uint8_t *frame, int fw, int bx, int by) {
    uint32_t h = 2166136261u;
    for (int row = 0; row < SCREEN_IBC_MIN_SIZE; row++) {
        const uint8_t *p = frame + (by + row) * fw + bx;
        for (int col = 0; col < SCREEN_IBC_MIN_SIZE; col++) {
            h ^= (uint32_t)p[col];
            h *= 16777619u;
        }
    }
    return h;
}

/* Linear probe depth for IBC hash table collision resolution */
#define SCREEN_IBC_PROBE_DEPTH 4

bool screen_ibc_search(const uint8_t *frame, int fw, int fh,
                       int bx, int by, int N,
                       const uint32_t *table, int table_size,
                       int *bv_x, int *bv_y) {
    if (!table || !frame) return false;
    if (bx + N > fw || by + N > fh) return false;

    uint32_t h    = screen_ibc_hash(frame, fw, bx, by);
    int      base = (int)(h % (uint32_t)table_size);

    for (int probe = 0; probe < SCREEN_IBC_PROBE_DEPTH; probe++) {
        int slot = (base + probe) % table_size;
        uint32_t entry = table[slot];

        if (entry == 0xFFFFFFFFu) break;  /* empty slot — stop probing */

        int ref_x = (int)((entry >> 16) & 0xFFFF);
        int ref_y = (int)(entry & 0xFFFF);

        /* Must be entirely before the current block in raster-scan order */
        if (ref_y * fw + ref_x + N > by * fw + bx) continue;
        if (ref_x + N > fw || ref_y + N > fh)      continue;

        /* Verify match quality: SAD must be below threshold */
        uint32_t sad = 0;
        for (int row = 0; row < N; row++)
            for (int col = 0; col < N; col++)
                sad += (uint32_t)abs((int)frame[(by + row) * fw + bx + col] -
                                     (int)frame[(ref_y + row) * fw + ref_x + col]);

        if (sad > (uint32_t)(N * N * 4)) continue;

        *bv_x = ref_x - bx;
        *bv_y = ref_y - by;
        return true;
    }
    return false;
}

void screen_ibc_update(const uint8_t *frame, int fw,
                       int bx, int by,
                       uint32_t *table, int table_size) {
    if (!table || !frame) return;
    uint32_t h    = screen_ibc_hash(frame, fw, bx, by);
    int      base = (int)(h % (uint32_t)table_size);
    uint32_t val  = ((uint32_t)(bx & 0xFFFF) << 16) | (uint32_t)(by & 0xFFFF);

    /* Linear probing: find first empty or same-hash slot */
    for (int probe = 0; probe < SCREEN_IBC_PROBE_DEPTH; probe++) {
        int slot = (base + probe) % table_size;
        if (table[slot] == 0xFFFFFFFFu || table[slot] == val) {
            table[slot] = val;
            return;
        }
    }
    /* All probe slots occupied — evict the base slot */
    table[base] = val;
}

/* screen_ibc_update_block: deprecated — delegates to screen_ibc_update via frame.
 * The frame pointer must be provided; use screen_ibc_update() directly instead. */
void screen_ibc_update_block(const uint8_t *frame, int fw,
                              int bx, int by,
                              uint32_t *table, int table_size) {
    screen_ibc_update(frame, fw, bx, by, table, table_size);
}
