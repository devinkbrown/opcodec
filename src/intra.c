/* opcodec/intra.c — Intra prediction: legacy 6-mode + HEVC 35-mode
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/intra.h"
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* ========================================================================
 * 16x16 MACROBLOCK INTRA PREDICTION
 * ======================================================================== */

void intra_predict_16x16(intra_mode_t mode,
                         const uint8_t *top, const uint8_t *left,
                         uint8_t top_left,
                         uint8_t pred[16][16]) {

    switch (mode) {
        case INTRA_DC: {
            /* DC mode: fill with average of top + left neighbors */
            int sum = 0, count = 0;

            if (top) {
                for (int i = 0; i < 16; i++) {
                    sum += top[i];
                }
                count += 16;
            }

            if (left) {
                for (int i = 0; i < 16; i++) {
                    sum += left[i];
                }
                count += 16;
            }

            uint8_t dc = (count > 0) ? (sum + count/2) / count : 128;

            /* Fill entire block with dc value */
            for (int y = 0; y < 16; y++) {
                for (int x = 0; x < 16; x++) {
                    pred[y][x] = dc;
                }
            }
            break;
        }

        case INTRA_HORIZ: {
            /* Horizontal mode: copy left column across each row */
            if (!left) {
                /* No left neighbors available, fill with default */
                for (int y = 0; y < 16; y++) {
                    for (int x = 0; x < 16; x++) {
                        pred[y][x] = 128;
                    }
                }
                return;
            }

            for (int y = 0; y < 16; y++) {
                for (int x = 0; x < 16; x++) {
                    pred[y][x] = left[y];
                }
            }
            break;
        }

        case INTRA_VERT: {
            /* Vertical mode: copy top row down each column */
            if (!top) {
                /* No top neighbors available, fill with default */
                for (int y = 0; y < 16; y++) {
                    for (int x = 0; x < 16; x++) {
                        pred[y][x] = 128;
                    }
                }
                return;
            }

            for (int y = 0; y < 16; y++) {
                for (int x = 0; x < 16; x++) {
                    pred[y][x] = top[x];
                }
            }
            break;
        }

        case INTRA_DIAG_DL: {
            /* Diagonal down-left: top-right to bottom-left */
            if (!top) {
                /* No top neighbors available, fill with default */
                for (int y = 0; y < 16; y++) {
                    for (int x = 0; x < 16; x++) {
                        pred[y][x] = 128;
                    }
                }
                return;
            }

            for (int y = 0; y < 16; y++) {
                for (int x = 0; x < 16; x++) {
                    int idx = x + y + 1;
                    if (idx < 16) {
                        pred[y][x] = top[idx];
                    } else {
                        /* Use rightmost available pixel */
                        pred[y][x] = top[15];
                    }
                }
            }
            break;
        }

        case INTRA_DIAG_DR: {
            /* Diagonal down-right: top-left to bottom-right */
            for (int y = 0; y < 16; y++) {
                for (int x = 0; x < 16; x++) {
                    if (x > y && top) {
                        /* Above diagonal: use top neighbors */
                        int idx = x - y - 1;
                        pred[y][x] = (idx >= 0 && idx < 16) ? top[idx] : top_left;
                    } else if (x < y && left) {
                        /* Below diagonal: use left neighbors */
                        int idx = y - x - 1;
                        pred[y][x] = (idx >= 0 && idx < 16) ? left[idx] : top_left;
                    } else {
                        /* On diagonal or no neighbors: use top_left */
                        pred[y][x] = top_left;
                    }
                }
            }
            break;
        }

        case INTRA_PLANE: {
            /* Planar prediction: H.264-style plane mode */
            if (!top || !left) {
                /* Fall back to DC mode if neighbors not available */
                intra_predict_16x16(INTRA_DC, top, left, top_left, pred);
                return;
            }

            /* Calculate horizontal and vertical gradients */
            int H = 0, V = 0;

            for (int i = 0; i < 8; i++) {
                H += (i + 1) * (top[8 + i] - top[6 - i]);
                V += (i + 1) * (left[8 + i] - left[6 - i]);
            }

            /* Base value and gradient coefficients */
            int a = 16 * (top[15] + left[15]);
            int b = (5 * H + 32) >> 6;
            int c = (5 * V + 32) >> 6;

            /* Generate planar prediction */
            for (int y = 0; y < 16; y++) {
                for (int x = 0; x < 16; x++) {
                    int val = (a + b * (x - 7) + c * (y - 7) + 16) >> 5;
                    pred[y][x] = CLAMP(val, 0, 255);
                }
            }
            break;
        }

        default:
            /* Unknown mode, fall back to DC */
            intra_predict_16x16(INTRA_DC, top, left, top_left, pred);
            break;
    }
}

intra_mode_t intra_find_best_mode(const uint8_t orig[16][16],
                                  const uint8_t *top, const uint8_t *left,
                                  uint8_t top_left) {
    uint32_t best_sad = UINT32_MAX;
    intra_mode_t best_mode = INTRA_DC;

    for (int m = 0; m < INTRA_NUM_MODES; m++) {
        uint8_t pred[16][16];
        intra_predict_16x16((intra_mode_t)m, top, left, top_left, pred);

        /* Calculate SAD between original and prediction */
        uint32_t sad = 0;
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 16; x++) {
                sad += abs((int)orig[y][x] - (int)pred[y][x]);
            }
        }

        if (sad < best_sad) {
            best_sad = sad;
            best_mode = (intra_mode_t)m;
        }
    }

    return best_mode;
}

/* ========================================================================
 * 4x4 SUB-BLOCK INTRA PREDICTION
 * ======================================================================== */

void intra_predict_4x4(intra_mode_t mode,
                       const uint8_t *top, const uint8_t *left,
                       uint8_t top_left,
                       uint8_t pred[4][4]) {

    switch (mode) {
        case INTRA_DC: {
            /* DC mode: fill with average of top + left neighbors */
            int sum = 0, count = 0;

            if (top) {
                for (int i = 0; i < 4; i++) {
                    sum += top[i];
                }
                count += 4;
            }

            if (left) {
                for (int i = 0; i < 4; i++) {
                    sum += left[i];
                }
                count += 4;
            }

            uint8_t dc = (count > 0) ? (sum + count/2) / count : 128;

            /* Fill entire block with dc value */
            for (int y = 0; y < 4; y++) {
                for (int x = 0; x < 4; x++) {
                    pred[y][x] = dc;
                }
            }
            break;
        }

        case INTRA_HORIZ: {
            /* Horizontal mode: copy left column across each row */
            if (!left) {
                /* No left neighbors available, fill with default */
                for (int y = 0; y < 4; y++) {
                    for (int x = 0; x < 4; x++) {
                        pred[y][x] = 128;
                    }
                }
                return;
            }

            for (int y = 0; y < 4; y++) {
                for (int x = 0; x < 4; x++) {
                    pred[y][x] = left[y];
                }
            }
            break;
        }

        case INTRA_VERT: {
            /* Vertical mode: copy top row down each column */
            if (!top) {
                /* No top neighbors available, fill with default */
                for (int y = 0; y < 4; y++) {
                    for (int x = 0; x < 4; x++) {
                        pred[y][x] = 128;
                    }
                }
                return;
            }

            for (int y = 0; y < 4; y++) {
                for (int x = 0; x < 4; x++) {
                    pred[y][x] = top[x];
                }
            }
            break;
        }

        case INTRA_DIAG_DL: {
            /* Diagonal down-left: top-right to bottom-left */
            if (!top) {
                /* No top neighbors available, fill with default */
                for (int y = 0; y < 4; y++) {
                    for (int x = 0; x < 4; x++) {
                        pred[y][x] = 128;
                    }
                }
                return;
            }

            for (int y = 0; y < 4; y++) {
                for (int x = 0; x < 4; x++) {
                    int idx = x + y + 1;
                    if (idx < 4) {
                        pred[y][x] = top[idx];
                    } else {
                        /* Use rightmost available pixel */
                        pred[y][x] = top[3];
                    }
                }
            }
            break;
        }

        case INTRA_DIAG_DR: {
            /* Diagonal down-right: top-left to bottom-right */
            for (int y = 0; y < 4; y++) {
                for (int x = 0; x < 4; x++) {
                    if (x > y && top) {
                        /* Above diagonal: use top neighbors */
                        int idx = x - y - 1;
                        pred[y][x] = (idx >= 0 && idx < 4) ? top[idx] : top_left;
                    } else if (x < y && left) {
                        /* Below diagonal: use left neighbors */
                        int idx = y - x - 1;
                        pred[y][x] = (idx >= 0 && idx < 4) ? left[idx] : top_left;
                    } else {
                        /* On diagonal or no neighbors: use top_left */
                        pred[y][x] = top_left;
                    }
                }
            }
            break;
        }

        case INTRA_PLANE: {
            /* Planar prediction adapted for 4x4 blocks */
            if (!top || !left) {
                /* Fall back to DC mode if neighbors not available */
                intra_predict_4x4(INTRA_DC, top, left, top_left, pred);
                return;
            }

            /* Simplified planar prediction for 4x4 */
            /* Calculate horizontal and vertical gradients */
            int H = 0, V = 0;

            for (int i = 0; i < 2; i++) {
                H += (i + 1) * (top[2 + i] - top[1 - i]);
                V += (i + 1) * (left[2 + i] - left[1 - i]);
            }

            /* Base value and gradient coefficients */
            int a = 16 * (top[3] + left[3]);
            int b = (5 * H + 32) >> 6;
            int c = (5 * V + 32) >> 6;

            /* Generate planar prediction */
            for (int y = 0; y < 4; y++) {
                for (int x = 0; x < 4; x++) {
                    int val = (a + b * (x - 1) + c * (y - 1) + 16) >> 5;
                    pred[y][x] = CLAMP(val, 0, 255);
                }
            }
            break;
        }

        default:
            /* Unknown mode, fall back to DC */
            intra_predict_4x4(INTRA_DC, top, left, top_left, pred);
            break;
    }
}

intra_mode_t intra_find_best_mode_4x4(const uint8_t orig[4][4],
                                       const uint8_t *top, const uint8_t *left,
                                       uint8_t top_left) {
    uint32_t best_sad = UINT32_MAX;
    intra_mode_t best_mode = INTRA_DC;

    for (int m = 0; m < INTRA_NUM_MODES; m++) {
        uint8_t pred[4][4];
        intra_predict_4x4((intra_mode_t)m, top, left, top_left, pred);

        /* Calculate SAD between original and prediction */
        uint32_t sad = 0;
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                sad += abs((int)orig[y][x] - (int)pred[y][x]);
            }
        }

        if (sad < best_sad) {
            best_sad = sad;
            best_mode = (intra_mode_t)m;
        }
    }

    return best_mode;
}

/* ========================================================================
 * NEIGHBOR PIXEL EXTRACTION
 * ======================================================================== */

bool intra_get_neighbors_16x16(const uint8_t *frame, uint16_t width, uint16_t height,
                               uint16_t mb_x, uint16_t mb_y,
                               uint8_t top_out[16], uint8_t left_out[16],
                               uint8_t *top_left_out) {
    bool any_neighbors = false;

    /* Clear output buffers first */
    if (top_out) {
        memset(top_out, 128, 16);
    }
    if (left_out) {
        memset(left_out, 128, 16);
    }
    if (top_left_out) {
        *top_left_out = 128;
    }

    /* Calculate pixel coordinates */
    const int mb_pixel_x = mb_x * 16;
    const int mb_pixel_y = mb_y * 16;

    /* Extract top neighbors (above the macroblock) */
    if (mb_y > 0 && top_out) {
        const int top_row = mb_pixel_y - 1;
        for (int x = 0; x < 16; x++) {
            if (mb_pixel_x + x < width) {
                top_out[x] = frame[top_row * width + mb_pixel_x + x];
            }
        }
        any_neighbors = true;
    } else if (top_out) {
        /* Set to NULL to indicate no top neighbors */
        top_out = NULL;
    }

    /* Extract left neighbors (left of the macroblock) */
    if (mb_x > 0 && left_out) {
        const int left_col = mb_pixel_x - 1;
        for (int y = 0; y < 16; y++) {
            if (mb_pixel_y + y < height) {
                left_out[y] = frame[(mb_pixel_y + y) * width + left_col];
            }
        }
        any_neighbors = true;
    } else if (left_out) {
        /* Set to NULL to indicate no left neighbors */
        left_out = NULL;
    }

    /* Extract top-left corner pixel */
    if (mb_x > 0 && mb_y > 0 && top_left_out) {
        const int top_left_y = mb_pixel_y - 1;
        const int top_left_x = mb_pixel_x - 1;
        *top_left_out = frame[top_left_y * width + top_left_x];
        any_neighbors = true;
    }

    return any_neighbors;
}

bool intra_get_neighbors_4x4(const uint8_t *frame, uint16_t width, uint16_t height,
                              uint16_t block_x, uint16_t block_y,
                              uint8_t top_out[4], uint8_t left_out[4],
                              uint8_t *top_left_out) {
    bool any_neighbors = false;

    /* Clear output buffers first */
    if (top_out) {
        memset(top_out, 128, 4);
    }
    if (left_out) {
        memset(left_out, 128, 4);
    }
    if (top_left_out) {
        *top_left_out = 128;
    }

    /* Calculate pixel coordinates */
    const int block_pixel_x = block_x * 4;
    const int block_pixel_y = block_y * 4;

    /* Extract top neighbors (above the 4x4 block) */
    if (block_y > 0 && top_out) {
        const int top_row = block_pixel_y - 1;
        for (int x = 0; x < 4; x++) {
            if (block_pixel_x + x < width) {
                top_out[x] = frame[top_row * width + block_pixel_x + x];
            }
        }
        any_neighbors = true;
    } else if (top_out) {
        /* Set to NULL to indicate no top neighbors */
        top_out = NULL;
    }

    /* Extract left neighbors (left of the 4x4 block) */
    if (block_x > 0 && left_out) {
        const int left_col = block_pixel_x - 1;
        for (int y = 0; y < 4; y++) {
            if (block_pixel_y + y < height) {
                left_out[y] = frame[(block_pixel_y + y) * width + left_col];
            }
        }
        any_neighbors = true;
    } else if (left_out) {
        /* Set to NULL to indicate no left neighbors */
        left_out = NULL;
    }

    /* Extract top-left corner pixel */
    if (block_x > 0 && block_y > 0 && top_left_out) {
        const int top_left_y = block_pixel_y - 1;
        const int top_left_x = block_pixel_x - 1;
        *top_left_out = frame[top_left_y * width + top_left_x];
        any_neighbors = true;
    }

    return any_neighbors;
}

/* ========================================================================
 * HEVC 35-MODE INTRA PREDICTION
 *
 * Mode layout (HEVC spec, Table 8-5):
 *   Mode  0: Planar
 *   Mode  1: DC
 *   Modes 2-18: angular, vertical family (use top reference)
 *     mode  2: angle= 32 ('\' diagonal)
 *     mode 10: angle=  0 (pure vertical)
 *     mode 18: angle=-32 ('/' diagonal)
 *   Modes 19-34: angular, horizontal family (use left ref, transposed)
 *     mode 19: angle=-26
 *     mode 26: angle=  0 (pure horizontal)
 *     mode 34: angle= 32
 *
 * Positive angle means reference shifts toward higher indices as rows/cols
 * increase.  Negative angle means reference shifts toward lower indices.
 * ======================================================================== */

/* intraPredAngle[mode-2] for modes 2..34 — from HEVC spec Table 8-5 */
static const int8_t kAngle[33] = {
    32, 26, 21, 17, 13,  9,  5,  2,   /* modes  2.. 9 */
     0,                                /* mode  10 (true vertical) */
    -2, -5, -9,-13,-17,-21,-26,-32,   /* modes 11..18 */
   -26,-21,-17,-13, -9, -5, -2,  0,   /* modes 19..26 (mode 26 = true horizontal) */
     2,  5,  9, 13, 17, 21, 26, 32    /* modes 27..34 */
};

/* invAngle[mode-2] = round(8192 / |intraPredAngle|), 0 where angle==0.
 * Used to extend the reference array in the opposite direction.
 */
static const int16_t kInvAngle[33] = {
      256,  315,  390,  482,  630, 1024, 2048, 4096,  /* modes  2.. 9 */
        0,                                              /* mode  10 */
     4096, 2048, 1024,  630,  482,  390,  315,  256,  /* modes 11..18 */
      315,  390,  482,  630, 1024, 2048, 4096,    0,  /* modes 19..26 */
     4096, 2048, 1024,  630,  482,  390,  315,  256   /* modes 27..34 */
};

/*
 * Maximum side-reference extension needed: angle=-32, N=64 → 64 samples.
 * We over-provision to 68 for safety.
 */
#define REF_SIDE_EXT 68

/*
 * Build the reference array for angular modes.
 *
 * For vertical modes (horizontal=false):
 *   p[0]     = top_left
 *   p[1..N]  = top[0..N-1]
 *   p[N+1..2N] = top_right (if avail) or replicated top[N-1]
 *   p[-1..-M] = derived from left column (when angle < 0)
 *
 * For horizontal modes (horizontal=true): swap top↔left in above.
 *
 * buf must have at least REF_SIDE_EXT + 1 + 2*N bytes.
 * Returns pointer to p[0] (= top_left position) inside buf.
 */
static uint8_t *build_ref(bool horizontal, int N,
                           const uint8_t *top,  const uint8_t *top_right,
                           const uint8_t *left, uint8_t top_left,
                           int8_t angle, int16_t inv_angle,
                           uint8_t *buf)
{
    uint8_t *p = buf + REF_SIDE_EXT;   /* p[0] = top_left */
    const uint8_t *main_ref  = horizontal ? left : top;
    const uint8_t *right_ext = horizontal ? NULL : top_right;
    const uint8_t *side_ref  = horizontal ? top  : left;

    /* p[0] = top_left corner */
    p[0] = top_left;

    /* p[1..N] = main reference (top or left) */
    for (int k = 1; k <= N; k++)
        p[k] = main_ref ? main_ref[k - 1] : top_left;

    /* p[N+1..2N] = extension (top_right for vertical, replicate otherwise) */
    for (int k = N + 1; k <= 2 * N; k++) {
        if (right_ext)
            p[k] = right_ext[k - N - 1];
        else
            p[k] = p[N];
    }

    /* p[-1..-M] = side reference for negative-angle modes */
    if (angle < 0) {
        int n_ext = ((-angle) * N + 31) >> 5;
        if (n_ext > REF_SIDE_EXT) n_ext = REF_SIDE_EXT;
        for (int k = 1; k <= n_ext; k++) {
            int idx = (k * inv_angle + 128) >> 8;
            if (idx < 0) idx = 0;
            if (idx >= N) idx = N - 1;
            p[-k] = side_ref ? side_ref[idx] : top_left;
        }
    }

    return p;
}

void intra_predict_hevc(intra_mode_hevc_t mode, int block_size,
                        const uint8_t *top, const uint8_t *top_right,
                        const uint8_t *left, uint8_t top_left,
                        uint8_t *pred)
{
    int N = block_size;

    /* ---- Mode 0: Planar ---- */
    if (mode == INTRA_HEVC_PLANAR) {
        /* HEVC Planar: bilinear blend from top, left, bottom-left, top-right */
        uint8_t tr = top  ? top[N - 1]  : top_left;
        uint8_t bl = left ? left[N - 1] : top_left;

        for (int y = 0; y < N; y++) {
            for (int x = 0; x < N; x++) {
                int v_top  = top  ? top[x]  : top_left;
                int v_left = left ? left[y] : top_left;
                int val = (N - 1 - x) * v_left + (x + 1) * (int)tr
                        + (N - 1 - y) * v_top  + (y + 1) * (int)bl
                        + N;
                pred[y * N + x] = (uint8_t)CLAMP(val >> (int)(log2f((float)N) + 1), 0, 255);
            }
        }
        return;
    }

    /* ---- Mode 1: DC ---- */
    if (mode == INTRA_HEVC_DC) {
        int sum = 0, count = 0;
        if (top)  { for (int i = 0; i < N; i++) sum += top[i];  count += N; }
        if (left) { for (int i = 0; i < N; i++) sum += left[i]; count += N; }
        uint8_t dc = count ? (uint8_t)((sum + count / 2) / count) : 128;
        for (int i = 0; i < N * N; i++)
            pred[i] = dc;
        return;
    }

    /* ---- Modes 2-34: Angular ---- */
    if (mode < 2 || mode > 34) {
        /* Fallback: DC */
        intra_predict_hevc(INTRA_HEVC_DC, N, top, top_right, left, top_left, pred);
        return;
    }

    int idx        = mode - 2;
    int8_t  angle  = kAngle[idx];
    int16_t inv_a  = kInvAngle[idx];
    bool horizontal = (mode >= 19);

    /* buf provides REF_SIDE_EXT negative slots, 1 TL, 2N forward */
    uint8_t buf[REF_SIDE_EXT + 1 + 2 * 64];
    uint8_t *p = build_ref(horizontal, N, top, top_right, left, top_left,
                           angle, inv_a, buf);

    if (angle == 0) {
        /* Pure vertical (mode 10) or horizontal (mode 26) */
        for (int y = 0; y < N; y++)
            for (int x = 0; x < N; x++)
                pred[y * N + x] = horizontal ? p[y + 1] : p[x + 1];
        return;
    }

    /*
     * Angular interpolation.
     * For vertical modes: outer=row(y), inner=col(x).
     * For horizontal modes: outer=col(x), inner=row(y) — transpose output.
     */
    for (int j = 0; j < N; j++) {
        int step = (j + 1) * (int)angle;   /* 1/32-pixel displacement */
        int iIdx = step >> 5;               /* integer part (signed) */
        int iFact = step & 31;              /* fractional part 0-31 */

        for (int i = 0; i < N; i++) {
            int r0 = p[i + 1 + iIdx];
            int val;
            if (iFact) {
                int r1 = p[i + 2 + iIdx];
                val = ((32 - iFact) * r0 + iFact * r1 + 16) >> 5;
            } else {
                val = r0;
            }
            if (val < 0) val = 0; else if (val > 255) val = 255;

            if (horizontal)
                pred[i * N + j] = (uint8_t)val;  /* transposed */
            else
                pred[j * N + i] = (uint8_t)val;
        }
    }
}

/* 4×4 Hadamard SATD: compute the sum of absolute Hadamard-transformed differences
 * for a flat 4×4 residual block.  Uses the standard butterfly decomposition. */
static uint32_t satd_4x4(const int16_t res[4][4]) {
    int16_t t[4][4];
    /* Horizontal butterfly */
    for (int r = 0; r < 4; r++) {
        int a0 = res[r][0] + res[r][2], a1 = res[r][1] + res[r][3];
        int a2 = res[r][0] - res[r][2], a3 = res[r][1] - res[r][3];
        t[r][0] = (int16_t)(a0 + a1); t[r][1] = (int16_t)(a0 - a1);
        t[r][2] = (int16_t)(a2 + a3); t[r][3] = (int16_t)(a2 - a3);
    }
    uint32_t satd = 0;
    /* Vertical butterfly + accumulate */
    for (int c = 0; c < 4; c++) {
        int a0 = t[0][c] + t[2][c], a1 = t[1][c] + t[3][c];
        int a2 = t[0][c] - t[2][c], a3 = t[1][c] - t[3][c];
        satd += (uint32_t)(abs(a0 + a1) + abs(a0 - a1) +
                           abs(a2 + a3) + abs(a2 - a3));
    }
    return satd;
}

/* Compute SATD (sum of 4×4-block Hadamard transformed differences) for an N×N block.
 * Falls back to SAD for odd remainder sub-blocks to keep code simple. */
static uint32_t compute_satd(const uint8_t *orig, const uint8_t *pred, int N) {
    uint32_t total = 0;
    int blocks = N / 4;
    for (int br = 0; br < blocks; br++) {
        for (int bc = 0; bc < blocks; bc++) {
            int16_t res[4][4];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++) {
                    int idx = (br * 4 + r) * N + (bc * 4 + c);
                    res[r][c] = (int16_t)((int)orig[idx] - (int)pred[idx]);
                }
            total += satd_4x4(res);
        }
    }
    /* Any remaining pixels (when N is not divisible by 4): use SAD */
    for (int i = 0; i < N * N; i++) {
        int row = i / N, col = i % N;
        if (row < blocks * 4 && col < blocks * 4) continue;
        total += (uint32_t)abs((int)orig[i] - (int)pred[i]);
    }
    return total;
}

intra_mode_hevc_t intra_find_best_mode_hevc(const uint8_t *orig, int block_size,
                                            const uint8_t *top,
                                            const uint8_t *top_right,
                                            const uint8_t *left,
                                            uint8_t top_left)
{
    int N = block_size;
    uint32_t best_cost = UINT32_MAX;
    intra_mode_hevc_t best_mode = INTRA_HEVC_DC;

    uint8_t pred_buf[64 * 64];

    for (intra_mode_hevc_t m = 0; m < INTRA_NUM_MODES_HEVC; m++) {
        intra_predict_hevc(m, N, top, top_right, left, top_left, pred_buf);

        /* SATD gives better mode discrimination than SAD by penalising
         * low-frequency prediction errors more heavily than high-frequency ones. */
        uint32_t cost = compute_satd(orig, pred_buf, N);

        if (cost < best_cost) {
            best_cost = cost;
            best_mode = m;
        }
    }

    return best_mode;
}

bool intra_get_neighbors_hevc(const uint8_t *frame,
                              uint16_t frame_width, uint16_t frame_height,
                              uint16_t bx, uint16_t by, int block_size,
                              uint8_t *top_out, uint8_t *top_right_out,
                              uint8_t *left_out, uint8_t *top_left_out)
{
    int N = block_size;
    bool any = false;

    /* Top row */
    if (by > 0) {
        for (int x = 0; x < N; x++) {
            int cx = bx + x;
            top_out[x] = cx < frame_width
                ? frame[(by - 1) * frame_width + cx]
                : top_out[x > 0 ? x - 1 : 0];
        }
        any = true;
    } else {
        memset(top_out, 128, (size_t)N);
    }

    /* Top-right extension */
    if (top_right_out) {
        if (by > 0) {
            for (int x = 0; x < N; x++) {
                int cx = bx + N + x;
                top_right_out[x] = cx < frame_width
                    ? frame[(by - 1) * frame_width + cx]
                    : top_out[N - 1];
            }
        } else {
            memset(top_right_out, 128, (size_t)N);
        }
    }

    /* Left column */
    if (bx > 0) {
        for (int y = 0; y < N; y++) {
            int cy = by + y;
            left_out[y] = cy < frame_height
                ? frame[cy * frame_width + bx - 1]
                : left_out[y > 0 ? y - 1 : 0];
        }
        any = true;
    } else {
        memset(left_out, 128, (size_t)N);
    }

    /* Top-left corner */
    if (top_left_out) {
        if (bx > 0 && by > 0)
            *top_left_out = frame[(by - 1) * frame_width + bx - 1];
        else if (bx > 0)
            *top_left_out = frame[by * frame_width + bx - 1];
        else if (by > 0)
            *top_left_out = frame[(by - 1) * frame_width + bx];
        else
            *top_left_out = 128;

        any |= (bx > 0 || by > 0);
    }

    return any;
}