/* cdef.c — Constrained Directional Enhancement Filter (AV1-style)
 *
 * Computes a dominant gradient direction for each 8x8 block, then applies a
 * 1D damped filter along that direction. Damping clamps each neighbor
 * contribution by the configured strength.
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */
#include "opcodec/cdef.h"
#include <stdlib.h>
#include <string.h>

#ifndef CLAMP
#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#endif

/* 8 candidate directions: (dx, dy) per step. */
static const int8_t kDir[8][2] = {
    { 1,  0}, { 1,  1}, { 0,  1}, {-1,  1},
    {-1,  0}, {-1, -1}, { 0, -1}, { 1, -1},
};

static int dominant_dir_8x8(const uint8_t *frame, int w, int h, int x0, int y0) {
    int best_score = -1, best_dir = 0;
    for (int d = 0; d < 8; d++) {
        int score = 0;
        for (int dy = 0; dy < 8; dy++) {
            for (int dx = 0; dx < 8; dx++) {
                int xa = x0 + dx;
                int ya = y0 + dy;
                int xb = xa + kDir[d][0];
                int yb = ya + kDir[d][1];
                if (xa < 0 || xa >= w || ya < 0 || ya >= h ||
                    xb < 0 || xb >= w || yb < 0 || yb >= h) continue;
                int diff = (int)frame[ya * w + xa] - (int)frame[yb * w + xb];
                /* Higher score for SMALL gradient => smoother direction. */
                score += 64 - (diff < 0 ? -diff : diff);
            }
        }
        if (score > best_score) { best_score = score; best_dir = d; }
    }
    return best_dir;
}

void opvis_cdef_apply(uint8_t *frame, int width, int height, uint8_t strength) {
    if (!frame || strength == 0 || width < 8 || height < 8) return;
    const int D = (int)strength * 4;  /* damping ceiling */
    uint8_t *out = (uint8_t *)malloc((size_t)width * (size_t)height);
    if (!out) return;
    memcpy(out, frame, (size_t)width * (size_t)height);

    for (int by = 0; by + 8 <= height; by += 8) {
        for (int bx = 0; bx + 8 <= width; bx += 8) {
            int dir = dominant_dir_8x8(frame, width, height, bx, by);
            for (int dy = 0; dy < 8; dy++) {
                for (int dx = 0; dx < 8; dx++) {
                    int x = bx + dx, y = by + dy;
                    int center = frame[y * width + x];
                    int sum = 0, n = 0;
                    for (int s = -1; s <= 1; s += 2) {
                        int nx = x + s * kDir[dir][0];
                        int ny = y + s * kDir[dir][1];
                        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                        int diff = (int)frame[ny * width + nx] - center;
                        diff = CLAMP(diff, -D, D);
                        sum += diff;
                        n++;
                    }
                    int filtered = center + (n ? (sum + (n > 0 ? 1 : 0)) / (n * 2 + 1) : 0);
                    out[y * width + x] = (uint8_t)CLAMP(filtered, 0, 255);
                }
            }
        }
    }
    memcpy(frame, out, (size_t)width * (size_t)height);
    free(out);
}
