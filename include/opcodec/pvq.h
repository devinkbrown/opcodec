/*
 * opcodec/pvq.h — Pyramid Vector Quantization (PVQ)
 *
 * Algebraic codebook quantization used by CELT/Opus for MDCT coefficient bands.
 * PVQ provides zero-storage quantization by mathematically generating codebook
 * vectors through pulse distribution on an N-dimensional unit sphere.
 *
 * Algorithm:
 *   1. Separate coefficient band into gain (L2 norm) and shape (unit vector)
 *   2. Distribute K pulses among N dimensions to approximate the shape
 *   3. Use combinatorial indexing for entropy coding
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#ifndef OPCODEC_PVQ_H
#define OPCODEC_PVQ_H

#include <stdint.h>
#include <stddef.h>

/* Maximum dimensions (MDCT coefficients per band).
 * Must accommodate the widest band at any supported sample rate.
 * 48kHz band 28 (bins 358-416) has N=58, so 64 is the safe ceiling. */
#define PVQ_MAX_DIM    64
/* Maximum pulses per band */
#define PVQ_MAX_PULSES 64

/* Encode a band of MDCT coefficients using PVQ.
 *
 * Input:
 *   coeffs[N]  — MDCT coefficients for this band (float)
 *   N          — number of coefficients (band width, 1..PVQ_MAX_DIM)
 *   K          — number of pulses (controls resolution/bitrate)
 *
 * Output:
 *   gain       — L2 norm of the band (to be quantized separately)
 *   shape[N]   — integer pulse vector (L1 norm = K, each in [-K..K])
 *
 * The caller quantizes `gain` and transmits it alongside the PVQ index.
 */
void pvq_encode(const float *coeffs, int N, int K,
                float *gain, int16_t *shape);

/* Decode PVQ: reconstruct float coefficients from gain + shape.
 *
 * Output:
 *   coeffs[N] = gain * normalize(shape)
 */
void pvq_decode(float gain, const int16_t *shape, int N,
                float *coeffs);

/* Combinatorial indexing: encode a pulse vector to a unique index.
 *
 * The index uniquely identifies the shape vector for transmission.
 * Index range is [0, pvq_codebook_size(N, K)).
 *
 * Returns the index. For large N*K the index can exceed 32 bits;
 * we use uint32_t which is sufficient for N<=24, K<=32.
 */
uint32_t pvq_index_encode(const int16_t *shape, int N, int K);

/* Combinatorial indexing: decode an index back to a pulse vector. */
void pvq_index_decode(uint32_t index, int N, int K, int16_t *shape);

/* Return the codebook size (number of valid vectors) for given N, K.
 * Used for entropy coding to know the range of the index. */
uint32_t pvq_codebook_size(int N, int K);

/* Compute optimal K (number of pulses) for a given bit budget.
 * Returns the largest K such that log2(codebook_size(N,K)) <= bits. */
int pvq_optimal_k(int N, int bits);

/* Band splitting: if a band is too wide for efficient PVQ,
 * split it in half and encode each half independently.
 * Returns true if the band should be split. */
static inline int pvq_should_split(int N, int K)
{
    return (N > 16 && K > 4);
}

/* ---- Residual Vector Quantization (RVQ) ---- */

/* Maximum stages for multi-stage RVQ encoding */
#define RVQ_MAX_STAGES 4

/* RVQ result: stores multi-stage quantization data */
typedef struct {
    int16_t shapes[RVQ_MAX_STAGES][PVQ_MAX_DIM];  /* shape vectors for each stage */
    float   gains[RVQ_MAX_STAGES];                /* gain for each stage */
    int     num_stages;                           /* number of stages used */
    int     N;                                    /* vector dimension */
    int     K_per_stage[RVQ_MAX_STAGES];          /* pulses allocated to each stage */
} rvq_result_t;

/* Multi-stage RVQ encode: encode coefficients using multiple PVQ stages.
 *
 * Algorithm:
 *   Stage 0: encode coeffs with K0 = total_K/2 pulses using pvq_encode
 *   Compute residual = coeffs - decoded_stage0
 *   Stage 1: encode residual with K1 = total_K/4 pulses
 *   Continue for num_stages (up to 4), halving K each time
 *
 * Input:
 *   coeffs[N]   — MDCT coefficients to encode
 *   N           — vector dimension
 *   total_K     — total pulses to distribute across all stages
 *   num_stages  — number of stages (1..RVQ_MAX_STAGES)
 *
 * Output:
 *   result      — RVQ encoding result with all stage data
 */
void rvq_encode(const float *coeffs, int N, int total_K, int num_stages,
                rvq_result_t *result);

/* Multi-stage RVQ decode: reconstruct coefficients from RVQ result.
 *
 * Sums all stages: coeffs = sum(gain[i] * normalize(shape[i]))
 *
 * Input:
 *   result      — RVQ result from rvq_encode
 *
 * Output:
 *   coeffs[N]   — reconstructed MDCT coefficients
 */
void rvq_decode(const rvq_result_t *result, float *coeffs);

#endif /* OPCODEC_PVQ_H */