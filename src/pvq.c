/* opcodec/pvq.c — Pyramid Vector Quantization implementation
 *
 * PVQ provides algebraic codebook quantization for MDCT coefficient bands
 * used in audio coding. Separates gain/shape and distributes pulses on
 * an N-dimensional unit sphere.
 *
 * Key algorithms:
 *   - Greedy pulse allocation for encoding
 *   - Combinatorial indexing (Fischer enumeration)
 *   - Precomputed table for codebook sizes
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */

#include "opcodec/pvq.h"
#include <math.h>
#include <string.h>

/* Math helpers */
__attribute__((unused))
static inline float fabsf_safe(float x)
{
    return x < 0.0f ? -x : x;
}

static inline int16_t sign_i16(float x)
{
    if (x > 0.0f) return 1;
    if (x < 0.0f) return -1;
    return 0;
}

static inline int abs_i16(int16_t x)
{
    return x < 0 ? -x : x;
}

/* Precomputed table for V(N,K) — number of integer vectors in Z^N with L1 norm = K
 * V[n][k] = number of ways to distribute k pulses among n dimensions (with signs)
 * This table avoids runtime combinatorial computation and overflow issues.
 */
static uint32_t V_table[PVQ_MAX_DIM + 1][PVQ_MAX_PULSES + 1];
static int table_initialized = 0;

/* Initialize the V(N,K) combinatorial table using recurrence relation */
static void init_v_table(void)
{
    if (table_initialized)
        return;

    /* Base cases */
    for (int n = 0; n <= PVQ_MAX_DIM; n++) {
        V_table[n][0] = 1; /* One way to place 0 pulses: all zeros */
    }
    for (int k = 0; k <= PVQ_MAX_PULSES; k++) {
        V_table[0][k] = (k == 0) ? 1 : 0; /* Only 0 pulses can be placed in 0 dimensions */
    }

    /* Fill table using correct PVQ recurrence
     * V(n,k) = V(n-1,k) + 2*sum(V(n-1,k-i)) for i=1 to k
     * This counts all ways to place k pulses in n dimensions with signs
     */
    for (int n = 1; n <= PVQ_MAX_DIM; n++) {
        for (int k = 1; k <= PVQ_MAX_PULSES; k++) {
            uint64_t sum = 0;

            /* Put all k pulses in first n-1 dimensions (don't use last dimension) */
            sum += V_table[n-1][k];

            /* Put i pulses in last dimension (both + and - signs), rest in first n-1 */
            for (int i = 1; i <= k; i++) {
                uint64_t ways_in_first = V_table[n-1][k-i];
                uint64_t ways_with_sign = ways_in_first * 2; /* +i and -i in last dim */
                sum += ways_with_sign;

                /* Prevent overflow */
                if (sum > UINT32_MAX) {
                    sum = UINT32_MAX;
                    break;
                }
            }

            V_table[n][k] = (uint32_t)sum;
        }
    }

    table_initialized = 1;
}

/* Get V(n,k) from the precomputed table */
static uint32_t get_v(int n, int k)
{
    if (n < 0 || k < 0 || n > PVQ_MAX_DIM || k > PVQ_MAX_PULSES)
        return 0;

    if (!table_initialized)
        init_v_table();

    return V_table[n][k];
}

/* Compute L2 norm of a float vector */
static float norm_l2_f(const float *x, int n)
{
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        sum += x[i] * x[i];
    }
    return sqrtf(sum);
}

/* Compute L2 norm of an int16 vector */
static float norm_l2_i16(const int16_t *x, int n)
{
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        float xi = (float)x[i];
        sum += xi * xi;
    }
    return sqrtf(sum);
}

/* Compute L1 norm of an int16 vector */
static int norm_l1_i16(const int16_t *x, int n)
{
    int sum = 0;
    for (int i = 0; i < n; i++) {
        sum += abs_i16(x[i]);
    }
    return sum;
}

void pvq_encode(const float *coeffs, int N, int K,
                float *gain, int16_t *shape)
{
    if (!coeffs || !gain || !shape || N <= 0 || N > PVQ_MAX_DIM || K < 0 || K > PVQ_MAX_PULSES) {
        if (gain) *gain = 0.0f;
        if (shape) {
            for (int i = 0; i < N; i++) {
                shape[i] = 0;
            }
        }
        return;
    }

    /* Step 1: Compute gain (L2 norm) */
    *gain = norm_l2_f(coeffs, N);

    /* Step 2: Handle zero gain case or K=0 case */
    if (*gain < 1e-12f || K == 0) {
        /* For zero input or zero pulses, shape is all zeros regardless of K */
        if (*gain < 1e-12f) *gain = 0.0f;
        for (int i = 0; i < N; i++) {
            shape[i] = 0;
        }
        return;
    }

    /* Step 4: Normalize to unit vector */
    float unit[PVQ_MAX_DIM];
    for (int i = 0; i < N; i++) {
        unit[i] = coeffs[i] / (*gain);
    }

    /* Step 5: Greedy pulse allocation
     * Start with rounded allocation, then adjust to meet L1 constraint
     */

    /* Initial allocation: round(unit[i] * K) */
    float target[PVQ_MAX_DIM];
    for (int i = 0; i < N; i++) {
        target[i] = unit[i] * (float)K;
        shape[i] = (int16_t)roundf(target[i]);
    }

    /* Current L1 norm */
    int current_l1 = norm_l1_i16(shape, N);

    /* Adjust to meet L1 = K constraint */
    while (current_l1 != K) {
        if (current_l1 < K) {
            /* Need to add pulses — find best dimension */
            int best_i = -1;
            float best_gain = -1.0f;

            for (int i = 0; i < N; i++) {
                /* Compute gain from adding a pulse in direction of unit[i] */
                float new_val = (float)shape[i] + sign_i16(unit[i]);
                float old_correlation = (float)shape[i] * unit[i];
                float new_correlation = new_val * unit[i];
                float gain_improvement = new_correlation - old_correlation;

                if (gain_improvement > best_gain) {
                    best_gain = gain_improvement;
                    best_i = i;
                }
            }

            if (best_i >= 0) {
                if (unit[best_i] >= 0.0f) {
                    shape[best_i]++;
                } else {
                    shape[best_i]--;
                }
                current_l1++;
            } else {
                break; /* Shouldn't happen */
            }
        } else {
            /* Need to remove pulses — find least costly dimension */
            int best_i = -1;
            float best_cost = 1e30f;

            for (int i = 0; i < N; i++) {
                if (shape[i] == 0) continue; /* Can't remove from zero */

                /* Compute cost of removing a pulse */
                float new_val = (float)shape[i] - sign_i16((float)shape[i]);
                float old_correlation = (float)shape[i] * unit[i];
                float new_correlation = new_val * unit[i];
                float cost = old_correlation - new_correlation;

                if (cost < best_cost) {
                    best_cost = cost;
                    best_i = i;
                }
            }

            if (best_i >= 0) {
                if (shape[best_i] > 0) {
                    shape[best_i]--;
                } else {
                    shape[best_i]++;
                }
                current_l1--;
            } else {
                break; /* Shouldn't happen */
            }
        }
    }
}

void pvq_decode(float gain, const int16_t *shape, int N,
                float *coeffs)
{
    if (!shape || !coeffs || N <= 0 || N > PVQ_MAX_DIM) {
        if (coeffs) {
            for (int i = 0; i < N; i++) {
                coeffs[i] = 0.0f;
            }
        }
        return;
    }

    /* Compute L2 norm of shape vector */
    float shape_norm = norm_l2_i16(shape, N);

    /* Handle zero norm case */
    if (shape_norm < 1e-12f) {
        for (int i = 0; i < N; i++) {
            coeffs[i] = 0.0f;
        }
        return;
    }

    /* Reconstruct: coeffs[i] = gain * shape[i] / ||shape|| */
    for (int i = 0; i < N; i++) {
        coeffs[i] = gain * (float)shape[i] / shape_norm;
    }
}

/* PVQ indexing using a simplified combinatorial approach
 * Maps pulse configurations to unique indices for entropy coding
 */
uint32_t pvq_index_encode(const int16_t *shape, int N, int K)
{
    if (!shape || N <= 0 || N > PVQ_MAX_DIM || K < 0 || K > PVQ_MAX_PULSES)
        return 0;

    if (!table_initialized)
        init_v_table();

    /* Special cases */
    if (K == 0) return 0;
    if (N == 1) {
        /* Only one dimension: index is just the value offset by K */
        return (uint32_t)(shape[0] + K);
    }

    /* General case: use factorization approach
     * Separate magnitude and sign information
     */
    uint32_t index = 0;
    int k_remaining = K;

    /* Process each dimension */
    for (int i = 0; i < N - 1 && k_remaining > 0; i++) {
        int16_t xi = shape[i];
        int abs_xi = abs_i16(xi);

        /* Count all configurations with smaller magnitude at position i */
        for (int mag = 0; mag < abs_xi; mag++) {
            uint32_t count = get_v(N - i - 1, k_remaining - mag);
            /* For mag > 0, each magnitude can have + or - sign */
            if (mag > 0) count *= 2;
            index += count;
        }

        /* If this dimension is negative, add positive configurations */
        if (xi < 0 && abs_xi > 0) {
            uint32_t pos_count = get_v(N - i - 1, k_remaining - abs_xi);
            index += pos_count;
        }

        k_remaining -= abs_xi;
    }

    return index;
}

void pvq_index_decode(uint32_t index, int N, int K, int16_t *shape)
{
    if (!shape || N <= 0 || N > PVQ_MAX_DIM || K < 0 || K > PVQ_MAX_PULSES) {
        if (shape) {
            for (int i = 0; i < N; i++) {
                shape[i] = 0;
            }
        }
        return;
    }

    if (!table_initialized)
        init_v_table();

    /* Initialize shape */
    for (int i = 0; i < N; i++) {
        shape[i] = 0;
    }

    /* Special cases */
    if (K == 0) return;
    if (N == 1) {
        shape[0] = (int16_t)(index - K);
        return;
    }

    /* General case: inverse of encoding */
    uint32_t remaining_index = index;
    int k_remaining = K;

    for (int i = 0; i < N - 1 && k_remaining > 0; i++) {
        /* Find magnitude at position i */
        int abs_xi = 0;
        while (abs_xi <= k_remaining) {
            uint32_t count = get_v(N - i - 1, k_remaining - abs_xi);
            if (abs_xi > 0) count *= 2;

            if (remaining_index < count) {
                break; /* Found the right magnitude */
            }

            remaining_index -= count;
            abs_xi++;
        }

        /* Determine sign */
        if (abs_xi > 0) {
            uint32_t pos_count = get_v(N - i - 1, k_remaining - abs_xi);
            if (remaining_index < pos_count) {
                shape[i] = abs_xi; /* Positive */
            } else {
                shape[i] = -abs_xi; /* Negative */
                remaining_index -= pos_count;
            }
        } else {
            shape[i] = 0;
        }

        k_remaining -= abs_xi;
    }

    /* Handle last dimension: put all remaining pulses here */
    if (N > 0 && k_remaining > 0) {
        shape[N-1] = k_remaining; /* Put remaining pulses as positive */
    }
}

uint32_t pvq_codebook_size(int N, int K)
{
    if (N <= 0 || N > PVQ_MAX_DIM || K < 0 || K > PVQ_MAX_PULSES)
        return 0;

    return get_v(N, K);
}

int pvq_optimal_k(int N, int bits)
{
    if (N <= 0 || N > PVQ_MAX_DIM || bits < 0)
        return 0;

    if (!table_initialized)
        init_v_table();

    /* Binary search for largest K such that log2(codebook_size(N,K)) <= bits */
    int k_max = 0;
    uint32_t max_size = 1U << bits; /* 2^bits */

    for (int k = 0; k <= PVQ_MAX_PULSES; k++) {
        uint32_t size = pvq_codebook_size(N, k);
        if (size == 0 || size > max_size) {
            break;
        }
        k_max = k;
    }

    return k_max;
}

/* ---- Residual Vector Quantization (RVQ) Implementation ---- */

void rvq_encode(const float *coeffs, int N, int total_K, int num_stages,
                rvq_result_t *result)
{
    if (!coeffs || !result || N <= 0 || N > PVQ_MAX_DIM ||
        total_K < 0 || total_K > PVQ_MAX_PULSES ||
        num_stages < 1 || num_stages > RVQ_MAX_STAGES) {
        /* Clear result on error */
        if (result) {
            memset(result, 0, sizeof(*result));
        }
        return;
    }

    /* Initialize result */
    memset(result, 0, sizeof(*result));
    result->N = N;
    result->num_stages = num_stages;

    /* Working buffers */
    float current[PVQ_MAX_DIM];
    float residual[PVQ_MAX_DIM];

    /* Initialize with input coefficients */
    for (int i = 0; i < N; i++) {
        current[i] = coeffs[i];
    }

    /* Distribute pulses across stages (geometrically decreasing) */
    int K_remaining = total_K;
    for (int stage = 0; stage < num_stages; stage++) {
        /* Allocate roughly half the remaining pulses to this stage */
        int K_stage;
        if (stage == num_stages - 1) {
            /* Last stage gets all remaining pulses */
            K_stage = K_remaining;
        } else {
            /* Distribute geometrically: roughly K/2, K/4, K/8, etc. */
            K_stage = K_remaining / 2;
            if (K_stage < 1 && K_remaining > 0) {
                K_stage = 1;  /* Ensure at least 1 pulse if any remain */
            }
        }

        result->K_per_stage[stage] = K_stage;
        K_remaining -= K_stage;

        if (K_stage == 0) {
            /* No pulses for this stage, set to zero */
            result->gains[stage] = 0.0f;
            for (int i = 0; i < N; i++) {
                result->shapes[stage][i] = 0;
            }
            continue;
        }

        /* Encode current residual with PVQ */
        float stage_gain;
        pvq_encode(current, N, K_stage, &stage_gain, result->shapes[stage]);
        result->gains[stage] = stage_gain;

        /* Decode this stage to get quantized version */
        float decoded_stage[PVQ_MAX_DIM];
        pvq_decode(stage_gain, result->shapes[stage], N, decoded_stage);

        /* Compute residual for next stage */
        for (int i = 0; i < N; i++) {
            residual[i] = current[i] - decoded_stage[i];
        }

        /* Use residual as input for next stage */
        for (int i = 0; i < N; i++) {
            current[i] = residual[i];
        }
    }
}

void rvq_decode(const rvq_result_t *result, float *coeffs)
{
    if (!result || !coeffs || result->N <= 0 || result->N > PVQ_MAX_DIM ||
        result->num_stages < 0 || result->num_stages > RVQ_MAX_STAGES) {
        if (coeffs && result && result->N > 0) {
            for (int i = 0; i < result->N; i++) {
                coeffs[i] = 0.0f;
            }
        }
        return;
    }

    int N = result->N;

    /* Initialize output to zero */
    for (int i = 0; i < N; i++) {
        coeffs[i] = 0.0f;
    }

    /* Sum contributions from all stages */
    for (int stage = 0; stage < result->num_stages; stage++) {
        float stage_coeffs[PVQ_MAX_DIM];

        /* Decode this stage */
        pvq_decode(result->gains[stage], result->shapes[stage], N, stage_coeffs);

        /* Add to output */
        for (int i = 0; i < N; i++) {
            coeffs[i] += stage_coeffs[i];
        }
    }
}