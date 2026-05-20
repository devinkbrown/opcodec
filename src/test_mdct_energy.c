/*
 * test_mdct_energy.c — standalone MDCT energy preservation test.
 *
 * Copies mdct_forward, mdct_inverse, and compute_window (sine variant) verbatim
 * from audio.c, then runs a 440 Hz sine roundtrip at five N values that
 * correspond to the codec frame sizes (4/8/16/32/48 kHz, 20 ms frames).
 *
 * For each N:
 *   - Generates 2N samples of a 440 Hz sine at sr = 50*N Hz.
 *   - Applies the sine window.
 *   - Runs MDCT forward, then IMDCT.
 *   - Simulates one full TDAC overlap-add cycle (two frames).
 *   - Compares E_out / E_in.
 *
 * A correct TDAC MDCT/IMDCT with a power-complementary sine window must
 * produce ratio = 1.000 for every N.  Any systematic deviation indicates
 * a normalisation or window bug in audio.c.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MDCT_MAX 1920   /* 2 * 960, large enough for all tested N */

/* ------------------------------------------------------------------ */
/*  Window (sine only — that is what the 440 Hz roundtrip test uses)  */
/* ------------------------------------------------------------------ */
static void
compute_window_sine(float *window, uint16_t size)
{
    for (uint16_t n = 0; n < size; n++)
        window[n] = (float)sin(M_PI * (n + 0.5) / size);
}

/* ------------------------------------------------------------------ */
/*  Forward MDCT — copied verbatim from audio.c lines 320-352         */
/*  X[k] = sum_{n=0}^{2N-1} x[n]*w[n] * cos(π/N*(n+N/2+½)*(k+½))   */
/* ------------------------------------------------------------------ */
static void
mdct_forward(const float *x, float *X, uint16_t N, const float *window)
{
    float y[MDCT_MAX];
    uint16_t N2 = (uint16_t)(2u * N);
    for (uint16_t n = 0; n < N2; n++)
        y[n] = x[n] * window[n];

    const float pi_N   = (float)(M_PI) / (float)N;
    const float offset = 0.5f + (float)N * 0.5f;   /* N/2 + ½ */

    for (uint16_t k = 0; k < N; k++) {
        float delta_n = pi_N * ((float)k + 0.5f);
        float cos_dn  = cosf(delta_n);
        float sin_dn  = sinf(delta_n);
        float phase0  = delta_n * offset;
        float cos_p   = cosf(phase0);
        float sin_p   = sinf(phase0);

        float sum = 0.0f;
        for (uint16_t n = 0; n < N2; n++) {
            sum += y[n] * cos_p;
            float nc = cos_p * cos_dn - sin_p * sin_dn;
            float ns = sin_p * cos_dn + cos_p * sin_dn;
            cos_p = nc;
            sin_p = ns;
        }
        X[k] = sum;
    }
}

/* ------------------------------------------------------------------ */
/*  Inverse MDCT — copied verbatim from audio.c lines 354-384         */
/*  x[n] = (2/N) * w[n] * sum_{k=0}^{N-1} X[k]*cos(π/N*(n+N/2+½)*(k+½)) */
/* ------------------------------------------------------------------ */
static void
mdct_inverse(const float *X, float *x, uint16_t N, const float *window)
{
    const float pi_N   = (float)(M_PI) / (float)N;
    const float scale  = 2.0f / (float)N;
    const float offset = 0.5f + (float)N * 0.5f;
    uint16_t N2 = (uint16_t)(2u * N);

    for (uint16_t n = 0; n < N2; n++) {
        float delta_k = pi_N * ((float)n + offset);
        float cos_dk  = cosf(delta_k);
        float sin_dk  = sinf(delta_k);
        float phase0  = delta_k * 0.5f;
        float cos_p   = cosf(phase0);
        float sin_p   = sinf(phase0);

        float sum = 0.0f;
        for (uint16_t k = 0; k < N; k++) {
            sum += X[k] * cos_p;
            float nc = cos_p * cos_dk - sin_p * sin_dk;
            float ns = sin_p * cos_dk + cos_p * sin_dk;
            cos_p = nc;
            sin_p = ns;
        }
        x[n] = sum * scale * window[n];
    }
}

/* ------------------------------------------------------------------ */
/*  Test harness                                                        */
/* ------------------------------------------------------------------ */
int
main(void)
{
    /*
     * N values: 2N = frame_samples for each sample rate at 20 ms frames.
     * sr = 50*N  =>  frame_samples = sr * 0.020 = 50*N * 0.020 = N samples
     *                window_size   = 2 * frame_samples = 2N
     *
     * N   sr(Hz)
     * 80   4000
     * 160  8000
     * 320  16000
     * 640  32000
     * 960  48000
     */
    const uint16_t N_vals[] = { 80, 160, 320, 640, 960 };
    const int      n_cases  = (int)(sizeof(N_vals) / sizeof(N_vals[0]));

    printf("%-6s  %-12s  %-12s  %-12s  %-12s  %-10s\n",
           "N", "sr(Hz)", "E_in", "E_mdct", "E_out", "ratio");
    printf("%s\n",
           "------  ------------  ------------  ------------  ------------  ----------");

    for (int ci = 0; ci < n_cases; ci++) {
        uint16_t N  = N_vals[ci];
        uint16_t N2 = (uint16_t)(2u * N);
        float    sr = 50.0f * (float)N;   /* sample rate */

        /* ---- generate 2N samples of 440 Hz sine ---- */
        float x[MDCT_MAX];
        for (uint16_t n = 0; n < N2; n++)
            x[n] = 0.5f * sinf(2.0f * (float)M_PI * 440.0f / sr * (float)n);

        /* ---- sine window ---- */
        float window[MDCT_MAX];
        compute_window_sine(window, N2);

        /* ---- E_in: average energy over all 2N input samples ---- */
        double E_in = 0.0;
        for (uint16_t n = 0; n < N2; n++)
            E_in += (double)x[n] * (double)x[n];
        E_in /= N2;

        /* ---- forward MDCT ---- */
        float X[MDCT_MAX];
        mdct_forward(x, X, N, window);

        /* ---- E_mdct: normalised spectral energy ---- */
        double E_mdct = 0.0;
        for (uint16_t k = 0; k < N; k++)
            E_mdct += (double)X[k] * (double)X[k];
        E_mdct /= (double)N * (double)N;

        /* ----------------------------------------------------------------
         * TDAC overlap-add simulation — two consecutive frames:
         *
         *   Frame 1 input:  [  zeros(N)  | x[0..N-1]  ]   (cold-start, prev=0)
         *   Frame 2 input:  [ x[0..N-1]  | x[N..2N-1] ]   (sliding window)
         *
         * IMDCT of frame f gives 2N output samples split into two halves:
         *   imdct_f[0..N-1]   — overlap with PREVIOUS frame's second half
         *   imdct_f[N..2N-1]  — overlap with NEXT frame's first half
         *
         * Reconstructed sample n in frame 2's output region:
         *   out[n] = imdct_frame2[n] + imdct_frame1[N + n],  n = 0..N-1
         *
         * For a sine window the TDAC condition guarantees perfect reconstruction
         * of the middle N samples (which correspond to x[0..N-1]).
         * ---------------------------------------------------------------- */

        /* Frame 1: input buffer = [zeros | x[0..N-1]] */
        float buf1[MDCT_MAX];
        memset(buf1, 0, sizeof(float) * N);
        for (uint16_t n = 0; n < N; n++)
            buf1[N + n] = x[n];

        float X1[MDCT_MAX], imdct1[MDCT_MAX];
        mdct_forward(buf1, X1, N, window);
        mdct_inverse(X1, imdct1, N, window);

        /* Frame 2: input buffer = [x[0..N-1] | x[N..2N-1]] */
        float buf2[MDCT_MAX];
        for (uint16_t n = 0; n < N; n++)
            buf2[n] = x[n];
        for (uint16_t n = 0; n < N; n++)
            buf2[N + n] = x[N + n];

        float X2[MDCT_MAX], imdct2[MDCT_MAX];
        mdct_forward(buf2, X2, N, window);
        mdct_inverse(X2, imdct2, N, window);

        /* Overlap-add: out[n] = imdct2[n] + imdct1[N + n] */
        float out[MDCT_MAX];
        for (uint16_t n = 0; n < N; n++)
            out[n] = imdct2[n] + imdct1[N + n];

        /* E_out: average energy of reconstructed N samples */
        double E_out = 0.0;
        for (uint16_t n = 0; n < N; n++)
            E_out += (double)out[n] * (double)out[n];
        E_out /= N;

        /* Reference energy: average of the corresponding x[0..N-1] */
        double E_ref = 0.0;
        for (uint16_t n = 0; n < N; n++)
            E_ref += (double)x[n] * (double)x[n];
        E_ref /= N;

        double ratio = (E_ref > 0.0) ? E_out / E_ref : 0.0;

        printf("%-6u  %-12.0f  %-12.6f  %-12.6f  %-12.6f  %-10.6f\n",
               (unsigned)N, (double)sr, E_ref, E_mdct, E_out, ratio);
    }

    return 0;
}
