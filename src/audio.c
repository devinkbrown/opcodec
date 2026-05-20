/*
 * opcodec/audio.c — OPVOX audio codec implementation
 *
 * A modern sub-band MDCT audio codec supporting 8kHz/16kHz/48kHz sample rates,
 * mono+stereo, 4 quality levels, 20ms frames.
 *
 * Enhanced algorithm:
 *   1. Pre-emphasis filtering
 *   2. Pitch detection and pre-filter (removes pitch harmonics)
 *   3. VMD (Voice/Music Detection) content classification
 *   4. MDCT transform with sine windowing
 *   5. SNS (Spectral Noise Shaping) analysis and flattening
 *   6. PNS (Perceptual Noise Substitution) — skip PVQ on noise bands
 *   7. TNS (Temporal Noise Shaping) for transients
 *   8. BWE (Bandwidth Extension) cutoff determination
 *   9. PVQ (Pyramid Vector Quantization) for MDCT coefficients
 *  10. Raw-bit PVQ index coding (RANS removed: uniform distribution has zero entropy gain)
 *
 * Decoder reverses the process with pitch post-filter at the end.
 *
 * Copyright (c) 2026 Ophion Development Team.  GPL v2.
 */

#include "opcodec/audio.h"
#include "opcodec/pvq.h"
#include "opcodec/pitch.h"
#include "opcodec/bwe.h"
#include "opcodec/tns.h"
#include "opcodec/energy.h"
#include "opcodec/pns.h"
#include "opcodec/vmd.h"
#include <string.h>
#include <math.h>
#include <stdio.h>  /* diagnostic fprintf */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* Return the number of raw bits needed to represent indices 0..codebook_size-1.
 * Equivalent to ceil(log2(codebook_size)), clamped to [0, 32]. */
static uint8_t pvq_index_bits(uint32_t codebook_size) {
    if (codebook_size <= 1) return 0;
    uint8_t bits = 0;
    uint32_t n = codebook_size - 1;
    while (n > 0) { bits++; n >>= 1; }
    return bits;
}

/* Per-rate pre/de-emphasis coefficients.
 *
 * α=0.97 is standard for narrowband (8kHz) speech: places the high-pass
 * inflection at ~1.6kHz, appropriate spectral whitening for voiced speech.
 *
 * At higher sample rates the same α causes excessive de-emphasis amplification
 * of quantization noise (average gain = 1/(1-α²); for α=0.97: ×16.9).
 * Reducing α at higher rates keeps the voice-band shaping useful while
 * cutting the noise floor amplification:
 *
 *   8 kHz → α=0.97   de-emph noise gain ×16.9
 *  16 kHz → α=0.97   de-emph noise gain ×16.9  (same filter, useful for wideband)
 *  32 kHz → α=0.94   de-emph noise gain ×10.8  (−2.4 dB noise floor)
 *  48 kHz → α=0.90   de-emph noise gain  ×5.3  (−5.0 dB noise floor)
 */
static float
pre_emph_coeff(uint32_t sample_rate)
{
	(void)sample_rate;
	return 0.97f;
}
static const float SILENCE_THRESHOLD = 0.001f;
static const float ENERGY_SMOOTH = 0.9f;
static const float COMFORT_NOISE_LEVEL = 0.002f;

/* PVQ K selection based on quality level — used as a minimum-K floor */
static const int base_k_table[4] = {2, 4, 8, 24};  /* LOW, NORMAL, HIGH, ULTRA */

/* Bit-allocation based K selection for PVQ.
 *
 * Uses 45% of the target_bits budget for PVQ shape coding.
 * Distributes bits proportional to band energy (dB scale), then maps
 * bits → K via pvq_optimal_k so the actual codebook size matches budget.
 *
 * Output: per-band K values, minimum 1.
 */
static void pvq_allocate_k_per_band(
    const float *coarse_dB,     /* energy ctx coarse_dB[n_bands] */
    const bool  *is_noise,      /* pns noise flags */
    const float *threshold_dB,  /* absolute hearing threshold per band (dB, same scale as coarse_dB) */
    uint8_t      n_coded_bands, /* number of bands to code */
    const int   *band_N,        /* MDCT coefficients per band */
    int          base_k,        /* quality-level floor */
    int          pvq_bits,      /* total PVQ bit budget */
    int         *K_out)         /* output K per coded band */
{
    /* Bandwidth-adjusted energy weights with hearing threshold masking.
     *
     * Weight = max(energy_dB_offset - threshold_margin, min_weight) × sqrt(N)
     *
     * sqrt(N): wider bands need proportionally more pulses for equal shape quality.
     * threshold_dB: bands near or below hearing threshold get sharply reduced weight;
     *   their PVQ bits are redistributed to clearly audible bands. */
    float min_dB = 0.0f;
    for (uint8_t b = 0; b < n_coded_bands; b++) {
        if (!is_noise[b] && coarse_dB[b] < min_dB)
            min_dB = coarse_dB[b];
    }

    /* Compute effective weight for each band, masking near-threshold bands. */
    float weights[OPVOX_MAX_BANDS];
    float total_weight = 0.0f;
    for (uint8_t b = 0; b < n_coded_bands; b++) {
        if (is_noise[b]) {
            weights[b] = 0.0f;
            continue;
        }
        float bw_scale = sqrtf((float)band_N[b]);
        float energy_offset = coarse_dB[b] - min_dB + 6.0f;

        /* Apply hearing threshold: reduce weight for near-threshold bands.
         * threshold_dB is relative to coarse_dB scale (both in dB, same origin).
         * A band 20+ dB above threshold gets full weight; below threshold: minimum. */
        float threshold_margin = (threshold_dB != NULL)
            ? (coarse_dB[b] - (min_dB + threshold_dB[b]))
            : energy_offset;
        /* Smoothly taper weight as band approaches or goes below threshold */
        if (threshold_dB != NULL) {
            float fade = threshold_margin / 20.0f;    /* full weight at 20 dB above threshold */
            fade = fade < 0.05f ? 0.05f : (fade > 1.0f ? 1.0f : fade);
            energy_offset *= fade;
        }

        weights[b] = energy_offset * bw_scale;
        total_weight += weights[b];
    }

    for (uint8_t b = 0; b < n_coded_bands; b++) {
        if (is_noise[b]) {
            K_out[b] = 0;
            continue;
        }
        int bits_b = 1;
        if (total_weight > 1e-6f) {
            bits_b = (int)(pvq_bits * weights[b] / total_weight + 0.5f);
        }
        bits_b = CLAMP(bits_b, 1, 18);  /* 2^18 = 256k entries, well within uint32 */
        int k = pvq_optimal_k(band_N[b], bits_b);
        /* Cap K at band dimension: K > N gives no useful extra shape resolution */
        int k_max = band_N[b] < PVQ_MAX_PULSES ? band_N[b] : PVQ_MAX_PULSES;
        K_out[b] = CLAMP(k, base_k > 0 ? base_k : 1, k_max);
    }
}

/* Target bits per frame for each quality level */
static const uint16_t target_bits_table[4][4] = {
	/* 8kHz   16kHz   32kHz   48kHz */
	{  128,    256,    448,    640  },  /* LOW */
	{  256,    512,    896,   1280  },  /* NORMAL */
	{  384,    768,   1536,   2560  },  /* HIGH */
	{  512,   1024,   2048,   4096  }   /* ULTRA — 4096 bits/frame (512 B) at 48kHz */
};

/* Band boundaries for psychoacoustic model */
static const uint16_t band_boundaries_8k[OPVOX_BANDS_8K + 1] = {
	0, 4, 8, 12, 16, 20, 26, 34, 80
};
static const uint16_t band_boundaries_16k[OPVOX_BANDS_16K + 1] = {
	0, 4, 8, 12, 16, 20, 24, 28, 32, 38, 44, 52, 62, 74, 90, 110, 160
};
/* 32kHz: 640 samples/frame → 320 MDCT bins (0..319), each bin ≈ 50 Hz */
static const uint16_t band_boundaries_32k[OPVOX_BANDS_32K + 1] = {
	0, 4, 8, 12, 16, 20, 24, 28, 32, 38, 44, 52, 62, 74, 90, 110,
	132, 158, 190, 226, 264, 286, 302, 312, 320
};
/* 48kHz: 960 samples/frame → 480 MDCT bins (0..479), each bin ≈ 50 Hz */
static const uint16_t band_boundaries_48k[OPVOX_MAX_BANDS + 1] = {
	0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 54, 60, 66,
	74, 82, 92, 104, 118, 134, 152, 174, 200, 230, 266, 308, 358, 416, 440, 460, 480
};

/* Absolute hearing threshold (in dB SPL) per band */
static const float hearing_threshold_8k[OPVOX_BANDS_8K] = {
	60.0f, 40.0f, 20.0f, 10.0f, 5.0f, 5.0f, 10.0f, 20.0f
};
static const float hearing_threshold_16k[OPVOX_BANDS_16K] = {
	60.0f, 40.0f, 20.0f, 10.0f, 5.0f, 2.0f, 0.0f, -2.0f,
	0.0f, 2.0f, 5.0f, 8.0f, 12.0f, 16.0f, 20.0f, 30.0f
};
/* 32kHz: thresholds follow ISO 226 equal-loudness curve to ~16kHz */
static const float hearing_threshold_32k[OPVOX_BANDS_32K] = {
	60.0f, 40.0f, 20.0f, 10.0f, 5.0f, 2.0f, 0.0f, -2.0f,
	0.0f, 2.0f, 5.0f, 8.0f, 12.0f, 16.0f, 20.0f, 30.0f,
	40.0f, 52.0f, 62.0f, 70.0f, 76.0f, 82.0f, 88.0f, 94.0f
};
static const float hearing_threshold_48k[OPVOX_MAX_BANDS] = {
	60.0f, 40.0f, 20.0f, 10.0f, 5.0f, 2.0f, 0.0f, -2.0f,
	-4.0f, -5.0f, -4.0f, -2.0f, 0.0f, 2.0f, 4.0f, 6.0f,
	8.0f, 10.0f, 12.0f, 14.0f, 16.0f, 18.0f, 20.0f, 22.0f,
	24.0f, 26.0f, 28.0f, 30.0f, 35.0f, 40.0f, 50.0f, 60.0f
};

/* Simple bit writer for header fields */
typedef struct {
	uint8_t *buf;
	size_t pos;     /* byte position */
	uint8_t bit;    /* bit position (0-7) */
	size_t cap;     /* buffer capacity */
} bit_writer_t;

typedef struct {
	const uint8_t *buf;
	size_t pos;
	uint8_t bit;
	size_t len;
} bit_reader_t;

/* Bit writer functions for header fields */
static void
bit_writer_init(bit_writer_t *w, uint8_t *buf, size_t cap)
{
	w->buf = buf;
	w->pos = 0;
	w->bit = 0;
	w->cap = cap;
	memset(buf, 0, cap);
}

static int
bit_writer_write(bit_writer_t *w, uint32_t value, uint8_t bits)
{
	if (bits > 32 || w->pos >= w->cap)
		return -1;

	for (int i = bits - 1; i >= 0; i--) {
		if (w->pos >= w->cap)
			return -1;

		if ((value >> i) & 1)
			w->buf[w->pos] |= (1u << (7 - w->bit));

		w->bit++;
		if (w->bit == 8) {
			w->bit = 0;
			w->pos++;
		}
	}
	return 0;
}

static size_t
bit_writer_bytes(const bit_writer_t *w)
{
	return w->pos + (w->bit > 0 ? 1 : 0);
}

static int
bit_writer_align(bit_writer_t *w)
{
	if (w->bit > 0) {
		uint8_t pad = 8 - w->bit;
		if (bit_writer_write(w, 0, pad) < 0)
			return -1;
	}
	return 0;
}

/* Bit reader functions for header fields */
static void
bit_reader_init(bit_reader_t *r, const uint8_t *buf, size_t len)
{
	r->buf = buf;
	r->pos = 0;
	r->bit = 0;
	r->len = len;
}

static uint32_t
bit_reader_read(bit_reader_t *r, uint8_t bits)
{
	uint32_t value = 0;

	for (int i = 0; i < bits; i++) {
		if (r->pos >= r->len)
			return 0;

		if (r->buf[r->pos] & (1u << (7 - r->bit)))
			value |= (1u << (bits - 1 - i));

		r->bit++;
		if (r->bit == 8) {
			r->bit = 0;
			r->pos++;
		}
	}
	return value;
}

static void
bit_reader_align(bit_reader_t *r)
{
	if (r->bit > 0) {
		r->pos++;
		r->bit = 0;
	}
}

/* MDCT implementation — cosine recurrence eliminates O(N²) cos() calls.
 *
 * The MDCT kernel is cos(π/N · (n + ½ + N/2) · (k + ½)).
 * Fixing k, consecutive n values differ in angle by π(k+½)/N (= delta_n).
 * Fixing n, consecutive k values differ in angle by π(n + ½ + N/2)/N (= delta_k).
 *
 * Recurrence: cos(θ + δ) = cos(θ)cos(δ) − sin(θ)sin(δ)
 *             sin(θ + δ) = sin(θ)cos(δ) + cos(θ)sin(δ)
 * Cost: 4 mults + 2 adds per step vs. 1 cos() call (≈ 20–50 cycles).
 * Speedup: ~10–20× at 48 kHz (N=960, 2N=1920 window).
 */
static void
mdct_forward(const float *x, float *X, uint16_t N, const float *window)
{
	/* Pre-apply window once — avoids re-multiplying for each k */
	float y[OPVOX_MDCT_MAX];
	uint16_t N2 = (uint16_t)(2u * N);
	for (uint16_t n = 0; n < N2; n++)
		y[n] = x[n] * window[n];

	const float pi_N   = (float)(M_PI) / (float)N;
	const float offset = 0.5f + (float)N * 0.5f;   /* N/2 + ½ */

	for (uint16_t k = 0; k < N; k++) {
		/* delta_n: angle step as n increases by 1 for fixed k */
		float delta_n  = pi_N * ((float)k + 0.5f);
		float cos_dn   = cosf(delta_n);
		float sin_dn   = sinf(delta_n);
		/* Initial phase at n=0 */
		float phase0   = delta_n * offset;
		float cos_p    = cosf(phase0);
		float sin_p    = sinf(phase0);

		float sum = 0.0f;
		for (uint16_t n = 0; n < N2; n++) {
			sum += y[n] * cos_p;
			/* Advance by delta_n */
			float nc = cos_p * cos_dn - sin_p * sin_dn;
			float ns = sin_p * cos_dn + cos_p * sin_dn;
			cos_p = nc;
			sin_p = ns;
		}
		X[k] = sum;
	}
}

static void
mdct_inverse(const float *X, float *x, uint16_t N, const float *window)
{
	/* delta_k: angle step as k increases by 1 for fixed n.
	 * We iterate over n in the outer loop and accumulate over k. */
	const float pi_N   = (float)(M_PI) / (float)N;
	const float scale  = 2.0f / (float)N;
	const float offset = 0.5f + (float)N * 0.5f;
	uint16_t N2 = (uint16_t)(2u * N);

	for (uint16_t n = 0; n < N2; n++) {
		/* delta_k: angle step for k+1 vs k at this n */
		float delta_k  = pi_N * ((float)n + offset);
		float cos_dk   = cosf(delta_k);
		float sin_dk   = sinf(delta_k);
		/* Initial phase at k=0 */
		float phase0   = delta_k * 0.5f;
		float cos_p    = cosf(phase0);
		float sin_p    = sinf(phase0);

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

/* Zeroth-order modified Bessel function I0(x) - computed via power series */
static double
bessel_i0(double x)
{
	double sum = 1.0;
	double term = 1.0;
	double x_sq_over_4 = (x * x) / 4.0;

	/* Power series expansion: I0(x) = sum(k=0..inf, (x^2/4)^k / (k!)^2) */
	for (int k = 1; k <= 20; k++) {  /* 20 terms gives excellent precision */
		term *= x_sq_over_4 / (k * k);
		sum += term;
		if (term < 1e-15 * sum) break;  /* Early termination for convergence */
	}

	return sum;
}

/* Generate Kaiser window with given alpha parameter */
static void
compute_kaiser_window(float *window, uint16_t size, double alpha)
{
	double inv_i0_alpha = 1.0 / bessel_i0(alpha);

	for (uint16_t n = 0; n < size; n++) {
		double x = (2.0 * n) / (size - 1) - 1.0;  /* normalize to [-1, 1] */
		double arg = alpha * sqrt(1.0 - x * x);
		window[n] = (float)(bessel_i0(arg) * inv_i0_alpha);
	}
}

/* Generate KBD (Kaiser-Bessel Derived) window from Kaiser window.
 *
 * Standard AAC/Opus KBD construction uses a half-length Kaiser window
 * of size N/2+1 samples, then mirrors it so the full window satisfies
 * the power-complementary property: w[n]^2 + w[n + N/2]^2 = 1.
 * This is required for TDAC (perfect reconstruction via overlap-add).
 *
 * Previous implementation used a full-length Kaiser which did NOT satisfy
 * the power-complementary property, causing systematic energy loss in
 * HIGH/ULTRA quality modes. */
static void
compute_kbd_window(float *window, uint16_t size, double alpha)
{
	uint16_t half = size / 2;           /* N/2 */
	uint16_t kaiser_len = half + 1;     /* N/2 + 1 Kaiser samples */

	float kaiser_window[OPVOX_MDCT_MAX / 2 + 1];
	double w_sum[OPVOX_MDCT_MAX / 2 + 1];

	/* Generate Kaiser window of length N/2+1 */
	compute_kaiser_window(kaiser_window, kaiser_len, alpha);

	/* Cumulative sum over N/2+1 points */
	w_sum[0] = kaiser_window[0];
	for (uint16_t n = 1; n < kaiser_len; n++) {
		w_sum[n] = w_sum[n - 1] + kaiser_window[n];
	}

	double total_sum = w_sum[half]; /* W[N/2] */

	/* Rising half: w[n] = sqrt(W[n] / W[N/2]) for n = 0..N/2-1 */
	for (uint16_t n = 0; n < half; n++) {
		window[n] = (float)sqrt(w_sum[n] / total_sum);
	}

	/* Falling half: mirror — satisfies w[n]^2 + w[n+N/2]^2 = 1 */
	for (uint16_t n = 0; n < half; n++) {
		window[size - 1 - n] = window[n];
	}
}

/* Precompute window (sine or KBD based on type) */
static void
compute_window(float *window, uint16_t size, opvox_window_t window_type)
{
	switch (window_type) {
	case OPVOX_WINDOW_SINE:
		for (uint16_t n = 0; n < size; n++) {
			window[n] = (float)sin(M_PI * (n + 0.5) / size);
		}
		break;

	case OPVOX_WINDOW_KBD:
		/* Use alpha = 4.0 for good sidelobe attenuation */
		compute_kbd_window(window, size, 4.0);
		break;

	default:
		/* Fallback to sine window */
		for (uint16_t n = 0; n < size; n++) {
			window[n] = (float)sin(M_PI * (n + 0.5) / size);
		}
		break;
	}
}

/* Short block MDCT forward transform */
static void
mdct_short_forward(const float *pcm_channel, float *mdct_coeffs,
                   uint16_t frame_samples, float short_overlap[OPVOX_SHORT_BLOCKS][OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS],
                   opvox_window_t window_type)
{
	uint16_t short_size = frame_samples / OPVOX_SHORT_BLOCKS;
	uint16_t window_size = short_size * 2;
	float short_window[OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS * 2];
	float short_buf[OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS * 2];

	/* Compute short window with specified type */
	compute_window(short_window, window_size, window_type);

	for (int blk = 0; blk < OPVOX_SHORT_BLOCKS; blk++) {
		/* Fill with overlap from previous short block and current */
		for (uint16_t i = 0; i < short_size; i++) {
			short_buf[i] = short_overlap[blk][i];  /* previous */
			short_buf[i + short_size] = pcm_channel[blk * short_size + i];
		}

		/* Update overlap for next short block */
		for (uint16_t i = 0; i < short_size; i++) {
			short_overlap[blk][i] = pcm_channel[blk * short_size + i];
		}

		/* Forward MDCT on short block */
		float short_coeffs[OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS];
		mdct_forward(short_buf, short_coeffs, short_size, short_window);

		/* Store in mdct_coeffs at offset blk * short_size */
		memcpy(mdct_coeffs + blk * short_size, short_coeffs, short_size * sizeof(float));
	}
}

/* Short block MDCT inverse transform */
static void
mdct_short_inverse(const float *mdct_coeffs, float *pcm_channel,
                   uint16_t frame_samples, float short_overlap[OPVOX_SHORT_BLOCKS][OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS],
                   opvox_window_t window_type)
{
	uint16_t short_size = frame_samples / OPVOX_SHORT_BLOCKS;
	uint16_t window_size = short_size * 2;
	float short_window[OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS * 2];
	float short_buf[OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS * 2];

	/* Compute short window with specified type */
	compute_window(short_window, window_size, window_type);

	for (int blk = 0; blk < OPVOX_SHORT_BLOCKS; blk++) {
		float short_coeffs[OPVOX_MAX_FRAME / OPVOX_SHORT_BLOCKS];
		memcpy(short_coeffs, mdct_coeffs + blk * short_size, short_size * sizeof(float));

		/* Inverse MDCT on short block */
		mdct_inverse(short_coeffs, short_buf, short_size, short_window);

		/* Overlap-add within the frame */
		for (uint16_t i = 0; i < short_size; i++) {
			pcm_channel[blk * short_size + i] = short_buf[i] + short_overlap[blk][i];
			short_overlap[blk][i] = short_buf[i + short_size];
		}
	}
}

/* Pre-emphasis correction factor for a single MDCT band.
 *
 * Pre-emphasis FIR: H(z) = 1 - α·z^{-1}
 * |H(ω)|² = 1 - 2α·cos(ω) + α²,   ω = 2π·center_bin / N_mdct
 *
 * At high sample rates a 440 Hz tone sits at very low normalised frequency,
 * so |H_pe|² ≈ 0.004.  Quantising pre-emphasised energies directly means
 * any 6 dB coarse-step error is amplified ×250 after de-emphasis.
 *
 * The encoder divides band_energies[b] by this value before energy_encode_*
 * (converting to the original domain) and the decoder multiplies band_energies[b]
 * by it after energy_decode_* (converting back to the pre-emphasised domain
 * needed for MDCT coefficient scaling).  Both sides compute the same value
 * from the shared sample-rate and band-boundary information so no extra bits
 * are transmitted.
 */
static float
pe_band_correction(uint16_t center_bin, uint16_t frame_samples, float alpha)
{
	float omega = (float)(2.0 * M_PI) * (float)center_bin / (2.0f * (float)frame_samples);
	float h2 = 1.0f - 2.0f * alpha * cosf(omega) + alpha * alpha;
	return (h2 > 1e-8f) ? h2 : 1e-8f;
}

/* Pre-emphasis filter */
static void
pre_emphasis(const int16_t *pcm, float *out, uint16_t samples, float *state, uint8_t channels, float alpha)
{
	for (uint16_t i = 0; i < samples; i++) {
		for (uint8_t ch = 0; ch < channels; ch++) {
			float x = pcm[i * channels + ch] / 32768.0f;
			float y = x - alpha * state[ch];
			out[i * channels + ch] = y;
			state[ch] = x;
		}
	}
}

/* De-emphasis filter */
static void
de_emphasis(const float *in, int16_t *pcm, uint16_t samples, float *state, uint8_t channels, float alpha)
{
	for (uint16_t i = 0; i < samples; i++) {
		for (uint8_t ch = 0; ch < channels; ch++) {
			float x = in[i * channels + ch];
			float y = x + alpha * state[ch];
			state[ch] = y;

			/* Clamp and convert to 16-bit */
			if (y > 1.0f) y = 1.0f;
			if (y < -1.0f) y = -1.0f;
			pcm[i * channels + ch] = (int16_t)(y * 32767.0f);
		}
	}
}

/* Joint stereo processing */
static bool
should_use_joint_stereo(const float *left, const float *right, uint16_t frame_samples)
{
	float mid_energy = 0.0f, side_energy = 0.0f;

	for (uint16_t i = 0; i < frame_samples; i++) {
		float mid = (left[i] + right[i]) * 0.5f;
		float side = (left[i] - right[i]) * 0.5f;

		mid_energy += mid * mid;
		side_energy += side * side;
	}

	/* Use joint stereo if side energy is less than 50% of mid energy */
	return (side_energy < 0.5f * mid_energy);
}

static void
encode_joint_stereo(const float *left, const float *right, float *mid, float *side, uint16_t frame_samples)
{
	for (uint16_t i = 0; i < frame_samples; i++) {
		mid[i] = (left[i] + right[i]) * 0.5f;
		side[i] = (left[i] - right[i]) * 0.5f;
	}
}

static void
decode_joint_stereo(const float *mid, const float *side, float *left, float *right, uint16_t frame_samples)
{
	for (uint16_t i = 0; i < frame_samples; i++) {
		left[i] = mid[i] + side[i];
		right[i] = mid[i] - side[i];
	}
}

/* Detect transients by analyzing sub-frame energy */
static bool
detect_transients(const float *pcm, uint16_t frame_samples, uint8_t channels,
                  float *quarter_energies, float *prev_quarter_energies)
{
	uint16_t quarter_size = frame_samples / 4;
	float total_energy = 0.0f;

	/* Compute energy for each quarter of the frame */
	for (int q = 0; q < 4; q++) {
		quarter_energies[q] = 0.0f;
		for (uint16_t i = q * quarter_size; i < (q + 1) * quarter_size; i++) {
			for (uint8_t ch = 0; ch < channels; ch++) {
				float sample = pcm[i * channels + ch];
				quarter_energies[q] += sample * sample;
			}
		}
		quarter_energies[q] /= (quarter_size * channels);
		total_energy += quarter_energies[q];
	}

	float avg_energy = total_energy / 4.0f;

	/* Check for energy spikes (transients) */
	for (int q = 0; q < 4; q++) {
		if (quarter_energies[q] > 4.0f * avg_energy) {
			return true;
		}
		/* Also check for sudden increases from previous frame */
		if (quarter_energies[q] > 3.0f * prev_quarter_energies[q] && prev_quarter_energies[q] > 1e-6f) {
			return true;
		}
	}

	return false;
}

/* Comfort noise generation */
static uint32_t
xorshift32(uint32_t *state)
{
	*state ^= *state << 13;
	*state ^= *state >> 17;
	*state ^= *state << 5;
	return *state;
}

static void
generate_comfort_noise(float *out, uint16_t samples, uint32_t *rng_state, uint8_t channels)
{
	for (uint16_t i = 0; i < samples; i++) {
		for (uint8_t ch = 0; ch < channels; ch++) {
			float noise = ((float)xorshift32(rng_state) / (float)UINT32_MAX - 0.5f) * 2.0f;
			out[i * channels + ch] = noise * COMFORT_NOISE_LEVEL;
		}
	}
}

/* Pre-echo attack gain limiting - apply gain ramp to initial coefficients if energy jump is too large */
static void
apply_gain_limiting(float *mdct_coeffs, uint16_t frame_samples,
                    float current_max_energy, float *prev_frame_max_energy, float *gain_limit)
{
	const float ATTACK_THRESHOLD = 10.0f;  /* 10:1 energy ratio threshold */
	const float MIN_GAIN = 0.1f;           /* minimum gain limit */
	const float GAIN_RECOVERY = 0.95f;     /* gain recovery factor */

	(void)current_max_energy;

	/* Compute current frame's maximum energy */
	float max_energy = 0.0f;
	for (uint16_t i = 0; i < frame_samples; i++) {
		float energy = mdct_coeffs[i] * mdct_coeffs[i];
		if (energy > max_energy) {
			max_energy = energy;
		}
	}

	/* Check if we have a sudden energy attack */
	float energy_ratio = 1.0f;
	if (*prev_frame_max_energy > 1e-10f) {
		energy_ratio = max_energy / *prev_frame_max_energy;
	}

	/* Apply gain limiting if energy ratio exceeds threshold */
	if (energy_ratio > ATTACK_THRESHOLD) {
		/* Calculate new gain limit - more aggressive limiting for larger attacks */
		float target_gain = ATTACK_THRESHOLD / energy_ratio;
		*gain_limit = fmaxf(target_gain, MIN_GAIN);

		/* Apply linear gain ramp to first N coefficients (N = frame_size/8) */
		uint16_t ramp_length = frame_samples / 8;
		for (uint16_t i = 0; i < ramp_length && i < frame_samples; i++) {
			float ramp_factor = *gain_limit + (1.0f - *gain_limit) * ((float)i / (float)(ramp_length - 1));
			mdct_coeffs[i] *= ramp_factor;
		}
	} else {
		/* Gradually recover gain limit */
		*gain_limit = fminf(*gain_limit / GAIN_RECOVERY, 1.0f);
	}

	/* Update previous frame energy for next frame */
	*prev_frame_max_energy = max_energy;
}

/* Get band boundaries and hearing threshold for sample rate */
static const uint16_t *
get_band_bounds(uint32_t sample_rate, uint8_t *n_bands, const float **threshold)
{
	switch (sample_rate) {
	case OPVOX_RATE_8K:
		*n_bands = OPVOX_BANDS_8K;
		if (threshold) *threshold = hearing_threshold_8k;
		return band_boundaries_8k;
	case OPVOX_RATE_16K:
		*n_bands = OPVOX_BANDS_16K;
		if (threshold) *threshold = hearing_threshold_16k;
		return band_boundaries_16k;
	case OPVOX_RATE_32K:
		*n_bands = OPVOX_BANDS_32K;
		if (threshold) *threshold = hearing_threshold_32k;
		return band_boundaries_32k;
	case OPVOX_RATE_48K:
		*n_bands = OPVOX_MAX_BANDS;
		if (threshold) *threshold = hearing_threshold_48k;
		return band_boundaries_48k;
	default:
		return NULL;
	}
}

/* Convert band boundaries to band_range_t arrays */
static void
build_band_ranges(const uint16_t *bounds, uint8_t n_bands, band_range_t *ranges)
{
	for (uint8_t b = 0; b < n_bands; b++) {
		ranges[b].start = bounds[b];
		ranges[b].end = bounds[b + 1];
	}
}

/* Public API implementation */
int
opvox_encoder_init(opvox_encoder_t *enc, uint32_t sample_rate,
                   uint8_t channels, opvox_quality_t quality)
{
	if (!enc || (channels != 1 && channels != 2))
		return -1;

	uint16_t frame_samples = opvox_frame_samples(sample_rate);
	if (frame_samples == 0)
		return -1;

	memset(enc, 0, sizeof(*enc));

	enc->sample_rate = sample_rate;
	enc->channels = channels;
	enc->quality = quality;
	enc->frame_samples = frame_samples;
	enc->mdct_size = frame_samples * 2;

	/* Set window type based on quality level */
	if (quality >= OPVOX_QUALITY_HIGH) {
		enc->window_type = OPVOX_WINDOW_KBD;  /* Use KBD for HIGH and ULTRA quality */
	} else {
		enc->window_type = OPVOX_WINDOW_SINE; /* Use sine for LOW and NORMAL quality */
	}

	const float *threshold;
	const uint16_t *band_bounds = get_band_bounds(sample_rate, &enc->n_bands, &threshold);
	if (!band_bounds)
		return -1;

	/* Copy hearing threshold */
	memcpy(enc->hearing_threshold, threshold, enc->n_bands * sizeof(float));

	/* Set target bits */
	uint8_t rate_idx = (sample_rate == OPVOX_RATE_8K)  ? 0 :
	                   (sample_rate == OPVOX_RATE_16K) ? 1 :
	                   (sample_rate == OPVOX_RATE_32K) ? 2 : 3;
	enc->target_bits = target_bits_table[quality][rate_idx];
	if (channels == 2)
		enc->target_bits *= 2;

	/* Compute window of selected type */
	compute_window(enc->window, enc->mdct_size, enc->window_type);

	enc->silence_threshold = SILENCE_THRESHOLD;
	enc->energy_avg = 1.0f;

	/* Initialize pre-echo attack gain limiting — per-channel */
	enc->prev_frame_max_energy = 1e-10f;
	enc->gain_limit = 1.0f;
	enc->prev_frame_max_energy_r = 1e-10f;
	enc->gain_limit_r = 1.0f;

	/* Initialize short block state */
	enc->prev_transient = false;
	memset(enc->short_overlap, 0, sizeof(enc->short_overlap));
	memset(enc->short_overlap_r, 0, sizeof(enc->short_overlap_r));

	/* Initialize new modules */
	pitch_detect_init(&enc->pitch_det, sample_rate);
	pitch_filter_init(&enc->pitch_filt);
	pitch_filter_init(&enc->pitch_filt_r);
	sns_init(&enc->sns);
	bwe_init(&enc->bwe, 0, enc->n_bands);  /* cutoff will be determined per frame */
	tns_init(&enc->tns);
	energy_init(&enc->energy, enc->n_bands);
	energy_init(&enc->energy_r, enc->n_bands);
	pns_init(&enc->pns);
	vmd_init(&enc->vmd, sample_rate);

	/* Psychoacoustic masking model: initialize with band boundaries */
	{
		band_range_t band_ranges[OPVOX_MAX_BANDS];
		build_band_ranges(band_bounds, enc->n_bands, band_ranges);
		uint16_t band_starts[OPVOX_MAX_BANDS], band_ends[OPVOX_MAX_BANDS];
		for (uint8_t b = 0; b < enc->n_bands; b++) {
			band_starts[b] = band_ranges[b].start;
			band_ends[b]   = band_ranges[b].end;
		}
		psych_init(&enc->psych, band_starts, band_ends, enc->n_bands, sample_rate);
	}

	return 0;
}

int
opvox_decoder_init(opvox_decoder_t *dec, uint32_t sample_rate,
                   uint8_t channels, opvox_quality_t quality)
{
	if (!dec || (channels != 1 && channels != 2))
		return -1;

	uint16_t frame_samples = opvox_frame_samples(sample_rate);
	if (frame_samples == 0)
		return -1;

	memset(dec, 0, sizeof(*dec));

	dec->sample_rate = sample_rate;
	dec->channels = channels;
	dec->quality = quality;
	dec->frame_samples = frame_samples;
	dec->mdct_size = frame_samples * 2;

	/* Set window type based on quality level (same as encoder) */
	if (quality >= OPVOX_QUALITY_HIGH) {
		dec->window_type = OPVOX_WINDOW_KBD;  /* Use KBD for HIGH and ULTRA quality */
	} else {
		dec->window_type = OPVOX_WINDOW_SINE; /* Use sine for LOW and NORMAL quality */
	}

	const float *threshold;
	const uint16_t *band_bounds = get_band_bounds(sample_rate, &dec->n_bands, &threshold);
	if (!band_bounds)
		return -1;

	/* Copy hearing threshold */
	memcpy(dec->hearing_threshold, threshold, dec->n_bands * sizeof(float));

	/* Compute window of selected type */
	compute_window(dec->window, dec->mdct_size, dec->window_type);

	/* Initialize RNG for comfort noise */
	dec->rng = 0x12345678;

	/* Initialize short block state */
	dec->prev_was_short = false;
	memset(dec->short_overlap, 0, sizeof(dec->short_overlap));
	memset(dec->short_overlap_r, 0, sizeof(dec->short_overlap_r));

	/* Initialize new modules */
	pitch_filter_init(&dec->pitch_filt);
	pitch_filter_init(&dec->pitch_filt_r);
	sns_init(&dec->sns);
	bwe_init(&dec->bwe, 0, dec->n_bands);
	tns_init(&dec->tns);
	energy_init(&dec->energy, dec->n_bands);
	energy_init(&dec->energy_r, dec->n_bands);
	pns_init(&dec->pns);

	return 0;
}

int
opvox_encode(opvox_encoder_t *enc, const int16_t *pcm,
             uint8_t *out, size_t out_cap)
{
	if (!enc || !pcm || !out || out_cap < 2)
		return -1;

	float pcm_float[OPVOX_MAX_FRAME * 2];
	float mdct_buf[OPVOX_MDCT_MAX];
	float mdct_coeffs[OPVOX_MAX_FRAME];
	float mdct_coeffs_r[OPVOX_MAX_FRAME];

	/* Variables for later use */
	float quarter_energies[4];
	bool has_transients, is_silence, use_joint_stereo = false, voiced, use_tns = false;
	uint8_t flags = 0;
	float frame_energy = 0.0f;
	uint16_t cutoff_band;
	const uint16_t *band_bounds;
	band_range_t band_ranges[OPVOX_MAX_BANDS];
	float left_channel[OPVOX_MAX_FRAME];
	pitch_info_t pitch;
	tns_params_t tns_params = {0};
	uint8_t side_info[256];
	bit_writer_t writer;
	size_t side_info_bytes;
	uint8_t bwe_buf[64], pvq_buf[512];
	int bwe_bytes, base_k;
	size_t pvq_bytes, total_payload, offset;
	const uint8_t *pvq_data;

	/* VAD — compute energy from raw PCM so pre-emphasis doesn't
	 * false-positive on low frequencies at high sample rates. */
	for (uint16_t i = 0; i < enc->frame_samples * enc->channels; i++) {
		float s = pcm[i] / 32768.0f;
		frame_energy += s * s;
	}
	frame_energy /= (enc->frame_samples * enc->channels);

	/* Pre-emphasis */
	pre_emphasis(pcm, pcm_float, enc->frame_samples, enc->pre_emph, enc->channels, pre_emph_coeff(enc->sample_rate));

	/* Update energy average */
	enc->energy_avg = ENERGY_SMOOTH * enc->energy_avg + (1.0f - ENERGY_SMOOTH) * frame_energy;

	/* Check for silence */
	is_silence = (frame_energy < enc->silence_threshold * enc->energy_avg);

	/* Detect transients for TNS */
	has_transients = detect_transients(pcm_float, enc->frame_samples, enc->channels,
	                                   quarter_energies, enc->prev_quarter_energy);
	memcpy(enc->prev_quarter_energy, quarter_energies, sizeof(quarter_energies));

	/* Frame header */
	if (enc->channels == 2)
		flags |= OPVOX_FLAG_STEREO;

	switch (enc->sample_rate) {
	case OPVOX_RATE_8K:  flags |= OPVOX_FLAG_RATE_8K;  break;
	case OPVOX_RATE_16K: flags |= OPVOX_FLAG_RATE_16K; break;
	case OPVOX_RATE_32K: flags |= OPVOX_FLAG_RATE_32K; break;
	case OPVOX_RATE_48K: flags |= OPVOX_FLAG_RATE_48K; break;
	}

	flags |= (enc->quality << OPVOX_FLAG_QUAL_SHIFT);

	if (is_silence) {
		flags |= OPVOX_FLAG_SILENCE;
		out[0] = flags;
		out[1] = 0;  /* No encoded data */
		return 2;
	}

	if (has_transients)
		flags |= OPVOX_FLAG_SHORT_BLOCKS;

	/* Joint stereo decision */
	if (enc->channels == 2) {
		float right_only[OPVOX_MAX_FRAME];
		for (uint16_t i = 0; i < enc->frame_samples; i++) {
			left_channel[i] = pcm_float[i * 2];
			right_only[i] = pcm_float[i * 2 + 1];
		}
		use_joint_stereo = should_use_joint_stereo(left_channel, right_only, enc->frame_samples);
	}

	if (use_joint_stereo)
		flags |= OPVOX_FLAG_JOINT_STEREO;

	out[0] = flags;

	/* Extract left channel for analysis */
	for (uint16_t i = 0; i < enc->frame_samples; i++) {
		left_channel[i] = pcm_float[i * enc->channels];
	}

	/* Pitch detection and pre-filtering.
	 * IMPORTANT: use the same quantized period/gain the decoder will receive,
	 * so the IIR post-filter at the decoder perfectly cancels this FIR pre-filter.
	 * Using the raw (unquantized) pitch.gain here while the decoder uses the
	 * bitstream-decoded value causes a gain mismatch that degrades voiced quality. */
	pitch = pitch_detect(&enc->pitch_det, left_channel, enc->frame_samples);
	voiced = (pitch.period > 0);

	/* Quantize period and gain for pre-filter — must match decoder's values */
	int   pitch_period_q = 0;
	float pitch_gain_q   = 0.0f;
	if (voiced) {
		uint8_t period_code_q = pitch_encode_period(pitch.period, enc->sample_rate);
		uint8_t gain_code_q   = pitch_encode_gain(pitch.gain);
		pitch_period_q = pitch_decode_period(period_code_q, enc->sample_rate);
		pitch_gain_q   = pitch_decode_gain(gain_code_q);
		if (pitch_period_q == 0) voiced = false;  /* quantization pushed out of range */
	}

	if (voiced) {
		/* Apply pitch pre-filter to L channel using quantized params */
		pitch_prefilter(&enc->pitch_filt, left_channel, enc->frame_samples,
		               pitch_period_q, pitch_gain_q);
		for (uint16_t i = 0; i < enc->frame_samples; i++) {
			pcm_float[i * enc->channels] = left_channel[i];
		}

		/* Apply same pitch pre-filter to R channel in stereo */
		if (enc->channels == 2) {
			float right_channel[OPVOX_MAX_FRAME];
			for (uint16_t i = 0; i < enc->frame_samples; i++) {
				right_channel[i] = pcm_float[i * 2 + 1];
			}
			pitch_prefilter(&enc->pitch_filt_r, right_channel, enc->frame_samples,
			               pitch_period_q, pitch_gain_q);
			for (uint16_t i = 0; i < enc->frame_samples; i++) {
				pcm_float[i * 2 + 1] = right_channel[i];
			}
		}
	} else if (enc->channels == 2) {
		/* Advance R pre-filter delay buffer even for unvoiced frames */
		float right_channel[OPVOX_MAX_FRAME];
		for (uint16_t i = 0; i < enc->frame_samples; i++) {
			right_channel[i] = pcm_float[i * 2 + 1];
		}
		pitch_prefilter(&enc->pitch_filt_r, right_channel, enc->frame_samples, 0, 0.0f);
	}

	/* MDCT analysis - handle long vs short blocks */
	if (has_transients && (flags & OPVOX_FLAG_SHORT_BLOCKS)) {
		/* Short block mode: 8 sub-transforms - left channel */
		float left_channel[OPVOX_MAX_FRAME];
		for (uint16_t i = 0; i < enc->frame_samples; i++) {
			left_channel[i] = pcm_float[i * enc->channels];
		}
		mdct_short_forward(left_channel, mdct_coeffs, enc->frame_samples, enc->short_overlap, enc->window_type);

		/* Process right channel if stereo */
		if (enc->channels == 2) {
			float right_channel[OPVOX_MAX_FRAME];
			for (uint16_t i = 0; i < enc->frame_samples; i++) {
				right_channel[i] = pcm_float[i * 2 + 1];
			}
			mdct_short_forward(right_channel, mdct_coeffs_r, enc->frame_samples, enc->short_overlap_r, enc->window_type);
		}
	} else {
		/* Long block mode (normal) - left channel */
		for (uint16_t i = 0; i < enc->frame_samples; i++) {
			mdct_buf[i] = enc->overlap[i];
			mdct_buf[i + enc->frame_samples] = pcm_float[i * enc->channels];
		}
		for (uint16_t i = 0; i < enc->frame_samples; i++) {
			enc->overlap[i] = pcm_float[i * enc->channels];
		}
		mdct_forward(mdct_buf, mdct_coeffs, enc->frame_samples, enc->window);

		/* Process right channel if stereo */
		if (enc->channels == 2) {
			for (uint16_t i = 0; i < enc->frame_samples; i++) {
				mdct_buf[i] = enc->overlap_r[i];
				mdct_buf[i + enc->frame_samples] = pcm_float[i * 2 + 1];
			}
			for (uint16_t i = 0; i < enc->frame_samples; i++) {
				enc->overlap_r[i] = pcm_float[i * 2 + 1];
			}
			mdct_forward(mdct_buf, mdct_coeffs_r, enc->frame_samples, enc->window);
		}
	}

	/* Apply pre-echo attack gain limiting for long blocks only */
	if (!(has_transients && (flags & OPVOX_FLAG_SHORT_BLOCKS))) {
		apply_gain_limiting(mdct_coeffs, enc->frame_samples,
		                   frame_energy, &enc->prev_frame_max_energy, &enc->gain_limit);

		/* Apply gain limiting to right channel with its own per-channel state */
		if (enc->channels == 2) {
			apply_gain_limiting(mdct_coeffs_r, enc->frame_samples,
			                   frame_energy, &enc->prev_frame_max_energy_r, &enc->gain_limit_r);
		}
	}

	/* Convert to mid/side if joint stereo */
	if (use_joint_stereo) {
		float mdct_mid[OPVOX_MAX_FRAME], mdct_side[OPVOX_MAX_FRAME];
		encode_joint_stereo(mdct_coeffs, mdct_coeffs_r, mdct_mid, mdct_side, enc->frame_samples);
		memcpy(mdct_coeffs, mdct_mid, enc->frame_samples * sizeof(float));
		memcpy(mdct_coeffs_r, mdct_side, enc->frame_samples * sizeof(float));
	}

	/* Build band ranges */
	band_bounds = get_band_bounds(enc->sample_rate, &enc->n_bands, NULL);
	build_band_ranges(band_bounds, enc->n_bands, band_ranges);

	/* SNS analysis */
	sns_analyze(&enc->sns, mdct_coeffs, enc->frame_samples, band_ranges, enc->n_bands);

	/* VMD: classify content (voice/music/mixed) for adaptive codec decisions.
	 * Done after SNS analysis so mdct_coeffs reflect pre-flatten spectral shape. */
	vmd_content_t vmd_content = vmd_classify(&enc->vmd, left_channel, enc->frame_samples,
	                                         mdct_coeffs, enc->frame_samples,
	                                         voiced ? pitch.gain : 0.0f, frame_energy);

	/* PNS: identify noise-like bands before SNS normalization.
	 * Skip for voiced or pitched content — the pitch pre-filter attenuates
	 * the fundamental, making periodic signals look spectrally flat (noise-like).
	 * For stereo: use consensus — only flag as noise if BOTH L and R agree.
	 * This prevents a noise-like L band from erasing tonal R content. */
	if (!voiced && vmd_content != VMD_VOICE) {
		pns_analyze(&enc->pns, mdct_coeffs, enc->frame_samples, band_ranges, enc->n_bands);
		if (enc->channels == 2) {
			pns_ctx_t pns_r;
			pns_init(&pns_r);
			pns_analyze(&pns_r, mdct_coeffs_r, enc->frame_samples, band_ranges, enc->n_bands);
			for (uint8_t b = 0; b < enc->n_bands; b++)
				enc->pns.is_noise[b] = enc->pns.is_noise[b] && pns_r.is_noise[b];
		}
	} else {
		memset(enc->pns.is_noise, 0, sizeof(enc->pns.is_noise));
		enc->pns.num_bands = enc->n_bands;
	}

	/* Compute band energies from PRE-flatten MDCT coefficients.
	 * These pre-flatten energies are the true signal levels per band.
	 * The decoder's energy scaler uses them to recover gain after PVQ decode,
	 * so SNS scale factors do NOT need to be transmitted. */
	float band_energies[OPVOX_MAX_BANDS];
	float band_energies_r[OPVOX_MAX_BANDS];
	for (uint8_t b = 0; b < enc->n_bands; b++) {
		band_energies[b] = 0.0f;
		for (uint16_t k = band_ranges[b].start; k < band_ranges[b].end; k++) {
			band_energies[b] += mdct_coeffs[k] * mdct_coeffs[k];
		}
		band_energies[b] += 1e-10f;

		band_energies_r[b] = 0.0f;
		if (enc->channels == 2) {
			for (uint16_t k = band_ranges[b].start; k < band_ranges[b].end; k++) {
				band_energies_r[b] += mdct_coeffs_r[k] * mdct_coeffs_r[k];
			}
		}
		band_energies_r[b] += 1e-10f;
	}

	/* Convert pre-emphasised band energies to original domain before quantisation.
	 * Pre-emphasis attenuates low frequencies; at 48kHz |H_pe|^2 ≈ 0.004 for
	 * a 440 Hz band, so quantising directly would amplify every energy error by
	 * ~250× after de-emphasis.  Dividing by |H_pe|^2 here makes the quantiser
	 * operate on original-domain energies where 6 dB steps are perceptually
	 * uniform.  The decoder multiplies by the same factor after decoding. */
	for (uint8_t b = 0; b < enc->n_bands; b++) {
		uint16_t center = (band_ranges[b].start + band_ranges[b].end) / 2;
		float corr = pe_band_correction(center, enc->frame_samples, pre_emph_coeff(enc->sample_rate));
		band_energies[b]   /= corr;
		band_energies_r[b] /= corr;
	}
	/* Energy quantization — coarse pass */
	int8_t coarse_codes[OPVOX_MAX_BANDS];
	energy_encode_coarse(&enc->energy, band_energies, enc->n_bands, coarse_codes);
	int8_t coarse_codes_r[OPVOX_MAX_BANDS];
	if (enc->channels == 2) {
		energy_encode_coarse(&enc->energy_r, band_energies_r, enc->n_bands, coarse_codes_r);
	}

	/* SNS flatten — normalize spectrum for quantization noise shaping.
	 * Done after energy coding so energy codes carry pre-flatten signal levels. */
	sns_flatten(&enc->sns, mdct_coeffs, enc->frame_samples, band_ranges, enc->n_bands);
	if (enc->channels == 2) {
		sns_flatten(&enc->sns, mdct_coeffs_r, enc->frame_samples, band_ranges, enc->n_bands);
	}

	/* Zero noise band coefficients — PVQ will skip these bands. */
	pns_zero_noise_bands(&enc->pns, mdct_coeffs, enc->frame_samples, band_ranges, enc->n_bands);
	if (enc->channels == 2) {
		pns_zero_noise_bands(&enc->pns, mdct_coeffs_r, enc->frame_samples, band_ranges, enc->n_bands);
	}

	/* TNS analysis and filtering if transient */
	if (has_transients) {
		tns_band_t tns_bands[OPVOX_MAX_BANDS];
		for (uint8_t b = 0; b < enc->n_bands; b++) {
			tns_bands[b].start = band_ranges[b].start;
			tns_bands[b].end = band_ranges[b].end;
		}

		use_tns = tns_analyze(&enc->tns, mdct_coeffs, enc->frame_samples, tns_bands, enc->n_bands, &tns_params);
		if (use_tns) {
			tns_filter_encode(&enc->tns, mdct_coeffs, enc->frame_samples, tns_bands, enc->n_bands, &tns_params);
			if (enc->channels == 2) {
				tns_filter_encode(&enc->tns, mdct_coeffs_r, enc->frame_samples, tns_bands, enc->n_bands, &tns_params);
			}
		}
	}

	/* BWE cutoff determination */
	cutoff_band = bwe_optimal_cutoff(enc->target_bits / enc->channels, enc->n_bands, enc->sample_rate);
	enc->bwe.cutoff_band = cutoff_band;
	enc->bwe.total_bands = enc->n_bands;
	enc->bwe.num_high_bands = (cutoff_band < enc->n_bands) ? (enc->n_bands - cutoff_band) : 0;

	/* Encode data in side info section */
	bit_writer_init(&writer, side_info, sizeof(side_info));

	/* Pitch info — write the same codes we already derived for the pre-filter */
	if (bit_writer_write(&writer, voiced ? 1 : 0, 1) < 0) return -1;
	if (voiced) {
		uint8_t period_code = pitch_encode_period(pitch_period_q, enc->sample_rate);
		uint8_t gain_code   = pitch_encode_gain(pitch_gain_q);
		if (bit_writer_write(&writer, period_code, 8) < 0) return -1;
		if (bit_writer_write(&writer, gain_code,   6) < 0) return -1;
	}

	/* TNS params — 1-bit flag; if set, 4-bit byte count then raw bytes */
	if (bit_writer_write(&writer, use_tns ? 1 : 0, 1) < 0) return -1;
	if (use_tns) {
		int tns_bytes = tns_encode_params(&tns_params, bwe_buf, sizeof(bwe_buf));
		if (tns_bytes <= 0) return -1;
		if (bit_writer_write(&writer, (uint32_t)tns_bytes, 4) < 0) return -1;
		for (int i = 0; i < tns_bytes; i++) {
			if (bit_writer_write(&writer, bwe_buf[i], 8) < 0) return -1;
		}
	}

	/* BWE cutoff */
	if (bit_writer_write(&writer, cutoff_band, 6) < 0) return -1;

	/* PNS: 1 noise flag per coded band (energy carried by coarse codes) */
	for (uint8_t b = 0; b < cutoff_band; b++) {
		if (bit_writer_write(&writer, enc->pns.is_noise[b] ? 1u : 0u, 1) < 0) return -1;
	}

	/* Coarse energy codes — mid/left channel (7 bits each) */
	for (uint8_t b = 0; b < enc->n_bands; b++) {
		uint8_t unsigned_code = (uint8_t)((int16_t)coarse_codes[b] + 64);
		if (bit_writer_write(&writer, unsigned_code, 7) < 0) return -1;
	}

	/* Coarse energy codes — right/side channel (7 bits each, stereo only) */
	if (enc->channels == 2) {
		for (uint8_t b = 0; b < enc->n_bands; b++) {
			uint8_t unsigned_code = (uint8_t)((int16_t)coarse_codes_r[b] + 64);
			if (bit_writer_write(&writer, unsigned_code, 7) < 0) return -1;
		}
	}

	/* Pad to byte boundary before fine energy length placeholder */
	if (bit_writer_align(&writer) < 0) return -1;
	/* Reserve 8 bits for fine energy length (0-255 bytes) */
	size_t fine_len_pos = bit_writer_bytes(&writer);
	if (bit_writer_write(&writer, 0, 8) < 0) return -1;  /* placeholder */
	side_info_bytes = bit_writer_bytes(&writer);

	/* Encode BWE high band energies */
	bwe_bytes = bwe_encode(&enc->bwe, mdct_coeffs, enc->frame_samples, band_ranges, enc->n_bands, bwe_buf, sizeof(bwe_buf));
	if (bwe_bytes < 0) return -1;

	/* Encode PVQ shape indices as raw bits.
	 * PVQ shapes from SNS-flattened bands are uniformly distributed, so RANS
	 * gives zero compression gain.  Raw bits also remove the 256-symbol cap
	 * that was silently corrupting shapes for large codebooks (e.g. N=4, K=46
	 * → codebook 259808 >> old RANS_MAX_SYMBOLS=256). */
	bit_writer_t pvq_writer;
	bit_writer_init(&pvq_writer, pvq_buf, sizeof(pvq_buf));

	/* Base K value from quality level */
	base_k = base_k_table[enc->quality];

	/* Build band_N array for PVQ bit allocation */
	int band_N_enc[OPVOX_MAX_BANDS];
	for (uint8_t b = 0; b < cutoff_band; b++)
		band_N_enc[b] = (int)(band_ranges[b].end - band_ranges[b].start);

	/* Psychoacoustic masking: compute spreading-function-based masking thresholds.
	 * These replace the static ATH in pvq_allocate_k_per_band, allowing the PVQ
	 * bit allocator to skip bits on perceptually masked bands and concentrate
	 * them on prominent tonal bands.
	 *
	 * mask_dB[b] = max(ATH[b], signal-based spreading threshold[b])
	 * Bands where signal is below mask_dB get fewer PVQ pulses → smaller packet. */
	psych_result_t psych_res;
	psych_analyze(&enc->psych, enc->energy.coarse_dB,
	              enc->hearing_threshold, cutoff_band, &psych_res);

	/* PVQ bit budget: 45% of per-channel target bits for shape coding.
	 * Divide by channels so stereo K allocation matches the decoder's
	 * per-channel computation (decoder uses mono target_bits_table). */
	int pvq_bits_enc = (int)((float)enc->target_bits / (float)enc->channels * 0.45f);
	int K_per_band[OPVOX_MAX_BANDS];
	pvq_allocate_k_per_band(enc->energy.coarse_dB, enc->pns.is_noise,
	                        psych_res.mask_dB,   /* spreading-function thresholds */
	                        cutoff_band, band_N_enc, base_k,
	                        pvq_bits_enc, K_per_band);

	/* For stereo: boost K for bands where right channel has more energy than left.
	 * This prevents bands silent in L (but loud in R) from getting K=0. */
	if (enc->channels == 2) {
		psych_result_t psych_res_r;
		psych_analyze(&enc->psych, enc->energy_r.coarse_dB,
		              enc->hearing_threshold, cutoff_band, &psych_res_r);
		int K_r[OPVOX_MAX_BANDS];
		pvq_allocate_k_per_band(enc->energy_r.coarse_dB, enc->pns.is_noise,
		                        psych_res_r.mask_dB,
		                        cutoff_band, band_N_enc, base_k,
		                        pvq_bits_enc, K_r);
		for (uint8_t b = 0; b < cutoff_band; b++)
			if (K_r[b] > K_per_band[b])
				K_per_band[b] = K_r[b];
	}

	/* Encode each coded band with PVQ (noise bands handled by PNS, not PVQ) */
	for (uint8_t b = 0; b < cutoff_band; b++) {
		if (enc->pns.is_noise[b]) continue;
		uint16_t start = band_ranges[b].start;
		uint16_t end = band_ranges[b].end;
		int N = end - start;
		int K = K_per_band[b];

		/* Encode left/mid channel */
		float gain;
		int16_t shape[PVQ_MAX_DIM];
		pvq_encode(mdct_coeffs + start, N, K, &gain, shape);

		uint32_t idx = pvq_index_encode(shape, N, K);
		uint32_t codebook_size = pvq_codebook_size(N, K);
		uint8_t idx_bits = pvq_index_bits(codebook_size);
		bit_writer_write(&pvq_writer, idx, idx_bits);

		/* Encode right/side channel if stereo */
		if (enc->channels == 2) {
			pvq_encode(mdct_coeffs_r + start, N, K, &gain, shape);
			idx = pvq_index_encode(shape, N, K);
			bit_writer_write(&pvq_writer, idx, idx_bits);
		}
	}

	bit_writer_align(&pvq_writer);
	pvq_bytes = bit_writer_bytes(&pvq_writer);
	pvq_data = pvq_buf;

	/* Calculate remaining bits for fine energy quantization */
	size_t used_bits = (side_info_bytes + bwe_bytes + pvq_bytes) * 8;
	size_t target_bits = (size_t)enc->target_bits;  /* already accounts for all channels */
	uint16_t fine_bits_budget = 0;
	if (target_bits > used_bits) {
		fine_bits_budget = (uint16_t)((target_bits - used_bits) / 2);  /* Use up to half for fine energy */
	}

	/* Fine energy quantization if we have spare bits */
	uint8_t fine_bits_per_band[OPVOX_MAX_BANDS];
	uint8_t fine_codes[OPVOX_MAX_BANDS];
	uint8_t fine_bits_per_band_r[OPVOX_MAX_BANDS];
	uint8_t fine_codes_r[OPVOX_MAX_BANDS];
	int fine_bytes = 0;
	uint8_t fine_buf[64];

	if (fine_bits_budget > 0) {
		/* Split budget between channels */
		uint16_t fine_budget_per_ch = (enc->channels == 2) ? fine_bits_budget / 2 : fine_bits_budget;

		energy_allocate_fine_bits(fine_budget_per_ch, enc->energy.coarse_dB, enc->n_bands, fine_bits_per_band);
		energy_encode_fine(&enc->energy, band_energies, enc->n_bands, fine_bits_per_band, fine_codes);

		if (enc->channels == 2) {
			energy_allocate_fine_bits(fine_budget_per_ch, enc->energy_r.coarse_dB, enc->n_bands, fine_bits_per_band_r);
			energy_encode_fine(&enc->energy_r, band_energies_r, enc->n_bands, fine_bits_per_band_r, fine_codes_r);
		}

		/* Pack fine bits into bytes */
		bit_writer_t fine_writer;
		bit_writer_init(&fine_writer, fine_buf, sizeof(fine_buf));

		/* Write mid/left fine bits per band (3 bits each) and codes */
		for (uint8_t b = 0; b < enc->n_bands; b++) {
			if (bit_writer_write(&fine_writer, fine_bits_per_band[b], 3) < 0) break;
		}
		for (uint8_t b = 0; b < enc->n_bands; b++) {
			if (fine_bits_per_band[b] > 0) {
				if (bit_writer_write(&fine_writer, fine_codes[b], fine_bits_per_band[b]) < 0) break;
			}
		}

		/* Write right/side fine bits per band (3 bits each) and codes — stereo only */
		if (enc->channels == 2) {
			for (uint8_t b = 0; b < enc->n_bands; b++) {
				if (bit_writer_write(&fine_writer, fine_bits_per_band_r[b], 3) < 0) break;
			}
			for (uint8_t b = 0; b < enc->n_bands; b++) {
				if (fine_bits_per_band_r[b] > 0) {
					if (bit_writer_write(&fine_writer, fine_codes_r[b], fine_bits_per_band_r[b]) < 0) break;
				}
			}
		}

		fine_bytes = (int)bit_writer_bytes(&fine_writer);

		/* Update fine energy length in side info */
		side_info[fine_len_pos] = (uint8_t)fine_bytes;
	}

	/* Commit fine-corrected energy as temporal prediction for next frame */
	energy_commit(&enc->energy);
	if (enc->channels == 2) {
		energy_commit(&enc->energy_r);
	}

	/* Pack final output */
	total_payload = side_info_bytes + bwe_bytes + pvq_bytes + fine_bytes;
	if (2 + 1 + total_payload > out_cap)
		return -1;

	offset = 2;
	out[offset++] = (uint8_t)side_info_bytes;
	memcpy(out + offset, side_info, side_info_bytes);
	offset += side_info_bytes;
	memcpy(out + offset, bwe_buf, bwe_bytes);
	offset += bwe_bytes;
	memcpy(out + offset, pvq_data, pvq_bytes);
	offset += pvq_bytes;
	if (fine_bytes > 0) {
		memcpy(out + offset, fine_buf, fine_bytes);
		offset += fine_bytes;
	}

	out[1] = (uint8_t)(total_payload);

	/* Update transient state for next frame */
	enc->prev_transient = has_transients;

	return (int)offset;
}

int
opvox_decode(opvox_decoder_t *dec, const uint8_t *in, size_t in_len,
             int16_t *pcm)
{
	if (!dec || !pcm)
		return -1;

#ifdef OPVOX_DEBUG_ENERGY
	static int __dbg_frame_32ku = 0;
	int __this_frame = 0;
	int __do_dbg = (dec->sample_rate == OPVOX_RATE_32K
	                && dec->channels == 1
	                && dec->quality == OPVOX_QUALITY_ULTRA);
	if (__do_dbg) {
		__this_frame = __dbg_frame_32ku++;
		__do_dbg = (__this_frame >= 5 && __this_frame <= 7);
	}
#define DBG_ENERGY(fmt, ...) do { if (__do_dbg) fprintf(stderr, "[DBG f%d] " fmt "\n", __this_frame, ##__VA_ARGS__); } while(0)
#else
#define DBG_ENERGY(fmt, ...) do {} while(0)
#endif

	/* Handle silence/comfort noise */
	if (!in || in_len == 0 || (in_len >= 2 && (in[0] & OPVOX_FLAG_SILENCE))) {
		float comfort[OPVOX_MAX_FRAME * 2];
		generate_comfort_noise(comfort, dec->frame_samples, &dec->rng, dec->channels);
		de_emphasis(comfort, pcm, dec->frame_samples, dec->de_emph, dec->channels, pre_emph_coeff(dec->sample_rate));
		dec->plc_count = 0;
		return 0;
	}

	if (in_len < 3)
		return -1;

	uint8_t flags = in[0];
	uint8_t total_len = in[1];
	uint8_t side_info_len = in[2];

	if (3 + (size_t)total_len > in_len)
		return -1;

	/* Verify header matches decoder config */
	bool is_stereo = (flags & OPVOX_FLAG_STEREO) != 0;
	if ((is_stereo ? 2 : 1) != dec->channels)
		return -1;

	bool use_joint_stereo = (flags & OPVOX_FLAG_JOINT_STEREO) != 0;
	bool has_transients = (flags & OPVOX_FLAG_SHORT_BLOCKS) != 0;

	/* Parse side info */
	bit_reader_t reader;
	bit_reader_init(&reader, in + 3, side_info_len);

	/* Decode pitch info */
	bool voiced = bit_reader_read(&reader, 1);
	int pitch_period = 0;
	float pitch_gain = 0.0f;
	if (voiced) {
		uint8_t period_code = bit_reader_read(&reader, 8);
		uint8_t gain_code = bit_reader_read(&reader, 6);
		pitch_period = pitch_decode_period(period_code, dec->sample_rate);
		pitch_gain = pitch_decode_gain(gain_code);
	}

	/* Decode TNS params — read 4-bit byte count then raw bytes */
	bool use_tns = bit_reader_read(&reader, 1);
	tns_params_t tns_params = {0};
	if (use_tns) {
		uint32_t tns_bytes = bit_reader_read(&reader, 4);
		if (tns_bytes > 0 && tns_bytes <= 8) {
			uint8_t tns_buf[8];
			for (uint32_t i = 0; i < tns_bytes; i++)
				tns_buf[i] = (uint8_t)bit_reader_read(&reader, 8);
			tns_decode_params_from_stream(&tns_params, tns_buf, tns_bytes);
		}
	}

	/* Decode BWE cutoff */
	uint16_t cutoff_band = bit_reader_read(&reader, 6);
	dec->bwe.cutoff_band = cutoff_band;
	dec->bwe.total_bands = dec->n_bands;
	dec->bwe.num_high_bands = (cutoff_band < dec->n_bands) ? (dec->n_bands - cutoff_band) : 0;

	/* PNS: decode noise flags (energy handled by coarse codes + energy scaler) */
	memset(dec->pns.is_noise, 0, sizeof(dec->pns.is_noise));
	memset(dec->pns.noise_energy, 0, sizeof(dec->pns.noise_energy));
	dec->pns.num_bands = dec->n_bands;
	for (uint8_t b = 0; b < cutoff_band; b++) {
		dec->pns.is_noise[b] = (bit_reader_read(&reader, 1) != 0);
		if (dec->pns.is_noise[b])
			dec->pns.noise_energy[b] = 1.0f;  /* energy scaler sets final level */
	}

	/* Decode coarse energy codes — mid/left channel (7 bits each) */
	int8_t coarse_codes[OPVOX_MAX_BANDS];
	for (uint8_t b = 0; b < dec->n_bands; b++) {
		uint8_t unsigned_code = bit_reader_read(&reader, 7);
		coarse_codes[b] = (int8_t)((int16_t)unsigned_code - 64);
	}

	/* Decode coarse energy codes — right/side channel (7 bits each, stereo only) */
	int8_t coarse_codes_r[OPVOX_MAX_BANDS];
	if (dec->channels == 2) {
		for (uint8_t b = 0; b < dec->n_bands; b++) {
			uint8_t unsigned_code = bit_reader_read(&reader, 7);
			coarse_codes_r[b] = (int8_t)((int16_t)unsigned_code - 64);
		}
	}

	/* Decode coarse energy to band energies */
	float band_energies[OPVOX_MAX_BANDS];
	energy_decode_coarse(&dec->energy, coarse_codes, dec->n_bands, band_energies);
	float band_energies_r[OPVOX_MAX_BANDS];
	if (dec->channels == 2)
		energy_decode_coarse(&dec->energy_r, coarse_codes_r, dec->n_bands, band_energies_r);

	/* Pad to byte boundary before fine energy length (mirrors encoder alignment) */
	bit_reader_align(&reader);
	/* Decode fine energy length */
	uint8_t fine_energy_len = bit_reader_read(&reader, 8);

	/* Decode sections */
	size_t offset = 3 + side_info_len;

	/* Decode BWE energies */
	const uint8_t *bwe_data = in + offset;
	int bwe_bytes = bwe_decode(&dec->bwe, bwe_data, in_len - offset);
	if (bwe_bytes < 0) return -1;
	offset += bwe_bytes;

	/* Calculate PVQ data length (total remaining minus fine energy) */
	size_t remaining_len = in_len - offset;
	size_t pvq_len = remaining_len - fine_energy_len;

	/* Decode PVQ shape indices from raw bits */
	const uint8_t *pvq_data = in + offset;

	bit_reader_t pvq_reader;
	bit_reader_init(&pvq_reader, pvq_data, pvq_len);

	float mdct_coeffs[OPVOX_MAX_FRAME];
	float mdct_coeffs_r[OPVOX_MAX_FRAME];
	memset(mdct_coeffs, 0, sizeof(mdct_coeffs));
	memset(mdct_coeffs_r, 0, sizeof(mdct_coeffs_r));

	/* Build band ranges */
	const uint16_t *band_bounds = get_band_bounds(dec->sample_rate, &dec->n_bands, NULL);
	band_range_t band_ranges[OPVOX_MAX_BANDS];
	build_band_ranges(band_bounds, dec->n_bands, band_ranges);

	/* Base K value from quality level */
	int base_k = base_k_table[dec->quality];

	/* Build band_N array for PVQ bit allocation — mirror encoder exactly */
	int band_N_dec[OPVOX_MAX_BANDS];
	for (uint8_t b = 0; b < cutoff_band; b++)
		band_N_dec[b] = (int)(band_ranges[b].end - band_ranges[b].start);

	int rate_idx_dec = 0;
	switch (dec->sample_rate) {
	case OPVOX_RATE_8K:  rate_idx_dec = 0; break;
	case OPVOX_RATE_16K: rate_idx_dec = 1; break;
	case OPVOX_RATE_32K: rate_idx_dec = 2; break;
	default:             rate_idx_dec = 3; break;
	}
	int pvq_bits_dec = (int)(target_bits_table[dec->quality][rate_idx_dec] * 0.45f);
	int K_per_band_dec[OPVOX_MAX_BANDS];
	pvq_allocate_k_per_band(dec->energy.coarse_dB, dec->pns.is_noise,
	                        dec->hearing_threshold,
	                        cutoff_band, band_N_dec, base_k,
	                        pvq_bits_dec, K_per_band_dec);

	/* For stereo: boost K for bands where right channel has more energy.
	 * Must mirror the encoder's K calculation exactly. */
	if (dec->channels == 2) {
		int K_r_dec[OPVOX_MAX_BANDS];
		pvq_allocate_k_per_band(dec->energy_r.coarse_dB, dec->pns.is_noise,
		                        dec->hearing_threshold,
		                        cutoff_band, band_N_dec, base_k,
		                        pvq_bits_dec, K_r_dec);
		for (uint8_t b = 0; b < cutoff_band; b++)
			if (K_r_dec[b] > K_per_band_dec[b])
				K_per_band_dec[b] = K_r_dec[b];
	}

	/* Decode each coded band with PVQ (noise bands are left at zero for pns_fill_noise) */
	for (uint8_t b = 0; b < cutoff_band; b++) {
		if (dec->pns.is_noise[b]) continue;
		uint16_t start = band_ranges[b].start;
		uint16_t end = band_ranges[b].end;
		int N = end - start;
		int K = K_per_band_dec[b];

		uint32_t codebook_size = pvq_codebook_size(N, K);
		uint8_t idx_bits = pvq_index_bits(codebook_size);

		/* Decode left/mid channel */
		uint32_t idx_val = bit_reader_read(&pvq_reader, idx_bits);
		if (codebook_size > 0 && idx_val >= codebook_size) idx_val = 0;
		int16_t shape[PVQ_MAX_DIM];
		pvq_index_decode(idx_val, N, K, shape);

		float gain = 1.0f;
		pvq_decode(gain, shape, N, mdct_coeffs + start);

		/* Decode right/side channel if stereo */
		if (dec->channels == 2) {
			idx_val = bit_reader_read(&pvq_reader, idx_bits);
			if (codebook_size > 0 && idx_val >= codebook_size) idx_val = 0;
			pvq_index_decode(idx_val, N, K, shape);
			pvq_decode(gain, shape, N, mdct_coeffs_r + start);
		}
	}

	/* Update offset to point to fine energy data */
	offset += pvq_len;

	/* Decode fine energy if present */
	if (fine_energy_len > 0) {
		const uint8_t *fine_data = in + offset;
		bit_reader_t fine_reader;
		bit_reader_init(&fine_reader, fine_data, fine_energy_len);

		/* Decode mid/left fine bits per band and codes */
		uint8_t fine_bits_per_band[OPVOX_MAX_BANDS];
		for (uint8_t b = 0; b < dec->n_bands; b++) {
			fine_bits_per_band[b] = bit_reader_read(&fine_reader, 3);
		}
		uint8_t fine_codes[OPVOX_MAX_BANDS];
		for (uint8_t b = 0; b < dec->n_bands; b++) {
			fine_codes[b] = (fine_bits_per_band[b] > 0)
			              ? bit_reader_read(&fine_reader, fine_bits_per_band[b]) : 0;
		}
		energy_decode_fine(&dec->energy, fine_codes, fine_bits_per_band, dec->n_bands, band_energies);

		/* Decode right/side fine bits per band and codes — stereo only */
		if (dec->channels == 2) {
			uint8_t fine_bits_per_band_r[OPVOX_MAX_BANDS];
			for (uint8_t b = 0; b < dec->n_bands; b++) {
				fine_bits_per_band_r[b] = bit_reader_read(&fine_reader, 3);
			}
			uint8_t fine_codes_r[OPVOX_MAX_BANDS];
			for (uint8_t b = 0; b < dec->n_bands; b++) {
				fine_codes_r[b] = (fine_bits_per_band_r[b] > 0)
				                ? bit_reader_read(&fine_reader, fine_bits_per_band_r[b]) : 0;
			}
			energy_decode_fine(&dec->energy_r, fine_codes_r, fine_bits_per_band_r, dec->n_bands, band_energies_r);
		}
	}

	/* Commit fine-corrected energy as temporal prediction for next frame */
	energy_commit(&dec->energy);
	if (dec->channels == 2) {
		energy_commit(&dec->energy_r);
	}

	/* Convert decoded energies back from original domain to pre-emphasised domain.
	 * The encoder divided by |H_pe|^2 before quantising; we multiply by the same
	 * factor so that the MDCT coefficient scaler operates in the correct domain
	 * (pre-emphasised), which is then cancelled by the de-emphasis filter. */
	for (uint8_t b = 0; b < dec->n_bands; b++) {
		uint16_t center = (band_ranges[b].start + band_ranges[b].end) / 2;
		float corr = pe_band_correction(center, dec->frame_samples, pre_emph_coeff(dec->sample_rate));
		DBG_ENERGY("band %2u: E_decoded_orig=%.6e  corr=%.6f  E_preemph=%.6e",
		           b, band_energies[b], corr, band_energies[b] * corr);
		band_energies[b]   *= corr;
		if (dec->channels == 2)
			band_energies_r[b] *= corr;
	}
	/* BWE synthesis for high bands */
	bwe_synthesize(&dec->bwe, mdct_coeffs, dec->frame_samples, band_ranges, dec->n_bands);
	if (dec->channels == 2) {
		bwe_synthesize(&dec->bwe, mdct_coeffs_r, dec->frame_samples, band_ranges, dec->n_bands);
	}

	/* PNS: fill noise bands with shaped noise at post-flatten energy. */
	pns_fill_noise(&dec->pns, mdct_coeffs, dec->frame_samples, band_ranges, dec->n_bands);
	if (dec->channels == 2) {
		pns_fill_noise(&dec->pns, mdct_coeffs_r, dec->frame_samples, band_ranges, dec->n_bands);
	}

	/* Apply decoded band energies to recover original signal levels.
	 * Mid/left and right/side channels use their own target energies. */
	for (uint8_t b = 0; b < cutoff_band; b++) {
		float current_energy = 0.0f;
		for (uint16_t k = band_ranges[b].start; k < band_ranges[b].end; k++) {
			current_energy += mdct_coeffs[k] * mdct_coeffs[k];
		}
		DBG_ENERGY("energy_scale band %2u: pvq_E=%.6e  target_E=%.6e  scale=%.4f",
		           b, current_energy, band_energies[b],
		           (current_energy > 1e-10f) ? sqrtf(band_energies[b] / current_energy) : 0.0f);
		if (current_energy > 1e-10f) {
			float scale = sqrtf(band_energies[b] / current_energy);
			for (uint16_t k = band_ranges[b].start; k < band_ranges[b].end; k++) {
				mdct_coeffs[k] *= scale;
			}
		}

		if (dec->channels == 2) {
			float current_energy_r = 0.0f;
			for (uint16_t k = band_ranges[b].start; k < band_ranges[b].end; k++) {
				current_energy_r += mdct_coeffs_r[k] * mdct_coeffs_r[k];
			}
			if (current_energy_r > 1e-10f) {
				float scale_r = sqrtf(band_energies_r[b] / current_energy_r);
				for (uint16_t k = band_ranges[b].start; k < band_ranges[b].end; k++) {
					mdct_coeffs_r[k] *= scale_r;
				}
			}
		}
	}

	/* TNS synthesis filtering */
	if (use_tns && has_transients) {
		tns_band_t tns_bands[OPVOX_MAX_BANDS];
		for (uint8_t b = 0; b < dec->n_bands; b++) {
			tns_bands[b].start = band_ranges[b].start;
			tns_bands[b].end = band_ranges[b].end;
		}
		tns_filter_decode(&dec->tns, mdct_coeffs, dec->frame_samples, tns_bands, dec->n_bands, &tns_params);
		if (dec->channels == 2) {
			tns_filter_decode(&dec->tns, mdct_coeffs_r, dec->frame_samples, tns_bands, dec->n_bands, &tns_params);
		}
	}

	/* Handle joint stereo decoding */
	if (dec->channels == 2 && use_joint_stereo) {
		float left_coeffs[OPVOX_MAX_FRAME], right_coeffs[OPVOX_MAX_FRAME];
		decode_joint_stereo(mdct_coeffs, mdct_coeffs_r, left_coeffs, right_coeffs, dec->frame_samples);
		memcpy(mdct_coeffs, left_coeffs, dec->frame_samples * sizeof(float));
		memcpy(mdct_coeffs_r, right_coeffs, dec->frame_samples * sizeof(float));
	}

	/* Save coefficients for PLC */
	memcpy(dec->prev_mdct, mdct_coeffs, dec->frame_samples * sizeof(float));
	if (dec->channels == 2) {
		memcpy(dec->prev_mdct_r, mdct_coeffs_r, dec->frame_samples * sizeof(float));
	}

	/* Inverse MDCT and overlap-add - handle long vs short blocks */
	float mdct_buf[OPVOX_MDCT_MAX];
	float pcm_float[OPVOX_MAX_FRAME * 2];

	if (has_transients) {
		/* Short block mode: 8 sub-inverse-transforms - left channel */
		float left_channel[OPVOX_MAX_FRAME];
		mdct_short_inverse(mdct_coeffs, left_channel, dec->frame_samples, dec->short_overlap, dec->window_type);
		for (uint16_t i = 0; i < dec->frame_samples; i++) {
			pcm_float[i * dec->channels] = left_channel[i];
		}

		/* Right channel if stereo */
		if (dec->channels == 2) {
			float right_channel[OPVOX_MAX_FRAME];
			mdct_short_inverse(mdct_coeffs_r, right_channel, dec->frame_samples, dec->short_overlap_r, dec->window_type);
			for (uint16_t i = 0; i < dec->frame_samples; i++) {
				pcm_float[i * 2 + 1] = right_channel[i];
			}
		}
	} else {
		/* Long block mode (normal) - left channel */
		mdct_inverse(mdct_coeffs, mdct_buf, dec->frame_samples, dec->window);
		for (uint16_t i = 0; i < dec->frame_samples; i++) {
			pcm_float[i * dec->channels] = mdct_buf[i] + dec->overlap[i];
			dec->overlap[i] = mdct_buf[i + dec->frame_samples];
		}

		/* Right channel if stereo */
		if (dec->channels == 2) {
			mdct_inverse(mdct_coeffs_r, mdct_buf, dec->frame_samples, dec->window);
			for (uint16_t i = 0; i < dec->frame_samples; i++) {
				pcm_float[i * 2 + 1] = mdct_buf[i] + dec->overlap_r[i];
				dec->overlap_r[i] = mdct_buf[i + dec->frame_samples];
			}

		}
	}

	/* Pitch post-filter if voiced */
#ifdef OPVOX_DEBUG_ENERGY
	if (__do_dbg) {
		float __e_pcm_prepost = 0.0f;
		for (uint16_t i = 0; i < dec->frame_samples; i++) {
			float s = pcm_float[i * dec->channels];
			__e_pcm_prepost += s * s;
		}
		fprintf(stderr, "[DBG f%d] PCM E before post-filter: %.6e  voiced=%d  period=%d  gain=%.4f\n",
		        __this_frame, __e_pcm_prepost, voiced, pitch_period, pitch_gain);
	}
#endif
	if (voiced) {
		float left_channel[OPVOX_MAX_FRAME];
		for (uint16_t i = 0; i < dec->frame_samples; i++) {
			left_channel[i] = pcm_float[i * dec->channels];
		}
		pitch_postfilter(&dec->pitch_filt, left_channel, dec->frame_samples, pitch_period, pitch_gain);
		for (uint16_t i = 0; i < dec->frame_samples; i++) {
			pcm_float[i * dec->channels] = left_channel[i];
		}

		/* Apply same pitch post-filter to R channel in stereo */
		if (dec->channels == 2) {
			float right_channel[OPVOX_MAX_FRAME];
			for (uint16_t i = 0; i < dec->frame_samples; i++) {
				right_channel[i] = pcm_float[i * 2 + 1];
			}
			pitch_postfilter(&dec->pitch_filt_r, right_channel, dec->frame_samples, pitch_period, pitch_gain);
			for (uint16_t i = 0; i < dec->frame_samples; i++) {
				pcm_float[i * 2 + 1] = right_channel[i];
			}
		}
	} else if (dec->channels == 2) {
		/* Advance R post-filter delay buffer even for unvoiced frames */
		float right_channel[OPVOX_MAX_FRAME];
		for (uint16_t i = 0; i < dec->frame_samples; i++) {
			right_channel[i] = pcm_float[i * 2 + 1];
		}
		pitch_postfilter(&dec->pitch_filt_r, right_channel, dec->frame_samples, 0, 0.0f);
		for (uint16_t i = 0; i < dec->frame_samples; i++) {
			pcm_float[i * 2 + 1] = right_channel[i];
		}
	}

#ifdef OPVOX_DEBUG_ENERGY
	if (__do_dbg) {
		float __e_post = 0.0f;
		for (uint16_t i = 0; i < dec->frame_samples; i++) {
			float s = pcm_float[i * dec->channels];
			__e_post += s * s;
		}
		fprintf(stderr, "[DBG f%d] PCM E after post-filter: %.6e\n", __this_frame, __e_post);
	}
#endif

	/* De-emphasis and convert to 16-bit */
	de_emphasis(pcm_float, pcm, dec->frame_samples, dec->de_emph, dec->channels, pre_emph_coeff(dec->sample_rate));

	/* Update short block state for next frame */
	dec->prev_was_short = has_transients;

	dec->plc_count = 0;
	return 0;
}

int
opvox_decode_plc(opvox_decoder_t *dec, int16_t *pcm)
{
	if (!dec || !pcm)
		return -1;

	float mdct_buf[OPVOX_MDCT_MAX];
	float pcm_float[OPVOX_MAX_FRAME * 2];

	dec->plc_count++;

	/* After 3+ consecutive losses, fade to comfort noise */
	if (dec->plc_count > 3) {
		float comfort[OPVOX_MAX_FRAME * 2];
		generate_comfort_noise(comfort, dec->frame_samples, &dec->rng, dec->channels);

		/* Apply pitch post-filter to comfort noise for continuity */
		if (dec->channels >= 1) {
			float left_channel[OPVOX_MAX_FRAME];
			for (uint16_t i = 0; i < dec->frame_samples; i++) {
				left_channel[i] = comfort[i * dec->channels];
			}
			pitch_postfilter(&dec->pitch_filt, left_channel, dec->frame_samples, 0, 0.1f);
			for (uint16_t i = 0; i < dec->frame_samples; i++) {
				comfort[i * dec->channels] = left_channel[i];
			}
		}

		de_emphasis(comfort, pcm, dec->frame_samples, dec->de_emph, dec->channels, pre_emph_coeff(dec->sample_rate));
		return 0;
	}

	/* Decay previous frame coefficients */
	float decay = (float)pow(0.9, dec->plc_count);

	/* Scale previous MDCT coefficients */
	float decayed_mdct[OPVOX_MAX_FRAME];
	float decayed_mdct_r[OPVOX_MAX_FRAME];

	for (uint16_t i = 0; i < dec->frame_samples; i++) {
		decayed_mdct[i] = dec->prev_mdct[i] * decay;
		if (dec->channels == 2) {
			decayed_mdct_r[i] = dec->prev_mdct_r[i] * decay;
		}
	}

	/* Inverse MDCT - left channel */
	mdct_inverse(decayed_mdct, mdct_buf, dec->frame_samples, dec->window);

	/* Overlap-add */
	for (uint16_t i = 0; i < dec->frame_samples; i++) {
		pcm_float[i * dec->channels] = mdct_buf[i] + dec->overlap[i];
		dec->overlap[i] = mdct_buf[i + dec->frame_samples];
	}

	/* Process right channel if stereo */
	if (dec->channels == 2) {
		mdct_inverse(decayed_mdct_r, mdct_buf, dec->frame_samples, dec->window);

		for (uint16_t i = 0; i < dec->frame_samples; i++) {
			pcm_float[i * 2 + 1] = mdct_buf[i] + dec->overlap_r[i];
			dec->overlap_r[i] = mdct_buf[i + dec->frame_samples];
		}
	}

	/* Apply pitch post-filter for concealment */
	float left_channel[OPVOX_MAX_FRAME];
	for (uint16_t i = 0; i < dec->frame_samples; i++) {
		left_channel[i] = pcm_float[i * dec->channels];
	}
	pitch_postfilter(&dec->pitch_filt, left_channel, dec->frame_samples, 0, 0.2f);
	for (uint16_t i = 0; i < dec->frame_samples; i++) {
		pcm_float[i * dec->channels] = left_channel[i];
	}

	/* De-emphasis and convert to 16-bit */
	de_emphasis(pcm_float, pcm, dec->frame_samples, dec->de_emph, dec->channels, pre_emph_coeff(dec->sample_rate));

	return 0;
}