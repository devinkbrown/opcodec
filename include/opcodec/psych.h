/*
 * opcodec/psych.h — Psychoacoustic Masking Model
 *
 * Implements simultaneous masking (spreading function) to compute per-band
 * masking thresholds that adapt to the actual signal content.  This allows
 * the encoder to allocate fewer bits to perceptually masked bands (where
 * quantization noise is inaudible) and more bits to prominent tonal bands.
 *
 * Algorithm (ISO 11172-3 / Johnston 1988 psychoacoustic model):
 *
 *   1. Convert band energies to Bark scale (critical band rate).
 *   2. Compute per-band tonality index from spectral flatness measure (SFM).
 *      Tonal bands (sharp peaks) have higher masking power (~26 dB attenuation).
 *      Noise-like bands (flat spectrum) have lower masking power (~6 dB).
 *   3. Compute the spreading function SF(Δz): Johnston's asymmetric formula
 *      in the Bark domain, giving the shape of masking energy spread from
 *      one band into neighboring bands.
 *   4. Convolve masker power with spreading function across all bands.
 *   5. Masking threshold = max(ATH, peak spread contribution to that band).
 *   6. Signal-to-Mask Ratio (SMR) = signal power dB - masking threshold dB.
 *
 * The SMR replaces the static ATH in pvq_allocate_k_per_band, enabling the
 * PVQ bit allocation to respond to the actual perceptual salience of each band.
 * A band well above its masking threshold gets bits; a masked band gets none.
 *
 * Expected gains over ATH-only allocation:
 *   - 20–40% effective bitrate reduction at same perceived quality, or
 *   - ~3–6 dB PESQ/MUSHRA improvement at the same bitrate.
 *
 * Copyright (c) 2026 Ophion Development Team.  GPL v2.
 */

#ifndef OPCODEC_PSYCH_H
#define OPCODEC_PSYCH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Maximum number of bands this model handles.
 * Must match OPVOX_MAX_BANDS in audio.h (32 at 48kHz). */
#define PSYCH_MAX_BANDS  32

/* Tonality-dependent masking offset (dB attenuation from masker power):
 * Tonal masker (pure tone): ~26 dB below masker = 26 dB attenuation.
 * Noise-like masker:        ~6  dB below masker = 6  dB attenuation.
 * Linear interpolation between based on SFM. */
#define PSYCH_TONAL_OFFSET  26.0f   /* dB: for pure tonal content */
#define PSYCH_NOISE_OFFSET   6.0f   /* dB: for noise-like content */

/* Psychoacoustic context (pre-computed per encoder init) */
typedef struct {
    float bark[PSYCH_MAX_BANDS];      /* Bark frequency for each band center */
    float atk[PSYCH_MAX_BANDS];       /* Spreading function kernel (pre-computed) — symmetric part */
    int   n_bands;
    bool  initialized;
} psych_ctx_t;

/* Per-frame analysis output */
typedef struct {
    float smr_dB[PSYCH_MAX_BANDS];    /* Signal-to-Mask Ratio per band (dB) */
    float mask_dB[PSYCH_MAX_BANDS];   /* Masking threshold per band (dB) */
    float tonality[PSYCH_MAX_BANDS];  /* Tonality index [0=noise, 1=tonal] */
} psych_result_t;

/* ── API ── */

/*
 * Initialize psychoacoustic context from the band structure of the codec.
 * band_starts: array of n_bands band start indices (MDCT bin indices)
 * band_ends:   array of n_bands band end indices
 * sample_rate: audio sample rate (8000, 16000, 32000, 48000)
 */
void psych_init(psych_ctx_t *ctx,
                const uint16_t *band_starts, const uint16_t *band_ends,
                int n_bands, uint32_t sample_rate);

/*
 * Analyze one frame and compute per-band masking thresholds.
 *
 * band_energy_dB: signal energy per band in dB (from energy.c coarse_dB).
 * ath_dB:         absolute threshold in quiet per band (hearing_threshold array).
 * result:         output SMR, mask_dB, and tonality arrays.
 *
 * The result->mask_dB values can be passed as threshold_dB to
 * pvq_allocate_k_per_band() in audio.c, replacing the static ATH.
 */
void psych_analyze(const psych_ctx_t *ctx,
                   const float *band_energy_dB,
                   const float *ath_dB,
                   int n_bands,
                   psych_result_t *result);

#endif /* OPCODEC_PSYCH_H */
