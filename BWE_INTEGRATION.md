# BWE/SNS Integration with OPVOX Audio Codec

This document explains how to integrate the BWE (Bandwidth Extension) and SNS (Spectral Noise Shaping) modules with the existing OPVOX audio codec.

## Overview

The BWE and SNS modules work in the MDCT domain to improve perceptual quality at low bitrates:

- **SNS (Spectral Noise Shaping)**: Shapes quantization noise to follow the spectral envelope
- **BWE (Bandwidth Extension)**: Saves bits by only coding low frequencies and regenerating high frequencies

## Integration Points

### 1. Encoder Pipeline

```c
// Current OPVOX encoder flow:
// PCM → pre-emphasis → MDCT → bit allocation → quantization → entropy coding

// New flow with BWE/SNS:
// PCM → pre-emphasis → MDCT → 
//   ↓ (if BWE enabled)
//   BWE analysis (determine cutoff, extract high-band energies) →
//   ↓ (if SNS enabled) 
//   SNS analysis (compute scale factors) → SNS flatten →
//   ↓
//   bit allocation (only for coded bands) → PVQ quantization →
//   ↓
//   entropy coding (coefficients + scale factors + high-band energies)
```

### 2. Decoder Pipeline

```c
// Current OPVOX decoder flow:
// entropy decode → dequantization → MDCT synthesis → de-emphasis → PCM

// New flow with BWE/SNS:
// entropy decode (coefficients + scale factors + high-band energies) →
//   ↓
//   PVQ dequantization →
//   ↓ (if SNS enabled)
//   SNS restore (multiply by scale factors) →
//   ↓ (if BWE enabled)
//   BWE synthesis (fold low bands into high bands) →
//   ↓
//   MDCT synthesis → de-emphasis → PCM
```

### 3. Band Boundary Conversion

Convert OPVOX band_boundaries to BWE band_range_t format:

```c
void convert_bands(const uint16_t *boundaries, int num_bands, band_range_t *bands)
{
    for (int i = 0; i < num_bands; i++) {
        bands[i].start = boundaries[i];
        bands[i].end = boundaries[i + 1];
    }
}
```

### 4. Frame Structure Changes

The frame header needs to encode:
- BWE enabled flag and cutoff band
- SNS enabled flag
- Additional bitstream sections for scale factors and high-band energies

Suggested header extension:
```
Header byte 2 (new):
  bit 7:    BWE enabled
  bit 6-4:  BWE cutoff band index (0-7, maps to actual band via table)
  bit 3:    SNS enabled
  bit 2-0:  reserved
```

### 5. Bitrate Allocation

With BWE, fewer bands need explicit coding:

```c
uint16_t get_coded_bands(opvox_encoder_t *enc, int target_bits)
{
    if (target_bits >= 160) return enc->n_bands;  // Code all bands
    
    uint16_t cutoff = bwe_optimal_cutoff(target_bits, enc->n_bands, enc->sample_rate);
    return cutoff;
}
```

## Code Integration Example

```c
// In opvox_encoder_t, add:
typedef struct opvox_encoder {
    // ... existing fields ...
    
    bwe_ctx_t bwe_ctx;
    sns_ctx_t sns_ctx;
    band_range_t bands[OPVOX_MAX_BANDS];
    uint16_t coded_bands;  // BWE cutoff
    bool bwe_enabled;
    bool sns_enabled;
} opvox_encoder_t;

// In encoder initialization:
int opvox_encoder_init(opvox_encoder_t *enc, ...)
{
    // ... existing init ...
    
    // Convert band boundaries to band_range_t format
    const uint16_t *boundaries = get_band_bounds(sample_rate, &enc->n_bands, &threshold);
    convert_bands(boundaries, enc->n_bands, enc->bands);
    
    // Enable BWE/SNS for low bitrates
    enc->bwe_enabled = (enc->target_bits < 256);
    enc->sns_enabled = (enc->target_bits < 512);
    
    if (enc->bwe_enabled) {
        enc->coded_bands = bwe_optimal_cutoff(enc->target_bits, enc->n_bands, sample_rate);
        bwe_init(&enc->bwe_ctx, enc->coded_bands, enc->n_bands);
    } else {
        enc->coded_bands = enc->n_bands;
    }
    
    if (enc->sns_enabled) {
        sns_init(&enc->sns_ctx);
    }
}

// In the encoder main loop:
int opvox_encode(opvox_encoder_t *enc, const int16_t *pcm, uint8_t *out, size_t out_cap)
{
    // ... PCM preprocessing, MDCT ...
    
    // BWE: extract high-band energies before they're lost
    uint8_t bwe_data[32];
    int bwe_bytes = 0;
    if (enc->bwe_enabled) {
        bwe_bytes = bwe_encode(&enc->bwe_ctx, mdct, mdct_size/2, 
                               enc->bands, enc->n_bands, bwe_data, sizeof(bwe_data));
    }
    
    // SNS: flatten spectrum before quantization
    uint8_t sns_data[64];
    int sns_bytes = 0;
    if (enc->sns_enabled) {
        sns_analyze(&enc->sns_ctx, mdct, mdct_size/2, 
                    enc->bands, enc->coded_bands);
        sns_bytes = sns_encode_scales(&enc->sns_ctx, sns_data, sizeof(sns_data));
        sns_flatten(&enc->sns_ctx, mdct, mdct_size/2, 
                    enc->bands, enc->coded_bands);
    }
    
    // Bit allocation and quantization (only for coded bands)
    allocate_bits(mdct, enc->coded_bands, remaining_bits, ...);
    
    // ... PVQ quantization, entropy coding ...
    
    // Pack frame: header + SNS data + BWE data + coefficients
}
```

## Performance Impact

- **Memory**: +~200 bytes per encoder/decoder instance
- **CPU**: +~10% for SNS, +~5% for BWE  
- **Bitrate savings**: 20-40% at low bitrates (< 32 kbps)
- **Quality**: Improved perceptual quality especially for music content

## Testing

Use `test_bwe` to verify BWE/SNS functionality:

```bash
cd subprojects/opcodec
./test_bwe
```

The test validates:
- SNS scale factor accuracy and round-trip behavior
- BWE spectral folding and energy matching
- Encode/decode bitstream compatibility

## Configuration Recommendations

- Enable BWE for bitrates < 32 kbps per channel
- Enable SNS for bitrates < 64 kbps per channel  
- For voice-only (8/16 kHz): BWE cutoff should preserve at least 50% of bands
- For music (48 kHz): BWE more aggressive, can code as few as 33% of bands