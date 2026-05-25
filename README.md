# opcodec

Real-time audio and video codec library for the [ophion](https://github.com/devinkbrown/ophion) IRC server.
Pure C23, no runtime dependencies beyond libc and libm.
Compiles to native shared libraries and a single WASM module for browser clients.

---

## What it is

opcodec powers voice and video calls transmitted over IRC using the [LADON media relay protocol](../doc/technical/ladon-media-wire.md).
It replaces general-purpose libraries (libopus, libvpx) with a stack tuned specifically for IRC:
low-latency, low-bitrate, resilient to lossy and bursty connections, and small enough to ship as a WASM payload.

| Property | Typical media stack | opcodec |
|---|---|---|
| Deployment | native binary, system lib | native + single WASM blob |
| Audio floor | Opus minimum: ~6 kbps | SAM mode: ~700 bps |
| Spatial audio | external library required | built-in OPFIELD (PRISM / HVSS / PERQ) |
| Prosody at low bitrate | lost | EPSC side channel preserves it at 200 bps |
| Acoustic environment | not transmitted | AEC2 compact room impulse |
| Video fallback | freeze or blank | LVC VQ mode — recognizable at < 2 kbps |
| Saliency-driven QP | heuristic or off | SARDO: per-CTU allocation from face/motion map |
| Frame encryption | external or none | per-frame ChaCha20-Poly1305 with key ratcheting |

---

## Codecs

### Audio — OPVOX

Sub-band MDCT codec. 8–48 kHz, mono and stereo, 700 bps (SAM) to 512 kbps.

| Module | Role |
|---|---|
| **psych** | Psychoacoustic masking — drives bit allocation |
| **bwe** | Bandwidth extension + spectral noise shaping |
| **tns** | Temporal noise shaping (pre-echo / smearing reduction) |
| **pns** | Perceptual noise substitution (noisy/unvoiced bands) |
| **pvq** | Pyramid vector quantization (low per-band bit counts) |
| **pitch** | F0 tracker — drives EPSC and SAM |
| **ns2** | Noise suppression (Wiener / Ephraim–Malah MMSE-STSA) |
| **dtx** | Discontinuous transmission + comfort noise |
| **aec2** | Acoustic environment codec (room impulse transmission) |
| **epsc** | Emotional prosody side channel — 200 bps F0/energy/rate |
| **sam** | Semantic audio mode (LPC vocoder, ~700 bps) |
| **separator** | Real-time speaker source separation (up to 4 voices) |
| **opfield** | Spatial audio: binaural HRTF, HOA 3rd-order, VBAP |

### Video — OPVIS

Integer wavelet codec with full-frame motion compensation. Up to 1920×1080.

| Module | Role |
|---|---|
| **saliency** | Per-CTU visual importance — drives SARDO bit allocation |
| **tfi** | Temporal frame interpolation (synthesize INTERP frames) |
| **gbs** | Generative background synthesis |
| **hdr** | PQ / HLG EOTF, BT.2020 ↔ BT.709 color matrix |
| **screen** | Palette mode + Intra Block Copy for screen content |
| **svf** | Scene video fingerprinting (forced I-frame on cut) |
| **lvc** | Latent video codec — VQ fallback at < 2 kbps |
| **cdef** | AV1-style directional enhancement filter |
| **tnr** | Temporal noise reduction |
| **vplc** | Video packet loss concealment |
| **vidutil** | YUV utilities and SIMD-friendly inner loops |

### Session and transport

| Module | Role |
|---|---|
| **avsession** | A/V session: jitter buffer, lip-sync, unified packet format |
| **secure** | Per-frame ChaCha20-Poly1305 AEAD with key ratcheting |
| **fec** | XOR-based 1D/2D forward error correction |
| **jitter** | Adaptive jitter buffer with NACK tracking |
| **netadapt** | Bandwidth-adaptive bitrate controller (RTT + loss) |

---

## Building

### As part of ophion (normal)

opcodec is a Meson subproject. It is built automatically when you build ophion:

```bash
meson setup build
ninja -C build
```

### Standalone

```bash
cd subprojects/opcodec
meson setup builddir
ninja -C builddir
ninja -C builddir test
```

Requirements: C23 compiler (GCC 13+ or Clang 17+), libm, libop (subproject), opssl (subproject).

### WASM (browser client)

Requires Emscripten ≥ 3.1 in `PATH`:

```bash
meson setup buildwasm \
  --cross-file cross/emscripten.ini \
  -Dwasm_client=true \
  subprojects/opcodec
ninja -C buildwasm
```

Outputs `opcodec_wasm.js` + `opcodec_wasm.wasm`. Load with `createOpcodec()`.

The WASM build omits modules with native dependencies (opssl AEAD uses WebCrypto instead, opfield HOA convolution is excluded).

---

## API quick-start

### Audio encode / decode (native)

```c
#include "opcodec/audio.h"

/* Encoder */
opvox_encoder_t *enc = opvox_encoder_create(16000, 1, 24000);
uint8_t frame[256];
int n = opvox_encode(enc, pcm_samples, frame, sizeof(frame));

/* Decoder */
opvox_decoder_t *dec = opvox_decoder_create(16000, 1);
int16_t out[320];
opvox_decode(dec, frame, n, out);
```

### Video encode / decode (native)

```c
#include "opcodec/video.h"

opvis_encoder_t *enc = opvis_encoder_create(1280, 720, 80);
uint8_t vframe[65536];
int n = opvis_encode(enc, yuv_plane_y, yuv_plane_u, yuv_plane_v, vframe, sizeof(vframe));

opvis_decoder_t *dec = opvis_decoder_create();
opvis_decode(dec, vframe, n, &out_y, &out_u, &out_v, &width, &height);
```

### WASM (browser)

```js
const codec = await createOpcodec();
const enc = codec._opvox_wasm_enc_create(16000, 1, 24000);
const pcmPtr = codec._opcodec_alloc_i16(320);
// ... fill HEAP16 at pcmPtr with 320 samples ...
const framePtr = codec._opcodec_alloc_u8(256);
const n = codec._opvox_wasm_encode(enc, pcmPtr, framePtr, 256);
```

---

## Integration with ophion

opcodec is used by:

- **`cap_ladon_media`** — `ophion/ladon-media` IRCv3 capability; negotiates sessions and relays MEDIAFRAME/MCHUNK packets.
- **`m_ladon_client`** — chunked MEDIAFRAME transport (splits video frames into 160-char base64 segments for the 512-byte IRC line limit).

The LADON media wire protocol is documented in [`doc/technical/ladon-media-wire.md`](../../doc/technical/ladon-media-wire.md).

---

## Documentation

| Document | Contents |
|---|---|
| [`OPCODEC.md`](OPCODEC.md) | Full technical reference: every module, all bitstream formats, all algorithms |
| [`BWE_INTEGRATION.md`](BWE_INTEGRATION.md) | How to integrate BWE/SNS into the OPVOX encode/decode pipeline |
| [`VIDUTIL_INTEGRATION.md`](VIDUTIL_INTEGRATION.md) | vidutil API reference and pixel-format helpers |
| [`doc/technical/ladon-media-wire.md`](../../doc/technical/ladon-media-wire.md) | LADON media frame wire format |
| [`doc/technical/ladon-media-client.md`](../../doc/technical/ladon-media-client.md) | IRC client integration and CAP negotiation |
| [`doc/technical/opcodec.md`](../../doc/technical/opcodec.md) | ophion-side integration notes |

---

## License

GPL-2.0-or-later. See the [ophion LICENSE](../../LICENSE) file.
