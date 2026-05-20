# opcodec — Ophion Audio/Video Codec Library

opcodec is a custom real-time audio and video codec library written for the ophion IRC server.
It powers browser and native media calls transmitted over IRC using the LADON media relay protocol.

Everything in opcodec is pure C (C23), with no runtime platform dependencies beyond libc and
libm. It compiles to both native shared libraries and a single WASM module (`opcodec.js`) for
use in browser clients.

---

## What makes it different

Most media libraries (libopus, libvpx, FFmpeg) are general-purpose and optimized for the broadest
possible use case. opcodec is tuned specifically for IRC voice and video calls: low-latency,
low-bitrate, resilient to the lossy and bursty networks IRC clients run on, and small enough to
ship as a WASM payload.

Key differentiators:

| Property | Typical media stack | opcodec |
|---|---|---|
| Deployment | native binary, system lib | native + single WASM blob |
| Audio floor | Opus minimum: ~6 kbps | SAM mode: ~700 bps |
| Spatial audio | external library required | built-in OPFIELD with novel PRISM/HVSS/PERQ |
| Prosody at low bitrate | lost | EPSC side channel preserves it at 200 bps |
| Acoustic environment | not transmitted | AEC2 transmits compact room impulse |
| Video fallback | freeze or blank | LVC VQ mode — recognizable at <2 kbps |
| Saliency-driven QP | heuristic or off | SARDO: per-CTU allocation from face/motion map |
| Frame encryption | external or none | per-frame ChaCha20-Poly1305 with key ratcheting |

---

## Audio codec: OPVOX

**OPVOX** is the primary audio codec. It is a sub-band MDCT codec covering the full range from
narrowband voice to fullband stereo music.

### Supported configurations

| Sample rate | Channels | Use case | Bitrate range |
|---|---|---|---|
| 8 000 Hz | mono | narrowband voice | 6 – 16 kbps |
| 16 000 Hz | mono | wideband voice | 12 – 32 kbps |
| 32 000 Hz | mono | super-wideband | 24 – 64 kbps |
| 48 000 Hz | mono | fullband music | 32 – 128 kbps |
| 48 000 Hz | stereo | fullband stereo | 64 – 512 kbps |

Frame duration is 20 ms at all rates.

### Encoding pipeline

1. Pre-emphasis + windowing
2. MDCT (Modified Discrete Cosine Transform)
3. Psychoacoustic bit allocation (via `psych` module)
4. Scalar quantization of MDCT coefficients
5. Rice/Golomb entropy coding

### Bitstream header (2 bytes)

```
byte 0:
  bit 7:    stereo (1) / mono (0)
  bit 6-5:  sample rate  00=8k  01=16k  10=48k  11=32k
  bit 4-3:  quality  00=low  01=normal  10=high  11=ultra
  bit 2:    silence/comfort-noise frame
  bit 1:    short blocks (transients)
  bit 0:    joint stereo

byte 1: encoded frame length in bytes
```

---

## Audio modules

### psych — Psychoacoustic masking model

Computes simultaneous masking thresholds per frequency band using a spreading function.
The encoder skips bits in bands where quantization noise is masked by the signal.
This is the core quality-at-bitrate mechanism of OPVOX.

### bwe — Bandwidth Extension + Spectral Noise Shaping

BWE regenerates high-frequency bands at the decoder from transmitted low-band coefficients via
spectral folding and energy scaling. SNS shapes quantization noise to follow the spectral
envelope so it stays below the masking threshold. Both operate in the MDCT domain.

### tns — Temporal Noise Shaping

Pre-whitens spectrally non-flat signals (e.g. music) with an all-pole LPC filter before
quantization, then inverts at the decoder. Reduces pre-echo and spectral smearing artifacts.

### pns — Perceptual Noise Substitution

For noisy or unvoiced bands the encoder transmits only an energy level; the decoder synthesizes
spectrally matched noise. Saves bits on bands where exact coefficients are perceptually irrelevant.

### pvq — Pyramid Vector Quantization

Allocates discrete bits per MDCT band and quantizes coefficients jointly on a unit sphere.
More efficient than scalar quantization at very low per-band bit counts.

### pitch — Pitch estimation

Autocorrelation-based fundamental frequency (F0) tracker. Drives voiced/unvoiced decisions,
EPSC pitch coding, and SAM excitation synthesis.

### ns2 — Noise Suppression v2

Wiener filter with Ephraim–Malah MMSE-STSA noise estimation. Removes stationary and
quasi-stationary background noise without the musical-noise artefacts of naive spectral
subtraction. Applied pre-encode to clean the microphone signal.

### dtx — Discontinuous Transmission + Comfort Noise

Detects silence and suppresses packet transmission entirely during quiet periods. The receiver
generates perceptually matched comfort noise (CNG) from a transmitted SID (silence insert
descriptor) so callers do not hear unnatural dead air.

### aec2 — Acoustic Environment Codec v2

Encodes and transmits a compact Room Impulse Response (RIR) descriptor. The receiver applies
it as convolution reverb on synthesized audio, giving the listener a perceptual sense of the
remote room — office, hallway, bathroom, outdoors, and so on.

This is distinct from echo cancellation. AEC2 is a *presence* feature: it makes remote speech
feel spatially grounded rather than flat.

### epsc — Emotional Prosody Side Channel

At 6–24 kbps, conventional codecs destroy pitch variation, energy dynamics, and speaking rate —
the cues that convey urgency, humor, and emotion. EPSC transmits these as a 200-bps side channel.

Per 20 ms frame (2 bytes):

| Field | Bits | Detail |
|---|---|---|
| F0 (pitch) | 7 | MIDI semitone index 0–127; 0 = unvoiced |
| Energy | 5 | log-energy in 3 dB steps over 96 dB |
| Voiced flag | 1 | voiced / unvoiced |
| Speaking rate | 3 | syllables/sec, quantized |

The decoder resynthesizes the prosodic envelope on top of the decoded audio.

### sam — Semantic Audio Mode

Ultra-low-bitrate parametric vocoder for conditions where even basic Opus is too expensive.
Encodes speech as LPC parameters + pitch + energy and resynthesizes at the decoder.

**Target: ~700 bps** (Opus minimum is ~6 000 bps — approximately 9× more efficient).

Per 40 ms frame (3.5 bytes):

| Field | Bits |
|---|---|
| LSF coefficients (10-order LPC, VQ codebook index) | 12 |
| Pitch period | 7 |
| Voiced/unvoiced | 1 |
| Log-energy | 6 |
| Frame-type flags | 2 |

Synthesis: voiced frames use a pulse-train excitation at the pitch period; unvoiced use shaped
noise. Both are filtered through the LPC synthesis filter and passed through a post-filter for
naturalness.

### separator — Real-time speaker source separation

Separates a mono mixture of up to four simultaneous voices into individual speaker streams using
STFT-domain magnitude-ratio masking (lightweight Conv-TasNet-inspired). Each speaker must be
enrolled from ≥ 1 s of clean audio. Optimized for speech (100–8 000 Hz); degrades above three
simultaneous speakers.

### opfield — Spatial audio (OPFIELD)

Object-based spatial audio supporting binaural HRTF rendering, Higher-Order Ambisonics (HOA)
up to 3rd order, and VBAP for loudspeaker arrays.

**Novel algorithms:**

**OPFIELD-PRISM** (Phase-Resolved Interaural Spectral Morphing) — Splits the signal at 700 Hz
and applies Duplex-theory-aware weighting:
- Low band (<700 Hz): full ITD (time delay), minimal ILD (level difference) — phase dominates
- High band (>700 Hz): partial ITD, full ILD — amplitude dominates

This produces more accurate localization than single-band HRTF approximations.

**OPFIELD-HVSS** (HRTF Velocity Smooth Scan) — Smoothly lerps ILD filter coefficients toward
new target values on every update, eliminating the comb-filtering artefacts that occur when a
sound source moves. Smoothing speed adapts to object velocity: fast objects get responsive
tracking; slow objects get stable, artifact-free rendering.

**OPFIELD-PERQ** (Perceptual JND-Zone Quantization) — Quantizes azimuth/elevation metadata at
coarser resolution in zones where the ear cannot distinguish small angular differences (based on
Mills 1958 and Blauert 1997 data):
- Front: 1° per zone
- Side: 3° per zone
- Rear: 5° per zone

Saves ~33% metadata bandwidth versus fixed fine-resolution quantization.

Mathematical foundations are clean-room from published theory: Woodworth-Schlosberg sphere model
(ITD), Brown-Duda first-order shelving (ILD), Raykar et al. (2004) N1 notch measurements (pinna).

---

## Video codec: OPVIS

**OPVIS** is the primary video codec. It is an integer wavelet codec with full-frame motion
compensation, designed for real-time IRC video calls.

### Supported resolutions

Up to 1920×1080 (full HD). Coding tree unit (CTU) size: 64×64 pixels. Macroblock size: 16×16.

### Frame types

| Type | Description |
|---|---|
| I | Intra-coded — no reference needed |
| P | Inter-coded — one reference frame |
| B | Low-delay B — both references are past frames |
| SKIP | Repeat previous frame (no data transmitted) |
| INTERP | Synthesized by TFI at the decoder — not transmitted |

### Bitstream header (v1 — 18 bytes)

```
byte 0:    version = 1
byte 1:    frame type (I/P/B/SKIP)
byte 2:    quality 0-100
bytes 3-4: width (BE)
bytes 5-6: height (BE)
bytes 7-10: frame_num (BE u32)
byte 11:   color info flags
             bit 7: HDR metadata present
             bit 6: 10-bit
             bits 5-4: transfer (SDR / PQ / HLG / LINEAR)
             bits 3-2: primaries (BT.709 / BT.2020 / P3-D65 / sRGB)
             bits 1-0: subsampling (4:2:0 / 4:2:2 / 4:4:4)
byte 12:   flags (ALF present, screen mode, B-ref distance)
bytes 13-16: payload length (BE u32)
[+6 bytes if HDR]: max_lum, min_lum, knee, knee_gain
```

A legacy v0 header (14 bytes, byte 0 = 0) is also decoded for backward compatibility.

---

## Video modules

### saliency — Perceptual saliency estimation

Estimates visual importance per CTU (64×64 block) using lightweight heuristics: skin-tone luma
range, edge density, and motion vector magnitude. No ML inference needed; runs in < 0.5 ms/frame.

Drives **SARDO** (Saliency-Aware Rate-Distortion Optimization): the encoder allocates more bits
to perceptually important regions — faces, motion, text — and fewer bits to backgrounds that the
visual system has already "memorized" from previous frames.

### tfi — Temporal Frame Interpolation

Synthesizes intermediate INTERP frames from two decoded reference frames using motion-compensated
blending. The encoder can mark every other frame as INTERP and skip transmitting it entirely,
saving approximately 50% of video bitrate. The decoder reconstructs the missing frame locally.

### gbs — Generative Background Synthesis

Separates each frame into foreground (speaker / subject) and background, transmits only the
foreground at full quality, and sends a compact background descriptor. The receiver synthesizes
the background locally. Effective for static or slowly changing backgrounds in call scenarios.

### hdr — HDR / color science utilities

SMPTE ST 2084 PQ EOTF/OETF (Dolby Vision / HDR10), BT.2100 HLG OETF (broadcast HDR), and
BT.2020 ↔ BT.709 color matrix conversions. The OPVIS v1 header carries HDR metadata when
present. SDR content is unaffected.

### screen — Screen content coding

Palette mode (blocks with ≤ 16 distinct luma values encoded as index maps) and Intra Block Copy
(copying previously decoded pixels from the current frame). Significantly improves coding
efficiency for screen sharing, terminal output, and UI-heavy content.

### svf — Scene Video Fingerprinting

Detects hard cuts and gradual scene transitions in real time to trigger adaptive forced I-frames.
Without this, inter-coded frames following a scene change produce severe visual corruption as
motion vectors reference the wrong content.

### lvc — Latent Video Codec mode

Extreme-compression fallback for sustained low-bandwidth conditions (< 64 kbps). Replaces
full-frame coding with VQ (vector quantization) against a 256-entry offline-trained codebook:

1. Divide luma plane into 16×16 macroblocks
2. Find best-match code vector in the codebook
3. Transmit 8-bit codebook index per macroblock (plus optional 4-bit gain scalar)
4. Reconstruct at the decoder via table lookup

A 640×480 frame becomes ~1 800 bytes. At 15 fps with I/P mix: ~43 kbps luma. Quality is visibly
blocky but recognizable — comparable to circa-2000 webcam video at 28.8k modem speeds. Acceptable
as emergency fallback or IoT deployment.

### cdef — Directional enhancement filter

AV1-style CDEF (Constrained Directional Enhancement Filter). Applied post-decode to reduce ringing
artefacts along edges by filtering each 8×8 block along its dominant edge direction.

### tnr — Temporal Noise Reduction

Motion-adaptive recursive temporal filter applied to the luma plane before encoding. Suppresses
camera sensor noise in static regions while preserving motion edges (no ghosting).

### vplc — Video Packet Loss Concealment

When packets are lost, VPLC synthesizes missing macroblocks from neighboring decoded data rather
than displaying frozen blocks or hard cuts. Complements FEC for last-mile packet loss conditions.

### vidutil — Video utility functions

Shared pixel-format conversion, YUV plane manipulation, subsampling helpers, and SIMD-friendly
inner loop primitives used by both encoder and decoder paths.

---

## Session and transport layer

### avsession — Audio/Video session

Integrates OPVOX and OPVIS with jitter buffering and A/V synchronization. Provides a unified
packet wire format suitable for IRC DCC or UDP transport:

```
byte 0:    type (AUDIO=0, VIDEO=1, KEYFRAME=2, SKIP=3)
bytes 1-4: sequence number (BE u32)
bytes 5-8: timestamp (BE u32)  — audio: sample count; video: frame_num × video_ts_scale
bytes 9-10: payload length (BE u16)
bytes 11+: payload
```

Lip-sync offset = `audio_ts − video_ts × video_ts_scale`. If the offset exceeds the configured
threshold, the decoder holds video or skips audio to stay aligned.

### secure — Per-frame AEAD encryption

Per-frame ChaCha20-Poly1305 encryption using an SFrame/DAVE-inspired header format:

```
[version:1][frame_type:1][sender_id:4][counter:4][epoch:2][codec_id:1][flags:1]
[encrypted_payload:N][auth_tag:16]
```

Features:
- Per-sender key derivation for group calls
- Generation-based key ratcheting with forward secrecy
- 512-packet anti-replay window with grace-period key retention for out-of-order frames
- AAD covers entire cleartext header

### fec — Forward Error Correction

XOR-based FEC with 1D and 2D protection schemes. Groups up to 8 packets; a single XOR parity
packet per group allows recovery of any one lost packet without retransmission. 2D FEC extends
this to recover any single row or column loss in a matrix arrangement.

### jitter — Adaptive jitter buffer

Handles packet reordering, loss detection, and adaptive playout delay. Tracks NACK entries for
selective retransmission. Shared by audio and video decoder paths.

### netadapt — Network-adaptive bitrate controller

Observes RTT, packet loss rate, and bandwidth estimate to dynamically adjust codec parameters.
Uses EWMA smoothing and Kalman-style uncertainty tracking. State machine:

```
STABLE → PROBING (+10% target for 3 s)
PROBING → DRAINING if loss increases (−30% multiplicative)
DRAINING → RECOVERY (+5% additive per interval until stable)
```

Distributes target bitrate across audio, video, and FEC allocations.

---

## WASM deployment

The full stack (OPVOX audio encode/decode, OPVIS video encode/decode, NS2, DTX) compiles to a
single `opcodec.js` Emscripten module (`createOpcodec()`) with `ALLOW_MEMORY_GROWTH` and a 64 MB
initial heap. All exported functions are prefixed with `_opvox_wasm_*`, `_opvis_wasm_*`, etc.

The WASM build omits modules with platform dependencies (opssl, opfield HOA convolution) and
uses separate JavaScript shims for operations that are native in browser (WebCrypto for AEAD).

---

## Integration with ophion

opcodec is used by:

- **`cap_ladon_media`** — IRCv3 `ophion/ladon-media` capability. Negotiates media sessions and
  relays MEDIAFRAME/MCHUNK packets between IRC clients and browser WebSocket connections.
- **`m_ladon_client`** — MEDIAFRAME/MCHUNK chunked transport. Splits video frames into
  160-character base64 segments to fit within the IRC 512-byte line limit.

See `doc/technical/ladon-media-client.md` for the full wire protocol.

---

## Build

opcodec is a Meson subproject under `subprojects/opcodec/`. It is built automatically as part of
the ophion build. To build it standalone:

```
meson setup builddir
ninja -C builddir
ninja -C builddir test
```

Requires: C23 compiler, libm, libop (subproject), opssl (subproject).

Optional: Emscripten (`emcc`) for the WASM target.

License: GPL v2.
