project('opcodec', 'c',
  version: '0.1.0',
  license: 'GPL-2.0-or-later',
  default_options: [
    'c_std=c23',
    'warning_level=2',
    'b_pie=true',
    'b_lto=true',
  ],
  meson_version: '>= 1.1.0',
)

cc = meson.get_compiler('c')

# Math library — required for sin/cos/sqrt in audio codec
math_dep = cc.find_library('m', required: true)

# libop — required transitively by opssl headers (op_lib.h)
libop_dep = dependency('libop', fallback: ['libop', 'libop_dep'])

# opssl — required for AEAD encryption and HKDF in secure codec layer
opssl_dep = dependency('opssl', fallback: ['opssl', 'opssl_dep'])

# Additional warnings beyond warning_level=2
_warn_flags = cc.get_supported_arguments(
  '-Wpedantic',
)
add_project_arguments(_warn_flags, language: 'c')

# Include directories
opcodec_inc = include_directories('include')

# Source files
opcodec_src = files(
  'src/audio.c',
  'src/bwe.c',
  'src/energy.c',
  'src/fec.c',
  'src/intra.c',
  'src/jitter.c',
  'src/netadapt.c',
  'src/pitch.c',
  'src/plc.c',
  'src/pns.c',
  'src/pvq.c',
  'src/rans.c',
  'src/secure.c',
  'src/stereo.c',
  'src/tns.c',
  'src/hdr.c',
  'src/screen.c',
  'src/tnr.c',
  'src/cdef.c',
  'src/video.c',
  'src/vidutil.c',
  'src/vmd.c',
  'src/opfield.c',
  'src/opfield_hrtf.c',
  'src/opfield_hoa.c',
  'src/avsession.c',
  'src/saliency.c',
  'src/tfi.c',
  'src/epsc.c',
  'src/separator.c',
  'src/sam.c',
  'src/aec2.c',
  'src/lvc.c',
  'src/gbs.c',
  'src/ns2.c',
  'src/vplc.c',
  'src/svf.c',
  'src/dtx.c',
  'src/psych.c',
)

# Static library — opcodec is consumed by ophion ircd core
libopcodec = static_library('opcodec',
  opcodec_src,
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  install: false,
)

# Dependency for consumers (parent build tree)
opcodec_dep = declare_dependency(
  include_directories: opcodec_inc,
  link_with: libopcodec,
)

# Test executables
vidutil_test = executable('vidutil_test',
  'src/vidutil_test.c',
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  link_with: libopcodec,
  install: false,
)

energy_test = executable('energy_test',
  'src/energy_test.c',
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  link_with: libopcodec,
  install: false,
)

vidutil_benchmark = executable('vidutil_benchmark',
  'src/vidutil_benchmark.c',
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  link_with: libopcodec,
  install: false,
)

plc_test = executable('plc_test',
  'src/plc_test.c',
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  link_with: libopcodec,
  install: false,
)

rans_test = executable('rans_test',
  'src/rans_test.c',
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  link_with: libopcodec,
  install: false,
)

pns_rvq_test = executable('pns_rvq_test',
  'src/pns_rvq_test.c',
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  link_with: libopcodec,
  install: false,
)

audio_test = executable('audio_test',
  'src/audio_test.c',
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  link_with: libopcodec,
  install: false,
)

codec_suite_test = executable('codec_suite_test',
  'src/codec_suite_test.c',
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  link_with: libopcodec,
  install: false,
)

opfield_test = executable('opfield_test',
  'src/opfield_test.c',
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  link_with: libopcodec,
  install: false,
)

video_test = executable('video_test',
  'src/video_test.c',
  include_directories: opcodec_inc,
  dependencies: [math_dep, libop_dep, opssl_dep],
  link_with: libopcodec,
  install: false,
)

# ── WASM browser client ──────────────────────────────────────────────────
#
# Build opcodec_wasm.js + opcodec_wasm.wasm for browser video clients.
#
# Prerequisites:
#   - emcc ≥ 3.1 in PATH
#   - Configure with Emscripten cross-file:
#       meson setup buildwasm \
#         --cross-file subprojects/opcodec/cross/emscripten.ini \
#         -Dwasm_client=true \
#         subprojects/opcodec
#
# The WASM build links only the decode-side sources (no opssl, no libop)
# and exports the wasm_api.c surface via Emscripten cwrap/ccall.

option_wasm = get_option('wasm_client')

if option_wasm
  # WASM encode+decode sources (no opssl/libop dependency)
  wasm_src = files(
    'src/audio.c',
    'src/energy.c',
    'src/fec.c',
    'src/intra.c',
    'src/jitter.c',
    'src/pitch.c',
    'src/plc.c',
    'src/pns.c',
    'src/pvq.c',
    'src/rans.c',
    'src/tns.c',
    'src/hdr.c',
    'src/video.c',
    'src/vidutil.c',
    'src/vmd.c',
    'src/saliency.c',
    'src/tfi.c',
    'src/ns2.c',
    'src/dtx.c',
    'src/vplc.c',
    'src/svf.c',
    'src/wasm_api.c',
  )

  wasm_exported_fns = [
    # Heap helpers
    '_opcodec_alloc_f32',
    '_opcodec_alloc_i16',
    '_opcodec_alloc_u8',
    '_opcodec_free',
    # NS2 noise suppression
    '_ns2_wasm_create',
    '_ns2_wasm_process',
    '_ns2_wasm_get_noise_db',
    '_ns2_wasm_destroy',
    # DTX comfort noise decoder
    '_dtx_wasm_dec_create',
    '_dtx_wasm_set_sid',
    '_dtx_wasm_voice_onset',
    '_dtx_wasm_generate',
    '_dtx_wasm_destroy',
    # Opvox audio encoder
    '_opvox_wasm_enc_create',
    '_opvox_wasm_encode',
    '_opvox_wasm_enc_destroy',
    # Opvox audio decoder
    '_opvox_wasm_dec_create',
    '_opvox_wasm_decode',
    '_opvox_wasm_dec_destroy',
    # Opvis video encoder
    '_opvis_wasm_enc_create',
    '_opvis_wasm_enc_encode',
    '_opvis_wasm_enc_flush',
    '_opvis_wasm_enc_set_quality',
    '_opvis_wasm_enc_destroy',
    # Opvis video decoder
    '_opvis_wasm_dec_create',
    '_opvis_wasm_dec_decode',
    '_opvis_wasm_dec_destroy',
    # Raw malloc/free for caller convenience
    '_malloc',
    '_free',
  ]

  exported_fns_str = ','.join(wasm_exported_fns)

  executable('opcodec_wasm',
    wasm_src,
    include_directories: opcodec_inc,
    dependencies: [math_dep],
    c_args: ['-DOPCODEC_WASM=1'],
    link_args: [
      '-sEXPORTED_FUNCTIONS=[' + exported_fns_str + ']',
      '-sEXPORTED_RUNTIME_METHODS=["cwrap","getValue","setValue","HEAPU8","HEAP16","HEAPF32"]',
      '-sALLOW_MEMORY_GROWTH=1',
      '-sMODULARIZE=1',
      '-sEXPORT_NAME=createOpcodec',
      '-sINITIAL_MEMORY=67108864',
      '--no-entry',
      '-O2',
    ],
    name_suffix: 'js',
    install: false,
  )
endif

# Meson subproject support
meson.override_dependency('opcodec', opcodec_dep)
