#!/usr/bin/env bash
# Build opcodec as an Emscripten WASM module and copy output to nexus/public/.
# Requires: emcc >= 3.1 in PATH, meson, ninja.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build-wasm"
NEXUS_PUBLIC="/home/kain/nexus/public"

if ! command -v emcc >/dev/null 2>&1; then
  echo "ERROR: emcc not found. Install Emscripten and activate it with 'source emsdk_env.sh'." >&2
  echo "Skipping WASM build — TypeScript stubs remain functional without WASM." >&2
  exit 1
fi

echo "==> Setting up Meson WASM build in ${BUILD_DIR}"
meson setup "${BUILD_DIR}" "${SCRIPT_DIR}" \
  --cross-file "${SCRIPT_DIR}/cross/emscripten.ini" \
  -Dwasm_client=true \
  --wipe

echo "==> Building"
ninja -C "${BUILD_DIR}"

echo "==> Copying output to nexus/public/"
mkdir -p "${NEXUS_PUBLIC}"
cp "${BUILD_DIR}/opcodec_wasm.js"   "${NEXUS_PUBLIC}/opcodec_wasm.js"
cp "${BUILD_DIR}/opcodec_wasm.wasm" "${NEXUS_PUBLIC}/opcodec_wasm.wasm"

echo "==> Done: ${NEXUS_PUBLIC}/opcodec_wasm.{js,wasm}"
