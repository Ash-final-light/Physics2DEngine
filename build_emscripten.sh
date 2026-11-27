#!/usr/bin/env bash
set -euo pipefail
# Activate emsdk before calling this script. All paths are relative to this file.
BOX2D_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$BOX2D_SCRIPT_DIR"
emcmake cmake -S . -B build-emscripten -DBUILD_TESTING=ON -DBOX2D_DISABLE_SIMD=OFF -DCMAKE_BUILD_TYPE=Debug
cmake --build build-emscripten --parallel 8
