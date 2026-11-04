#!/usr/bin/env bash
# Build the Box3D static library with emscripten, then link the embind glue
# into isomorphic ES modules (browser + node).
#
# Flavours (standard and deluxe use wasm SIMD, which is baseline everywhere in 2026):
#   standard  single threaded
#   deluxe    wasm threads (SharedArrayBuffer, pthreads)
#   scalar    single threaded, no wasm SIMD (Box3D's scalar math)
#
# The auto-detecting entry (src/entry.mjs) is copied to dist/ and picks
# scalar on a host without SIMD, deluxe when threads are usable, standard otherwise.
#
# Usage:
#   scripts/build.sh                 build every flavour, Release
#   FLAVOURS=standard scripts/build.sh
#   TARGET_TYPE=Debug scripts/build.sh
set -euo pipefail

# pwd -W gives a Windows path under Git Bash (MSYS), which the Windows emsdk tools need; elsewhere it fails and pwd runs.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && { pwd -W 2>/dev/null || pwd; })"
ROOT="$(dirname "$DIR")"

FLAVOURS="${FLAVOURS:-standard scalar deluxe}"
TARGET_TYPE="${TARGET_TYPE:-Release}"
BOX3D_SRC="$ROOT/deps/box3d"

if [ ! -f "$BOX3D_SRC/CMakeLists.txt" ]; then
  echo "deps/box3d missing. Run: npm run fetch-deps" >&2
  exit 1
fi

command -v emcc >/dev/null || { echo "emcc not found. Activate emsdk first." >&2; exit 1; }

mkdir -p "$ROOT/dist" "$ROOT/build"

for FLAVOUR in $FLAVOURS; do
  CMAKE_BUILD_DIR="$ROOT/build/cmake-$FLAVOUR-$TARGET_TYPE"

  FLAVOUR_FLAGS=()
  BASENAME="box3d"
  DISABLE_SIMD=OFF
  case "$FLAVOUR" in
    standard)
      ;;
    scalar)
      # Box3D's scalar math and no wasm SIMD, for hosts without wasm SIMD.
      DISABLE_SIMD=ON
      BASENAME="box3d.scalar"
      ;;
    deluxe)
      FLAVOUR_FLAGS=(-pthread)
      BASENAME="box3d.deluxe"
      ;;
    *)
      echo "Unknown flavour: $FLAVOUR (expected standard, scalar or deluxe)" >&2
      exit 1
      ;;
  esac

  # The single-threaded flavours: -Os with LTO is as fast as -O3 without it and 38 KB smaller gzipped (the standard
  # flavour); not measured on the threaded one, which stays at -O3.
  SIZE_FLAGS=(-Os -flto)
  OPT_FLAGS="-Os"
  LTO_FLAGS="-flto"
  if [ "$FLAVOUR" = "deluxe" ] || [ "$TARGET_TYPE" = "Debug" ]; then
    SIZE_FLAGS=(-O3)
    OPT_FLAGS="-O3"
    LTO_FLAGS=""
  fi

  echo "==> cmake ($FLAVOUR, $TARGET_TYPE)"
  emcmake cmake \
    -S "$BOX3D_SRC" \
    -B "$CMAKE_BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$TARGET_TYPE" \
    -DCMAKE_C_FLAGS="${FLAVOUR_FLAGS[*]:-} $LTO_FLAGS" \
    -DCMAKE_C_FLAGS_RELEASE="$OPT_FLAGS -DNDEBUG" \
    -DBOX3D_SAMPLES=OFF \
    -DBOX3D_UNIT_TESTS=OFF \
    -DBOX3D_BENCHMARKS=OFF \
    -DBOX3D_DOCS=OFF \
    -DBOX3D_VALIDATE=OFF \
    -DBOX3D_DISABLE_SIMD="$DISABLE_SIMD" \
    > "$CMAKE_BUILD_DIR.cmake.log" 2>&1 || { cat "$CMAKE_BUILD_DIR.cmake.log" >&2; exit 1; }

  echo "==> build libbox3d.a ($FLAVOUR)"
  cmake --build "$CMAKE_BUILD_DIR" -j"$(nproc)" > "$CMAKE_BUILD_DIR.build.log" 2>&1 \
    || { tail -50 "$CMAKE_BUILD_DIR.build.log" >&2; exit 1; }

  LIB="$CMAKE_BUILD_DIR/src/libbox3d.a"
  [ -f "$LIB" ] || { echo "missing $LIB" >&2; exit 1; }

  EMCC_OPTS=(
    -std=c++17
    -lembind
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=Box3D
    -sENVIRONMENT=web,worker,node
    -sALLOW_MEMORY_GROWTH=1
    -sALLOW_TABLE_GROWTH=1
    -sFILESYSTEM=0
    -sEXPORTED_RUNTIME_METHODS=HEAPF32,HEAPU8,HEAPU32
  )
  if [ "$DISABLE_SIMD" = OFF ]; then
    EMCC_OPTS+=(-msimd128 -msse2)
  fi

  if [ "$FLAVOUR" = "deluxe" ]; then
    EMCC_OPTS+=(
      -pthread
      "-sPTHREAD_POOL_SIZE=(globalThis.navigator&&navigator.hardwareConcurrency)||8"
    )
  fi

  case "$TARGET_TYPE" in
    Debug)
      EMCC_OPTS+=(-g3 -gsource-map -sASSERTIONS=2)
      ;;
    *)
      EMCC_OPTS+=("${SIZE_FLAGS[@]}")
      ;;
  esac

  # Box3D reads the clock around every stage and every time-of-impact sweep; in WebAssembly each read crosses into
  # JavaScript. The glue's __wrap_clock_gettime skips it unless enableStepTimers(true) is called. Not on the threaded
  # flavour: there libc may wait on the clock, and the gain is not measured.
  if [ "$FLAVOUR" != "deluxe" ]; then
    EMCC_OPTS+=(-Wl,--wrap=clock_gettime)
  fi

  echo "==> emcc link ($FLAVOUR) -> dist/$BASENAME.mjs"
  em++ "$ROOT/csrc/glue.cpp" "$ROOT/csrc/flat.cpp" "$LIB" \
    -I "$BOX3D_SRC/include" \
    "${EMCC_OPTS[@]}" \
    "${FLAVOUR_FLAGS[@]:-}" \
    -o "$ROOT/dist/$BASENAME.mjs"
done

cp "$ROOT/src/entry.mjs" "$ROOT/dist/entry.mjs"

# The isomorphic surface: the shared TS frontend (type-STRIPPED, not
# transformed -- the same file must compile under a native host's static
# dialect, so nothing here may depend on tsc semantics), its wasm backend,
# and the iso entry that wires them to a flavour.
echo "==> frontend (shared TS -> dist/frontend.js)"
node node_modules/typescript/bin/tsc "$ROOT/src/frontend.ts" \
  --target es2022 --module esnext --moduleResolution bundler \
  --outDir "$ROOT/dist" --declaration false --skipLibCheck
cp "$ROOT/src/backend-wasm.mjs" "$ROOT/dist/backend.js"
cp "$ROOT/src/iso.mjs" "$ROOT/dist/iso.mjs"

echo "==> done"
ls -la "$ROOT/dist"
