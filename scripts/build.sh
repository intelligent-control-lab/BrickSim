#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)

# Setup toolchain environment
"$ROOT_DIR/scripts/setup_toolchain.sh"
source "$ROOT_DIR/_toolchain/env.sh"

BUILD_PROFILE=${1:-RelWithDebInfo}  # Debug, Release, RelWithDebInfo, MinSizeRel
BRICKSIM_NATIVE_OUTPUT=${BRICKSIM_NATIVE_OUTPUT:-}

SRC="$ROOT_DIR/native"
BUILD="$SRC/.build/${BUILD_PROFILE}"
BUILD_TESTS=OFF
if [ -n "${RUN_TESTS:-}" ]; then
  BUILD_TESTS=ON
fi

mkdir -p "$BUILD"
cmake -S "$SRC" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=${BUILD_PROFILE} \
  -DBRICKSIM_BUILD_TESTS=${BUILD_TESTS} \
  -DCMAKE_COLOR_DIAGNOSTICS=ON \
  -Wno-deprecated \
  -G Ninja
cmake --build "$BUILD" --parallel

native_outputs=("$BUILD"/_native.*.so)
if [ "${#native_outputs[@]}" -ne 1 ]; then
  echo "[ERROR] Expected exactly one _native shared library in $BUILD" >&2
  exit 1
fi

if [ -n "$BRICKSIM_NATIVE_OUTPUT" ]; then
  mkdir -p "$(dirname "$BRICKSIM_NATIVE_OUTPUT")"
  cp -v "${native_outputs[0]}" "$BRICKSIM_NATIVE_OUTPUT"
else
  cp -v "${native_outputs[0]}" "python/bricksim/"
fi

# Run tests only when RUN_TESTS is set
if [ -n "${RUN_TESTS:-}" ]; then
  cd "$BUILD"
  ctest --output-on-failure
  cd "$ROOT_DIR"
fi
