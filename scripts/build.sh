#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)

# Setup toolchain environment
"$ROOT_DIR/scripts/setup_toolchain.sh" && source "$ROOT_DIR/_toolchain/env.sh"

BUILD_PROFILE=${1:-RelWithDebInfo}  # Debug, Release, RelWithDebInfo, MinSizeRel

SRC="$ROOT_DIR/native"
BUILD="$SRC/.build/${BUILD_PROFILE}"

mkdir -p "$BUILD"
cmake -S "$SRC" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=${BUILD_PROFILE} \
  -DCMAKE_COLOR_DIAGNOSTICS=ON \
  -Wno-deprecated \
  -G Ninja
cmake --build "$BUILD" --parallel

cp -v "$BUILD/"_native.*.so "exts/bricksim/bricksim/"

# Run tests only when RUN_TESTS is set
if [ -n "${RUN_TESTS:-}" ]; then
  cd "$BUILD"
  ctest --output-on-failure
  cd "$ROOT_DIR"
fi
