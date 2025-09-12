#!/usr/bin/env bash
set -eo pipefail

export CC=gcc-11 CXX=g++-11 NVCC_CCBIN=gcc-11

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)

source "$ROOT_DIR/.venv/bin/activate"

SRC="$ROOT_DIR/extension/source/lego_assemble/cpp"
BUILD="$SRC/.build/Debug"
OUT="$ROOT_DIR/extension/source/lego_assemble/lego_assemble"

mkdir -p "$BUILD"

# Generator and ccache preferences
GEN_ARGS=()
if command -v ninja >/dev/null 2>&1; then
  GEN_ARGS=("-G" "Ninja")
fi

CCACHE_ARGS=()
if command -v ccache >/dev/null 2>&1; then
  CCACHE_ARGS=("-DCMAKE_C_COMPILER_LAUNCHER=ccache" "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache")
fi

# Always (re)configure to ensure compile_commands.json is regenerated
cmake -S "$SRC" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DEXT_OUTPUT_DIR="$OUT" \
  -DTARGET_DEPS_DIR="$ISAACSIM_TARGET_DEPS" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
  -DCMAKE_COLOR_DIAGNOSTICS=ON \
  "${GEN_ARGS[@]}" \
  "${CCACHE_ARGS[@]}"

# Copy compile_commands.json to the C++ source dir for clangd/IDE
if [[ -f "$BUILD/compile_commands.json" ]]; then
  cp -f "$BUILD/compile_commands.json" "$SRC/compile_commands.json"
fi

# Build with parallel jobs (defaults to all cores if not set)
JOBS=${CMAKE_BUILD_PARALLEL_LEVEL:-}
if [[ -n "$JOBS" ]]; then
  cmake --build "$BUILD" --parallel "$JOBS"
else
  cmake --build "$BUILD" --parallel
fi

echo "Built module(s) in: $OUT"
