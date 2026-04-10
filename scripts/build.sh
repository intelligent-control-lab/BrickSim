#!/usr/bin/env bash
set -eo pipefail

die() {
    echo "[ERROR] $*" >&2
    exit 1
}

read_prebuilt_manifest() {
    PREBUILT_ARTIFACT=""
    PREBUILT_SHA256=""
    PREBUILT_DOWNLOAD_URL=""
    PREBUILT_COMMIT=""

    local line key value
    while IFS= read -r line || [[ -n "$line" ]]; do
        line="${line%$'\r'}"
        if [[ -z "$line" || "$line" == \#* ]]; then
            continue
        fi
        IFS='=' read -r key value <<< "$line"
        case "$key" in
            ARTIFACT) PREBUILT_ARTIFACT="$value" ;;
            SHA256) PREBUILT_SHA256="$value" ;;
            DOWNLOAD_URL) PREBUILT_DOWNLOAD_URL="$value" ;;
            COMMIT) PREBUILT_COMMIT="$value" ;;
        esac
    done < "$PREBUILT_NATIVE_MANIFEST"
}

validate_prebuilt_manifest() {
    [[ -n "$PREBUILT_ARTIFACT" ]] || die "Prebuilt native manifest $PREBUILT_NATIVE_MANIFEST is missing ARTIFACT"
    [[ "$PREBUILT_ARTIFACT" != */* ]] || die "Prebuilt native manifest $PREBUILT_NATIVE_MANIFEST has invalid ARTIFACT=$PREBUILT_ARTIFACT"
    [[ "$PREBUILT_SHA256" =~ ^[0-9A-Fa-f]{64}$ ]] || die "Prebuilt native manifest $PREBUILT_NATIVE_MANIFEST has invalid SHA256=$PREBUILT_SHA256"
    [[ -n "$PREBUILT_DOWNLOAD_URL" ]] || die "Prebuilt native manifest $PREBUILT_NATIVE_MANIFEST is missing DOWNLOAD_URL"
    [[ "$PREBUILT_COMMIT" =~ ^[0-9A-Fa-f]{40}$ ]] || die "Prebuilt native manifest $PREBUILT_NATIVE_MANIFEST has invalid COMMIT=$PREBUILT_COMMIT"
}

use_prebuilt_native_if_configured() {
    if [[ ! -f "$PREBUILT_NATIVE_MANIFEST" ]]; then
        return 0
    fi

    if [[ -n "${RUN_TESTS:-}" ]]; then
        die "RUN_TESTS=1 cannot be used with a prebuilt native cache at $PREBUILT_NATIVE_MANIFEST. Delete the prebuilt cache and rebuild locally to run native tests."
    fi

    read_prebuilt_manifest
    validate_prebuilt_manifest

    local cache_artifact="$PREBUILT_NATIVE_DIR/$PREBUILT_ARTIFACT"
    [[ -f "$cache_artifact" ]] || die "Prebuilt native manifest exists at $PREBUILT_NATIVE_MANIFEST, but cached artifact is missing: $cache_artifact"

    local actual_sha256
    actual_sha256=$(sha256sum "$cache_artifact" | awk '{print $1}')
    if [[ "$actual_sha256" != "$PREBUILT_SHA256" ]]; then
        die "Prebuilt native cache is corrupted: expected SHA256 $PREBUILT_SHA256 for $cache_artifact, got $actual_sha256. Rerun scripts/download_prebuilt_native.sh or delete $PREBUILT_NATIVE_MANIFEST."
    fi

    local output_path
    if [[ -n "$BRICKSIM_NATIVE_OUTPUT" ]]; then
        output_path="$BRICKSIM_NATIVE_OUTPUT"
    else
        output_path="$ROOT_DIR/python/bricksim/$PREBUILT_ARTIFACT"
    fi
    mkdir -p "$(dirname "$output_path")"
    cp -v "$cache_artifact" "$output_path"

    echo "Skipping native build because a prebuilt native cache is active via .prebuilt-native/manifest.env"
    echo "Using cached artifact .prebuilt-native/$PREBUILT_ARTIFACT"
    echo "To resume normal C++ builds, delete .prebuilt-native/manifest.env"
    exit 0
}

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)

BUILD_PROFILE=${1:-RelWithDebInfo}  # Debug, Release, RelWithDebInfo, MinSizeRel
BRICKSIM_NATIVE_OUTPUT=${BRICKSIM_NATIVE_OUTPUT:-}
PREBUILT_NATIVE_DIR="$ROOT_DIR/.prebuilt-native"
PREBUILT_NATIVE_MANIFEST="$PREBUILT_NATIVE_DIR/manifest.env"

# Respect an explicit prebuilt cache before touching the toolchain or local build dirs.
use_prebuilt_native_if_configured

# Setup toolchain environment
"$ROOT_DIR/scripts/setup_toolchain.sh"
source "$ROOT_DIR/_toolchain/env.sh"

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

native_outputs=("$BUILD"/core.*.so)
if [ "${#native_outputs[@]}" -ne 1 ]; then
  die "Expected exactly one core.*.so shared library in $BUILD" 
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
