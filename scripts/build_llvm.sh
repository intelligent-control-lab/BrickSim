#!/usr/bin/env bash
set -euo pipefail

# Defaults (override with flags)
LLVM_VER="${LLVM_VER:-22}"
LLVM_COMMIT="${LLVM_COMMIT:-0246f331d4438375389dfbbd8132fb70638ee64f}"
IMAGE_TAG="${IMAGE_TAG:-llvm-builder}"
OUT_DIR="${OUT_DIR:-dist}"
BUILD_KIND="${BUILD_KIND:-release}" # release | debug

# Parse flags: --llvm-ver X --commit SHA --out DIR --tag NAME [--debug]
while [[ $# -gt 0 ]]; do
  case "$1" in
    --llvm-ver) LLVM_VER="$2"; shift 2;;
    --commit)   LLVM_COMMIT="$2"; shift 2;;
    --out)      OUT_DIR="$2"; shift 2;;
    --tag)      IMAGE_TAG="$2"; shift 2;;
    --debug)    BUILD_KIND="debug"; shift 1;;
    *) echo "Unknown arg: $1"; exit 1;;
  esac
done

mkdir -p "$OUT_DIR"

# Map build kind -> CMake/LLVM config
if [[ "$BUILD_KIND" == "debug" ]]; then
  # Debug-friendly LLVM:
  #   - RelWithDebInfo: optimized with debug info
  #   - Assertions ON for internal checks
  #   - No strip so symbols remain in the tarball
  CMAKE_BUILD_TYPE="RelWithDebInfo"
  LLVM_ENABLE_ASSERTIONS="ON"
  STRIP_BINS="OFF"
  TARBALL_SUFFIX="-debug"
else
  # Default "release" build: small and fast
  CMAKE_BUILD_TYPE="Release"
  LLVM_ENABLE_ASSERTIONS="OFF"
  STRIP_BINS="ON"
  TARBALL_SUFFIX=""
fi

# Use an empty context so nothing gets sent
CTX="$(mktemp -d)"
trap 'rm -rf "$CTX"' EXIT

# Build from Dockerfile via stdin (no files in context)
docker build -t "$IMAGE_TAG" \
  --build-arg LLVM_VER="$LLVM_VER" \
  --build-arg LLVM_COMMIT="$LLVM_COMMIT" \
  --build-arg CMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE" \
  --build-arg LLVM_ENABLE_ASSERTIONS="$LLVM_ENABLE_ASSERTIONS" \
  --build-arg STRIP_BINS="$STRIP_BINS" \
  --build-arg TARBALL_SUFFIX="$TARBALL_SUFFIX" \
  -f - "$CTX" <<'DOCKERFILE'
# syntax=docker/dockerfile:1
FROM ubuntu:22.04 AS build
ARG LLVM_VER
ARG LLVM_COMMIT
ARG DEFAULT_TRIPLE=x86_64-pc-linux-gnu
ARG TARGETS_TO_BUILD=X86
ARG CMAKE_BUILD_TYPE=Release
ARG LLVM_ENABLE_ASSERTIONS=OFF
ARG STRIP_BINS=ON
ARG TARBALL_SUFFIX=""
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential ninja-build cmake python3 \
      ca-certificates curl xz-utils patchelf file \
      zlib1g-dev libzstd-dev libxml2-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /src
RUN mkdir -p /src/llvm-project && \
    curl -fsSL "https://github.com/llvm/llvm-project/archive/${LLVM_COMMIT}.tar.gz" \
    | tar -xzf - -C /src/llvm-project --strip-components=1 --no-same-owner

RUN cmake -S /src/llvm-project/llvm -B /build -G Ninja \
      -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
      -DLLVM_ENABLE_ASSERTIONS="${LLVM_ENABLE_ASSERTIONS}" \
      -DCMAKE_INSTALL_PREFIX="/opt/llvm-${LLVM_VER}" \
      -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra;lld" \
      -DLLVM_ENABLE_RUNTIMES="compiler-rt" \
      -DCOMPILER_RT_BUILD_SANITIZERS=ON \
      -DCOMPILER_RT_DEFAULT_TARGET_ONLY=ON \
      -DLLVM_TARGETS_TO_BUILD="${TARGETS_TO_BUILD}" \
      -DLLVM_INCLUDE_TESTS=OFF -DCLANG_INCLUDE_TESTS=OFF \
      -DLLVM_BUILD_LLVM_DYLIB=ON -DLLVM_LINK_LLVM_DYLIB=ON \
      -DLLVM_DEFAULT_TARGET_TRIPLE="${DEFAULT_TRIPLE}" \
      -DLLVM_APPEND_VC_REV=ON \
      -DLLVM_FORCE_VC_REVISION="${LLVM_COMMIT}" \
      -DLLVM_FORCE_VC_REPOSITORY="https://github.com/llvm/llvm-project" \
      -DCLANG_REPOSITORY_STRING="https://github.com/llvm/llvm-project" \
      -DCMAKE_INSTALL_RPATH="\$ORIGIN/../lib" -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON

RUN cmake --build /build -j"$(nproc)" && \
    if [ "${STRIP_BINS}" = "ON" ]; then \
      cmake --build /build --target install/strip; \
    else \
      cmake --build /build --target install; \
    fi

RUN set -eux; \
    mkdir -p "/opt/llvm-${LLVM_VER}/lib"; \
    need_bins="/opt/llvm-${LLVM_VER}/bin/clang /opt/llvm-${LLVM_VER}/bin/clang++ /opt/llvm-${LLVM_VER}/bin/clangd /opt/llvm-${LLVM_VER}/bin/clang-scan-deps /opt/llvm-${LLVM_VER}/bin/lld /opt/llvm-${LLVM_VER}/bin/ld.lld"; \
    for b in $need_bins; do \
      if [ -x "$b" ]; then \
        ldd "$b" | awk '/=>/ {print $3} $2 ~ /^\// {print $2}' | sort -u | while read -r so; do \
          case "$so" in ""|*ld-linux*|*/libc.so.*|*/libm.so.*|*/libdl.so.*|*/libpthread.so.*|*/librt.so.*|*/libgcc_s.so.*) ;; \
            *) cp -n "$so" "/opt/llvm-${LLVM_VER}/lib/" || true ;; \
          esac; \
        done; \
      fi; \
    done; \
    for p in /opt/llvm-${LLVM_VER}/bin/* /opt/llvm-${LLVM_VER}/lib/*.so*; do \
      if [ -f "$p" ] && file "$p" | grep -q ELF; then \
        patchelf --force-rpath --set-rpath '$ORIGIN/../lib' "$p" || true; \
      fi; \
    done

RUN set -eux; short="$(printf '%.7s' "${LLVM_COMMIT}")"; \
    mkdir -p /out; \
    XZ_OPT='-T0 -0' tar -C /opt -cvJf "/out/llvm-${LLVM_VER}+${short}${TARBALL_SUFFIX}.tar.xz" "llvm-${LLVM_VER}"

FROM scratch AS artifact
ARG LLVM_VER
ARG LLVM_COMMIT
COPY --from=build /out/ /out/
CMD ["artifact"]
DOCKERFILE

# Copy artifact out
cid="$(docker create "$IMAGE_TAG")"
mkdir -p "$OUT_DIR"
docker cp "$cid:/out/." "$OUT_DIR"
docker rm "$cid"

echo "Done -> $OUT_DIR"
ls -lh "$OUT_DIR"
