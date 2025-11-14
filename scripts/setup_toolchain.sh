#!/usr/bin/env bash
set -eo pipefail

#### Dependencies
CMAKE_URL="https://github.com/Kitware/CMake/releases/download/v4.2.0-rc2/cmake-4.2.0-rc2-linux-x86_64.tar.gz"
CMAKE_SHA256="50c445ff77648556e4d1f68edf763a3d8186c8c90d3433e604a613d51bf41297"

P7ZIP_URL="https://www.7-zip.org/a/7z2501-linux-x64.tar.xz"
P7ZIP_SHA256="4ca3b7c6f2f67866b92622818b58233dc70367be2f36b498eb0bdeaaa44b53f4"

NINJA_URL="https://github.com/ninja-build/ninja/releases/download/v1.13.1/ninja-linux.zip"
NINJA_FILENAME="ninja-v1.13.1-linux.zip"
NINJA_SHA256="0830252db77884957a1a4b87b05a1e2d9b5f658b8367f82999a941884cbe0238"

LLVM_URL="https://www.cs.cmu.edu/~haoweiw/llvm-22+0246f33.tar.xz"
LLVM_SHA256="82cc1d93c95c544f6c7231efd9f5a0a3602082c4fb4d9dddc8ba8e425ded6e9c"

GCC_DIRNAME="gcc-15.2.0-7ubuntu1_amd64"
GCC_VER="15"
GCC_DEB_URLS=(
    "https://snapshot.ubuntu.com/ubuntu/20251108T100000Z/pool/main/g/gcc-15/libstdc++-15-dev_15.2.0-7ubuntu1_amd64.deb"
    "https://snapshot.ubuntu.com/ubuntu/20251108T100000Z/pool/main/g/gcc-15/libgcc-15-dev_15.2.0-7ubuntu1_amd64.deb"
    "https://snapshot.ubuntu.com/ubuntu/20251108T100000Z/pool/main/g/glibc/libc6-dev_2.39-0ubuntu8_amd64.deb"
)
GCC_DEB_SHA256S=(
    "c3e207a04665c0a3be55ce44cf8a6bd2d1f77972ab7ac25683c41ae731f9ca47"
    "75a4a840031b1d03a52a51baaef09a9ffa16c933a6a350b48e1217cd3167dd26"
    "1f03b79223d9cf1a6ed3a648f6942619dcb97b2884dba8fd12e262b6771e247a"
)

####

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)

TC_DIR="$ROOT_DIR/_toolchain"
mkdir -p "$TC_DIR"
TC_DOWNLOADS_DIR="$TC_DIR/downloads"
mkdir -p "$TC_DOWNLOADS_DIR"

#### Setup cmake
CMAKE_DIRNAME="$(basename "$CMAKE_URL" .tar.gz)"
CMAKE_DIR="$TC_DIR/$CMAKE_DIRNAME"
if [[ ! -d "$CMAKE_DIR" ]]; then
    CMAKE_ARCHIVE_PATH="$TC_DOWNLOADS_DIR/$(basename "$CMAKE_URL")"
    [ -f "$CMAKE_ARCHIVE_PATH" ] || wget -O "$CMAKE_ARCHIVE_PATH" "$CMAKE_URL"
    CMAKE_SHA256_ACTUAL=$(sha256sum "$CMAKE_ARCHIVE_PATH" | awk '{print $1}')
    if [[ "$CMAKE_SHA256_ACTUAL" != "$CMAKE_SHA256" ]]; then
        echo "CMake download is corrupted (expected SHA256: $CMAKE_SHA256, actual: $CMAKE_SHA256_ACTUAL)"
        exit 1
    fi
    tar -xzf "$CMAKE_ARCHIVE_PATH" -C "$TC_DIR"
fi

### Setup p7zip
P7ZIP_DIRNAME="$(basename "$P7ZIP_URL" .tar.xz)"
P7ZIP_DIR="$TC_DIR/$P7ZIP_DIRNAME"
if [[ ! -d "$P7ZIP_DIR" ]]; then
    P7ZIP_ARCHIVE_PATH="$TC_DOWNLOADS_DIR/$(basename "$P7ZIP_URL")"
    [ -f "$P7ZIP_ARCHIVE_PATH" ] || wget -O "$P7ZIP_ARCHIVE_PATH" "$P7ZIP_URL"
    P7ZIP_SHA256_ACTUAL=$(sha256sum "$P7ZIP_ARCHIVE_PATH" | awk '{print $1}')
    if [[ "$P7ZIP_SHA256_ACTUAL" != "$P7ZIP_SHA256" ]]; then
        echo "p7zip download is corrupted (expected SHA256: $P7ZIP_SHA256, actual: $P7ZIP_SHA256_ACTUAL)"
        exit 1
    fi
    mkdir -p "$P7ZIP_DIR"
    tar -xJf "$P7ZIP_ARCHIVE_PATH" -C "$P7ZIP_DIR"
fi
P7ZIP_BIN="$P7ZIP_DIR/7zz"

### Setup ninja
NINJA_DIRNAME="$(basename "$NINJA_FILENAME" .zip)"
NINJA_DIR="$TC_DIR/$NINJA_DIRNAME"
if [[ ! -d "$NINJA_DIR" ]]; then
    NINJA_ARCHIVE_PATH="$TC_DOWNLOADS_DIR/$NINJA_FILENAME"
    [ -f "$NINJA_ARCHIVE_PATH" ] || wget -O "$NINJA_ARCHIVE_PATH" "$NINJA_URL"
    NINJA_SHA256_ACTUAL=$(sha256sum "$NINJA_ARCHIVE_PATH" | awk '{print $1}')
    if [[ "$NINJA_SHA256_ACTUAL" != "$NINJA_SHA256" ]]; then
        echo "Ninja download is corrupted (expected SHA256: $NINJA_SHA256, actual: $NINJA_SHA256_ACTUAL)"
        exit 1
    fi
    mkdir -p "$NINJA_DIR"
    "$P7ZIP_BIN" x "$NINJA_ARCHIVE_PATH" -o"$NINJA_DIR" -y
fi

### Setup llvm
LLVM_DIRNAME="$(basename "$LLVM_URL" .tar.xz)"
LLVM_DIR="$TC_DIR/$LLVM_DIRNAME"
if [[ ! -d "$LLVM_DIR" ]]; then
    LLVM_ARCHIVE_PATH="$TC_DOWNLOADS_DIR/$(basename "$LLVM_URL")"
    [ -f "$LLVM_ARCHIVE_PATH" ] || wget -O "$LLVM_ARCHIVE_PATH" "$LLVM_URL"
    LLVM_SHA256_ACTUAL=$(sha256sum "$LLVM_ARCHIVE_PATH" | awk '{print $1}')
    if [[ "$LLVM_SHA256_ACTUAL" != "$LLVM_SHA256" ]]; then
        echo "LLVM download is corrupted (expected SHA256: $LLVM_SHA256, actual: $LLVM_SHA256_ACTUAL)"
        exit 1
    fi
    mkdir -p "$LLVM_DIR"
    tar -xJf "$LLVM_ARCHIVE_PATH" -C "$LLVM_DIR" --strip-components=1
fi

### Setup gcc
extract_deb() {
    local deb="$1" dest="$2"
    local tmp
    tmp="$(mktemp -d)"
    ar x "$deb" --output="$tmp"
    local data_archive
    data_archive=$(find "$tmp" -maxdepth 1 -type f -name 'data.tar*' | head -n 1)
    if [[ -z "$data_archive" ]]; then
        echo "No data archive found in $deb"
        exit 1
    fi
    tar -xaf "$data_archive" -C "$dest"
    rm -rf "$tmp"
}
GCC_DIR="$TC_DIR/$GCC_DIRNAME"
if [[ ! -d "$GCC_DIR" ]]; then
    mkdir -p "$GCC_DIR"
    for i in "${!GCC_DEB_URLS[@]}"; do
        DEB_URL="${GCC_DEB_URLS[$i]}"
        DEB_SHA256="${GCC_DEB_SHA256S[$i]}"
        DEB_ARCHIVE_PATH="$TC_DOWNLOADS_DIR/$(basename "$DEB_URL")"
        [ -f "$DEB_ARCHIVE_PATH" ] || wget -O "$DEB_ARCHIVE_PATH" "$DEB_URL"
        DEB_SHA256_ACTUAL=$(sha256sum "$DEB_ARCHIVE_PATH" | awk '{print $1}')
        if [[ "$DEB_SHA256_ACTUAL" != "$DEB_SHA256" ]]; then
            echo "GCC download $(basename "$DEB_URL") is corrupted (expected SHA256: $DEB_SHA256, actual: $DEB_SHA256_ACTUAL)"
            exit 1
        fi
        extract_deb "$DEB_ARCHIVE_PATH" "$GCC_DIR"
    done
    # Patch libstdc++ modules JSON paths to be relative
    sed -e "s@/usr/include@../../../../../usr/include@g" -i "$GCC_DIR/usr/lib/gcc/x86_64-linux-gnu/$GCC_VER/libstdc++.modules.json"
fi

### Generate env.sh
ENV_SH_PATH="$TC_DIR/env.sh"
cat > "$ENV_SH_PATH" << EOF
if [[ -z "\${LEGO_TC_DIR:-}" ]]; then
    export LEGO_TC_DIR="\$(dirname -- "\$(realpath -- "\${BASH_SOURCE[0]}")")"
    export PATH="\$LEGO_TC_DIR/$CMAKE_DIRNAME/bin:\$LEGO_TC_DIR/$P7ZIP_DIRNAME:\$LEGO_TC_DIR/$NINJA_DIRNAME:\$LEGO_TC_DIR/$LLVM_DIRNAME/bin:\$PATH"
    export CC="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/clang"
    export CXX="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/clang++"
    export AR="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/llvm-ar"
    export RANLIB="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/llvm-ranlib"
    export NM="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/llvm-nm"
    export STRIP="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/llvm-strip"
    export OBJCOPY="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/llvm-objcopy"
    export OBJDUMP="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/llvm-objdump"
    export READELF="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/llvm-readelf"
    export ADDR2LINE="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/llvm-addr2line"
    export CMAKE_CXX_COMPILER_AR="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/llvm-ar"
    export CMAKE_CXX_COMPILER_RANLIB="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/llvm-ranlib"
    export CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS="\$LEGO_TC_DIR/$LLVM_DIRNAME/bin/clang-scan-deps"
    export CCC_OVERRIDE_OPTIONS="\${CCC_OVERRIDE_OPTIONS:+\$CCC_OVERRIDE_OPTIONS }# ^--gcc-install-dir=\$LEGO_TC_DIR/$GCC_DIRNAME/usr/lib/gcc/x86_64-linux-gnu/$GCC_VER"
    export CXXFLAGS="\${CXXFLAGS:+\$CXXFLAGS } -nostdlib++"
    export LDFLAGS="\${LDFLAGS:+\$LDFLAGS }-fuse-ld=lld -L/usr/lib/x86_64-linux-gnu -Wl,-rpath-link,/usr/lib/x86_64-linux-gnu -Wl,-Bdynamic -l:libstdc++.so.6"
    export CPLUS_INCLUDE_PATH="\$LEGO_TC_DIR/$GCC_DIRNAME/usr/include/c++/$GCC_VER:\$LEGO_TC_DIR/$GCC_DIRNAME/usr/include/x86_64-linux-gnu/c++/$GCC_VER:\$LEGO_TC_DIR/$GCC_DIRNAME/usr/include:\$LEGO_TC_DIR/$GCC_DIRNAME/usr/include/x86_64-linux-gnu\${CPLUS_INCLUDE_PATH:+:\$CPLUS_INCLUDE_PATH}"
fi
EOF

### Generate CMakePresets.json
cat > "$ROOT_DIR/native/CMakePresets.json" <<JSON
{
    "version": 8,
    "configurePresets": [
        {
            "name": "dev",
            "generator": "Ninja",
            "binaryDir": "\${sourceDir}/.build/\${presetName}",
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Debug",
                "CMAKE_C_COMPILER": "clang",
                "CMAKE_CXX_COMPILER": "clang++",
                "CMAKE_AR": "llvm-ar",
                "CMAKE_RANLIB": "llvm-ranlib",
                "CMAKE_NM": "llvm-nm",
                "CMAKE_STRIP": "llvm-strip",
                "CMAKE_OBJCOPY": "llvm-objcopy",
                "CMAKE_OBJDUMP": "llvm-objdump",
                "CMAKE_READELF": "llvm-readelf",
                "CMAKE_ADDR2LINE": "llvm-addr2line",
                "CMAKE_CXX_COMPILER_AR": "llvm-ar",
                "CMAKE_CXX_COMPILER_RANLIB": "llvm-ranlib",
                "CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS": "clang-scan-deps",
                "CMAKE_CXX_FLAGS": "-nostdlib++ --gcc-install-dir=\${sourceDir}/../_toolchain/$GCC_DIRNAME/usr/lib/gcc/x86_64-linux-gnu/$GCC_VER",
                "CMAKE_EXE_LINKER_FLAGS": "-fuse-ld=lld -L/usr/lib/x86_64-linux-gnu -Wl,-rpath-link,/usr/lib/x86_64-linux-gnu -Wl,-Bdynamic -l:libstdc++.so.6",
                "CMAKE_SHARED_LINKER_FLAGS": "-fuse-ld=lld -L/usr/lib/x86_64-linux-gnu -Wl,-rpath-link,/usr/lib/x86_64-linux-gnu -Wl,-Bdynamic -l:libstdc++.so.6",
                "CMAKE_MODULE_LINKER_FLAGS": "-fuse-ld=lld -L/usr/lib/x86_64-linux-gnu -Wl,-rpath-link,/usr/lib/x86_64-linux-gnu -Wl,-Bdynamic -l:libstdc++.so.6"
            },
            "environment": {
                "PATH": "\${sourceDir}/../_toolchain/$CMAKE_DIRNAME/bin:\${sourceDir}/../_toolchain/$P7ZIP_DIRNAME:\${sourceDir}/../_toolchain/$NINJA_DIRNAME:\${sourceDir}/../_toolchain/$LLVM_DIRNAME/bin:\$penv{PATH}",
                "CPLUS_INCLUDE_PATH": "\${sourceDir}/../_toolchain/$GCC_DIRNAME/usr/include/c++/$GCC_VER:\${sourceDir}/../_toolchain/$GCC_DIRNAME/usr/include/x86_64-linux-gnu/c++/$GCC_VER:\${sourceDir}/../_toolchain/$GCC_DIRNAME/usr/include:\${sourceDir}/../_toolchain/$GCC_DIRNAME/usr/include/x86_64-linux-gnu"
            }
        }
    ],
    "buildPresets": [
        { "name": "dev", "configurePreset": "dev" }
    ],
    "testPresets": [
        { "name": "dev", "configurePreset": "dev" }
    ]
}
JSON
