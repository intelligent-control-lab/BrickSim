#!/usr/bin/env bash
set -eo pipefail

#### Dependencies
CMAKE_URL="https://github.com/Kitware/CMake/releases/download/v4.2.3/cmake-4.2.3-linux-x86_64.tar.gz"
CMAKE_SHA256="5bb505d5e0cca0480a330f7f27ccf52c2b8b5214c5bba97df08899f5ef650c23"

P7ZIP_URL="https://www.7-zip.org/a/7z2600-linux-x64.tar.xz"
P7ZIP_SHA256="c74dc4a48492cde43f5fec10d53fb2a66f520e4a62a69d630c44cb22c477edc6"

NINJA_URL="https://github.com/ninja-build/ninja/releases/download/v1.13.2/ninja-linux.zip"
NINJA_FILENAME="ninja-v1.13.2-linux.zip"
NINJA_SHA256="5749cbc4e668273514150a80e387a957f933c6ed3f5f11e03fb30955e2bbead6"

LLVM_URL="https://github.com/llvm/llvm-project/releases/download/llvmorg-22.1.0-rc3/LLVM-22.1.0-rc3-Linux-X64.tar.xz"
LLVM_SHA256="5360d8bef7b715c4cc2689d548d13e676d755cbf425a6bcb07c2959403da3047"

GCC_DIRNAME="gcc-15.2.0-12ubuntu1_amd64"
GCC_VER="15"
GCC_DEB_URLS=(
    "https://www.cs.cmu.edu/~haoweiw/bricksim_downloads/libstdc++-15-dev_15.2.0-12ubuntu1_amd64.deb"
    "https://www.cs.cmu.edu/~haoweiw/bricksim_downloads/libgcc-15-dev_15.2.0-12ubuntu1_amd64.deb"
    "https://www.cs.cmu.edu/~haoweiw/bricksim_downloads/libc6-dev_2.42-2ubuntu5_amd64.deb"
)
GCC_DEB_SHA256S=(
    "ab6ee3f4f3aae070cd4da95601f96c79e37d015d94968f22c3c0096c42607c25"
    "500bd80c40374f47636d6bc5a348210c7cfca0feddd5736967067ffcd04153fc"
    "d027e5c4f42ae23677d606b7e6969971c85104fa1dd17b49cdca54199c66829b"
)

####

download_with_retry() {
    local url="$1"
    local dest="$2"
    local expected_sha256="$3"

    if [[ -f "$dest" ]]; then
        local existing_sha256
        existing_sha256=$(sha256sum "$dest" | awk '{print $1}')
        if [[ "$existing_sha256" == "$expected_sha256" ]]; then
            return 0
        fi
        echo "Existing file has wrong SHA256, redownloading: $dest" >&2
        rm -f "$dest"
    fi

    echo "Downloading with retry: ${url}" >&2
    if ! wget --tries=10 --retry-on-http-error=500 -O "$dest" "$url"; then
        echo "Download failed for $url" >&2
        exit 1
    fi

    local actual_sha256
    actual_sha256=$(sha256sum "$dest" | awk '{print $1}')
    if [[ "$actual_sha256" != "$expected_sha256" ]]; then
        echo "SHA256 mismatch for $dest (expected: $expected_sha256, actual: $actual_sha256)" >&2
        exit 1
    fi
}

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
    download_with_retry "$CMAKE_URL" "$CMAKE_ARCHIVE_PATH" "$CMAKE_SHA256"
    tar -xzf "$CMAKE_ARCHIVE_PATH" -C "$TC_DIR"
fi

### Setup p7zip
P7ZIP_DIRNAME="$(basename "$P7ZIP_URL" .tar.xz)"
P7ZIP_DIR="$TC_DIR/$P7ZIP_DIRNAME"
if [[ ! -d "$P7ZIP_DIR" ]]; then
    P7ZIP_ARCHIVE_PATH="$TC_DOWNLOADS_DIR/$(basename "$P7ZIP_URL")"
    download_with_retry "$P7ZIP_URL" "$P7ZIP_ARCHIVE_PATH" "$P7ZIP_SHA256"
    mkdir -p "$P7ZIP_DIR"
    tar -xJf "$P7ZIP_ARCHIVE_PATH" -C "$P7ZIP_DIR"
fi
P7ZIP_BIN="$P7ZIP_DIR/7zz"

### Setup ninja
NINJA_DIRNAME="$(basename "$NINJA_FILENAME" .zip)"
NINJA_DIR="$TC_DIR/$NINJA_DIRNAME"
if [[ ! -d "$NINJA_DIR" ]]; then
    NINJA_ARCHIVE_PATH="$TC_DOWNLOADS_DIR/$NINJA_FILENAME"
    download_with_retry "$NINJA_URL" "$NINJA_ARCHIVE_PATH" "$NINJA_SHA256"
    mkdir -p "$NINJA_DIR"
    "$P7ZIP_BIN" x "$NINJA_ARCHIVE_PATH" -o"$NINJA_DIR" -y
fi

### Setup llvm
LLVM_DIRNAME="$(basename "$LLVM_URL" .tar.xz)"
LLVM_DIR="$TC_DIR/$LLVM_DIRNAME"
if [[ ! -d "$LLVM_DIR" ]]; then
    LLVM_ARCHIVE_PATH="$TC_DOWNLOADS_DIR/$(basename "$LLVM_URL")"
    download_with_retry "$LLVM_URL" "$LLVM_ARCHIVE_PATH" "$LLVM_SHA256"
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
    # Ensure all GCC debs are downloaded and verified before creating the target dir.
    for i in "${!GCC_DEB_URLS[@]}"; do
        DEB_URL="${GCC_DEB_URLS[$i]}"
        DEB_SHA256="${GCC_DEB_SHA256S[$i]}"
        DEB_ARCHIVE_PATH="$TC_DOWNLOADS_DIR/$(basename "$DEB_URL")"
        download_with_retry "$DEB_URL" "$DEB_ARCHIVE_PATH" "$DEB_SHA256"
    done

    mkdir -p "$GCC_DIR"
    for i in "${!GCC_DEB_URLS[@]}"; do
        DEB_URL="${GCC_DEB_URLS[$i]}"
        DEB_ARCHIVE_PATH="$TC_DOWNLOADS_DIR/$(basename "$DEB_URL")"
        extract_deb "$DEB_ARCHIVE_PATH" "$GCC_DIR"
    done
    # Patch libstdc++ modules JSON paths to be relative
    sed -e "s@/usr/include@../../../../../usr/include@g" -i "$GCC_DIR/usr/lib/gcc/x86_64-linux-gnu/$GCC_VER/libstdc++.modules.json"
fi

### Generate env.sh
ENV_SH_PATH="$TC_DIR/env.sh"
cat > "$ENV_SH_PATH" << EOF
if [[ -z "\${BRICKSIM_TC_DIR:-}" ]]; then
    export BRICKSIM_TC_DIR="\$(dirname -- "\$(realpath -- "\${BASH_SOURCE[0]}")")"
    export PATH="\$BRICKSIM_TC_DIR/$CMAKE_DIRNAME/bin:\$BRICKSIM_TC_DIR/$P7ZIP_DIRNAME:\$BRICKSIM_TC_DIR/$NINJA_DIRNAME:\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin:\$PATH"
    export CC="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/clang"
    export CXX="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/clang++"
    export AR="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/llvm-ar"
    export RANLIB="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/llvm-ranlib"
    export NM="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/llvm-nm"
    export STRIP="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/llvm-strip"
    export OBJCOPY="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/llvm-objcopy"
    export OBJDUMP="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/llvm-objdump"
    export READELF="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/llvm-readelf"
    export ADDR2LINE="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/llvm-addr2line"
    export CMAKE_CXX_COMPILER_AR="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/llvm-ar"
    export CMAKE_CXX_COMPILER_RANLIB="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/llvm-ranlib"
    export CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS="\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/clang-scan-deps"
    export CCC_OVERRIDE_OPTIONS="\${CCC_OVERRIDE_OPTIONS:+\$CCC_OVERRIDE_OPTIONS }# ^--gcc-install-dir=\$BRICKSIM_TC_DIR/$GCC_DIRNAME/usr/lib/gcc/x86_64-linux-gnu/$GCC_VER"
    export CXXFLAGS="\${CXXFLAGS:+\$CXXFLAGS } -nostdlib++"
    export LDFLAGS="\${LDFLAGS:+\$LDFLAGS }-fuse-ld=lld -L/usr/lib/x86_64-linux-gnu -Wl,-rpath-link,/usr/lib/x86_64-linux-gnu -Wl,-Bdynamic -l:libstdc++.so.6"
    export CPLUS_INCLUDE_PATH="\$BRICKSIM_TC_DIR/$GCC_DIRNAME/usr/include/c++/$GCC_VER:\$BRICKSIM_TC_DIR/$GCC_DIRNAME/usr/include/c++/$GCC_VER/backward:\$BRICKSIM_TC_DIR/$GCC_DIRNAME/usr/include/x86_64-linux-gnu/c++/$GCC_VER:\$BRICKSIM_TC_DIR/$GCC_DIRNAME/usr/include:\$BRICKSIM_TC_DIR/$GCC_DIRNAME/usr/include/x86_64-linux-gnu\${CPLUS_INCLUDE_PATH:+:\$CPLUS_INCLUDE_PATH}"
fi
EOF

### Generate clangd-wrapper.sh
CLANGD_WRAPPER_PATH="$TC_DIR/clangd-wrapper.sh"
cat > "$CLANGD_WRAPPER_PATH" << EOF
#!/usr/bin/env bash
SCRIPT_DIR=\$(cd -- "\$(dirname -- "\$0")" && pwd -P)
source "\$SCRIPT_DIR/env.sh"
exec "\$BRICKSIM_TC_DIR/$LLVM_DIRNAME/bin/clangd" "\$@"
EOF
chmod +x "$CLANGD_WRAPPER_PATH"

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
                "CPLUS_INCLUDE_PATH": "\${sourceDir}/../_toolchain/$GCC_DIRNAME/usr/include/c++/$GCC_VER:\${sourceDir}/../_toolchain/$GCC_DIRNAME/usr/include/c++/$GCC_VER/backward:\${sourceDir}/../_toolchain/$GCC_DIRNAME/usr/include/x86_64-linux-gnu/c++/$GCC_VER:\${sourceDir}/../_toolchain/$GCC_DIRNAME/usr/include:\${sourceDir}/../_toolchain/$GCC_DIRNAME/usr/include/x86_64-linux-gnu"
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
