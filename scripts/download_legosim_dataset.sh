#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)

DATASET_URL="https://www.cs.cmu.edu/~haoweiw/legosim_dataset.tar.xz"

DOWNLOAD_DIR="$ROOT_DIR/resources/legosim_dataset"
mkdir -p "$DOWNLOAD_DIR"
cd "$DOWNLOAD_DIR"
wget -O- "$DATASET_URL" | tar -xJf -
