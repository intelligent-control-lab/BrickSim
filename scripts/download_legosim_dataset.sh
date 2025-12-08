#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)

DATASET_URL="https://www.cs.cmu.edu/~haoweiw/legosim_dataset.tar.xz"

cd "$ROOT_DIR/resources/legosim_dataset"
wget -O- "$DATASET_URL" | tar -xJf -
