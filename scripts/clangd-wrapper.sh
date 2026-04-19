#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)
PIXI_BIN=${PIXI:-pixi}

if ! command -v "$PIXI_BIN" >/dev/null 2>&1; then
    if [[ -x "$HOME/.pixi/bin/pixi" ]]; then
        PIXI_BIN="$HOME/.pixi/bin/pixi"
    else
        echo "[ERROR] pixi executable not found" >&2
        exit 127
    fi
fi

cd "$ROOT_DIR"
exec "$PIXI_BIN" run clangd "$@"
