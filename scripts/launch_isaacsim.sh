#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)
source "$ROOT_DIR/.venv/bin/activate"

EXPERIENCE="isaacsim.exp.full"

DEBUG_HOST="${DEBUG_HOST:-127.0.0.1}"
DEBUG_PORT="${DEBUG_PORT:-5678}"
WAIT_FOR_CLIENT="${WAIT_FOR_CLIENT:-false}"

STAGE_PATH="${STAGE_PATH:-$ROOT_DIR/resources/demo.usda}"

exec isaacsim "$EXPERIENCE" \
  --ext-folder "$ROOT_DIR/IsaacLab/source" \
  --ext-folder "$ROOT_DIR/exts" \
  --enable lego_assemble \
  --enable omni.kit.debug.python \
  --/exts/omni.kit.debug.python/mode=listen \
  --/exts/omni.kit.debug.python/host="$DEBUG_HOST" \
  --/exts/omni.kit.debug.python/port="$DEBUG_PORT" \
  --/exts/omni.kit.debug.python/waitForClient="$WAIT_FOR_CLIENT" \
  --/exts/omni.kit.debug.python/debugpyLogging=true \
  --/app/content/emptyStageOnStart=false \
  --/crashreporter/enabled=false \
  --exec "open_stage.py $STAGE_PATH" \
  -v
