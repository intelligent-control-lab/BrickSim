#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)

resolve_bricksim_env() {
  if [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/python" ]]; then
    BRICKSIM_ENV_DIR="${VIRTUAL_ENV}"
  elif [[ -x "$ROOT_DIR/.venv/bin/python" ]]; then
    source "$ROOT_DIR/.venv/bin/activate"
    BRICKSIM_ENV_DIR="${VIRTUAL_ENV}"
    echo "[INFO] Sourced repo-local virtualenv: $BRICKSIM_ENV_DIR"
  else
    cat >&2 <<EOF
[ERROR] No Python environment found.
Run from the repo root:
  uv sync --locked
EOF
    exit 1
  fi

  BRICKSIM_ENV_ISAACSIM="$BRICKSIM_ENV_DIR/bin/isaacsim"
  if [[ ! -x "$BRICKSIM_ENV_ISAACSIM" ]]; then
    echo "[ERROR] Isaac Sim executable not found: $BRICKSIM_ENV_ISAACSIM" >&2
    exit 1
  fi
}

resolve_installed_isaaclab_source_root() {
  BRICKSIM_ENV_ISAACLAB_SOURCE_ROOT="$("$BRICKSIM_ENV_DIR/bin/python" - <<'PY'
import importlib.util
from pathlib import Path


def resolve_isaaclab_source_root() -> Path:
    spec = importlib.util.find_spec("isaaclab")
    if spec is None or spec.origin is None:
        raise RuntimeError("Missing required Python package: isaaclab")

    module_dir = Path(spec.origin).resolve().parent

    nested_source_root = module_dir / "source"
    if (nested_source_root / "isaaclab" / "config" / "extension.toml").is_file():
        return nested_source_root

    editable_source_root = module_dir.parent.parent
    if (editable_source_root / "isaaclab" / "config" / "extension.toml").is_file():
        return editable_source_root

    raise RuntimeError(f"Unable to resolve Isaac Lab source root from {module_dir}")


print(resolve_isaaclab_source_root())
PY
  )"
}

resolve_bricksim_env
resolve_installed_isaaclab_source_root

EXPERIENCE="isaacsim.exp.full"

DEBUG_HOST="${DEBUG_HOST:-127.0.0.1}"
DEBUG_PORT="${DEBUG_PORT:-5678}"
WAIT_FOR_CLIENT="${WAIT_FOR_CLIENT:-false}"
HEADLESS="${HEADLESS:-false}"

ISAAC_ARGS=(
  "$EXPERIENCE"
  --ext-folder "$ROOT_DIR/exts"
  --enable bricksim
  --enable omni.kit.debug.python
  --/exts/omni.kit.debug.python/mode=listen
  --/exts/omni.kit.debug.python/host="$DEBUG_HOST"
  --/exts/omni.kit.debug.python/port="$DEBUG_PORT"
  --/exts/omni.kit.debug.python/waitForClient="$WAIT_FOR_CLIENT"
  --/exts/omni.kit.debug.python/debugpyLogging=true
  --/app/content/emptyStageOnStart=false
  --/crashreporter/enabled=false
  --/log/outputStreamLevel=info
  '--/log/channels/*'=warn
  '--/log/channels/bricksim'=info
  '--/log/channels/bricksim.*'=info
)

ISAAC_ARGS+=( --ext-folder "$BRICKSIM_ENV_ISAACLAB_SOURCE_ROOT" )
if [[ "$HEADLESS" == "true" ]]; then
  ISAAC_ARGS+=( --no-window )
fi

if [ "$#" -ge 1 ]; then
  # First argument is treated as the target (module, module:func, or script path)
  # to run via kit_runner. Use a simple script invocation to avoid characters
  # that confuse Kit's wordexp-based parser.
  TARGET="$1"
  EXEC_COMMAND="$ROOT_DIR/scripts/run_demo.py $TARGET"
  ISAAC_ARGS+=( --exec "$EXEC_COMMAND" )
else
  # Open demo stage
  ISAAC_ARGS+=( --exec "open_stage.py $ROOT_DIR/resources/demo.usda" )
fi

# exec $ISAACSIM_PATH/isaac-sim.sh "${ISAAC_ARGS[@]}"
exec "$BRICKSIM_ENV_ISAACSIM" "${ISAAC_ARGS[@]}"
