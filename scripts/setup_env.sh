#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)
source "$ROOT_DIR/scripts/_resolve_env.sh"

require_commands() {
    local missing=()
    local command_name
    for command_name in git wget tar ar sha256sum uv; do
        if ! command -v "$command_name" >/dev/null 2>&1; then
            missing+=("$command_name")
        fi
    done

    if (( ${#missing[@]} > 0 )); then
        {
            echo "[ERROR] Missing required host tools: ${missing[*]}"
            echo "Install the missing commands for your Linux distribution and rerun ./scripts/setup_env.sh."
        } >&2
        exit 1
    fi
}

sync_environment() {
    if [[ -n "${CONDA_PREFIX:-}" && -x "${CONDA_PREFIX}/bin/python" ]]; then
        echo "[INFO] Syncing BrickSim dependencies into active conda environment: $CONDA_PREFIX"
        uv sync --directory "$ROOT_DIR" --locked --active --inexact
        return
    fi

    if [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/python" ]]; then
        echo "[INFO] Syncing BrickSim dependencies into active virtualenv: $VIRTUAL_ENV"
        uv sync --directory "$ROOT_DIR" --locked --active --inexact
        return
    fi

    echo "[INFO] Syncing BrickSim dependencies into repo-local environment: $ROOT_DIR/.venv"
    uv sync --directory "$ROOT_DIR" --locked
}

check_python_version() {
    "$BRICKSIM_ENV_PYTHON" - <<'PY'
import sys

if sys.version_info[:2] != (3, 11):
    raise SystemExit(
        f"[ERROR] BrickSim requires Python 3.11.x in the resolved environment, found {sys.version.split()[0]}."
    )
PY
}

verify_python_packages() {
    "$BRICKSIM_ENV_PYTHON" - <<'PY'
import importlib
import importlib.util

for module_name in ("isaacsim", "isaaclab_tasks"):
    if importlib.util.find_spec(module_name) is None:
        raise SystemExit(f"[ERROR] Failed to locate the {module_name} Python package in the resolved environment.")

for module_name in ("torch", "isaaclab", "bricksim"):
    importlib.import_module(module_name)
PY
}

verify_isaacsim_entrypoint() {
    if [[ ! -x "$BRICKSIM_ENV_ISAACSIM" ]]; then
        echo "[ERROR] Isaac Sim executable is missing from the resolved environment: $BRICKSIM_ENV_ISAACSIM" >&2
        exit 1
    fi
}

verify_native_module_artifact() {
    shopt -s nullglob
    local native_modules=("$ROOT_DIR"/exts/bricksim/bricksim/_native.*.so)
    shopt -u nullglob

    if (( ${#native_modules[@]} == 0 )); then
        echo "[ERROR] BrickSim native module was not built: exts/bricksim/bricksim/_native.*.so" >&2
        exit 1
    fi
}

main() {
    require_commands
    sync_environment
    bricksim_require_env
    check_python_version

    echo "[INFO] Using BrickSim environment: $BRICKSIM_ENV_DIR ($BRICKSIM_ENV_KIND)"

    "$ROOT_DIR/scripts/build.sh"

    verify_isaacsim_entrypoint
    verify_python_packages
    verify_native_module_artifact

    echo "[INFO] BrickSim environment setup complete."
}

main "$@"
