#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)
source "$ROOT_DIR/scripts/_resolve_env.sh"
bricksim_require_env

readonly NVIDIA_PYPI_URL="https://pypi.nvidia.com"
readonly PYTORCH_CU128_INDEX_URL="https://download.pytorch.org/whl/cu128"

require_commands() {
    local missing=()
    local command_name
    for command_name in git wget tar ar sha256sum; do
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

check_python_version() {
    "$BRICKSIM_ENV_PYTHON" - <<'PY'
import sys

if sys.version_info[:2] != (3, 11):
    raise SystemExit(
        f"[ERROR] BrickSim requires Python 3.11.x in the resolved environment, found {sys.version.split()[0]}."
    )
PY
}

pip_install() {
    "$BRICKSIM_ENV_PYTHON" -m pip install "$@"
}

pip_install_editable() {
    "$BRICKSIM_ENV_PYTHON" -m pip install --no-build-isolation -e "$1"
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
    check_python_version

    echo "[INFO] Using BrickSim environment: $BRICKSIM_ENV_DIR ($BRICKSIM_ENV_KIND)"

    pip_install "setuptools<81" "wheel<0.46" "packaging==23.0" "poetry-core"
    pip_install "isaacsim[all,extscache]==5.1.0" --extra-index-url "$NVIDIA_PYPI_URL"
    pip_install --index-url "$PYTORCH_CU128_INDEX_URL" \
        "torch==2.7.0" \
        "torchvision==0.22.0" \
        "torchaudio==2.7.0"

    pip_install_editable "$ROOT_DIR/IsaacLab/source/isaaclab"
    pip_install_editable "$ROOT_DIR/IsaacLab/source/isaaclab_assets"
    pip_install_editable "$ROOT_DIR/IsaacLab/source/isaaclab_tasks"
    pip_install --no-build-isolation -e "$ROOT_DIR/IsaacLab/source/isaaclab_rl[all]"
    pip_install --no-build-isolation -e "$ROOT_DIR/IsaacLab/source/isaaclab_mimic[all]"

    "$ROOT_DIR/scripts/build.sh"
    pip_install -e "$ROOT_DIR/exts/bricksim"

    verify_isaacsim_entrypoint
    verify_python_packages
    verify_native_module_artifact

    echo "[INFO] BrickSim environment setup complete."
}

main "$@"
