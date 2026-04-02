#!/usr/bin/env bash

_BRICKSIM_RESOLVER_SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
_BRICKSIM_RESOLVER_ROOT_DIR=$(cd -- "$_BRICKSIM_RESOLVER_SCRIPT_DIR/.." && pwd -P)

bricksim_print_env_resolution_help() {
    cat >&2 <<EOF
[ERROR] Unable to resolve a BrickSim Python environment.

BrickSim scripts use the first available environment in this order:
1. Active conda environment
2. Active virtualenv
3. Repository-local .venv (without sourcing it)

Fix one of the following and rerun the command:
- Activate a conda environment containing Python 3.11, then run:
    uv sync --directory ${_BRICKSIM_RESOLVER_ROOT_DIR} --locked --active --inexact
- Activate a Python virtualenv containing Python 3.11, then run:
    uv sync --directory ${_BRICKSIM_RESOLVER_ROOT_DIR} --locked --active --inexact
- Create a repo-local environment with:
    uv sync --directory ${_BRICKSIM_RESOLVER_ROOT_DIR} --locked
  or run:
    ${_BRICKSIM_RESOLVER_ROOT_DIR}/scripts/setup_env.sh
EOF
}

bricksim_resolve_env() {
    BRICKSIM_ROOT_DIR="${_BRICKSIM_RESOLVER_ROOT_DIR}"
    BRICKSIM_ENV_KIND=""
    BRICKSIM_ENV_DIR=""

    if [[ -n "${CONDA_PREFIX:-}" && -x "${CONDA_PREFIX}/bin/python" ]]; then
        BRICKSIM_ENV_KIND="conda"
        BRICKSIM_ENV_DIR="${CONDA_PREFIX}"
    elif [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/python" ]]; then
        BRICKSIM_ENV_KIND="virtualenv"
        BRICKSIM_ENV_DIR="${VIRTUAL_ENV}"
    elif [[ -x "${BRICKSIM_ROOT_DIR}/.venv/bin/python" ]]; then
        BRICKSIM_ENV_KIND="repo_venv"
        BRICKSIM_ENV_DIR="${BRICKSIM_ROOT_DIR}/.venv"
    else
        return 1
    fi

    BRICKSIM_ENV_BIN_DIR="${BRICKSIM_ENV_DIR}/bin"
    BRICKSIM_ENV_PYTHON="${BRICKSIM_ENV_BIN_DIR}/python"
    BRICKSIM_ENV_ISAACSIM="${BRICKSIM_ENV_BIN_DIR}/isaacsim"
    export BRICKSIM_ROOT_DIR BRICKSIM_ENV_KIND BRICKSIM_ENV_DIR BRICKSIM_ENV_BIN_DIR
    export BRICKSIM_ENV_PYTHON BRICKSIM_ENV_ISAACSIM
}

bricksim_require_env() {
    if ! bricksim_resolve_env; then
        bricksim_print_env_resolution_help
        return 1
    fi
}
