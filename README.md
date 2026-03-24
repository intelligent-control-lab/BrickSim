<div align="center">

# BrickSim
### A Physics-Based Simulator for Manipulating Interlocking Brick Assemblies

<p>
  <a href="https://arxiv.org/abs/2603.16853"><img src="https://img.shields.io/badge/arXiv-2603.16853-b31b1b.svg?logo=arxiv" alt="arXiv 2603.16853"></a>
  <a href="https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html"><img src="https://img.shields.io/badge/Isaac%20Sim-5.1-76B900?logo=nvidia&amp;logoColor=white" alt="Isaac Sim 5.1"></a>
  <a href="https://en.cppreference.com/w/cpp/26"><img src="https://img.shields.io/badge/C%2B%2B-26-00599C?logo=cplusplus&amp;logoColor=white" alt="C++26"></a>
  <a href="https://docs.python.org/3.11/"><img src="https://img.shields.io/badge/python-3.11-3776AB?logo=python&amp;logoColor=white" alt="Python 3.11"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-MIT-yellow.svg" alt="MIT License"></a>
</p>

<p>
  <a href="https://arxiv.org/abs/2603.16853">Paper</a> •
  <a href="#quickstart">Quickstart</a> •
  <a href="#build--test">Build &amp; Test</a> •
  <a href="#run-demos">Demos</a> •
  <a href="https://www.youtube.com/watch?v=VGuHfz4yxLU">Video</a>
</p>

</div>

<p align="center">
  <img src="docs/assets/bricksim_teaser.png" alt="BrickSim teaser showing a multi-robot brick assembly workflow and a gallery of simulated brick structures." width="100%">
</p>

## Quickstart

```bash
git clone --recursive https://github.com/intelligent-control-lab/BrickSim BrickSim
cd BrickSim

python3.11 -m venv --symlinks --prompt bricksim --upgrade-deps .venv
./scripts/setup_env.sh

./scripts/launch_isaacsim.sh demos/demo_assembly.py
```

## Prerequisites

- BrickSim currently supports the x86-64 Linux platform. Support for other platforms is coming.
- Ubuntu 22.04+ or another Linux distribution with `GLIBC >= 2.35`, `GLIBCXX >= 3.4.30`, and `CXXABI >= 1.3.13`
- Python 3.11
- A working NVIDIA driver compatible with Isaac Sim RTX requirements

## Installation

### 1. Clone the repository

```bash
git clone --recursive https://github.com/intelligent-control-lab/BrickSim BrickSim
cd BrickSim
```

### 2. Prepare a Python virtual environment

BrickSim scripts resolve the Python virtual environment in this order:

1. Active conda environment
2. Active Python virtualenv
3. Repository-local `.venv` without sourcing it

Use one of the following supported setups.

Repository-local virtualenv:

```bash
python3.11 -m venv --symlinks --prompt bricksim --upgrade-deps .venv
```

Conda:

```bash
conda create -n bricksim python=3.11
conda activate bricksim
```

### 3. Install required host tools

Debian/Ubuntu example:

```bash
sudo apt install build-essential wget python3.11-full xz-utils zstd
```

### 4. Run the BrickSim setup script

```bash
./scripts/setup_env.sh
```

### 5. Launch Isaac Sim

Only Isaac Sim 5.1 is currently supported.

```bash
./scripts/launch_isaacsim.sh demos/demo_assembly.py
```

## Build & Test

Build the native extension:

```bash
scripts/build.sh
```

Build and run the C++ sanity checks:

```bash
RUN_TESTS=1 scripts/build.sh
```

The build copies the compiled `_native` module into `exts/bricksim/bricksim/`.

## Run Demos

Launch the assembly demo:

```bash
./scripts/launch_isaacsim.sh demos/demo_assembly.py
```

Other useful entry points:

- `demos/demo_inhand.py` for in-hand manipulation experiments
- `demos/demo_keyboard_teleop.py` for keyboard-driven interaction
- `demos/demo_teleop.py` for teleoperation, recording, and replay workflows

The teleoperation demo expects the `lerobot` package plus a configured leader device path inside `demos/demo_teleop.py`.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `native/` | C++26 core |
| `exts/bricksim/` | Python extension and API |
| `demos/` | End-to-end examples |
| `resources/` | USD assets, robot assets, and brick datasets |
| `scripts/` | Utility scripts |

## Development Setup

Generate dependency-aware Pyright configuration for VS Code completion:

```bash
python scripts/generate_pyrightconfig.py
```

This creates `pyrightconfig.deps.json`.
