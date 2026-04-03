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
  <a href="https://intelligent-control-lab.github.io/BrickSim/">Website</a> •
  <a href="https://arxiv.org/abs/2603.16853">Paper</a> •
  <a href="#quickstart">Quickstart</a> •
  <a href="https://www.youtube.com/watch?v=VGuHfz4yxLU">Video</a>
</p>

</div>

<p align="center">
  <a href="https://www.youtube.com/watch?v=VGuHfz4yxLU">
    <img src="docs/assets/bricksim_teaser.png" alt="BrickSim teaser showing a multi-robot brick assembly workflow and a gallery of simulated brick structures." width="100%">
  </a>
</p>

## Quickstart

### Prerequisites
- x86-64 Linux platform. Support for other platforms is coming.
- Ubuntu 22.04+ or another Linux distribution with `GLIBC >= 2.35`, `GLIBCXX >= 3.4.30`, and `CXXABI >= 1.3.13`
- [`uv` package manager](https://docs.astral.sh/uv/getting-started/installation/)
- A working NVIDIA driver compatible with [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html#system-requirements)

### Installation
We use `uv` for package management. If you don't have it installed, please refer to [Installing uv](https://docs.astral.sh/uv/getting-started/installation/).

```bash
# Install host tools (for Debian/Ubuntu)
sudo apt install build-essential wget xz-utils zstd git

# Clone the repository
git clone --recursive https://github.com/intelligent-control-lab/BrickSim BrickSim
cd BrickSim

# Set up the Python environment
# This will also install Isaac Sim 5.1, Isaac Lab, and the BrickSim package
uv sync --locked
```

### Run Demos
```bash
# Launch the assembly demo in Isaac Sim
./scripts/launch_isaacsim.sh demos/demo_assembly.py
```

Other demos include:
- `demos/demo_inhand.py` for in-hand manipulation experiments
- `demos/demo_keyboard_teleop.py` for keyboard-driven interaction
- `demos/demo_teleop.py` for teleoperation, recording, and replay workflows
  - The teleoperation demo expects the `lerobot` package plus a configured leader device path inside `demos/demo_teleop.py`.

## Development

### Repository Layout
| Path              | Purpose                                       |
| ----------------- | --------------------------------------------- |
| `native/`         | C++26 core                                    |
| `exts/bricksim/`  | Python extension and API                      |
| `demos/`          | Demos                                         |
| `resources/`      | USD assets, robot assets, and brick datasets  |
| `scripts/`        | Utility scripts                               |

### Generating Pyright Configuration

Generate dependency-aware Pyright configuration for VS Code completion:
```bash
uv run python scripts/generate_pyrightconfig.py
```

This creates `pyrightconfig.deps.json`.

### Building the C++ Extension
If you make changes to the C++ code in `native/`, you need to re-compile the native extension for the changes to take effect.

```bash
./scripts/build.sh

# To also build & run the tests, use:
RUN_TESTS=1 ./scripts/build.sh
```
