# lego-robot

## Build

### Prerequisites
Requires Ubuntu 22.04+ or other distributions satisfying:
 * GLIBC >= 2.35
 * GLIBCXX >= 3.4.30
 * CXXABI >= 1.3.13

Install build requirements (for Debian / Ubuntu):
```bash
sudo apt install build-essential wget python3.11-full xz-utils zstd
```

### Clone

```bash
git clone https://github.com/yushijinhun/lego-robot.git --recursive
cd lego-robot
```

### Setup virtualenv
```bash
python3.11 -m venv --symlinks --prompt lego --upgrade-deps .venv
source .venv/bin/activate
```

### Install Isaac Sim 5.1.0
```bash
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

### Install Isaac Lab
```bash
CMAKE_POLICY_VERSION_MINIMUM=3.5 ./IsaacLab/isaaclab.sh --install
```

### Build & Install This Extension
```bash
scripts/build.sh
pip install -e exts/lego_assemble -v
```

### Launch Isaac Sim
```bash
scripts/launch_isaacsim.sh
```

### VS Code Setup (optional)
```bash
source .venv/bin/activate
python scripts/collect_isaacsim_paths.py
```
This creates `pyrightconfig.deps.json` for code completion.

### Issues
1. `ModuleNotFoundError: No module named '_lzma'`.
Solution: Reinstall Python. The system Python may miss this package.
2. `The currently installed NVIDIA graphics driver is unsupported or has known issues. Reason for failure: The minimum Omniverse RTX requirement on Linux Installed driver: 535.18 The unsupported driver range: [0.0, 535.129)`
Solution: install another driver, e.g., 580. The 535.274 does not work according to Nvidia.
