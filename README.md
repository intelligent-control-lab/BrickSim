# lego-robot

## Build

### Prerequisites
Requires Ubuntu 22.04+ or other distributions satisfying:
 * GLIBC > 2.35
 * GLIBCXX_3.4.30
 * CXXABI_1.3.13

Install build requirements (for Debian / Ubuntu):
```bash
sudo apt install build-essential wget python3.11-full
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
