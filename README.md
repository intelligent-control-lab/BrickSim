# lego-robot

## Build

### Prerequisites

- Toolchain: clang-22, CMake >= 4.1.2, Python 3.11, ninja, ccache

For Debian / Ubuntu, clang and CMake can be installed from upstream repositories:
- clang-22: follow instructions at https://apt.llvm.org/
- CMake: follow instructions at https://apt.kitware.com/

Ninja and ccache can be installed with APT:
```bash
sudo apt install ninja-build ccache
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
pip install -e source/lego_assemble -v
```

### Launch Isaac Sim
```bash
scripts/launch_isaacsim.sh
```

### Debug builds
```bash
scripts/build_debug.sh
```
This script produces a Debug build of the C++ extension. It runs fast and is suitable for development.

