"""Installation script for the 'lego_assemble' python package."""

import os
import pathlib
import shutil
import subprocess
import sys
from typing import List

import toml

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
# Read the extension.toml file
EXTENSION_TOML_DATA = toml.load(os.path.join(EXTENSION_PATH, "config", "extension.toml"))

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # NOTE: Add dependencies
    "psutil",
]

# --- CMake-based extension wiring (minimal, no external build helpers) ---

class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = ""):
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(pathlib.Path(sourcedir).resolve())


class CMakeBuild(build_ext):
    def run(self):
        # Ensure CMake is available (will raise if missing)
        subprocess.check_output(["cmake", "--version"])

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext: CMakeExtension):
        # Require a single env var that points to IsaacSim's target-deps tree.
        # Example: export ISAACSIM_TARGET_DEPS=/home/you/IsaacSim/_build/target-deps
        env = os.environ.copy()
        env["CC"] = "clang"
        env["CXX"] = "clang++"
        env["NVCC_CCBIN"] = "gcc-11"
        target_deps = env.get("ISAACSIM_TARGET_DEPS")
        if not target_deps:
            raise RuntimeError(
                "ISAACSIM_TARGET_DEPS is not set. Please export ISAACSIM_TARGET_DEPS to your IsaacSim _build/target-deps directory."
            )
        if not pathlib.Path(target_deps).is_dir():
            raise RuntimeError(f"ISAACSIM_TARGET_DEPS does not exist: {target_deps}")

        ext_fullpath = pathlib.Path(self.get_ext_fullpath(ext.name))
        extdir = ext_fullpath.parent.resolve()

        cfg = "Debug" if self.debug else "Release"
        build_temp = pathlib.Path(self.build_temp) / ext.name
        build_temp.mkdir(parents=True, exist_ok=True)

        cmake_args: List[str] = [
            "-G", "Ninja",
            f"-DCMAKE_BUILD_TYPE={cfg}",
            f"-DEXT_OUTPUT_DIR={extdir}",
            f"-DTARGET_DEPS_DIR={target_deps}",
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        ]

        # Build
        subprocess.check_call(["cmake", ext.sourcedir] + cmake_args, cwd=build_temp, env=env)
        # Expose compile_commands.json for clangd at the C++ source dir
        cc_src = build_temp / "compile_commands.json"
        cc_dst = pathlib.Path(ext.sourcedir) / "compile_commands.json"
        if cc_src.exists():
            shutil.copy2(cc_src, cc_dst)

        subprocess.check_call(["cmake", "--build", ".", "--config", cfg], cwd=build_temp, env=env)

        # On some platforms pybind11 places the built .so/.pyd in EXT_OUTPUT_DIR already.
        # If not, copy any produced module-like artifacts into extdir as a fallback.
        artifacts = [p for p in build_temp.glob("**/_native.*") if p.is_file()]
        for art in artifacts:
            dest = extdir / art.name
            if not dest.exists():
                shutil.copy2(art, dest)

# Installation operation
setup(
    name="lego_assemble",
    packages=["lego_assemble"],
    package_data={"lego_assemble": ["_native.pyi"]},
    author=EXTENSION_TOML_DATA["package"]["author"],
    maintainer=EXTENSION_TOML_DATA["package"]["maintainer"],
    url=EXTENSION_TOML_DATA["package"]["repository"],
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    install_requires=INSTALL_REQUIRES,
    license="MIT",
    include_package_data=True,
    python_requires=">=3.11",
    # Build the C++ pybind module via CMake (placed inside the python package as lego_assemble/_native.*)
    ext_modules=[CMakeExtension("lego_assemble._native", sourcedir=os.path.join(EXTENSION_PATH, "cpp"))],
    cmdclass={"build_ext": CMakeBuild},
    classifiers=[
        "Natural Language :: English",
        "Programming Language :: Python :: 3.11",
        "Isaac Sim :: 5.0.0",
    ],
    zip_safe=False,
)
