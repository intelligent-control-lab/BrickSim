"""Setuptools bridge for building the BrickSim Python package."""

import os
import subprocess
from distutils.errors import DistutilsSetupError
from pathlib import Path

from setuptools import Extension, find_namespace_packages, setup
from setuptools.command.build_ext import build_ext
from setuptools.command.sdist import sdist


THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parents[1]
BUILD_SCRIPT = REPO_ROOT / "scripts" / "build.sh"
NATIVE_CMAKE_LISTS = REPO_ROOT / "native" / "CMakeLists.txt"
FULL_REPO_ERROR = (
    "Building bricksim from source requires the full BrickSim repository "
    "checkout (expected scripts/build.sh and native/CMakeLists.txt next to "
    "exts/bricksim). Install a wheel instead."
)

PACKAGES = find_namespace_packages(
    where=".",
    include=["bricksim*"],
)
NATIVE_EXTENSION = Extension(
    "bricksim._native",
    sources=[],
)


class BrickSimBuildExt(build_ext):
    def run(self):
        self._require_full_repo_checkout()
        if not self.extensions:
            return
        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        old_inplace = self.inplace
        self.inplace = False
        regular_output = self._project_path(self.get_ext_fullpath(ext.name))
        self.inplace = old_inplace

        inplace_output = None
        if self.inplace:
            inplace_output = self._project_path(self.get_ext_fullpath(ext.name))

        self.mkpath(str(regular_output.parent))
        env = os.environ.copy()
        env.pop("RUN_TESTS", None)
        env["BRICKSIM_NATIVE_OUTPUT"] = str(regular_output)

        # Reuse the repo's pinned LLVM/CMake/Ninja toolchain instead of
        # setuptools' normal compiler pipeline.
        subprocess.run([str(BUILD_SCRIPT)], cwd=REPO_ROOT, env=env, check=True)

        if inplace_output is not None and inplace_output != regular_output:
            self.mkpath(str(inplace_output.parent))
            self.copy_file(str(regular_output), str(inplace_output))

    @staticmethod
    def _project_path(path):
        path = Path(path)
        if path.is_absolute():
            return path
        return THIS_DIR / path

    @staticmethod
    def _require_full_repo_checkout():
        if BUILD_SCRIPT.is_file() and NATIVE_CMAKE_LISTS.is_file():
            return
        raise DistutilsSetupError(FULL_REPO_ERROR)


class BrickSimSdist(sdist):
    def run(self):
        raise DistutilsSetupError(
            "Standalone source distributions for exts/bricksim are unsupported. "
            + FULL_REPO_ERROR
        )


setup(
    packages=PACKAGES,
    ext_modules=[NATIVE_EXTENSION],
    cmdclass={"build_ext": BrickSimBuildExt, "sdist": BrickSimSdist},
    include_package_data=True,
    zip_safe=False,
)
