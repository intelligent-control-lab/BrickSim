"""Build configuration for the BrickSim Python package."""

import os
import subprocess
from distutils.errors import DistutilsSetupError
from pathlib import Path

from setuptools import Extension, find_namespace_packages, setup
from setuptools.command.build_ext import build_ext
from setuptools.command.sdist import sdist


class BrickSimBuildExt(build_ext):
    """Build the native extension through the repository build script."""

    def build_extension(self, ext):
        """Build one native extension module."""
        py_project_root = Path(__file__).resolve().parent
        repo_root = py_project_root.parent
        build_script = repo_root / "scripts" / "build.sh"
        if not build_script.is_file():
            raise DistutilsSetupError(
                "Building BrickSim from source requires the full BrickSim "
                "repository checkout. Either install a wheel or clone the full "
                "BrickSim repository to build from source."
            )

        output_path = Path(self.get_ext_fullpath(ext.name))
        if not output_path.is_absolute():
            output_path = py_project_root / output_path
        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        env = os.environ.copy()
        env.pop("RUN_TESTS", None)
        env["BRICKSIM_NATIVE_OUTPUT"] = str(output_path)
        subprocess.run([str(build_script)], cwd=repo_root, env=env, check=True)


class BrickSimSdist(sdist):
    """Reject standalone source distributions."""

    def run(self):
        """Fail because source distributions cannot build standalone."""
        raise DistutilsSetupError(
            "Standalone source distributions for BrickSim are unsupported. "
            "Either install a wheel or clone the full BrickSim repository to "
            "build from source."
        )


setup(
    packages=find_namespace_packages(where=".", include=["bricksim*"]),
    ext_modules=(
        []
        if os.environ.get("BRICKSIM_SKIP_NATIVE_BUILD") == "1"
        else [Extension("bricksim.core", sources=[])]
    ),
    cmdclass={
        "build_ext": BrickSimBuildExt,
        "sdist": BrickSimSdist,
    },
    include_package_data=True,
    zip_safe=False,
)
