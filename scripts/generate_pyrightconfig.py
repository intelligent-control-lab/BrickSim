#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path
from typing import Iterable, List


def _iter_subdirs(base: Path) -> Iterable[Path]:
    if not base.is_dir():
        return []
    for entry in base.iterdir():
        if entry.is_dir():
            yield entry


def collect_isaacsim_extension_paths() -> List[str]:
    import isaacsim  # must be importable from the current venv

    isaacsim_path = Path(os.path.realpath(isaacsim.__file__)).parent
    paths: list[Path] = []

    # Kit side (mirrors isaacsim.__main__.generate_vscode_settings)
    kit_path = isaacsim_path / "kit"

    kernel_py = kit_path / "kernel" / "py"
    if kernel_py.is_dir():
        paths.append(kernel_py)

    for folder in ("exts", "extscore"):
        base = kit_path / folder
        for ext_dir in _iter_subdirs(base):
            paths.append(ext_dir)

    # Isaac Sim side
    for folder in ("exts", "extscache", "extsDeprecated", "extsUser"):
        base = isaacsim_path / folder
        for ext_dir in _iter_subdirs(base):
            paths.append(ext_dir)

    # Omniverse global extensions (where many omni.* packages live)
    ov_base = Path.home() / ".local" / "share" / "ov" / "data" / "Kit"
    if ov_base.is_dir():
        # Example layout:
        # ~/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/exts/3/omni.usd-...
        for kit_dir in ov_base.iterdir():
            if not kit_dir.is_dir():
                continue
            # Look for any nested "exts" directories (depth >= 1).
            for exts_root in kit_dir.rglob("exts"):
                if not exts_root.is_dir():
                    continue
                for layer_dir in exts_root.iterdir():
                    if not layer_dir.is_dir():
                        continue
                    for ext_dir in layer_dir.iterdir():
                        if ext_dir.is_dir():
                            paths.append(ext_dir)

    # Deduplicate and normalize
    seen: set[str] = set()
    result: list[str] = []
    for p in paths:
        s = str(p.resolve())
        if s not in seen:
            seen.add(s)
            result.append(s)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect Isaac Sim / Omniverse python paths for pyright.")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the generated pyrightconfig.deps.json content without writing it.",
    )
    args = parser.parse_args()

    extra_paths = [
        "exts/bricksim",
        "IsaacLab/source/isaaclab",
        "IsaacLab/source/isaaclab_assets",
        "IsaacLab/source/isaaclab_mimic",
        "IsaacLab/source/isaaclab_rl",
        "IsaacLab/source/isaaclab_tasks",
    ]
    extra_paths += collect_isaacsim_extension_paths()

    deps_config = {
        "extends": "pyrightconfig.base.json",
        "extraPaths": extra_paths,
    }

    if args.dry_run:
        print(json.dumps(deps_config, indent=4))
        return

    deps_config_path = Path(__file__).parent.parent / "pyrightconfig.deps.json"
    with deps_config_path.open("w") as f:
        json.dump(deps_config, f, indent=4)
    print(f"Wrote {deps_config_path}")


if __name__ == "__main__":
    main()
