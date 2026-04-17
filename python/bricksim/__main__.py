"""Command-line launcher for running BrickSim inside Isaac Sim."""

import argparse
import shlex
import sys
from pathlib import Path


def _uncache_bricksim_package():
    package_name = __package__ or "bricksim"
    package = sys.modules.get(package_name)
    if package is None:
        return
    package_file = getattr(package, "__file__", None)
    if package_file is None:
        return
    package_dir = Path(package_file).resolve().parent
    if package_dir != Path(__file__).resolve().parent:
        return
    sys.modules.pop(package_name, None)


def main():
    """Launch Isaac Sim with BrickSim and optional forwarded script arguments."""
    parser = argparse.ArgumentParser(
        prog="bricksim",
        usage="bricksim [launcher args] [script] [args]",
        description=(
            "Launch Isaac Sim with BrickSim. If [script] is omitted, the "
            "packaged default stage is opened.\n"
            "  --</path/to/key>=<value>: (Kit) Instruct to supersede "
            "configuration key with given value."
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--exp",
        metavar="EXPERIENCE",
        help="(Kit) Isaac Sim experience file (app config).",
    )
    parser.add_argument(
        "--info",
        "-v",
        action="store_true",
        help="(Kit) Show info log output in console.",
    )
    parser.add_argument(
        "--verbose",
        "-vv",
        action="store_true",
        help="(Kit) show verbose log output in console.",
    )
    parser.add_argument(
        "--enable", action="append", metavar="EXT_ID", help="(Kit) Enable extension."
    )
    parser.add_argument(
        "--ext-folder",
        action="append",
        metavar="PATH",
        help="(Kit) Add extension folder to look extensions in.",
    )
    parser.add_argument(
        "--ext-path",
        action="append",
        metavar="PATH",
        help="(Kit) Add direct extension path.",
    )
    parser.add_argument(
        "script",
        nargs="?",
        help="Script path or module to run on startup, optionally with :func suffix.",
    )
    parser.add_argument(
        "args", nargs=argparse.REMAINDER, help="Arguments forwarded to the script."
    )
    args, forwarded = parser.parse_known_args()
    # Forwarded args must start with --/
    unsupported_args = [
        arg for arg in forwarded if (not arg.startswith("--/")) or ("=" not in arg)
    ]
    if unsupported_args:
        parser.error(f"Unsupported launcher arguments: {' '.join(unsupported_args)}")

    exec_args = [str(Path(__file__).resolve().with_name("_entry.py"))]
    if args.script is not None:
        exec_args.append(args.script)
        exec_args.extend(args.args)

    kit_args = []
    kit_args.append(args.exp or "isaacsim.exp.full")
    if args.info:
        kit_args.append("--info")
    if args.verbose:
        kit_args.append("--verbose")
    for ext_id in args.enable or []:
        kit_args.extend(["--enable", ext_id])
    for path in args.ext_folder or []:
        kit_args.extend(["--ext-folder", path])
    for path in args.ext_path or []:
        kit_args.extend(["--ext-path", path])
    kit_args.extend(
        [
            "--/app/content/emptyStageOnStart=false",
            "--/crashreporter/enabled=false",
            "--/log/outputStreamLevel=info",
            "--/log/channels/*=warn",
            "--/log/channels/bricksim=info",
            "--/log/channels/bricksim.*=info",
        ]
    )
    kit_args.extend(forwarded)
    kit_args.append("--exec")
    kit_args.append(" ".join(shlex.quote(arg) for arg in exec_args))

    sys.argv = [sys.argv[0], *kit_args]
    _uncache_bricksim_package()
    import isaacsim

    isaacsim.main()


if __name__ == "__main__":
    main()
