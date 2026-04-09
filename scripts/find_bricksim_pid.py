#!/usr/bin/env python3

import os
import sys
import time
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent
BRICKSIM_SCRIPT = str(ROOT_DIR / ".venv" / "bin" / "bricksim")
MAX_ATTEMPTS = int(os.environ.get("BRICKSIM_PID_MAX_ATTEMPTS", "300"))
POLL_INTERVAL_SECONDS = float(os.environ.get("BRICKSIM_PID_POLL_INTERVAL_SECONDS", "0.1"))


def _read_cmdline(pid: int) -> list[str] | None:
    try:
        data = Path(f"/proc/{pid}/cmdline").read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None
    if not data:
        return None
    return [arg for arg in data.rstrip(b"\0").decode(errors="replace").split("\0") if arg]


def _is_bricksim_cmd(argv: list[str]) -> bool:
    if BRICKSIM_SCRIPT in argv:
        return True
    for i, arg in enumerate(argv[:-1]):
        if arg == "-m" and argv[i + 1] == "bricksim":
            return True
    return False


def _is_debugpy_wrapper(argv: list[str]) -> bool:
    return any(arg.endswith("/debugpy/launcher") or arg.endswith("/debugpy/adapter") for arg in argv)


def find_matches() -> list[tuple[int, str]]:
    matches: list[tuple[int, str]] = []
    for proc_dir in Path("/proc").iterdir():
        if not proc_dir.name.isdigit():
            continue
        pid = int(proc_dir.name)
        argv = _read_cmdline(pid)
        if argv is None:
            continue
        if not _is_bricksim_cmd(argv):
            continue
        if _is_debugpy_wrapper(argv):
            continue
        matches.append((pid, " ".join(argv)))
    return matches


def main() -> int:
    for _ in range(MAX_ATTEMPTS):
        matches = find_matches()
        if len(matches) == 1:
            print(matches[0][0])
            return 0
        if len(matches) > 1:
            print(f"Expected exactly one BrickSim process, found {len(matches)}:", file=sys.stderr)
            for pid, cmdline in matches:
                print(f"{pid} {cmdline}", file=sys.stderr)
            return 1
        time.sleep(POLL_INTERVAL_SECONDS)

    print(
        f"Timed out waiting for a BrickSim process after {MAX_ATTEMPTS} attempts.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
