#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import numpy as np

from lego_assemble.importers.legolization import (
    _extract_bricks_from_lego_json,
    is_legolization_json,
    legolization_json_to_topology_json,
    load_default_lego_library,
)
from lego_assemble.importers.stabletext2brick import (
    _parse_bricks_text,
    bricks_text_to_topology_json,
    is_bricks_text,
)


def _parse_baseplate(s: str) -> tuple[int, int]:
    w_s, h_s = s.split("x")
    return int(w_s), int(h_s)


def _load_and_convert(
    input_path: Path,
    *,
    input_format: str,
    include_baseplate: bool,
    baseplate_size: tuple[int, int] | None,
) -> tuple[dict, list[tuple[int, int, int, int, int]], int]:
    text = input_path.read_text(encoding="utf-8")

    fmt = input_format
    if fmt == "auto":
        if is_legolization_json(text):
            fmt = "legolization"
        elif is_bricks_text(text):
            fmt = "stabletext2brick"
        else:
            raise ValueError("Could not auto-detect input format.")

    if fmt == "legolization":
        lego_structure = json.loads(text)
        lego_library = load_default_lego_library()
        bricks = _extract_bricks_from_lego_json(lego_structure, lego_library)
        topology = legolization_json_to_topology_json(
            lego_structure,
            lego_library=lego_library,
            include_base_plate=include_baseplate,
            base_plate_size=baseplate_size,
        )
    elif fmt == "stabletext2brick":
        bricks = _parse_bricks_text(text)
        topology = bricks_text_to_topology_json(
            text,
            include_base_plate=include_baseplate,
            base_plate_size=baseplate_size,
        )
    else:
        raise ValueError(f"Unknown format: {fmt}")

    pid_offset = 1 if include_baseplate else 0
    return topology, bricks, pid_offset


def _run_static_solve(topology: dict, solver_path: Path) -> dict:
    topo_text = json.dumps(topology)
    proc = subprocess.run(
        [str(solver_path), "-"],
        input=topo_text,
        text=True,
        stdout=subprocess.PIPE,
        check=True,
    )
    out = proc.stdout
    start = out.find("{")
    if start == -1:
        raise RuntimeError("static_solve produced no JSON on stdout.")
    return json.loads(out[start:])


def _render_heatmap(
    bricks: list[tuple[int, int, int, int, int]],
    pid_offset: int,
    per_part_score: dict[int, float],
    *,
    output_path: Path,
    draw_overutilized: bool,
) -> None:
    min_x = min(x for _, _, x, _, _ in bricks)
    min_y = min(y for _, _, _, y, _ in bricks)
    min_z = min(z for _, _, _, _, z in bricks)
    if min_x < 0 or min_y < 0 or min_z < 0:
        raise ValueError("Negative brick coordinates are not supported for voxel plotting.")

    util_values = [
        util for util in per_part_score.values() if draw_overutilized or util <= 1.0
    ]
    if util_values:
        umin = min(util_values)
        umax = max(util_values)
    else:
        umin = 0.0
        umax = 1.0
    denom = umax - umin

    world_dim = (
        max(x + L for L, _, x, _, _ in bricks),
        max(y + W for _, W, _, y, _ in bricks),
        max(z + 1 for _, _, _, _, z in bricks),
    )

    world_grid = np.zeros(world_dim, dtype=bool)
    heatmap_color = np.zeros((*world_dim, 3), dtype=float)

    for brick_idx, (L, W, x, y, z) in enumerate(bricks):
        pid = brick_idx + pid_offset
        util = per_part_score.get(pid, 0.0)
        if util > 1.0 and not draw_overutilized:
            color = (1.0, 1.0, 1.0)
        else:
            util_scaled = (util - umin) / denom if denom > 0.0 else 0.0
            util_scaled = max(0.0, min(1.0, util_scaled))
            color = (float(util_scaled), 0.0, 0.0)

        for i in range(x, x + L):
            for j in range(y, y + W):
                world_grid[i, j, z] = True
                heatmap_color[i, j, z, :] = color

    import matplotlib.pyplot as plt

    ax = plt.figure().add_subplot(projection="3d")
    ax.voxels(world_grid, facecolors=heatmap_color, edgecolor="k")
    plt.savefig(output_path)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run static_solve and visualize a 3D stability heatmap.")
    parser.add_argument("input", type=Path, help="Input lego json (legolization or StableText2Brick).")
    parser.add_argument(
        "--format",
        choices=["auto", "legolization", "stabletext2brick"],
        default="auto",
        help="Input format (default: auto).",
    )
    parser.add_argument("--baseplate", type=str, default=None, help="Optional baseplate type (e.g., '16x16', '32x32').")
    parser.add_argument("--output", type=Path, default=Path("stability_analysis.png"), help="Output PNG path.")
    parser.add_argument(
        "--draw-overutilized",
        action="store_true",
        help="Draw utilization > 1 with the heatmap scale instead of white.",
    )
    args = parser.parse_args()

    if args.baseplate:
        baseplate_size = _parse_baseplate(args.baseplate)
        include_baseplate = True
    else:
        baseplate_size = None
        include_baseplate = False

    repo_root = Path(__file__).resolve().parents[1]
    solver_path = repo_root / "native/.build/RelWithDebInfo/static_solve"
    if not solver_path.exists():
        raise FileNotFoundError(f"Missing solver binary at {solver_path}")

    topology, bricks, pid_offset = _load_and_convert(
        args.input,
        input_format=args.format,
        include_baseplate=include_baseplate,
        baseplate_size=baseplate_size,
    )
    result = _run_static_solve(topology, solver_path)

    per_part_score: dict[int, float] = {}
    utilizations = result.get("clutch_utilizations", {})
    for conn in topology.get("connections", []):
        cid = int(conn["id"])
        hole_id = int(conn["hole_id"])
        util = float(utilizations.get(str(cid), 0.0))
        per_part_score[hole_id] = max(per_part_score.get(hole_id, 0.0), util)

    _render_heatmap(
        bricks,
        pid_offset,
        per_part_score,
        output_path=args.output,
        draw_overutilized=args.draw_overutilized,
    )


if __name__ == "__main__":
    main()
