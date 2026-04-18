#!/usr/bin/env python3

import argparse
import json
import subprocess
from pathlib import Path

import numpy as np

from bricksim.topology.legolization import (
    _extract_bricks_from_lego_json,
    is_legolization_json,
    legolization_json_to_topology_json,
    load_default_lego_library,
)
from bricksim.topology.stabletext2brick import (
    _parse_bricks_text,
    bricks_text_to_topology_json,
    is_bricks_text,
)

def set_axes_equal_3d(ax, world_dim):
    x_dim, y_dim, z_dim = world_dim
    # Tight bounds so the object fills the viewport instead of being padded
    # by a cubic equal-range box.
    ax.set_xlim(0.0, float(x_dim))
    ax.set_ylim(0.0, float(y_dim))
    ax.set_zlim(0.0, float(z_dim))
    ax.set_proj_type("ortho")
    ax.set_box_aspect((x_dim, y_dim, z_dim))



def _parse_baseplate(s: str) -> tuple[int, int]:
    w_s, h_s = s.split("x")
    return int(w_s), int(h_s)


def _parse_viewpoint(s: str) -> tuple[float, float, float]:
    vals = s.split(",")
    if len(vals) != 3:
        raise ValueError(
            "Invalid --viewpoint format. Expected 'elevation,azimith,roll'."
        )
    try:
        elev = float(vals[0].strip())
        azim = float(vals[1].strip())
        roll = float(vals[2].strip())
    except ValueError as e:
        raise ValueError(
            "Invalid --viewpoint values. Expected 'elevation,azimith,roll' as floats."
        ) from e
    return elev, azim, roll


def _apply_viewpoint(ax, viewpoint: tuple[float, float, float]) -> None:
    elev, azim, roll = viewpoint
    try:
        ax.view_init(elev=elev, azim=azim, roll=roll)
    except TypeError:
        ax.view_init(elev=elev, azim=azim)


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


def _derive_heatmap_output_path(base_path: Path, *, suffix: str = "") -> Path:
    stem = base_path.stem
    if suffix:
        stem = f"{stem}{suffix}"
    return base_path.with_name(f"{stem}.npy")


def _resolve_util_scale(
    per_part_score: dict[int, float],
    *,
    scale_range: tuple[float, float] | None,
) -> tuple[float, float]:
    return 0.0, 1.0


def _build_fixed_heatmap(
    bricks: list[tuple[int, int, int, int, int]],
    pid_offset: int,
    per_part_score: dict[int, float],
    *,
    cmap_name: str,
    scale_range: tuple[float, float] | None = None,
    fixed_shape: tuple[int, int, int] = (20, 20, 20),
) -> np.ndarray:
    import matplotlib.pyplot as plt

    umin, umax = _resolve_util_scale(
        per_part_score,
        scale_range=scale_range,
    )
    denom = umax - umin
    cmap = plt.get_cmap(cmap_name)
    out = np.zeros((*fixed_shape, 3), dtype=float)

    for brick_idx, (L, W, x, y, z) in enumerate(bricks):
        pid = brick_idx + pid_offset
        util = per_part_score.get(pid, 0.0)
        if util >= 1.0:
            color = (1.0, 1.0, 1.0)
        else:
            util_scaled = (util - umin) / denom if denom > 0.0 else 0.0
            util_scaled = max(0.0, min(1.0, util_scaled))
            color = cmap(float(util_scaled))[:3]

        for i in range(x, x + L):
            for j in range(y, y + W):
                if not (0 <= i < fixed_shape[0] and 0 <= j < fixed_shape[1] and 0 <= z < fixed_shape[2]):
                    raise ValueError(
                        f"Brick voxel {(i, j, z)} is outside fixed heatmap shape {fixed_shape}"
                    )
                out[i, j, z, :] = color
    return out


def _render_heatmap(
    bricks: list[tuple[int, int, int, int, int]],
    pid_offset: int,
    per_part_score: dict[int, float],
    *,
    output_path: Path | None,
    cmap_name: str,
    interactive: bool,
    ax=None,
    title: str | None = None,
    scale_range: tuple[float, float] | None = None,
    viewpoint: tuple[float, float, float] = (0.0, -90.0, 0.0),
) -> np.ndarray:
    min_x = min(x for _, _, x, _, _ in bricks)
    min_y = min(y for _, _, _, y, _ in bricks)
    min_z = min(z for _, _, _, _, z in bricks)
    max_x = max(x + L for L, _, x, _, _ in bricks)
    max_y = max(y + W for _, W, _, y, _ in bricks)
    max_z = max(z + 1 for _, _, _, _, z in bricks)

    umin, umax = _resolve_util_scale(
        per_part_score,
        scale_range=scale_range,
    )
    denom = umax - umin

    world_dim = (
        max_x - min_x,
        max_y - min_y,
        max_z - min_z,
    )

    world_grid = np.zeros(world_dim, dtype=bool)
    heatmap_color = np.zeros((*world_dim, 3), dtype=float)

    import matplotlib.pyplot as plt

    cmap = plt.get_cmap(cmap_name)

    for brick_idx, (L, W, x, y, z) in enumerate(bricks):
        pid = brick_idx + pid_offset
        util = per_part_score.get(pid, 0.0)
        if util >= 1.0:
            color = (1.0, 1.0, 1.0)
        else:
            util_scaled = (util - umin) / denom if denom > 0.0 else 0.0
            util_scaled = max(0.0, min(1.0, util_scaled))
            color = cmap(float(util_scaled))[:3]

        x0 = x - min_x
        y0 = y - min_y
        z0 = z - min_z
        for i in range(x0, x0 + L):
            for j in range(y0, y0 + W):
                world_grid[i, j, z0] = True
                heatmap_color[i, j, z0, :] = color

    own_axis = ax is None
    if own_axis:
        ax = plt.figure().add_subplot(projection="3d")
    ax.voxels(world_grid, facecolors=heatmap_color, edgecolor="k")
    set_axes_equal_3d(ax, world_dim)
    _apply_viewpoint(ax, viewpoint)
    ax.set_axis_off()
    if title is not None:
        ax.set_title(title)
    if own_axis:
        plt.tight_layout()
        if output_path is not None:
            plt.savefig(output_path, dpi=300, bbox_inches="tight")
        if interactive:
            plt.show()
    return heatmap_color


def _sync_3d_view(fig, ax_a, ax_b) -> None:
    syncing = False

    def _copy_view(src, dst) -> None:
        nonlocal syncing
        if syncing:
            return
        syncing = True
        roll = getattr(src, "roll", None)
        if roll is None:
            dst.view_init(elev=src.elev, azim=src.azim)
        else:
            dst.view_init(elev=src.elev, azim=src.azim, roll=roll)
        dst.set_xlim3d(src.get_xlim3d())
        dst.set_ylim3d(src.get_ylim3d())
        dst.set_zlim3d(src.get_zlim3d())
        fig.canvas.draw_idle()
        syncing = False

    def _on_motion(event) -> None:
        if event.inaxes is ax_a:
            _copy_view(ax_a, ax_b)
        elif event.inaxes is ax_b:
            _copy_view(ax_b, ax_a)

    fig.canvas.mpl_connect("motion_notify_event", _on_motion)
    _copy_view(ax_a, ax_b)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run static_solve and visualize a 3D stability heatmap.")
    parser.add_argument("input", type=Path, help="Input lego json (legolization or StableText2Brick).")
    parser.add_argument(
        "--format",
        choices=["auto", "legolization", "stabletext2brick"],
        default="auto",
        help="Input format (default: auto).",
    )
    parser.add_argument("--baseplate", type=str, default='32x32', help="Optional baseplate type (e.g., '16x16', '32x32').")
    parser.add_argument("--output", type=Path, default=None, help="Output PNG path.")
    parser.add_argument(
        "--cmap",
        type=str,
        default="viridis",
        help="Matplotlib colormap name for utilization visualization (default: viridis).",
    )
    parser.add_argument(
        "--compare",
        action="store_true",
        help="Run both static_solve and StableLego and render side-by-side comparison.",
    )
    parser.add_argument(
        "--viewpoint",
        type=str,
        default="0,-90,0",
        help="3D viewpoint as 'elevation,azimith,roll' (default: 0,-90,0).",
    )
    parser.add_argument("--interactive", action="store_true")
    args = parser.parse_args()
    viewpoint = _parse_viewpoint(args.viewpoint)

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

    compare_input_text: str | None = None
    if args.compare:
        compare_input_text = args.input.read_text(encoding="utf-8")
        if args.format == "stabletext2brick" or not is_legolization_json(compare_input_text):
            raise ValueError("--compare requires legolization JSON input.")

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

    npy_base_path = args.output if args.output is not None else args.input

    if not args.compare:
        _render_heatmap(
            bricks,
            pid_offset,
            per_part_score,
            output_path=args.output,
            cmap_name=args.cmap,
            interactive=args.interactive,
            viewpoint=viewpoint,
        )
        np.save(
            _derive_heatmap_output_path(npy_base_path),
            _build_fixed_heatmap(
                bricks,
                pid_offset,
                per_part_score,
                cmap_name=args.cmap,
            ),
        )
        return

    assert compare_input_text is not None
    lego_structure = json.loads(compare_input_text)
    from bricksim.stable_lego import run_stable_lego

    stable_world_dim = (
        max(x + L for L, _, x, _, _ in bricks) + 1,
        max(y + W for _, W, _, y, _ in bricks) + 1,
        max(z + 1 for _, _, _, _, z in bricks) + 1,
    )
    stable_utilization, _, _, _, _ = run_stable_lego(
        lego_structure,
        world_dim=stable_world_dim,
        brick_library=load_default_lego_library(),
    )
    stable_per_part_score: dict[int, float] = {}
    for brick_idx, (L, W, x, y, z) in enumerate(bricks):
        pid = brick_idx + pid_offset
        stable_per_part_score[pid] = float(
            np.max(stable_utilization[x : x + L, y : y + W, z])
        )

    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(12, 6))
    ax_left = fig.add_subplot(1, 2, 1, projection="3d")
    ax_right = fig.add_subplot(1, 2, 2, projection="3d")
    shared_scale = (0.0, 1.0)
    _render_heatmap(
        bricks,
        pid_offset,
        per_part_score,
        output_path=None,
        cmap_name=args.cmap,
        interactive=False,
        ax=ax_left,
        title="Ours",
        scale_range=shared_scale,
        viewpoint=viewpoint,
    )
    _render_heatmap(
        bricks,
        pid_offset,
        stable_per_part_score,
        output_path=None,
        cmap_name=args.cmap,
        interactive=False,
        ax=ax_right,
        title="StableLego",
        scale_range=shared_scale,
        viewpoint=viewpoint,
    )
    np.save(
        _derive_heatmap_output_path(npy_base_path, suffix="_ours"),
        _build_fixed_heatmap(
            bricks,
            pid_offset,
            per_part_score,
            cmap_name=args.cmap,
            scale_range=shared_scale,
        ),
    )
    np.save(
        _derive_heatmap_output_path(npy_base_path, suffix="_stablelego"),
        _build_fixed_heatmap(
            bricks,
            pid_offset,
            stable_per_part_score,
            cmap_name=args.cmap,
            scale_range=shared_scale,
        ),
    )
    import matplotlib.colors as mcolors
    import matplotlib.cm as cm

    norm = mcolors.Normalize(vmin=shared_scale[0], vmax=shared_scale[1])
    mappable = cm.ScalarMappable(norm=norm, cmap=plt.get_cmap(args.cmap))
    mappable.set_array([])
    # Explicit placement keeps the two 3D panels tightly packed.
    ax_left.set_position([0.00, 0.04, 0.45, 0.90])
    ax_right.set_position([0.43, 0.04, 0.45, 0.90])
    cax = fig.add_axes([0.90, 0.15, 0.02, 0.70])
    fig.colorbar(mappable, cax=cax, orientation="vertical", label="Utilization")
    _sync_3d_view(fig, ax_left, ax_right)
    if args.output is not None:
        plt.savefig(args.output, dpi=300, bbox_inches="tight", pad_inches=0.0)
    if args.interactive:
        plt.show()


if __name__ == "__main__":
    main()
