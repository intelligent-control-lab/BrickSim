"""
Convert a legolization-style LEGO JSON structure (task_graph.json)
into a bricksim JsonTopology.

Input format (per brick, in JSON):
    {
      "brick_id": int,   # key into lego_library.json
      "x": int,          # grid x (studs)
      "y": int,          # grid y (studs)
      "z": int,          # grid z (brick layers)
      "ori": int         # 0 or 1, footprint orientation
    }

Example structure:
    {
      "1": {"brick_id": 10, "x": 11, "y": 10, "z": 0, "ori": 0},
      "2": {"brick_id":  9, "x": 11, "y":  8, "z": 0, "ori": 0},
      ...
    }

Output format (Python dict):
    {
      "schema": "bricksim/lego_topology@2",
      "parts": [ ... JsonPart ... ],
      "connections": [ ... JsonConnection ... ],
      "pose_hints": [ ... JsonPoseHint ... ],
    }

This mirrors the behavior of stabletext2brick.bricks_text_to_topology_json,
but takes the legolization JSON + lego_library.json instead of text lines.
"""

import os
import json
from typing import Any

from .grid_topology import bricks_grid_to_topology_json

DEFAULT_LEGO_LIBRARY_PATH = os.path.join(os.path.dirname(__file__), "lego_library.json")
DEFAULT_LEGO_LIBRARY = None

def load_default_lego_library() -> dict[str, dict[str, Any]]:
    global DEFAULT_LEGO_LIBRARY
    if DEFAULT_LEGO_LIBRARY is None:
        if not os.path.exists(DEFAULT_LEGO_LIBRARY_PATH):
            raise FileNotFoundError(f"Default lego library not found at {DEFAULT_LEGO_LIBRARY_PATH}\nPlease run scripts/download_legosim_dataset.py to download it.")
        with open(DEFAULT_LEGO_LIBRARY_PATH, "r", encoding="utf-8") as f:
            DEFAULT_LEGO_LIBRARY = json.load(f)
    return DEFAULT_LEGO_LIBRARY


def is_legolization_json(text: str) -> bool:
    """
    Lightweight heuristic check whether the given string looks like a
    legolization-style task_graph.json.

    Similar spirit to is_bricks_text: we only look at the top-level JSON
    object and the first digit key we find, and we do not validate types
    beyond basic structure.
    """
    try:
        obj = json.loads(text)
    except Exception:
        return False

    if not isinstance(obj, dict):
        return False

    for k, v in obj.items():
        if isinstance(k, str) and k.isdigit() and isinstance(v, dict):
            # Require the usual fields to exist, but don't enforce types.
            required = {"brick_id", "x", "y", "z", "ori"}
            return required.issubset(v.keys())

    return False


def _extract_bricks_from_lego_json(
    lego_structure: dict[str, Any],
    lego_library: dict[str, dict[str, Any]],
) -> list[tuple[int, int, int, int, int]]:
    """
    Convert a legolization JSON structure into a list of (h, w, x, y, z).

    lego_structure:
        Dict[str, dict] with keys "1", "2", ... and values containing
        "brick_id", "x", "y", "z", "ori".
    lego_library:
        Dict[str, dict] keyed by brick_id string with fields:
            "height": int, "width": int, ...

    Returns:
        List of (h, w, x, y, z) where h and w are footprint dimensions in studs.
    """
    bricks: list[tuple[int, int, int, int, int]] = []

    # Follow LegoStructure.from_json: only use digit keys and sort them.
    items = [(int(k), v) for k, v in lego_structure.items() if k.isdigit()]
    items.sort(key=lambda kv: kv[0])

    for _, brick in items:
        brick_id = brick["brick_id"]
        x = int(brick["x"])
        y = int(brick["y"])
        z = int(brick["z"])
        ori = int(brick["ori"])

        brick_type = lego_library[str(brick_id)]
        h = int(brick_type["height"])
        w = int(brick_type["width"])

        # ori == 1 means footprint is rotated 90°: swap height/width.
        if ori == 1:
            h, w = w, h

        bricks.append((h, w, x, y, z))

    return bricks


def legolization_json_to_topology_json(
    lego_structure: dict[str, Any],
    lego_library: dict[str, dict[str, Any]] | None = None,
    color: tuple[int, int, int] | list[tuple[int, int, int]] | None = (255, 255, 255),
    *,
    include_base_plate: bool = False,
    base_plate_size: tuple[int, int] | None = None,
    base_plate_color: tuple[int, int, int] | None = None,
) -> dict[str, Any]:
    """
    Convert a legolization LEGO JSON (task_graph-style) into a JsonTopology dict
    matching bricksim.io.json.JsonTopology.

    - Parts:
        * one BrickPart per brick in lego_structure
        * L = footprint length (studs) along x
        * W = footprint width  (studs) along y
        * H = 3 (one brick tall; same convention as StableText2Brick importer)
        * color:
            - if `color` is a single (r,g,b) tuple, all bricks share that color;
            - if `color` is a list of (r,g,b), it is applied per brick;
            - if `color` is None, bricks are white (255,255,255).

    - Connections:
        * vertical adjacencies: bricks occupying (x, y, z) and (x, y, z+1)
        * bottom brick provides studs  (StudId = 1)
        * top brick    provides holes  (HoleId = 0)
        * offset = (x_hole_origin - x_stud_origin, y_hole_origin - y_stud_origin)
        * yaw = 0
          (we currently do not encode per-brick orientation ori into yaw)

    - Pose hints:
        * exactly one anchor per connected component (by connections)
        * if a base plate is present and connected, it is the anchor;
          otherwise, the lowest brick in the component is used.
        * anchor origin (bottom center) is placed at z = 0.
    """
    if lego_library is None:
        lego_library = load_default_lego_library()
    bricks = _extract_bricks_from_lego_json(lego_structure, lego_library)
    return bricks_grid_to_topology_json(
        bricks,
        color=color,
        include_base_plate=include_base_plate,
        base_plate_size=base_plate_size,
        base_plate_color=base_plate_color,
    )
