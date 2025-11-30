"""
Convert a StableText2Brick-style brick string into lego_assemble JsonTopology.

Input format (per line):
    hxw (x,y,z)

Example line:
    2x6 (13,12,0)

Output format (Python dict):
    {
      "schema": "lego_assemble/lego_topology@1",
      "parts": [ ... JsonPart ... ],
      "connections": [ ... JsonConnection ... ],
      "pose_hints": [ ... JsonPoseHint ... ],
    }
"""

import re
from typing import Any

BRICK_UNIT_LENGTH = 0.008      # meters per stud
PLATE_HEIGHT      = 0.0032     # meters per plate
BRICK_HEIGHT      = 3 * PLATE_HEIGHT  # 1 brick = 3 plates = 0.0096 m

# Regex for lines like: "2x6 (13,12,0)"
_BRICK_LINE_RE = re.compile(
    r"^\s*(?P<h>\d+)x(?P<w>\d+)\s*"
    r"\(\s*(?P<x>-?\d+)\s*,\s*(?P<y>-?\d+)\s*,\s*(?P<z>-?\d+)\s*\)\s*$"
)

SCHEMA_STRING = "lego_assemble/lego_topology@1"

# From BrickPart in your C++:
# static constexpr InterfaceId HoleId = 0;
# static constexpr InterfaceId StudId = 1;
HOLE_IFACE_ID = 0
STUD_IFACE_ID = 1


def _parse_bricks_text(bricks_text: str) -> list[tuple[int, int, int, int, int]]:
    """
    Parse StableText2Brick 'bricks' text into a list of (h, w, x, y, z).

    Each line: "hxw (x,y,z)".
    """
    bricks: list[tuple[int, int, int, int, int]] = []
    for line in bricks_text.splitlines():
        line = line.strip()
        if not line:
            continue
        m = _BRICK_LINE_RE.match(line)
        if not m:
            raise ValueError(f"Cannot parse brick line: {line!r}")
        h = int(m.group("h"))
        w = int(m.group("w"))
        x = int(m.group("x"))
        y = int(m.group("y"))
        z = int(m.group("z"))
        bricks.append((h, w, x, y, z))
    return bricks


def is_bricks_text(text: str) -> bool:
    """
    Heuristic check if the given text is in StableText2Brick 'bricks' format by checking the first non-empty line.
    """
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        return _BRICK_LINE_RE.match(line) is not None
    return False

def bricks_text_to_topology_json(
    bricks_text: str,
    color: tuple[int, int, int] = (255, 255, 255),
    *,
    include_base_plate: bool = False,
    base_plate_size: tuple[int, int] | None = None,
    base_plate_color: tuple[int, int, int] | None = None,
) -> dict[str, Any]:
    """
    Convert a StableText2Brick brick string into a JsonTopology dict
    matching your C++ lego_assemble.io.json schema.

    - Parts: one BrickPart per line
      * L = h, W = w, H = 1 (you can change to BrickHeightPerPlate later)
      * color = [255, 255, 255] (placeholder)
    - Pose hints:
      * exactly one per connected component
      * part origin = bottom center of the brick
      * first layer component (base plate if present, otherwise lowest bricks)
        has z = 0
    - Connections:
      * vertical adjacencies: bricks at z and z+1 with overlapping (x, y)
      * bottom brick provides studs (StudId = 1)
      * top brick provides holes (HoleId = 0)
      * offset = (x_hole_origin - x_stud_origin, y_hole_origin - y_stud_origin)
      * yaw = 0 (all bricks axis-aligned in dataset)
    """
    bricks = _parse_bricks_text(bricks_text)
    num_bricks = len(bricks)

    if num_bricks == 0:
        return {
            "schema": SCHEMA_STRING,
            "parts": [],
            "connections": [],
            "pose_hints": [],
        }

    # Compute grid bounds (dataset is 20x20x20, but we infer from data).
    # x, y, z are discrete grid coordinates in stud / brick layers.
    min_x = min(x for (h, w, x, y, z) in bricks)
    min_y = min(y for (h, w, x, y, z) in bricks)
    min_z = min(z for (h, w, x, y, z) in bricks)
    max_x = max(x + h for (h, w, x, y, z) in bricks)
    max_y = max(y + w for (h, w, x, y, z) in bricks)
    max_z = max(z + 1 for (h, w, x, y, z) in bricks)  # z layer plus one

    # 3D occupancy grid: grid[x][y][z] = brick_id or -1
    grid = [
        [[-1 for _ in range(max_z)] for _ in range(max_y)]
        for _ in range(max_x)
    ]

    for brick_id, (h, w, x, y, z) in enumerate(bricks):
        for dx in range(h):
            for dy in range(w):
                gx = x + dx
                gy = y + dy
                gz = z
                curr = grid[gx][gy][gz]
                if curr != -1 and curr != brick_id:
                    raise ValueError(
                        f"Overlapping bricks at ({gx},{gy},{gz}): "
                        f"{curr} and {brick_id}"
                    )
                grid[gx][gy][gz] = brick_id

    # Build JsonPart list
    parts: list[dict[str, Any]] = []
    pose_hints: list[dict[str, Any]] = []

    pid_offset = 1 if include_base_plate else 0

    # Optional base plate as the first part (id == 0).
    # The base plate is modeled as a 1-plate-high brick whose footprint
    # tightly covers the projected footprint of all bricks by default,
    # unless overridden via base_plate_size.
    px_plate = py_plate = 0.0
    if include_base_plate:
        if base_plate_size is None:
            plate_L = int(max_x - min_x)
            plate_W = int(max_y - min_y)
        else:
            plate_L, plate_W = base_plate_size

        plate_color = base_plate_color if base_plate_color is not None else color
        plate_x = min_x
        plate_y = min_y
        cx_plate = plate_x + 0.5 * plate_L
        cy_plate = plate_y + 0.5 * plate_W
        px_plate = cx_plate * BRICK_UNIT_LENGTH
        py_plate = cy_plate * BRICK_UNIT_LENGTH

        parts.append(
            {
                "id": 0,
                "type": "brick",
                "payload": {
                    "L": int(plate_L),
                    "W": int(plate_W),
                    "H": 1,
                    "color": [plate_color[0], plate_color[1], plate_color[2]],
                },
            }
        )

    for brick_idx, (h, w, x, y, z) in enumerate(bricks):
        pid = brick_idx + pid_offset
        # JsonPart
        parts.append(
            {
                "id": int(pid),
                "type": "brick",
                "payload": {
                    "L": int(h),
                    "W": int(w),
                    "H": 3,
                    "color": [color[0], color[1], color[2]],
                },
            }
        )

        # Center of the brick in grid units (for x, y only).
        cx = x + 0.5 * h
        cy = y + 0.5 * w
        px = cx * BRICK_UNIT_LENGTH
        py = cy * BRICK_UNIT_LENGTH
        # z will be set via a single pose hint per connected component later.

    # Infer vertical stud–hole connections between bricks
    connections = []
    seen_keys = set()

    for gx in range(max_x):
        for gy in range(max_y):
            for gz in range(max_z - 1):
                b_bottom = grid[gx][gy][gz]
                b_top = grid[gx][gy][gz + 1]

                if b_bottom == -1 or b_top == -1 or b_bottom == b_top:
                    continue

                # Use bottom brick as stud, top as hole
                h0, w0, x0, y0, z0 = bricks[b_bottom]
                h1, w1, x1, y1, z1 = bricks[b_top]

                # Dataset: (x, y) = stud closest to origin.
                # BrickPart: interface grid origin is also at the corner closest to origin,
                # so offset is simply (x_hole_origin - x_stud_origin).
                offset_x = x1 - x0
                offset_y = y1 - y0
                yaw = 0  # all bricks axis-aligned; no 90° rotations in this representation

                key = (b_bottom, b_top, offset_x, offset_y, yaw)
                if key in seen_keys:
                    continue
                seen_keys.add(key)

                connections.append(
                    {
                        "stud_id": int(b_bottom + pid_offset),
                        "stud_iface": STUD_IFACE_ID,
                        "hole_id": int(b_top + pid_offset),
                        "hole_iface": HOLE_IFACE_ID,
                        "offset": [int(offset_x), int(offset_y)],
                        "yaw": int(yaw),
                    }
                )

    # Optional connections from the base plate (studs) to the bottom-layer bricks (holes)
    if include_base_plate:
        plate_id = 0
        plate_x = min_x
        plate_y = min_y
        bottom_z = min_z

        for brick_idx, (h, w, x, y, z) in enumerate(bricks):
            if z != bottom_z:
                continue

            offset_x = x - plate_x
            offset_y = y - plate_y
            yaw = 0

            connections.append(
                {
                    "stud_id": int(plate_id),
                    "stud_iface": STUD_IFACE_ID,
                    "hole_id": int(brick_idx + pid_offset),
                    "hole_iface": HOLE_IFACE_ID,
                    "offset": [int(offset_x), int(offset_y)],
                    "yaw": int(yaw),
                }
            )

    # Build pose hints: exactly one per connected component. For each component,
    # we choose an anchor part:
    #   * if the base plate is present in the component, use it as the anchor;
    #   * otherwise, use the lowest brick in that component (min z, then y, then x).
    # The anchor's origin (bottom center) is placed at z = 0.
    pose_hints = []
    num_parts = num_bricks + (1 if include_base_plate else 0)
    if num_parts > 0:
        # Build adjacency list from connections.
        adjacency: list[list[int]] = [[] for _ in range(num_parts)]
        for conn in connections:
            stud_id = int(conn["stud_id"])
            hole_id = int(conn["hole_id"])
            if 0 <= stud_id < num_parts and 0 <= hole_id < num_parts:
                adjacency[stud_id].append(hole_id)
                adjacency[hole_id].append(stud_id)

        visited = [False] * num_parts

        for start in range(num_parts):
            if visited[start]:
                continue

            # Collect one connected component via DFS.
            stack = [start]
            visited[start] = True
            component: list[int] = []
            while stack:
                node = stack.pop()
                component.append(node)
                for nbr in adjacency[node]:
                    if not visited[nbr]:
                        visited[nbr] = True
                        stack.append(nbr)

            # Choose anchor for this component.
            if include_base_plate and 0 in component:
                # Base plate is part of this component: use it as anchor.
                anchor_pid = 0
                anchor_px = px_plate
                anchor_py = py_plate
            else:
                # No base plate: choose the lowest brick in this component.
                best_key = None
                anchor_pid = None
                anchor_px = anchor_py = 0.0
                for pid in component:
                    if pid < pid_offset:
                        # Skip base plate id in this branch (shouldn't happen).
                        continue
                    brick_idx = pid - pid_offset
                    h, w, x, y, z = bricks[brick_idx]
                    key = (z, y, x, brick_idx)
                    if best_key is None or key < best_key:
                        best_key = key
                        anchor_pid = pid
                        cx = x + 0.5 * h
                        cy = y + 0.5 * w
                        anchor_px = cx * BRICK_UNIT_LENGTH
                        anchor_py = cy * BRICK_UNIT_LENGTH

                if anchor_pid is None:
                    # Component has no bricks; skip (should not occur in practice).
                    continue

            pose_hints.append(
                {
                    "part": int(anchor_pid),
                    "pos": [anchor_px, anchor_py, 0.0],
                    "rot": [1.0, 0.0, 0.0, 0.0],  # identity quaternion (wxyz)
                }
            )

    topology = {
        "schema": SCHEMA_STRING,
        "parts": parts,
        "connections": connections,
        "pose_hints": pose_hints,
    }

    return topology
