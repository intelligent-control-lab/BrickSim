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

def bricks_text_to_topology_json(bricks_text: str, color: tuple[int, int, int] = (255, 255, 255)) -> dict[str, Any]:
    """
    Convert a StableText2Brick brick string into a JsonTopology dict
    matching your C++ lego_assemble.io.json schema.

    - Parts: one BrickPart per line
      * L = h, W = w, H = 1 (you can change to BrickHeightPerPlate later)
      * color = [255, 255, 255] (placeholder)
    - Pose hints:
      * pos = [x, y, z] in grid units
      * rot = [1, 0, 0, 0] (identity quaternion)
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

    # Compute grid bounds (dataset is 20x20x20, but we infer from data)
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
    parts = []
    pose_hints = []

    for pid, (h, w, x, y, z) in enumerate(bricks):
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

        # Center of the brick in grid units
        cx = x + 0.5 * h
        cy = y + 0.5 * w
        cz = z + 0.5   # center of a 1-layer-tall brick

        # Convert to meters
        px = cx * BRICK_UNIT_LENGTH
        py = cy * BRICK_UNIT_LENGTH
        pz = cz * BRICK_HEIGHT

        pose_hints.append(
            {
                "part": int(pid),
                "pos": [px, py, pz],
                "rot": [1.0, 0.0, 0.0, 0.0],  # identity quaternion (wxyz)
            }
        )

    # Infer vertical stud–hole connections
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
                        "stud_id": int(b_bottom),
                        "stud_iface": STUD_IFACE_ID,
                        "hole_id": int(b_top),
                        "hole_iface": HOLE_IFACE_ID,
                        "offset": [int(offset_x), int(offset_y)],
                        "yaw": int(yaw),
                    }
                )

    topology = {
        "schema": SCHEMA_STRING,
        "parts": parts,
        "connections": connections,
        "pose_hints": pose_hints,
    }

    return topology
