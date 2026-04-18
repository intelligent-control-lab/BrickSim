"""Convert a StableText2Brick-style brick string into bricksim JsonTopology.

Input format (per line):
    hxw (x,y,z)

Example line:
    2x6 (13,12,0)

Output format (Python dict):
    {
      "schema": "bricksim/lego_topology@2",
      "parts": [ ... JsonPart ... ],
      "connections": [ ... JsonConnection ... ],
      "pose_hints": [ ... JsonPoseHint ... ],
    }
"""

import re

from .schema import JsonTopology
from .voxel_grid import Brick, ColorInput, InputColor, bricks_grid_to_topology_json

# Regex for lines like: "2x6 (13,12,0)"
_BRICK_LINE_RE = re.compile(
    r"^\s*(?P<h>\d+)x(?P<w>\d+)\s*"
    r"\(\s*(?P<x>-?\d+)\s*,\s*(?P<y>-?\d+)\s*,\s*(?P<z>-?\d+)\s*\)\s*$"
)


def _parse_bricks_text(bricks_text: str) -> list[Brick]:
    """Parse StableText2Brick 'bricks' text into a list of (h, w, x, y, z).

    Each line: "hxw (x,y,z)".

    Returns:
        Parsed bricks as ``(h, w, x, y, z)`` tuples.
    """
    bricks: list[Brick] = []
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
    """Check the first non-empty line for StableText2Brick brick format.

    Returns:
        ``True`` if the first non-empty line matches the brick-line format.
    """
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        return _BRICK_LINE_RE.match(line) is not None
    return False


def bricks_text_to_topology_json(
    bricks_text: str,
    color: ColorInput = (255, 255, 255),
    *,
    include_base_plate: bool = False,
    base_plate_size: tuple[int, int] | None = None,
    base_plate_color: InputColor | None = None,
) -> JsonTopology:
    """Convert a StableText2Brick brick string into a JsonTopology dict.

    The output matches the C++ bricksim.io.json schema.

    - Parts: one BrickPart per line
      * L = h, W = w, H = 3 plates (one brick tall)
      * color:
          - if `color` is a single (r,g,b) tuple, all bricks share that color;
          - if `color` is a list of (r,g,b), it is applied per brick;
          - if `color` is None, bricks are white (255,255,255).
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
      * yaw = 0 (all bricks axis-aligned in this representation)

    Returns:
        JsonTopology-compatible dictionary.
    """
    bricks = _parse_bricks_text(bricks_text)
    return bricks_grid_to_topology_json(
        bricks,
        color=color,
        include_base_plate=include_base_plate,
        base_plate_size=base_plate_size,
        base_plate_color=base_plate_color,
    )
