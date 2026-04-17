"""Color lookup and parsing helpers for BrickSim brick assets."""

import json
from pathlib import Path

_colors_path = Path(__file__).parent / "colors.json"
with _colors_path.open("r", encoding="utf-8") as f:
    Colors = dict(sorted(json.load(f).items()))


def parse_color(name: str) -> tuple[int, int, int]:
    """Parse a named or hex RGB color.

    Args:
        name: Color name from ``colors.json`` or ``#RRGGBB`` hex string.

    Returns:
        RGB tuple with integer channels in ``[0, 255]``.
    """
    if name.startswith("#") and (len(name) == 7):
        hex = name[1:]
    elif name in Colors:
        hex = Colors[name]
    else:
        raise ValueError(f"Unknown color name: {name}")
    return (int(hex[0:2], 16), int(hex[2:4], 16), int(hex[4:6], 16))
