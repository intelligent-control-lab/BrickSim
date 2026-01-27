import json
from pathlib import Path

_colors_path = Path(__file__).parent / "colors.json"
with _colors_path.open("r", encoding="utf-8") as f:
    Colors = dict(sorted(json.load(f).items()))

def parse_color(name: str) -> tuple[int, int, int]:
    if name.startswith("#") and (len(name) == 7):
        hex = name[1:]
    elif name in Colors:
        hex = Colors[name]
    else:
        raise ValueError(f"Unknown color name: {name}")
    return (int(hex[0:2], 16), int(hex[2:4], 16), int(hex[4:6], 16))
