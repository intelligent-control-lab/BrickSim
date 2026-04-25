#!/usr/bin/env python3
"""Generate the LEGO color-name to RGB JSON mapping."""

# Usage:
#   ./scripts/generate_color_json.py \
#       --input=resources/lego_colors_20250311.csv \
#       --output=python/bricksim/colors.json
# From https://rebrickable.com/colors/

from argparse import ArgumentParser
from csv import DictReader
from json import dump

arg_parser = ArgumentParser(description="Generate color JSON for LEGO bricks.")
arg_parser.add_argument("--input", type=str, required=True, help="Input CSV file path.")
arg_parser.add_argument(
    "--output", type=str, required=True, help="Output JSON file path."
)
args = arg_parser.parse_args()

with open(args.input, "r", encoding="utf-8") as f:
    reader = DictReader(f)
    color_data: dict[str, str] = {}
    for row in reader:
        color_name = row["name"]
        color_rgb = row["rgb"]
        if color_name.startswith("["):
            continue
        color_rgb = color_rgb.lower()
        color_data[color_name] = color_rgb
with open(args.output, "w", encoding="utf-8") as f:
    dump(color_data, f, indent=4)
