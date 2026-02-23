#!/usr/bin/env python3

# Usage: ./scripts/generate_color_json.py --input=resources/lego_colors_20250311.csv --output=exts/bricksim/bricksim/colors.json
# From https://rebrickable.com/colors/

import argparse
import csv
import json

arg_parser = argparse.ArgumentParser(description="Generate color JSON for LEGO bricks.")
arg_parser.add_argument("--input", type=str, required=True, help="Input CSV file path.")
arg_parser.add_argument("--output", type=str, required=True, help="Output JSON file path.")
args = arg_parser.parse_args()

with open(args.input, "r", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    color_data = {}
    for row in reader:
        color_name = row["name"]
        color_rgb = row["rgb"]
        if color_name.startswith("["):
            continue
        color_rgb = color_rgb.lower()
        color_data[color_name] = color_rgb
with open(args.output, "w", encoding="utf-8") as f:
    json.dump(color_data, f, indent=4)
