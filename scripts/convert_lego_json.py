#!/usr/bin/env python3

import argparse
import json
from lego_assemble.importers.stabletext2brick import bricks_text_to_topology_json, is_bricks_text
from lego_assemble.importers.legolization import legolization_json_to_topology_json, is_legolization_json

parser = argparse.ArgumentParser(description="Convert legolization or StableText2Brick JSON to topology JSON.")
parser.add_argument("--input", required=True, help="Input legolization JSON file")
parser.add_argument("--output", required=True, help="Output topology JSON file")
parser.add_argument("--format", choices=["auto", "legolization", "stabletext2brick"], default="auto", help="Input format (default: auto)")
parser.add_argument("--baseplate", type=str, default=None, help="Optional baseplate type (e.g., '16x16', '32x32')")
args = parser.parse_args()
if args.baseplate:
    w = int(args.baseplate.split("x")[0])
    h = int(args.baseplate.split("x")[1])
    include_baseplate = True
    baseplate_size = [w, h]
    print(f"Using baseplate size: {baseplate_size}")
else:
    include_baseplate = False
    baseplate_size = None
    print("No baseplate specified.")
with open(args.input, "r", encoding="utf-8") as f:
    input_text = f.read()
format = args.format
if format == "auto":
    if is_legolization_json(input_text):
        format = "legolization"
    elif is_bricks_text(input_text):
        format = "stabletext2brick"
    else:
        raise ValueError("Could not auto-detect input format.")
print(f"Detected input format: {format}")
if format == "legolization":
    topology_json = legolization_json_to_topology_json(json.loads(input_text), include_base_plate=include_baseplate, base_plate_size=baseplate_size)
elif format == "stabletext2brick":
    topology_json = bricks_text_to_topology_json(input_text, include_base_plate=include_baseplate, base_plate_size=baseplate_size)
else:
    raise ValueError(f"Unknown format: {format}")
print(f"{len(topology_json['parts'])} parts.")
print(f"{len(topology_json['connections'])} connections.")
print(f"{len(topology_json['pose_hints'])} pose hints.")
print(f"Writing topology JSON to {args.output}")
with open(args.output, "w", encoding="utf-8") as f:
    json.dump(topology_json, f, indent=2)
