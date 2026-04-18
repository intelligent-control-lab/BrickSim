"""CLI for converting supported LEGO structure formats to topology JSON."""

import argparse
import json
from pathlib import Path

from bricksim.topology.legolization import (
    is_legolization_json,
    legolization_json_to_topology_json,
)
from bricksim.topology.stabletext2brick import (
    bricks_text_to_topology_json,
    is_bricks_text,
)


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert legolization or StableText2Brick JSON to topology JSON."
    )
    parser.add_argument(
        "--input", type=Path, required=True, help="Input legolization JSON file"
    )
    parser.add_argument(
        "--output", type=Path, required=True, help="Output topology JSON file"
    )
    parser.add_argument(
        "--format",
        choices=["auto", "legolization", "stabletext2brick"],
        default="auto",
        help="Input format (default: auto)",
    )
    parser.add_argument(
        "--baseplate",
        type=str,
        default=None,
        help="Optional baseplate type (e.g., '16x16', '32x32')",
    )
    return parser


def _parse_baseplate(baseplate: str | None) -> tuple[bool, tuple[int, int] | None]:
    if baseplate is None:
        print("No baseplate specified.")
        return False, None

    width_str, height_str = baseplate.split("x", maxsplit=1)
    baseplate_size = (int(width_str), int(height_str))
    print(f"Using baseplate size: {list(baseplate_size)}")
    return True, baseplate_size


def _detect_input_format(input_text: str, requested_format: str) -> str:
    if requested_format != "auto":
        return requested_format

    if is_legolization_json(input_text):
        return "legolization"
    if is_bricks_text(input_text):
        return "stabletext2brick"
    raise ValueError("Could not auto-detect input format.")


def main() -> int:
    """Run the topology conversion command.

    Returns:
        Process exit code.
    """
    args = _build_argument_parser().parse_args()
    include_baseplate, baseplate_size = _parse_baseplate(args.baseplate)

    input_text = args.input.read_text(encoding="utf-8")
    input_format = _detect_input_format(input_text, args.format)
    print(f"Detected input format: {input_format}")

    if input_format == "legolization":
        topology_json = legolization_json_to_topology_json(
            json.loads(input_text),
            include_base_plate=include_baseplate,
            base_plate_size=baseplate_size,
        )
    elif input_format == "stabletext2brick":
        topology_json = bricks_text_to_topology_json(
            input_text,
            include_base_plate=include_baseplate,
            base_plate_size=baseplate_size,
        )
    else:
        raise ValueError(f"Unknown format: {input_format}")

    print(f"{len(topology_json['parts'])} parts.")
    print(f"{len(topology_json['connections'])} connections.")
    print(f"{len(topology_json['pose_hints'])} pose hints.")
    print(f"Writing topology JSON to {args.output}")
    args.output.write_text(json.dumps(topology_json, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
