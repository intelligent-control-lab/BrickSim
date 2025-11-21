#!/usr/bin/env python3
"""
Export a light index + brick text files from AvaLovelace/StableText2Brick.

Usage:
    python export_stabletext2brick_index.py /path/to/output_dir

It will create:

    /path/to/output_dir/
        index.json                     # [{"uuid": "...", "caption": "..."}, ...]
        structures/
            <uuid>.txt                # raw bricks text for that structure
"""

import sys
import json
from pathlib import Path

from datasets import load_dataset  # pip install datasets


def main() -> None:
    if len(sys.argv) != 2:
        print(
            "Usage: python export_stabletext2brick_index.py /path/to/output_dir",
            file=sys.stderr,
        )
        sys.exit(1)

    out_dir = Path(sys.argv[1])
    structures_dir = out_dir / "structures"

    out_dir.mkdir(parents=True, exist_ok=True)
    structures_dir.mkdir(parents=True, exist_ok=True)

    # Stream the dataset so we don't load everything into memory at once.
    dataset = load_dataset(
        "AvaLovelace/StableText2Brick",
        split="train",
        streaming=True,
    )

    index = []
    count = 0

    for ex in dataset:
        # Fields in the dataset
        structure_id = ex["structure_id"]   # UUID-like string
        captions = ex.get("captions") or []
        caption = captions[0] if captions else ""

        bricks_text = ex["bricks"]

        # Save brick text
        txt_path = structures_dir / f"{structure_id}.txt"
        txt_path.write_text(bricks_text, encoding="utf-8")

        # Add to index (only uuid + caption)
        index.append(
            {
                "uuid": structure_id,
                "caption": caption,
            }
        )

        count += 1
        if count % 1000 == 0:
            print(f"Processed {count} examples...", file=sys.stderr)

    # Write index.json
    index_path = out_dir / "index.json"
    index_path.write_text(
        json.dumps(index, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"Done. Wrote {count} entries to {index_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
