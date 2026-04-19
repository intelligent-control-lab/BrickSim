#!/usr/bin/env python3

import argparse
import json
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

from bricksim.topology.legolization import (
    is_legolization_json,
    legolization_json_to_topology_json,
    load_default_lego_library,
)
from bricksim.topology.stabletext2brick import (
    bricks_text_to_topology_json,
    is_bricks_text,
)


@dataclass
class EvalEntry:
    label: str
    meta: dict[str, Any]


@dataclass
class JsonMemberSpan:
    key: str
    key_start: int
    value_start: int
    value_end: int
    has_comma: bool


@dataclass
class JsonObjectSpan:
    members: list[JsonMemberSpan]
    close_index: int


def _parse_baseplate(s: str) -> tuple[int, int]:
    w_s, h_s = s.split("x")
    return int(w_s), int(h_s)


def _load_and_convert(
    structure_path: Path,
    *,
    input_format: str,
    include_baseplate: bool,
    baseplate_size: tuple[int, int],
    lego_library: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    text = structure_path.read_text(encoding="utf-8")

    fmt = input_format
    if fmt == "auto":
        if is_legolization_json(text):
            fmt = "legolization"
        elif is_bricks_text(text):
            fmt = "stabletext2brick"
        else:
            raise ValueError(f"Could not auto-detect input format: {structure_path}")

    if fmt == "legolization":
        lego_structure = json.loads(text)
        return legolization_json_to_topology_json(
            lego_structure,
            lego_library=lego_library,
            include_base_plate=include_baseplate,
            base_plate_size=baseplate_size,
        )
    if fmt == "stabletext2brick":
        return bricks_text_to_topology_json(
            text,
            include_base_plate=include_baseplate,
            base_plate_size=baseplate_size,
        )
    raise ValueError(f"Unknown format: {fmt}")


def _run_static_solve(topology: dict[str, Any], solver_path: Path) -> dict[str, Any]:
    proc = subprocess.run(
        [str(solver_path), "-"],
        input=json.dumps(topology),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        stderr = proc.stderr.strip()
        raise RuntimeError(
            f"static_solve failed with exit code {proc.returncode}: "
            f"{stderr if stderr else '<no stderr>'}"
        )
    out = proc.stdout
    start = out.find("{")
    if start == -1:
        raise RuntimeError("static_solve produced no JSON on stdout.")
    return json.loads(out[start:])


def _extract_solver_result(
    solve_result: dict[str, Any],
) -> tuple[bool, bool, float]:
    stable = solve_result.get("stable")
    solved = solve_result.get("solved")
    time_s = solve_result.get("time_s")
    if not isinstance(stable, bool):
        raise TypeError(f"Invalid static_solve output: stable={stable!r}")
    if not isinstance(solved, bool):
        raise TypeError(f"Invalid static_solve output: solved={solved!r}")
    if not isinstance(time_s, (int, float)):
        raise TypeError(f"Invalid static_solve output: time_s={time_s!r}")
    return stable, solved, float(time_s)


def _iter_eval_entries(node: Any, prefix: tuple[str, ...] = ()) -> Iterator[EvalEntry]:
    if isinstance(node, dict):
        if "json_fname" in node:
            label = "/".join(prefix) if prefix else "<root>"
            yield EvalEntry(label=label, meta=node)
            return
        for k, v in node.items():
            yield from _iter_eval_entries(v, prefix + (str(k),))
        return
    if isinstance(node, list):
        for i, item in enumerate(node):
            yield from _iter_eval_entries(item, prefix + (str(i),))


def _resolve_structure_path(datarootdir: Path, json_fname: str) -> Path:
    return datarootdir / json_fname.lstrip("/")


def _resolve_bricksim_dataset_path() -> Path:
    dataset_path = os.environ.get("BRICKSIM_DATASET_PATH")
    if dataset_path:
        return Path(dataset_path).expanduser()
    env_xdg_cache_home = os.environ.get("XDG_CACHE_HOME")
    if env_xdg_cache_home:
        cache_home = Path(env_xdg_cache_home).expanduser()
    else:
        cache_home = Path.home() / ".cache"
    return cache_home / "bricksim" / "bricksim_dataset"


def _as_optional_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value)
    raise TypeError(f"gt_stable must be bool or int, got {type(value).__name__}")


class _JsonSpanParser:
    def __init__(self, text: str) -> None:
        self.text = text
        self.i = 0
        self.entry_objects: list[JsonObjectSpan] = []

    def parse(self) -> list[JsonObjectSpan]:
        self._skip_ws()
        self._parse_value()
        self._skip_ws()
        if self.i != len(self.text):
            raise ValueError(f"Unexpected trailing data at index {self.i}")
        return self.entry_objects

    def _skip_ws(self) -> None:
        n = len(self.text)
        while self.i < n and self.text[self.i] in (" ", "\t", "\r", "\n"):
            self.i += 1

    def _parse_value(self) -> None:
        if self.i >= len(self.text):
            raise ValueError("Unexpected EOF while parsing JSON value")
        ch = self.text[self.i]
        if ch == "{":
            self._parse_object()
        elif ch == "[":
            self._parse_array()
        elif ch == '"':
            self._parse_string()
        elif ch == "-" or ch.isdigit():
            self._parse_number()
        elif self.text.startswith("true", self.i):
            self.i += 4
        elif self.text.startswith("false", self.i):
            self.i += 5
        elif self.text.startswith("null", self.i):
            self.i += 4
        else:
            raise ValueError(f"Invalid JSON token at index {self.i}: {self.text[self.i:self.i+16]!r}")

    def _parse_string(self) -> str:
        if self.text[self.i] != '"':
            raise ValueError(f"Expected string at index {self.i}")
        start = self.i
        self.i += 1
        n = len(self.text)
        while self.i < n:
            ch = self.text[self.i]
            if ch == "\\":
                self.i += 2
                continue
            if ch == '"':
                self.i += 1
                return json.loads(self.text[start:self.i])
            self.i += 1
        raise ValueError("Unterminated JSON string")

    def _parse_number(self) -> None:
        n = len(self.text)
        if self.text[self.i] == "-":
            self.i += 1
        if self.i >= n:
            raise ValueError("Invalid number")
        if self.text[self.i] == "0":
            self.i += 1
        else:
            if not self.text[self.i].isdigit():
                raise ValueError("Invalid number")
            while self.i < n and self.text[self.i].isdigit():
                self.i += 1
        if self.i < n and self.text[self.i] == ".":
            self.i += 1
            if self.i >= n or not self.text[self.i].isdigit():
                raise ValueError("Invalid number")
            while self.i < n and self.text[self.i].isdigit():
                self.i += 1
        if self.i < n and self.text[self.i] in ("e", "E"):
            self.i += 1
            if self.i < n and self.text[self.i] in ("+", "-"):
                self.i += 1
            if self.i >= n or not self.text[self.i].isdigit():
                raise ValueError("Invalid number")
            while self.i < n and self.text[self.i].isdigit():
                self.i += 1

    def _parse_array(self) -> None:
        if self.text[self.i] != "[":
            raise ValueError(f"Expected '[' at index {self.i}")
        self.i += 1
        self._skip_ws()
        if self.i < len(self.text) and self.text[self.i] == "]":
            self.i += 1
            return
        while True:
            self._parse_value()
            self._skip_ws()
            if self.i >= len(self.text):
                raise ValueError("Unexpected EOF in array")
            ch = self.text[self.i]
            if ch == ",":
                self.i += 1
                self._skip_ws()
                continue
            if ch == "]":
                self.i += 1
                return
            raise ValueError(f"Expected ',' or ']' at index {self.i}")

    def _parse_object(self) -> None:
        if self.text[self.i] != "{":
            raise ValueError(f"Expected '{{' at index {self.i}")
        self.i += 1
        self._skip_ws()
        members: list[JsonMemberSpan] = []
        has_json_fname = False
        if self.i < len(self.text) and self.text[self.i] == "}":
            close_index = self.i
            self.i += 1
            if has_json_fname:
                self.entry_objects.append(JsonObjectSpan(members=members, close_index=close_index))
            return
        while True:
            key_start = self.i
            key = self._parse_string()
            self._skip_ws()
            if self.i >= len(self.text) or self.text[self.i] != ":":
                raise ValueError(f"Expected ':' at index {self.i}")
            self.i += 1
            self._skip_ws()
            value_start = self.i
            self._parse_value()
            value_end = self.i
            if key == "json_fname":
                has_json_fname = True
            self._skip_ws()
            has_comma = False
            if self.i < len(self.text) and self.text[self.i] == ",":
                has_comma = True
                self.i += 1
                self._skip_ws()
            members.append(
                JsonMemberSpan(
                    key=key,
                    key_start=key_start,
                    value_start=value_start,
                    value_end=value_end,
                    has_comma=has_comma,
                )
            )
            if not has_comma:
                break
        if self.i >= len(self.text) or self.text[self.i] != "}":
            raise ValueError(f"Expected '}}' at index {self.i}")
        close_index = self.i
        self.i += 1
        if has_json_fname:
            self.entry_objects.append(JsonObjectSpan(members=members, close_index=close_index))


def _line_start(text: str, pos: int) -> int:
    idx = text.rfind("\n", 0, pos)
    if idx == -1:
        return 0
    return idx + 1


def _member_indent(text: str, obj: JsonObjectSpan) -> str:
    if obj.members:
        key_start = obj.members[0].key_start
        return text[_line_start(text, key_start):key_start]
    close_line_start = _line_start(text, obj.close_index)
    close_indent = text[close_line_start:obj.close_index]
    if "\t" in close_indent:
        return close_indent + "\t"
    return close_indent + "  "


def _json_scalar(value: bool | float) -> str:
    return json.dumps(value, ensure_ascii=False)


def _annotate_json_text(
    input_text: str,
    predictions: list[tuple[bool, bool, float]],
) -> str:
    parser = _JsonSpanParser(input_text)
    objects = parser.parse()
    if len(objects) != len(predictions):
        raise RuntimeError(
            f"Entry count mismatch between parser and dataset: {len(objects)} vs {len(predictions)}"
        )

    newline = "\r\n" if "\r\n" in input_text else "\n"
    edits: list[tuple[int, int, str]] = []

    for obj, (stable, solved, time_s) in zip(objects, predictions):
        desired: dict[str, bool | float] = {
            "bricksim_stable": stable,
            "bricksim_solved": solved,
            "bricksim_time": time_s,
        }
        member_by_key = {m.key: m for m in obj.members}
        missing_keys: list[str] = []

        for key, value in desired.items():
            member = member_by_key.get(key)
            if member is not None:
                edits.append(
                    (member.value_start, member.value_end, _json_scalar(value))
                )
                continue
            missing_keys.append(key)

        if missing_keys:
            if obj.members and not obj.members[-1].has_comma:
                edits.append((obj.members[-1].value_end, obj.members[-1].value_end, ","))

            indent = _member_indent(input_text, obj)
            lines = [f'{indent}"{key}": {_json_scalar(desired[key])}' for key in missing_keys]
            insertion = f",{newline}".join(lines) + newline
            close_line_start = _line_start(input_text, obj.close_index)
            edits.append((close_line_start, close_line_start, insertion))

    output_text = input_text
    for start, end, replacement in sorted(edits, key=lambda x: (x[0], x[1]), reverse=True):
        output_text = output_text[:start] + replacement + output_text[end:]
    return output_text


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate structures listed in a dataset JSON, predict static stability "
            "with static_solve, write bricksim_stable, and report accuracy."
        )
    )
    parser.add_argument("input", type=Path, help="Input evaluation dataset JSON path.")
    parser.add_argument("output", type=Path, help="Output JSON path.")
    parser.add_argument(
        "--datarootdir",
        type=Path,
        default=None,
        help=(
            "Root corresponding to /data in json_fname paths. "
            "Default: BRICKSIM_DATASET_PATH, else XDG_CACHE_HOME/bricksim/bricksim_dataset, "
            "else ~/.cache/bricksim/bricksim_dataset"
        ),
    )
    parser.add_argument(
        "--solver",
        type=Path,
        default=None,
        help="Path to static_solve binary. Default: <repo>/native/.build/release/static_solve",
    )
    parser.add_argument(
        "--format",
        choices=["auto", "legolization", "stabletext2brick"],
        default="auto",
        help="Input structure format (default: auto).",
    )
    parser.add_argument(
        "--baseplate",
        type=str,
        default="32x32",
        help="Baseplate size as WxH (default: 32x32).",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    datarootdir = args.datarootdir if args.datarootdir is not None else _resolve_bricksim_dataset_path()
    solver_path = args.solver if args.solver is not None else repo_root / "native" / ".build" / "release" / "static_solve"
    if not solver_path.is_file():
        raise FileNotFoundError(f"Missing solver binary at {solver_path}")

    baseplate_size = _parse_baseplate(args.baseplate)
    include_baseplate = True
    lego_library = load_default_lego_library()

    input_text = args.input.read_text(encoding="utf-8")
    dataset = json.loads(input_text)
    entries = list(_iter_eval_entries(dataset))
    if not entries:
        raise ValueError(f"No entries with json_fname found in {args.input}")

    total = len(entries)
    solved_total = 0
    solved_with_gt = 0
    solved_correct = 0
    solved_pred_unstable_gt_stable = 0
    solved_pred_stable_gt_unstable = 0
    predictions: list[tuple[bool, bool, float]] = []

    print(f"Evaluating {total} structures from {args.input}")
    print(f"Using datarootdir: {datarootdir}")
    print(f"Using solver: {solver_path}")
    print(f"Using baseplate: {baseplate_size[0]}x{baseplate_size[1]}")

    for idx, entry in enumerate(entries, start=1):
        json_fname = entry.meta.get("json_fname")
        if not isinstance(json_fname, str):
            raise TypeError(f"{entry.label}: json_fname must be a string")

        structure_path = _resolve_structure_path(datarootdir, json_fname)
        if not structure_path.is_file():
            raise FileNotFoundError(f"{entry.label}: structure file not found: {structure_path}")

        topology = _load_and_convert(
            structure_path,
            input_format=args.format,
            include_baseplate=include_baseplate,
            baseplate_size=baseplate_size,
            lego_library=lego_library,
        )
        solve_result = _run_static_solve(topology, solver_path)
        stable, solved, time_s = _extract_solver_result(solve_result)
        predictions.append((stable, solved, time_s))

        gt_stable = _as_optional_bool(entry.meta.get("gt_stable"))
        if not solved:
            correctness = "unsolved"
        elif gt_stable is None:
            correctness = "n/a"
        else:
            solved_with_gt += 1
            if stable == gt_stable:
                solved_correct += 1
                correctness = "ok"
            else:
                correctness = "wrong"
                if (not stable) and gt_stable:
                    solved_pred_unstable_gt_stable += 1
                elif stable and (not gt_stable):
                    solved_pred_stable_gt_unstable += 1

        if solved:
            solved_total += 1

        print(
            f"[{idx:03d}/{total:03d}] {entry.label} "
            f"stable={str(stable).lower()} "
            f"solved={str(solved).lower()} "
            f"time_s={time_s:.6f} "
            f"correctness={correctness}"
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(_annotate_json_text(input_text, predictions), encoding="utf-8")

    print()
    print(f"Saved annotated dataset to {args.output}")
    print(f"Total structures: {total}")

    solved_pct = (100.0 * solved_total / total) if total > 0 else 0.0
    print(f"Solved: {solved_total}/{total} ({solved_pct:.2f}%)")

    if solved_with_gt > 0:
        wrong_unstable_pct = 100.0 * solved_pred_unstable_gt_stable / solved_with_gt
        wrong_stable_pct = 100.0 * solved_pred_stable_gt_unstable / solved_with_gt
        accuracy_pct = 100.0 * solved_correct / solved_with_gt
        print(
            f"Solved miscls (pred unstable, gt stable): "
            f"{solved_pred_unstable_gt_stable}/{solved_with_gt} ({wrong_unstable_pct:.2f}%)"
        )
        print(
            f"Solved miscls (pred stable, gt unstable): "
            f"{solved_pred_stable_gt_unstable}/{solved_with_gt} ({wrong_stable_pct:.2f}%)"
        )
        print(f"Solved accuracy: {solved_correct}/{solved_with_gt} ({accuracy_pct:.2f}%)")
        if solved_with_gt != solved_total:
            print(
                f"Note: {solved_total - solved_with_gt} solved cases have no gt_stable and are excluded from solved accuracy."
            )
        print(f"Time avg (solved): {sum(p[2] for p in predictions if p[1]) / solved_total:.6f} s")
    else:
        print("Solved accuracy: N/A (no solved cases with gt_stable)")


if __name__ == "__main__":
    main()
