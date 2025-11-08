#!/usr/bin/env python3
"""
List Chromium clang builds from the public bucket and sort by date.

Default: list Linux_x64 clang packages (keys like "Linux_x64/clang-<rev>.*")
and print newest first. With --group, widen to the platform root to show
all components for the same revision (clang, clangd, clang-tidy, etc.).

Usage examples:
  scripts/list_clang_builds.py                         # latest 50 Linux_x64 builds
  scripts/list_clang_builds.py --platform Mac          # latest 50 Mac builds
  scripts/list_clang_builds.py --platform Mac_arm64    # latest 50 Mac arm64 builds
  scripts/list_clang_builds.py --platform Win --limit 20
  scripts/list_clang_builds.py --prefix Linux_x64/clang- --contains .tgz
  scripts/list_clang_builds.py --group                 # group by commit (revision)

Notes:
- Uses the public GCS JSON API and prints direct URLs on
  commondatastorage.googleapis.com.
- Standard library only; fail fast on errors.
"""

from __future__ import annotations

import argparse
import datetime as _dt
from datetime import timezone as _tz
import json
import sys
import urllib.parse
import urllib.request


BUCKET = "chromium-browser-clang"
JSON_LIST_ENDPOINT = f"https://storage.googleapis.com/storage/v1/b/{BUCKET}/o"

# Known extensions and markers (used for lightweight parsing/formatting)
_ARCH_EXTS = (
    ".tar.xz",
    ".tar.gz",
    ".tar.zst",
    ".tar.bz2",
    ".tgz",
    ".zip",
)
_OTHER_EXTS = (
    ".txt",
    ".json",
    ".sha256",
    ".sha1",
    ".sha512",
    ".xz",
    ".gz",
    ".zst",
    ".sig",
)
_SUFFIX_MARKERS = ("buildlog", "manifest", "metadata", "info")


def _http_get_json(url: str, timeout: int = 30) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        # GCS JSON API returns UTF-8 JSON
        return json.loads(resp.read().decode("utf-8"))


def _human_size(n: int) -> str:
    units = ["B", "KB", "MB", "GB", "TB"]
    size = float(n)
    for u in units:
        if size < 1024.0 or u == units[-1]:
            return f"{size:.1f} {u}" if u != "B" else f"{int(size)} B"
        size /= 1024.0
    return f"{int(n)} B"


def _strip_known_ext(filename: str) -> tuple[str, str]:
    """Return (stem_without_known_ext, ext) where ext includes the leading dot(s).

    Handles compound archive extensions like .tar.xz, .tar.gz.
    """
    for ext in (*_ARCH_EXTS, *_OTHER_EXTS):
        if filename.endswith(ext):
            return filename[: -len(ext)], ext
    # default split on last dot if any
    if "." in filename:
        i = filename.rfind(".")
        return filename[:i], filename[i:]
    return filename, ""


def _parse_component_and_rev(name: str) -> tuple[str | None, str | None, str | None]:
    """Best-effort parse of component and revision from an object name.

    Returns (component, revision, suffix) where suffix may be 'buildlog', 'manifest', etc.,
    or None when not present. Returns (None, None, None) if parsing fails.
    """
    base = name.split("/")[-1]
    if "-" not in base:
        return None, None, None
    stem, _ext = _strip_known_ext(base)
    # Remove known trailing markers like '-buildlog'
    suffix = None
    for marker in _SUFFIX_MARKERS:
        if stem.endswith("-" + marker):
            suffix = marker
            stem = stem[: -(len(marker) + 1)]
            break
    # stem is now 'component[-sub]-<rev>'. Prefer splitting at the
    # beginning of the revision marker when present. Chromium clang
    # artifacts consistently use '...-llvmorg-...' for the revision.
    idx = stem.find("-llvmorg-")
    if idx != -1:
        raw_component = stem[:idx]
        # drop the leading '-' before 'llvmorg'
        rev = stem[idx + 1 :]
        component = _normalize_component(raw_component)
        return component, rev, suffix

    # Fallbacks for older formats without 'llvmorg-'
    if stem.startswith("clang-tidy-"):
        component = "clang-tidy"
        rev = stem[len("clang-tidy-"):]
        return component, rev, suffix
    if "-" in stem:
        component, rev = stem.split("-", 1)
        return component, rev, suffix
    return None, None, suffix


def _normalize_component(s: str) -> str:
    """Normalize verbose component names to stable labels.

    - 'rust-toolchain-<hash>-N' -> 'rust-toolchain'
    - others returned unchanged.
    """
    if s.startswith("rust-toolchain-"):
        return "rust-toolchain"
    return s


def list_objects(prefix: str, contains: str | None, limit: int | None) -> list[dict]:
    params = {
        "prefix": prefix,
        # Only fetch fields we need to keep responses small.
        "fields": "nextPageToken,items(name,updated,size)",
        # Reasonable page size; server may return fewer.
        "pageSize": 1000,
    }

    # Maintain a top-N by updated timestamp using a min-heap to avoid
    # materializing all results when --limit is set.
    import heapq

    heap: list[tuple[_dt.datetime, int, dict]] = []
    seq = 0  # tie-breaker to keep heap entries unique
    page_token: str | None = None

    while True:
        q = dict(params)
        if page_token:
            q["pageToken"] = page_token
        url = JSON_LIST_ENDPOINT + "?" + urllib.parse.urlencode(q)
        data = _http_get_json(url)
        items = data.get("items", [])
        if contains:
            items = [it for it in items if contains in it.get("name", "")]

        for it in items:
            # Parse timestamp once for comparison and keep as hidden field.
            it_dt = _dt.datetime.fromisoformat(it["updated"].replace("Z", "+00:00"))
            it["_updated_dt"] = it_dt
            if limit is None:
                # No limit: collect everything.
                heap.append((it_dt, seq, it))
                seq += 1
            else:
                if len(heap) < limit:
                    heapq.heappush(heap, (it_dt, seq, it))
                    seq += 1
                else:
                    # Keep only the newest 'limit' entries.
                    if it_dt > heap[0][0]:
                        heapq.heapreplace(heap, (it_dt, seq, it))
                        seq += 1
        page_token = data.get("nextPageToken")
        if not page_token:
            break

    # If no limit was provided, 'heap' is a list of all items. If a limit
    # was provided, 'heap' already contains the top-N newest entries.
    # Sort newest first for output.
    heap.sort(key=lambda t: t[0], reverse=True)
    return [t[2] for t in heap]


def group_by_revision(items: list[dict]) -> list[dict]:
    """Group objects by parsed revision, aggregating across components.

    Returns a list of groups sorted by newest item time desc. Each group is:
      { 'rev': str, 'latest': datetime, 'items': [ { 'component', 'suffix', 'obj' } ] }
    Groups with unknown revision are ignored.
    """
    groups: dict[str, dict] = {}
    for it in items:
        comp, rev, suffix = _parse_component_and_rev(it["name"])
        if not rev:
            continue
        g = groups.get(rev)
        if not g:
            g = {"rev": rev, "latest": it["_updated_dt"], "items": []}
            groups[rev] = g
        else:
            if it["_updated_dt"] > g["latest"]:
                g["latest"] = it["_updated_dt"]
        g["items"].append({
            "component": comp or "unknown",
            "suffix": suffix,
            "obj": it,
        })

    out = list(groups.values())
    out.sort(key=lambda g: g["latest"], reverse=True)
    return out


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(
        description=(
            "Fetch clang builds from Chromium's public bucket and list them "
            "latest-first."
        )
    )
    ap.add_argument(
        "--platform",
        default="Linux_x64",
        help=(
            "Platform directory (e.g. Linux_x64, Linux_arm64, Mac, Mac_arm64, Win). "
            "Used to build the default prefix '<platform>/clang-'."
        ),
    )
    ap.add_argument(
        "--prefix",
        default=None,
        help=(
            "Override object prefix. Defaults to '<platform>/clang-'. "
            "Example: 'Mac/clang-' or 'Linux_arm64/clang-'."
        ),
    )
    ap.add_argument(
        "--contains",
        default=None,
        help=(
            "Substring filter on object name (applied client-side). "
            "Example: '.tgz' or '.tar.xz'."
        ),
    )
    ap.add_argument("--limit", type=int, default=50, help="Max results to show (default: 50)")
    ap.add_argument(
        "--iso",
        action="store_true",
        help="Print timestamps in strict UTC ISO-8601 instead of a friendly format.",
    )
    ap.add_argument(
        "--group",
        action="store_true",
        help=(
            "Group artifacts by parsed revision and list components per commit. "
            "If no --prefix is given, grouping widens the search to '<platform>/' "
            "so components like clang, clangd, clang-tidy, libclang, etc. are included."
        ),
    )

    args = ap.parse_args(argv)

    prefix = args.prefix if args.prefix is not None else f"{args.platform}/clang-"
    # In grouping mode, widen default prefix to platform root so all components
    # from the same commit (clang, clangd, clang-tidy, libclang, etc.) are included.
    if args.group and args.prefix is None:
        prefix = f"{args.platform}/"
    # Coerce non-positive limits to None to mean "no limit".
    limit = args.limit if args.limit and args.limit > 0 else None
    items = list_objects(prefix=prefix, contains=args.contains, limit=limit)

    if not args.group:
        for it in items:
            ts = _fmt_ts(it["_updated_dt"], args.iso)
            size = _human_size(int(it.get("size", 0)))
            print(f"{ts}\t{size}\t{_dl_url(it['name'])}")
        return 0

    # Grouping output
    groups = group_by_revision(items)
    for g in groups:
        latest = _fmt_ts(g["latest"], args.iso)
        print(f"# {g['rev']}\t{latest}\t{len(g['items'])} items")
        # Sort items: component, then by size desc to show archives before logs
        g_items = sorted(
            g["items"],
            key=lambda x: (x["component"], -int(x["obj"].get("size", 0))),
        )
        for entry in g_items:
            it = entry["obj"]
            size = _human_size(int(it.get("size", 0)))
            url = _dl_url(it["name"])
            label = entry["component"] or "unknown"
            if entry["suffix"]:
                label = f"{label}:{entry['suffix']}"
            print(f"  - {label}\t{size}\t{url}")
    return 0


def _dl_url(name: str) -> str:
    return f"https://commondatastorage.googleapis.com/{BUCKET}/{name}"


def _fmt_ts(dt: _dt.datetime, iso: bool) -> str:
    if iso:
        return dt.astimezone(_tz.utc).isoformat()
    # Force readable UTC label.
    return dt.astimezone(_tz.utc).strftime("%Y-%m-%d %H:%M:%S UTC")


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
