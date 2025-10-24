#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  echo "Usage: $0 --patch-exe PATH --include-dir DIR --patch-file FILE --strip N --stamp FILE" >&2
}

PATCH_EXE="patch"
INCLUDE_DIR=""
PATCH_FILE=""
STRIP_N="1"
STAMP_FILE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --patch-exe) PATCH_EXE="$2"; shift 2;;
    --include-dir) INCLUDE_DIR="$2"; shift 2;;
    --patch-file) PATCH_FILE="$2"; shift 2;;
    --strip) STRIP_N="$2"; shift 2;;
    --stamp) STAMP_FILE="$2"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2;;
  esac
done

if ! command -v "$PATCH_EXE" >/dev/null 2>&1; then
  echo "patch executable not found: $PATCH_EXE" >&2
  exit 127
fi
if [[ -z "$INCLUDE_DIR" || -z "$PATCH_FILE" || -z "$STAMP_FILE" ]]; then
  usage; exit 2
fi

# Collect target file paths from the patch (prefer +++ b/ lines)
mapfile -t files < <(awk '/^\+\+\+ b\//{print substr($2,3)}' "$PATCH_FILE" | awk '{print $1}' | sort -u)
if [[ ${#files[@]} -eq 0 ]]; then
  # Fallback to --- a/ paths
  mapfile -t files < <(awk '/^--- a\//{print substr($2,3)}' "$PATCH_FILE" | awk '{print $1}' | sort -u)
fi

declare -A existed=()

# Restore from .orig if present; otherwise create backup if file exists
for f in "${files[@]}"; do
  path="$INCLUDE_DIR/$f"
  if [[ -f "$path.orig" ]]; then
    cp -f "$path.orig" "$path"
    existed["$path"]=1
  elif [[ -f "$path" ]]; then
    cp -f "$path" "$path.orig"
    existed["$path"]=1
  else
    existed["$path"]=0
  fi
done

# Apply the patch strictly; rollback if anything fails
if ! "$PATCH_EXE" -p"$STRIP_N" --directory="${INCLUDE_DIR}" --input="${PATCH_FILE}" --batch --fuzz=0; then
  for f in "${files[@]}"; do
    path="$INCLUDE_DIR/$f"
    if [[ "${existed[$path]}" == "1" && -f "$path.orig" ]]; then
      cp -f "$path.orig" "$path"
    else
      rm -f "$path"
    fi
    # Remove reject if produced
    [[ -f "$path.rej" ]] && rm -f "$path.rej"
  done
  exit 1
fi

# Success
cmake -E touch "$STAMP_FILE"
exit 0

