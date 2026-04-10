#!/usr/bin/env bash
set -eo pipefail

die() {
    echo "[ERROR] $*" >&2
    exit 1
}

resolve_download_url() {
    local commit="$1"
    local template="${BRICKSIM_PREBUILT_NATIVE_URL_TEMPLATE:-https://generic.cloudsmith.io/public/bricksim/bricksim/commits/{commit}/wheel-url.txt}"
    printf '%s\n' "${template//\{commit\}/$commit}"
}

artifact_url_exists() {
    local url="$1"
    wget --quiet --spider --method=HEAD --tries=1 "$url"
}

read_wheel_url() {
    local url_file_url="$1"
    local wheel_url
    wheel_url=$(wget -qO- "$url_file_url" | tr -d '\r')
    [[ -n "$wheel_url" ]] || die "Wheel URL file is empty: $url_file_url"
    [[ "$wheel_url" != *$'\n'* ]] || die "Wheel URL file must contain exactly one URL: $url_file_url"
    printf '%s\n' "$wheel_url"
}

extract_native_from_wheel() {
    local wheel_path="$1"
    local output_dir="$2"
    local native_entry_lines
    native_entry_lines=$(unzip -Z1 "$wheel_path" | grep '^bricksim/core\..*\.so$' || true)
    [[ -n "$native_entry_lines" ]] || die "Wheel does not contain bricksim/core.*.so: $wheel_path"
    [[ "$native_entry_lines" != *$'\n'* ]] || die "Wheel contains multiple bricksim/core.*.so entries: $wheel_path"

    local native_entry="$native_entry_lines"
    local artifact="${native_entry##*/}"
    unzip -p "$wheel_path" "$native_entry" > "$output_dir/$artifact"
    printf '%s\n' "$artifact"
}

commit_touches_native() {
    local commit="$1"
    local parent_commit
    if parent_commit=$(git -C "$ROOT_DIR" rev-parse --verify "${commit}^1" 2>/dev/null); then
        if git -C "$ROOT_DIR" diff --quiet "$parent_commit" "$commit" -- native/; then
            return 1
        fi
        return 0
    fi

    if git -C "$ROOT_DIR" ls-tree -r --name-only "$commit" -- native/ | grep -q .; then
        return 0
    fi
    return 1
}

ensure_native_tree_clean() {
    if git -C "$ROOT_DIR" status --porcelain=v1 --untracked-files=all --ignored=no -- native/ | grep -q .; then
        die "Refusing to download a prebuilt native cache while native/ has local changes."
    fi
}

select_prebuilt() {
    local commit="$1"
    local strict="$2"
    local searched_commits=0
    local url_file_url

    while true; do
        url_file_url=$(resolve_download_url "$commit")
        if artifact_url_exists "$url_file_url"; then
            SELECTED_COMMIT="$commit"
            SELECTED_WHEEL_URL_FILE="$url_file_url"
            return 0
        fi

        if [[ "$strict" -eq 1 ]]; then
            die "No prebuilt native artifact found for commit $commit"
        fi
        if commit_touches_native "$commit"; then
            die "No prebuilt native artifact found for commit $commit, and that commit changes native/, so fallback to older prebuilts is invalid."
        fi

        searched_commits=$((searched_commits + 1))
        if [[ "$searched_commits" -ge "$MAX_FALLBACK_COMMITS" ]]; then
            die "No prebuilt native artifact found within the last $MAX_FALLBACK_COMMITS fallback-eligible first-parent commits from $RESOLVED_COMMIT. Pass a commit hash with a known prebuilt artifact."
        fi

        commit=$(git -C "$ROOT_DIR" rev-parse --verify "${commit}^1" 2>/dev/null) || die "No prebuilt native artifact found for commit $RESOLVED_COMMIT, and no older first-parent commit is available."
    done
}

SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd -P)
ROOT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)
PREBUILT_NATIVE_DIR="$ROOT_DIR/.prebuilt-native"
MAX_FALLBACK_COMMITS=100

if [[ $# -gt 1 ]]; then
    die "Usage: $0 [ref]"
fi

USER_REF="${1:-HEAD}"
RESOLVED_COMMIT=$(git -C "$ROOT_DIR" rev-parse --verify "${USER_REF}^{commit}") || die "Failed to resolve git ref: $USER_REF"
ensure_native_tree_clean

STRICT_COMMIT_MODE=0
if [[ "$USER_REF" =~ ^[0-9A-Fa-f]{7,40}$ ]]; then
    STRICT_COMMIT_MODE=1
fi

SELECTED_COMMIT=""
SELECTED_WHEEL_URL_FILE=""
select_prebuilt "$RESOLVED_COMMIT" "$STRICT_COMMIT_MODE"

if [[ "$SELECTED_COMMIT" != "$RESOLVED_COMMIT" ]]; then
    echo "No prebuilt native artifact found for $RESOLVED_COMMIT; using nearest ancestor $SELECTED_COMMIT" >&2
fi

temporary_wheel_path=$(mktemp)

echo "Downloading wheel URL file from $SELECTED_WHEEL_URL_FILE" >&2
WHEEL_URL=$(read_wheel_url "$SELECTED_WHEEL_URL_FILE")
echo "Downloading wheel from $WHEEL_URL" >&2
wget --tries=10 --retry-on-http-error=500 --retry-on-host-error -O "$temporary_wheel_path" "$WHEEL_URL"

rm -rf "$PREBUILT_NATIVE_DIR"
mkdir -p "$PREBUILT_NATIVE_DIR"

ARTIFACT=$(extract_native_from_wheel "$temporary_wheel_path" "$PREBUILT_NATIVE_DIR")
SHA256=$(sha256sum "$PREBUILT_NATIVE_DIR/$ARTIFACT" | awk '{print $1}')
rm -f "$temporary_wheel_path"

# Write the manifest last so build.sh only sees complete caches.
cat > "$PREBUILT_NATIVE_DIR/manifest.env" <<EOF
ARTIFACT=$ARTIFACT
SHA256=$SHA256
DOWNLOAD_URL=$WHEEL_URL
COMMIT=$SELECTED_COMMIT
EOF

echo "Stored prebuilt native cache at .prebuilt-native/" >&2
echo "Artifact: $ARTIFACT" >&2
echo "Commit: $SELECTED_COMMIT" >&2
echo "To resume normal C++ builds, delete .prebuilt-native/manifest.env" >&2
