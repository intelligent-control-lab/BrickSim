#!/usr/bin/env python3
"""Prepare archive artifacts for a build.

Reads a TOML file listing archives by name, URL, and SHA256. For each
artifact the script maintains a user-level cache of the downloaded archive
and a pristine extracted tree. Output directories are disposable: untouched
files are hardlinked from cache when possible, patch-touched files are copied
with ``cp --reflink=auto`` before patching, and hardlinking falls back to the
same reflink copy path across filesystems. Prepared files are marked
read-only; directories stay writable so staged outputs can be replaced and
removed normally.

This tool intentionally targets Linux/GNU environments. It uses
``fcntl.flock``, ``7zz``, ``patch``, GNU ``cp``, and ``wget`` for remote URLs.
Patch support is limited to textual unified diffs. Binary patches, renames,
symlink-touched paths, and paths that escape the artifact root are rejected.
"""

import argparse
import fcntl
import hashlib
import json
import os
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import tomllib
from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath
from urllib.parse import urlparse
from urllib.request import url2pathname

EXTRACT_VERSION = 1
OUTPUT_VERSION = 1
LOG_PREFIX = "[prepare_artifacts]"
RESERVED_ARTIFACT_NAMES = {".artifact-preparer"}


class UserError(RuntimeError):
    """Displayed without a traceback."""


def sha256_file(path: Path) -> str:
    """Return the SHA256 digest for a file."""
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def log(message: str, *, stderr: bool = False) -> None:
    """Print one script-owned log line."""
    print(
        f"{LOG_PREFIX} {message}", file=sys.stderr if stderr else sys.stdout, flush=True
    )


def log_tool(tool: str, message: str) -> None:
    """Print one external-tool log line."""
    print(f"[prepare_artifacts:{tool}] {message}", flush=True)


def read_table(raw: object, ctx: str) -> dict[str, object]:
    """Return a TOML table as a string-keyed dictionary."""
    if not isinstance(raw, Mapping):
        raise UserError(f"{ctx} must be a table")

    table: dict[str, object] = {}
    for key, value in raw.items():
        if not isinstance(key, str):
            raise UserError(f"{ctx} has a non-string key: {key}")
        table[key] = value
    return table


def read_string(table: dict[str, object], key: str, ctx: str) -> str:
    """Return a required non-empty string field from a TOML table."""
    value = table.get(key)
    if not isinstance(value, str) or not value:
        raise UserError(f"{ctx}.{key} must be a non-empty string")
    return value


def read_path_name(table: dict[str, object], key: str, ctx: str) -> str:
    """Return a string field that is safe to use as one path component."""
    value = read_string(table, key, ctx)
    if value in {".", ".."} or "/" in value or "\\" in value:
        raise UserError(f"{ctx}.{key} is not a safe path component: {value}")
    return value


def read_relative_path(table: dict[str, object], key: str, ctx: str) -> PurePosixPath:
    """Return an optional relative POSIX path from a TOML table."""
    value = table.get(key, ".")
    if not isinstance(value, str) or not value:
        raise UserError(f"{ctx}.{key} must be a non-empty string")
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts:
        raise UserError(f"{ctx}.{key} must be relative without '..': {path}")
    return path


@dataclass(frozen=True)
class Patch:
    """One patch applied to an artifact after output preparation."""

    path: Path
    display_path: str
    root: PurePosixPath
    strip: int
    sha256: str

    @classmethod
    def from_config(cls, raw: object, config_dir: Path, ctx: str) -> "Patch":
        """Read one patch entry from the TOML config.

        Returns:
            A validated patch description.
        """
        table = read_table(raw, ctx)

        display_path = read_string(table, "path", ctx)
        path = Path(display_path)
        if not path.is_absolute():
            path = config_dir / path
        if not path.is_file():
            raise UserError(f"{ctx}.path does not exist: {path}")

        root = read_relative_path(table, "root", ctx)

        strip = table.get("strip", 1)
        if type(strip) is not int or strip < 0:
            raise UserError(f"{ctx}.strip must be a non-negative integer")

        resolved = path.resolve()
        return cls(
            path=resolved,
            display_path=display_path,
            root=root,
            strip=strip,
            sha256=sha256_file(resolved),
        )

    def affected_files(self) -> set[PurePosixPath]:
        """Parse the unified diff and return paths it modifies.

        Walks ``---``/``+++`` header pairs, strips leading path components
        such as the ``a/`` and ``b/`` from ``git diff``, and prepends the
        configured root directory.

        Returns:
            Artifact-relative paths that must be copied before patching.
        """
        result: set[PurePosixPath] = set()
        prev: PurePosixPath | None = None
        in_file = False
        hunk_old = 0
        hunk_new = 0

        with self.path.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if hunk_old > 0 or hunk_new > 0:
                    hunk_old, hunk_new = self._consume_hunk_line(
                        line, hunk_old, hunk_new
                    )
                    continue

                if line.startswith(("GIT binary patch", "Binary files ")):
                    raise UserError(f"{self.path}: binary patches not supported")
                if line.startswith(("rename from ", "rename to ")):
                    raise UserError(f"{self.path}: renames not supported")
                if line.startswith(("--- ", "---\t")):
                    prev = self._parse_diff_path(line)
                    continue
                if not line.startswith(("+++ ", "+++\t")):
                    if line.startswith("@@ "):
                        if not in_file:
                            raise UserError(f"{self.path}: hunk before file header")
                        hunk_old, hunk_new = self._parse_hunk_header(line)
                    continue
                cur = self._parse_diff_path(line)
                paths = [p for p in (prev, cur) if p is not None]
                if len(paths) == 2 and paths[0] != paths[1]:
                    raise UserError(f"{self.path}: renames not supported")
                for p in paths:
                    rooted = self.root / p
                    if rooted.is_absolute() or ".." in rooted.parts:
                        raise UserError(f"{self.path}: path escapes root: {rooted}")
                    result.add(rooted)
                prev = None
                in_file = True

        if hunk_old > 0 or hunk_new > 0:
            raise UserError(f"{self.path}: truncated hunk")
        if prev is not None:
            raise UserError(f"{self.path}: missing +++ header after --- header")
        if not result:
            raise UserError(f"{self.path}: no file paths found in patch")
        return result

    def _parse_diff_path(self, line: str) -> PurePosixPath | None:
        """Extract a file path from a ``---`` or ``+++`` diff header.

        Returns:
            The stripped patch path, or ``None`` for ``/dev/null``.
        """
        header = line.removesuffix("\n").removesuffix("\r")
        if len(header) < 5 or header[:3] not in {"---", "+++"}:
            raise UserError(f"{self.path}: bad diff header: {line.rstrip()}")
        if header[3] not in {" ", "\t"}:
            raise UserError(f"{self.path}: bad diff header: {line.rstrip()}")

        raw = self._parse_diff_header_path(header[4:], line)
        if raw == "/dev/null":
            return None
        p = PurePosixPath(raw)
        if p.is_absolute():
            raise UserError(f"{self.path}: absolute diff path not supported")
        if self.strip > len(p.parts):
            raise UserError(f"{self.path}: can't strip {self.strip} from {raw}")
        stripped = PurePosixPath(*p.parts[self.strip :])
        if not stripped.parts:
            raise UserError(f"{self.path}: diff path strips to empty")
        if stripped.is_absolute() or ".." in stripped.parts:
            raise UserError(f"{self.path}: unsafe path: {stripped}")
        return stripped

    def _parse_diff_header_path(self, body: str, line: str) -> str:
        """Return the path field from a unified-diff file header."""
        if not body:
            raise UserError(f"{self.path}: bad diff header: {line.rstrip()}")

        if body.startswith('"'):
            raw, rest = self._parse_c_quoted_path(body, line)
            if rest and not rest[0].isspace():
                raise UserError(f"{self.path}: bad diff header: {line.rstrip()}")
            return raw

        tab = body.find("\t")
        if tab == -1:
            return body
        return body[:tab]

    def _parse_c_quoted_path(self, text: str, line: str) -> tuple[str, str]:
        """Parse the C-style quoted path syntax used by Git and GNU diff.

        Returns:
            The decoded path and the unconsumed header suffix.
        """
        escapes = {
            '"': b'"',
            "\\": b"\\",
            "a": b"\a",
            "b": b"\b",
            "f": b"\f",
            "n": b"\n",
            "r": b"\r",
            "t": b"\t",
            "v": b"\v",
        }
        out = bytearray()
        i = 1
        while i < len(text):
            char = text[i]
            if char == '"':
                try:
                    return out.decode("utf-8"), text[i + 1 :]
                except UnicodeDecodeError as error:
                    raise UserError(
                        f"{self.path}: diff path is not UTF-8: {line.rstrip()}"
                    ) from error
            if char != "\\":
                out.extend(char.encode("utf-8"))
                i += 1
                continue

            i += 1
            if i == len(text):
                raise UserError(f"{self.path}: bad diff header: {line.rstrip()}")
            escaped = text[i]
            if escaped in escapes:
                out.extend(escapes[escaped])
                i += 1
                continue
            if escaped in "01234567":
                digits = [escaped]
                i += 1
                while i < len(text) and len(digits) < 3 and text[i] in "01234567":
                    digits.append(text[i])
                    i += 1
                try:
                    out.append(int("".join(digits), 8))
                except ValueError as error:
                    raise UserError(
                        f"{self.path}: diff path octal out of bounds: {line.rstrip()}"
                    ) from error
                continue

            out.extend(escaped.encode("utf-8"))
            i += 1

        raise UserError(f"{self.path}: bad diff header: {line.rstrip()}")

    def _parse_hunk_header(self, line: str) -> tuple[int, int]:
        """Return old/new line counts from a unified-diff hunk header."""
        parts = line.split()
        if len(parts) < 4 or parts[0] != "@@" or parts[3] != "@@":
            raise UserError(f"{self.path}: bad hunk header: {line.rstrip()}")
        return (
            self._parse_hunk_range(parts[1], "-"),
            self._parse_hunk_range(parts[2], "+"),
        )

    def _parse_hunk_range(self, token: str, prefix: str) -> int:
        """Return the line count from one side of a hunk range."""
        if not token.startswith(prefix):
            raise UserError(f"{self.path}: bad hunk range: {token}")
        pieces = token[1:].split(",", maxsplit=1)
        try:
            if len(pieces) == 1:
                return 1
            return int(pieces[1])
        except ValueError as error:
            raise UserError(f"{self.path}: bad hunk range: {token}") from error

    def _consume_hunk_line(
        self, line: str, old_remaining: int, new_remaining: int
    ) -> tuple[int, int]:
        """Consume one unified-diff hunk body line.

        Returns:
            Remaining old-side and new-side line counts.
        """
        if line.startswith("\\"):
            return old_remaining, new_remaining
        if line.startswith(" ") or line in {"\n", "\r\n"}:
            old_remaining -= 1
            new_remaining -= 1
        elif line.startswith("-"):
            old_remaining -= 1
        elif line.startswith("+"):
            new_remaining -= 1
        else:
            raise UserError(f"{self.path}: bad hunk line: {line.rstrip()}")
        if old_remaining < 0 or new_remaining < 0:
            raise UserError(f"{self.path}: hunk has too many lines")
        return old_remaining, new_remaining

    def to_manifest(self) -> dict[str, object]:
        """Return the patch data that participates in output reuse."""
        return {
            "path": self.display_path,
            "root": str(self.root),
            "sha256": self.sha256,
            "strip": self.strip,
        }


@dataclass(frozen=True)
class Artifact:
    """One archive artifact from the config file."""

    name: str
    url: str
    local_archive: Path | None
    sha256: str
    subdir: PurePosixPath
    patches: tuple[Patch, ...]

    @classmethod
    def from_config(
        cls,
        raw: object,
        config_dir: Path,
        ctx: str,
    ) -> "Artifact":
        """Read one artifact entry from the TOML config.

        Returns:
            A validated artifact description.
        """
        table = read_table(raw, ctx)

        name = read_path_name(table, "name", ctx)
        if name in RESERVED_ARTIFACT_NAMES:
            raise UserError(f"{ctx}.name is reserved: {name}")
        url = read_string(table, "url", ctx)
        local_archive = cls._local_archive_path(url, config_dir, ctx)

        sha256 = read_string(table, "sha256", ctx).lower()
        if len(sha256) != 64 or not all(c in "0123456789abcdef" for c in sha256):
            raise UserError(f"{ctx}.sha256 must be a hex digest")
        subdir = read_relative_path(table, "subdir", ctx)

        patches_raw = table.get("patches", [])
        if not isinstance(patches_raw, list):
            raise UserError(f"{ctx}.patches must be an array")
        patches = tuple(
            Patch.from_config(p, config_dir, f"{ctx}.patches[{i}]")
            for i, p in enumerate(patches_raw)
        )
        return cls(
            name=name,
            url=url,
            local_archive=local_archive,
            sha256=sha256,
            subdir=subdir,
            patches=patches,
        )

    @staticmethod
    def _local_archive_path(url: str, config_dir: Path, ctx: str) -> Path | None:
        parsed = urlparse(url)
        if parsed.scheme == "":
            path = Path(url)
            if not path.is_absolute():
                path = config_dir / path
            return Artifact._checked_local_archive(path, ctx)
        if parsed.scheme != "file":
            return None
        if parsed.netloc not in {"", "localhost"}:
            raise UserError(f"{ctx}.url file host is not local: {parsed.netloc}")
        path = Path(url2pathname(parsed.path))
        if not path.is_absolute():
            path = config_dir / path
        return Artifact._checked_local_archive(path, ctx)

    @staticmethod
    def _checked_local_archive(path: Path, ctx: str) -> Path:
        resolved = path.resolve()
        if not resolved.exists():
            raise UserError(f"{ctx}.url local archive does not exist: {resolved}")
        if not resolved.is_file():
            raise UserError(f"{ctx}.url local archive is not a file: {resolved}")
        try:
            with resolved.open("rb"):
                pass
        except OSError as error:
            raise UserError(
                f"{ctx}.url local archive is not readable: {resolved}: {error.strerror}"
            ) from error
        return resolved


@dataclass(frozen=True)
class Config:
    """Validated TOML configuration for artifact preparation."""

    path: Path
    namespace: str
    artifacts: tuple[Artifact, ...]

    @classmethod
    def load(cls, path: Path) -> "Config":
        """Load and validate an artifact config file.

        Returns:
            The validated configuration.
        """
        resolved = path.resolve()
        with resolved.open("rb") as f:
            raw = tomllib.load(f)

        raw_config = read_table(raw, "config")

        cache_raw = raw_config.get("cache", {})
        cache = read_table(cache_raw, "cache")

        ns = cache.get("namespace", "default")
        if (
            not isinstance(ns, str)
            or not ns
            or ns.startswith("/")
            or any(p in {"", ".", ".."} for p in ns.split("/"))
        ):
            raise UserError(f"cache.namespace is not safe: {ns}")

        arts = raw_config.get("artifacts")
        if not isinstance(arts, list):
            raise UserError("artifacts must be an array")

        artifacts: list[Artifact] = []
        names: set[str] = set()
        for i, raw_artifact in enumerate(arts):
            artifact = Artifact.from_config(
                raw_artifact, resolved.parent, f"artifacts[{i}]"
            )
            if artifact.name in names:
                raise UserError(f"duplicate artifact name: {artifact.name}")
            names.add(artifact.name)
            artifacts.append(artifact)

        return cls(
            path=resolved,
            namespace=ns,
            artifacts=tuple(artifacts),
        )

    def input_paths(self) -> list[Path]:
        """Return local filesystem inputs for build-system dependencies."""
        out = [self.path]
        for a in self.artifacts:
            if a.local_archive is not None:
                out.append(a.local_archive)
            out.extend(p.path for p in a.patches)
        return out


@dataclass
class PatchPathTree:
    """Trie over path components for patch-affected-path lookups."""

    children: dict[str, "PatchPathTree"] = field(default_factory=dict)

    @classmethod
    def build(cls, paths: set[PurePosixPath]) -> "PatchPathTree":
        """Build a lookup trie from patch-touched paths.

        Returns:
            The trie root.
        """
        root = cls()
        for p in paths:
            node = root
            for part in p.parts:
                node = node.children.setdefault(part, cls())
        return root


class LockFile:
    """Advisory flock on one lock file."""

    def __init__(self, path: Path) -> None:
        """Remember the lock file path."""
        self._path = path
        self._f = None

    def __enter__(self) -> "LockFile":
        """Acquire the lock.

        Returns:
            This lock object.
        """
        self._path.parent.mkdir(parents=True, exist_ok=True)
        f = self._path.open("a")
        self._f = f
        try:
            fcntl.flock(f.fileno(), fcntl.LOCK_EX)
        except BaseException:
            self._f = None
            f.close()
            raise
        return self

    def __exit__(self, *exc: object) -> None:
        """Release the cache lock."""
        if self._f:
            fcntl.flock(self._f.fileno(), fcntl.LOCK_UN)
            self._f.close()


class ArtifactPreparer:
    """Downloads, extracts, patches, and installs build artifacts."""

    def __init__(
        self,
        config: Config,
        cache_root: Path,
        output_root: Path,
        *,
        sevenzip: str,
        patch: str,
        wget: str | None,
        cp: str,
    ) -> None:
        """Store configuration, output locations, and external tools."""
        self.config = config
        self.cache_root = cache_root
        self.output_root = output_root
        self._7z = sevenzip
        self._patch = patch
        self._wget = wget
        self._cp = cp

    def run(self) -> None:
        """Prepare every configured artifact."""
        self.cache_root.mkdir(parents=True, exist_ok=True)
        self.output_root.mkdir(parents=True, exist_ok=True)
        for artifact in self.config.artifacts:
            with (
                LockFile(self.cache_root / ".lock"),
                LockFile(self._output_lock_path(artifact)),
            ):
                self._prepare(artifact)

    def _prepare(self, art: Artifact) -> None:
        self._verify_local_archive(art)

        final = self.output_root / art.name
        manifest = self._output_manifest(art)

        if (
            self._is_real_dir(final)
            and self._read_manifest(self._output_manifest_path(art)) == manifest
        ):
            return

        extracted = self._extract(art)
        source = self._source_root(art, extracted)
        with self._staged(self.output_root, f".{art.name}.") as tmp:
            if art.patches:
                self._build_patched(art, source, tmp)
            else:
                self._build_unpatched(source, tmp)
            self._make_dirs_writable(tmp)
            self._make_files_readonly(tmp)
            self._swap(tmp, final)
            self._write_manifest(self._output_manifest_path(art), manifest)
            log(f"Prepared: {art.name}")

    @staticmethod
    def _verify_local_archive(art: Artifact) -> None:
        if art.local_archive is None:
            return
        got = sha256_file(art.local_archive)
        if got != art.sha256:
            raise UserError(
                f"{art.name} local archive SHA256 mismatch: "
                f"expected {art.sha256}, got {got}: {art.local_archive}"
            )

    def _download(self, art: Artifact) -> Path:
        archive_dir = self.cache_root / "archives" / art.sha256
        archive_dir.mkdir(parents=True, exist_ok=True)
        dest = archive_dir / "archive"

        if dest.is_file() and sha256_file(dest) == art.sha256:
            return dest
        if dest.exists() or dest.is_symlink():
            try:
                self._remove(dest)
            except OSError as error:
                raise UserError(
                    f"cannot clear cache entry {dest}: {error.strerror}"
                ) from error

        fd, tmp = tempfile.mkstemp(
            prefix=f".{art.name}.archive.",
            suffix=".tmp",
            dir=archive_dir,
        )
        os.close(fd)
        tmp = Path(tmp)
        try:
            if art.local_archive is not None:
                try:
                    shutil.copyfile(art.local_archive, tmp)
                except OSError as error:
                    raise UserError(
                        f"{art.name} local archive cannot be copied: "
                        f"{art.local_archive}: {error.strerror}"
                    ) from error
            else:
                if not self._wget:
                    raise UserError("wget is required for non-local URLs")
                log(f"Downloading {art.name}")
                self._run(self._wget, "-nv", "-O", str(tmp), art.url)

            got = sha256_file(tmp)
            if got != art.sha256:
                raise UserError(
                    f"{art.name} SHA256 mismatch: expected {art.sha256}, got {got}"
                )
            os.replace(tmp, dest)
            self._make_file_readonly(dest)
        except BaseException:
            tmp.unlink(missing_ok=True)
            raise
        return dest

    def _extract(self, art: Artifact) -> Path:
        archive = self._download(art)
        parent = self.cache_root / "extracted"
        parent.mkdir(parents=True, exist_ok=True)
        dest = parent / f"{art.name}-{art.sha256}-v{EXTRACT_VERSION}"
        manifest = self._extract_manifest(art)

        if (
            self._is_real_dir(dest)
            and self._read_manifest(self._extract_manifest_path(art)) == manifest
        ):
            return dest

        self._remove(dest)
        with self._staged(parent, f".{art.name}.") as tmp:
            log(f"Extracting {art.name}")
            self._run(self._7z, "x", "-snld20", "-y", f"-o{tmp}", str(archive))
            while self._expand_single_tar(tmp):
                pass
            self._make_dirs_writable(tmp)
            self._make_files_readonly(tmp)
            os.rename(tmp, dest)
            self._write_manifest(self._extract_manifest_path(art), manifest)
        return dest

    def _expand_single_tar(self, directory: Path) -> bool:
        entries = list(directory.iterdir())
        if len(entries) != 1:
            return False
        tar_file = entries[0]
        if not tar_file.is_file() or not tarfile.is_tarfile(tar_file):
            return False
        self._run(self._7z, "x", "-snld20", "-y", f"-o{directory}", str(tar_file))
        tar_file.unlink()
        return True

    def _source_root(self, art: Artifact, extracted: Path) -> Path:
        source = extracted / art.subdir
        try:
            extracted_root = extracted.resolve(strict=True)
            resolved = source.resolve(strict=True)
        except (OSError, RuntimeError) as error:
            raise UserError(
                f"{art.name} subdir does not exist: {art.subdir}"
            ) from error
        if not resolved.is_relative_to(extracted_root):
            raise UserError(f"{art.name} subdir escapes archive root: {art.subdir}")
        if not resolved.is_dir():
            raise UserError(f"{art.name} subdir is not a directory: {art.subdir}")
        return resolved

    def _build_unpatched(self, extracted: Path, dest: Path) -> None:
        if self._can_hardlink_into(extracted, dest):
            try:
                self._run(self._cp, "-al", "--", f"{extracted}/.", str(dest))
            except subprocess.CalledProcessError:
                self._remove(dest)
                dest.mkdir(mode=0o700)
                self._copy_with_reflink(f"{extracted}/.", dest)
        else:
            self._copy_with_reflink(f"{extracted}/.", dest)
        self._make_writable(dest)

    def _build_patched(
        self,
        art: Artifact,
        extracted: Path,
        dest: Path,
    ) -> None:
        affected: set[PurePosixPath] = set()
        for patch in art.patches:
            affected.update(patch.affected_files())

        self._materialize(extracted, dest, PatchPathTree.build(affected))

        for rel in affected:
            cur = dest
            self._make_writable(cur)
            for part in rel.parts[:-1]:
                cur = cur / part
                cur.mkdir(exist_ok=True)
                if not cur.is_symlink():
                    self._make_writable(cur)

        for patch in art.patches:
            patch_dir = dest
            for part in patch.root.parts:
                patch_dir = patch_dir / part
            patch_dir.mkdir(parents=True, exist_ok=True)
            self._run(
                self._patch,
                f"-p{patch.strip}",
                f"--directory={patch_dir}",
                f"--input={patch.path}",
                "--batch",
                "--fuzz=0",
            )

    def _materialize(self, src: Path, dst: Path, affected_tree: PatchPathTree) -> None:
        """Hardlink untouched entries; copy files that patches will modify."""
        dst.mkdir(exist_ok=True)
        shutil.copystat(src, dst, follow_symlinks=False)
        self._make_writable(dst)

        with os.scandir(src) as entries:
            for entry in entries:
                source_entry = src / entry.name
                output_entry = dst / entry.name
                child_tree = affected_tree.children.get(entry.name)

                if child_tree is None:
                    # Untouched — hardlink or symlink-copy.
                    if source_entry.is_symlink():
                        os.symlink(os.readlink(source_entry), output_entry)
                    elif source_entry.is_dir():
                        self._copy_tree_with_hardlinks_or_reflinks(
                            source_entry, output_entry
                        )
                    elif source_entry.is_file():
                        self._link_file_or_copy(source_entry, output_entry)
                    else:
                        self._run(
                            self._cp,
                            "-a",
                            "--",
                            str(source_entry),
                            str(output_entry),
                        )
                elif source_entry.is_symlink():
                    raise UserError(f"patch touches symlink: {source_entry}")
                elif source_entry.is_dir():
                    if not child_tree.children:
                        raise UserError(
                            f"patch expects file but got directory: {source_entry}"
                        )
                    self._materialize(source_entry, output_entry, child_tree)
                elif not source_entry.is_file():
                    raise UserError(f"patch touches unsupported type: {source_entry}")
                elif child_tree.children:
                    raise UserError(
                        f"patch expects directory but got file: {source_entry}"
                    )
                else:
                    # Patch-touched file — copy so the hardlinked cache stays clean.
                    self._copy_with_reflink(source_entry, output_entry)
                    self._make_writable(output_entry)

    def _copy_tree_with_hardlinks_or_reflinks(self, src: Path, dst: Path) -> None:
        if self._can_hardlink_into(src, dst.parent):
            try:
                self._run(self._cp, "-al", "--", str(src), str(dst))
                return
            except subprocess.CalledProcessError:
                self._remove(dst)
        self._copy_with_reflink(src, dst)

    def _link_file_or_copy(self, src: Path, dst: Path) -> None:
        try:
            os.link(src, dst)
        except OSError:
            self._copy_with_reflink(src, dst)

    def _copy_with_reflink(self, src: Path | str, dst: Path) -> None:
        self._run(self._cp, "-a", "--reflink=auto", "--", str(src), str(dst))

    @staticmethod
    def _is_real_dir(path: Path) -> bool:
        return path.is_dir() and not path.is_symlink()

    @staticmethod
    def _can_hardlink_into(src: Path, dst_parent: Path) -> bool:
        return src.stat(follow_symlinks=False).st_dev == dst_parent.stat().st_dev

    def _swap(self, tmp: Path, final: Path) -> None:
        backup: Path | None = None
        if final.exists() or final.is_symlink():
            backup = Path(
                tempfile.mkdtemp(prefix=f".{final.name}.old.", dir=final.parent)
            )
            backup.rmdir()
            os.rename(final, backup)
        try:
            os.rename(tmp, final)
        except BaseException:
            if backup and (backup.exists() or backup.is_symlink()):
                os.rename(backup, final)
            raise
        if backup:
            self._remove(backup)

    @staticmethod
    def _make_writable(path: Path) -> None:
        if path.is_symlink():
            return
        mode = path.lstat().st_mode | stat.S_IWUSR
        if stat.S_ISDIR(mode):
            mode |= stat.S_IRUSR | stat.S_IXUSR
        os.chmod(path, mode)

    @staticmethod
    def _make_file_readonly(path: Path) -> None:
        if not path.is_symlink():
            m = path.lstat().st_mode
            os.chmod(path, m & ~stat.S_IWUSR & ~stat.S_IWGRP & ~stat.S_IWOTH)

    def _make_files_readonly(self, root: Path) -> None:
        for dirpath, _dirs, files in os.walk(root):
            d = Path(dirpath)
            for n in files:
                self._make_file_readonly(d / n)

    def _make_dirs_writable(self, root: Path) -> None:
        self._make_writable(root)
        for dirpath, dirs, _files in os.walk(root):
            d = Path(dirpath)
            for n in dirs:
                child = d / n
                if not child.is_symlink():
                    self._make_writable(child)

    def _remove(self, path: Path) -> None:
        if not path.exists() and not path.is_symlink():
            return
        if path.is_symlink() or path.is_file():
            path.unlink()
            return
        self._make_writable(path)
        for dirpath, dirs, _files in os.walk(path):
            for d in dirs:
                dp = Path(dirpath) / d
                if not dp.is_symlink():
                    self._make_writable(dp)
        shutil.rmtree(path)

    @contextmanager
    def _staged(self, parent: Path, prefix: str) -> Iterator[Path]:
        tmp = Path(tempfile.mkdtemp(prefix=prefix, dir=parent))
        try:
            yield tmp
        except BaseException:
            self._remove(tmp)
            raise

    @staticmethod
    def _run(*args: str) -> None:
        tool = Path(args[0]).name
        try:
            proc = subprocess.Popen(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors="replace",
            )
        except OSError as error:
            raise UserError(f"failed to launch {tool}: {error}") from error
        if proc.stdout is None:
            raise RuntimeError("subprocess stdout was not captured")
        for line in proc.stdout:
            log_tool(tool, line.rstrip("\n"))
        rc = proc.wait()
        if rc != 0:
            raise subprocess.CalledProcessError(rc, args)

    @staticmethod
    def _read_manifest(path: Path) -> dict[str, object] | None:
        if not path.is_file():
            return None
        try:
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return None
        return {str(k): v for k, v in data.items()} if isinstance(data, dict) else None

    @staticmethod
    def _write_manifest(path: Path, data: dict[str, object]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, sort_keys=True)
            f.write("\n")

    def _extract_manifest_path(self, art: Artifact) -> Path:
        return (
            self.cache_root
            / "meta"
            / "extracted"
            / f"{art.name}-{art.sha256}-v{EXTRACT_VERSION}.json"
        )

    def _output_meta_root(self) -> Path:
        return self.output_root / ".artifact-preparer"

    def _output_manifest_path(self, art: Artifact) -> Path:
        return self._output_meta_root() / f"{art.name}.json"

    def _output_lock_path(self, art: Artifact) -> Path:
        return self._output_meta_root() / f"{art.name}.lock"

    @staticmethod
    def _extract_manifest(art: Artifact) -> dict[str, object]:
        manifest: dict[str, object] = {
            "extraction_format_version": EXTRACT_VERSION,
            "name": art.name,
            "sha256": art.sha256,
            "url": art.url,
        }
        if str(art.subdir) != ".":
            manifest["subdir"] = str(art.subdir)
        return manifest

    @staticmethod
    def _output_manifest(art: Artifact) -> dict[str, object]:
        manifest: dict[str, object] = {
            "algorithm": "hardlink-reflink-patch",
            "extraction_format_version": EXTRACT_VERSION,
            "name": art.name,
            "output_format_version": OUTPUT_VERSION,
            "patches": [p.to_manifest() for p in art.patches],
            "sha256": art.sha256,
            "url": art.url,
        }
        if str(art.subdir) != ".":
            manifest["subdir"] = str(art.subdir)
        return manifest


def main(argv: list[str]) -> int:
    """Run the command-line interface.

    Returns:
        Process exit status.
    """
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    p.add_argument("--output-root")
    p.add_argument("--cache-root")
    p.add_argument("--sevenzip")
    p.add_argument("--patch")
    p.add_argument("--wget")
    p.add_argument("--cp")
    p.add_argument("--print-inputs", action="store_true")
    args = p.parse_args(argv)

    config = Config.load(Path(args.config))

    if args.print_inputs:
        for path in config.input_paths():
            print(path)
        return 0

    if args.output_root is None:
        raise UserError("--output-root is required unless --print-inputs is used")

    # Resolve tools.
    def tool(explicit: str | None, name: str) -> str:
        if explicit:
            found = shutil.which(explicit)
            if found is None:
                raise UserError(f"required tool not found: {explicit}")
            return found
        found = shutil.which(name)
        if found is None:
            raise UserError(f"required tool not found: {name}")
        return found

    # Resolve cache root.
    if args.cache_root:
        cache_root = Path(args.cache_root).resolve()
    else:
        xdg = os.environ.get("XDG_CACHE_HOME")
        base = Path(xdg) if xdg else Path.home() / ".cache"
        cache_root = base / config.namespace

    ArtifactPreparer(
        config,
        cache_root,
        Path(args.output_root).resolve(),
        sevenzip=tool(args.sevenzip, "7zz"),
        patch=tool(args.patch, "patch"),
        wget=tool(args.wget, "wget") if args.wget else shutil.which("wget"),
        cp=tool(args.cp, "cp"),
    ).run()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except UserError as error:
        log(f"error: {error}", stderr=True)
        raise SystemExit(1)
    except subprocess.CalledProcessError as error:
        log(
            f"error: command failed with exit code {error.returncode}: {error.cmd}",
            stderr=True,
        )
        raise SystemExit(error.returncode or 1)
