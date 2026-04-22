#!/usr/bin/env python3
"""Resolve BrickSim's Isaac Sim dependency catalog from an installed Isaac Sim tree."""

import argparse
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import tomllib
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


PACKMAN_BASE_URL = "https://d4i3qtqj3r0z5.cloudfront.net"
DEFAULT_CACHE_NAMESPACE = "bricksim/isaac-deps"
PLATFORM = "linux-x86_64"
PLATFORM_TARGET_ABI = "manylinux_2_35_x86_64"
BUILD_CONFIG = "release"


@dataclass(frozen=True)
class PatchSpec:
    path: str
    root: str


@dataclass(frozen=True)
class ArtifactSpec:
    name: str
    url: str
    sha256: str
    patches: tuple[PatchSpec, ...] = ()


class ResolverError(RuntimeError):
    pass


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Resolve BrickSim's Isaac Sim dependency catalog from an installed Isaac Sim tree."
    )
    parser.add_argument(
        "--python",
        dest="python_executable",
        help="Interpreter whose environment contains the isaacsim package.",
    )
    parser.add_argument(
        "--isaacsim-root",
        type=Path,
        help="Explicit Isaac Sim install root. Overrides --python.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Path to write the resolved TOML catalog.",
    )
    parser.add_argument(
        "--cache-root",
        type=Path,
        help="Archive cache root. Defaults to ~/.cache/bricksim/isaac-deps.",
    )
    parser.add_argument(
        "--packman-cache-root",
        type=Path,
        help="Packman extracted package cache. Defaults to ~/.cache/packman/chk.",
    )
    args = parser.parse_args()

    cache_root = (
        args.cache_root.resolve()
        if args.cache_root is not None
        else default_cache_root()
    )
    packman_cache_root = (
        args.packman_cache_root.resolve()
        if args.packman_cache_root is not None
        else Path.home() / ".cache" / "packman" / "chk"
    )

    install_root = resolve_install_root(args.isaacsim_root, args.python_executable)
    ensure_install_root(install_root)

    specs = resolve_catalog(install_root, cache_root, packman_cache_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render_toml(specs), encoding="utf-8")
    print(f"Wrote {args.output}")
    return 0


def default_cache_root() -> Path:
    xdg_cache_home = os.environ.get("XDG_CACHE_HOME")
    base = Path(xdg_cache_home) if xdg_cache_home else Path.home() / ".cache"
    return (base / DEFAULT_CACHE_NAMESPACE).resolve()


def resolve_install_root(explicit_root: Path | None, python_executable: str | None) -> Path:
    if explicit_root is not None:
        return explicit_root.resolve()

    python = python_executable or sys.executable
    probe = subprocess.run(
        [
            python,
            "-c",
            (
                "import importlib.util, pathlib, sys; "
                "spec = importlib.util.find_spec('isaacsim'); "
                "if spec is None: raise SystemExit('isaacsim is not importable'); "
                "print(pathlib.Path(spec.origin).resolve().parent)"
            ),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if probe.returncode != 0:
        stderr = probe.stderr.strip() or probe.stdout.strip() or "isaacsim import probe failed"
        raise ResolverError(
            f"Could not resolve isaacsim install root from {python}: {stderr}"
        )
    return Path(probe.stdout.strip()).resolve()


def ensure_install_root(install_root: Path) -> None:
    required = (
        install_root / "kit",
        install_root / "extscache",
    )
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise ResolverError(
            f"{install_root} does not look like an Isaac Sim install root; missing: {', '.join(missing)}"
        )


def resolve_catalog(
    install_root: Path, cache_root: Path, packman_cache_root: Path
) -> list[ArtifactSpec]:
    all_deps_path = install_root / "kit" / "dev" / "all-deps.packman.xml"
    if not all_deps_path.is_file():
        raise ResolverError(f"Missing Kit dependency manifest: {all_deps_path}")

    kit_name, kit_version = resolve_kit_package(install_root)
    carb_name, carb_version = read_dependency_package(
        all_deps_path, dependency_name="carb_sdk_plugins"
    )
    omni_client_name, omni_client_version = read_dependency_package(
        all_deps_path, dependency_name="omni_client_library"
    )
    python_name, python_version = read_dependency_package(
        all_deps_path, dependency_name="python"
    )
    usd_name, usd_version = read_dependency_package(
        all_deps_path, dependency_name="usd-release"
    )

    omni_physics_root, omni_physics_version = locate_omni_physics_package(
        install_root, packman_cache_root
    )
    physx_name, physx_version = read_dependency_package(
        omni_physics_root / "deps" / "target-deps.packman.xml",
        dependency_name="physx",
        preferred_platform=PLATFORM_TARGET_ABI,
    )
    usd_ext_physics_name, usd_ext_physics_version = read_dependency_package(
        omni_physics_root / "deps" / "schema-deps.packman.xml",
        dependency_name="usd_ext_physics_${config}",
    )

    omni_usd_core_url = read_extension_archive_path(install_root, "omni.usd.core")
    omni_usd_schema_audio_url = read_extension_archive_path(
        install_root, "omni.usd.schema.audio"
    )

    packman_urls = {
        "kit": make_packman_url(kit_name, kit_version, ".zip"),
        "carb": make_packman_url(carb_name, carb_version, ".7z"),
        "omni_client": make_packman_url(omni_client_name, omni_client_version, ".7z"),
        "physx": make_packman_url(physx_name, physx_version, ".zip"),
        "usd": make_packman_url(usd_name, usd_version, ".7z"),
        "usd_ext_physics": make_packman_url(
            usd_ext_physics_name, usd_ext_physics_version, ".7z"
        ),
        "omni_physics": make_packman_url(
            "omni_physics", omni_physics_version, ".7z"
        ),
        "python": make_packman_url(python_name, python_version, ".7z"),
        "omni_usd_core": omni_usd_core_url,
        "omni_usd_schema_audio": omni_usd_schema_audio_url,
    }

    patch_map = {
        "kit": (PatchSpec("sdk_patches/kit_include.patch", "dev/include"),),
        "usd": (PatchSpec("sdk_patches/usd_include.patch", "include"),),
    }
    ordered_names = (
        "kit",
        "carb",
        "omni_client",
        "physx",
        "usd",
        "usd_ext_physics",
        "omni_physics",
        "python",
        "omni_usd_core",
        "omni_usd_schema_audio",
    )

    specs: list[ArtifactSpec] = []
    for artifact_name in ordered_names:
        url = packman_urls[artifact_name]
        sha256 = download_and_hash(cache_root, artifact_name, url)
        specs.append(
            ArtifactSpec(
                name=artifact_name,
                url=url,
                sha256=sha256,
                patches=patch_map.get(artifact_name, ()),
            )
        )
    return specs


def resolve_kit_package(install_root: Path) -> tuple[str, str]:
    kit_path = install_root / "kit"
    resolved = kit_path.resolve()
    packman_root = Path.home() / ".cache" / "packman" / "chk"
    try:
        rel = resolved.relative_to(packman_root)
    except ValueError:
        rel = None
    if rel is not None and len(rel.parts) >= 2:
        return rel.parts[0], rel.parts[1]

    manifest = install_root / "dev" / "deps" / "kit-sdk.packman.xml"
    if not manifest.is_file():
        raise ResolverError(f"Missing Kit SDK manifest: {manifest}")
    name, version = read_first_package(manifest)
    return name, substitute_packman_tokens(version)


def locate_omni_physics_package(
    install_root: Path, packman_cache_root: Path
) -> tuple[Path, str]:
    build_number = read_extension_build_number(install_root, "omni.physx")
    version = read_extension_version(install_root, "omni.physx")
    build_hash = build_number.split(".")[-2]

    root = packman_cache_root / "omni_physics"
    if not root.is_dir():
        raise ResolverError(f"Missing local omni_physics packman cache: {root}")

    matches = sorted(
        entry
        for entry in root.iterdir()
        if entry.is_dir()
        and entry.name.startswith(f"{version}-")
        and build_hash in entry.name
        and PLATFORM_TARGET_ABI in entry.name
    )
    if not matches:
        raise ResolverError(
            "Could not map installed omni.physx build metadata to a local omni_physics packman package. "
            f"Needed version={version}, build hash={build_hash}, platform={PLATFORM_TARGET_ABI} under {root}"
        )
    if len(matches) != 1:
        raise ResolverError(
            f"Ambiguous omni_physics package matches for version={version}, build hash={build_hash}: "
            + ", ".join(entry.name for entry in matches)
        )
    return matches[0], matches[0].name


def read_extension_build_number(install_root: Path, extension_name: str) -> str:
    package = read_extension_generated_package(install_root, extension_name)
    publish = read_required_table(package, "publish", f"{extension_name} package")
    build_number = publish.get("buildNumber")
    if not isinstance(build_number, str) or not build_number:
        raise ResolverError(f"{extension_name} extension.gen.toml is missing package.publish.buildNumber")
    return build_number


def read_extension_version(install_root: Path, extension_name: str) -> str:
    extension_toml = read_extension_config(install_root, extension_name, "extension.toml")
    package = read_required_table(extension_toml, "package", f"{extension_name} extension")
    version = package.get("version")
    if not isinstance(version, str) or not version:
        raise ResolverError(f"{extension_name} extension.toml is missing package.version")
    return version


def read_extension_archive_path(install_root: Path, extension_name: str) -> str:
    package = read_extension_generated_package(install_root, extension_name)
    archive_path = package.get("archivePath")
    if not isinstance(archive_path, str) or not archive_path:
        raise ResolverError(
            f"{extension_name} extension.gen.toml is missing package.archivePath"
        )
    return normalize_url(archive_path)


def read_extension_generated_package(
    install_root: Path, extension_name: str
) -> dict[str, object]:
    data = read_extension_config(install_root, extension_name, "extension.gen.toml")
    return read_required_table(data, "package", f"{extension_name} extension.gen")


def read_extension_config(
    install_root: Path, extension_name: str, filename: str
) -> dict[str, object]:
    extscache_root = install_root / "extscache"
    candidates = sorted(extscache_root.glob(f"{extension_name}-*/config/{filename}"))
    if not candidates:
        raise ResolverError(
            f"Could not find {filename} for installed extension {extension_name} under {extscache_root}"
        )

    preferred = select_preferred_extension_config(candidates)
    with preferred.open("rb") as file:
        return tomllib.load(file)


def select_preferred_extension_config(candidates: list[Path]) -> Path:
    def rank(path: Path) -> tuple[int, int, str]:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
        package = data.get("package", {})
        target = package.get("target", {})
        config = flatten_toml_values(target.get("config"))
        platform = flatten_toml_values(target.get("platform"))
        score = 0
        if BUILD_CONFIG in config:
            score += 2
        if PLATFORM in platform:
            score += 1
        return (-score, len(path.parent.parent.name), path.parent.parent.name)

    return sorted(candidates, key=rank)[0]


def flatten_toml_values(raw: object) -> list[str]:
    if isinstance(raw, str):
        return [raw]
    if isinstance(raw, list):
        return [item for item in raw if isinstance(item, str)]
    if isinstance(raw, dict):
        values = []
        for key in sorted(raw):
            value = raw[key]
            if isinstance(value, str):
                values.append(value)
        return values
    return []


def read_first_package(xml_path: Path) -> tuple[str, str]:
    root = parse_xml(xml_path)
    package = root.find(".//package")
    if package is None:
        raise ResolverError(f"No <package> entry found in {xml_path}")
    name = package.get("name")
    version = package.get("version")
    if not name or not version:
        raise ResolverError(f"Malformed <package> entry in {xml_path}")
    return name, version


def read_dependency_package(
    xml_path: Path,
    *,
    dependency_name: str,
    preferred_platform: str | None = None,
) -> tuple[str, str]:
    root = parse_xml(xml_path)
    dependency = None
    for candidate in root.findall("dependency"):
        if candidate.get("name") == dependency_name:
            dependency = candidate
            break
    if dependency is None:
        raise ResolverError(f"Missing dependency {dependency_name} in {xml_path}")

    packages = dependency.findall("package")
    if not packages:
        raise ResolverError(f"Dependency {dependency_name} in {xml_path} has no package entries")

    selected = select_package_by_platform(packages, preferred_platform)
    name = selected.get("name")
    version = selected.get("version")
    if not name or not version:
        raise ResolverError(f"Malformed package entry for {dependency_name} in {xml_path}")
    return name, substitute_packman_tokens(version)


def select_package_by_platform(
    packages: list[ET.Element], preferred_platform: str | None
) -> ET.Element:
    if preferred_platform is None:
        return packages[0]
    for package in packages:
        platforms = package.get("platforms", "")
        if preferred_platform in platforms.split():
            return package
    for package in packages:
        platforms = package.get("platforms", "")
        if PLATFORM in platforms.split():
            return package
    return packages[0]


def parse_xml(xml_path: Path) -> ET.Element:
    try:
        return ET.parse(xml_path).getroot()
    except ET.ParseError as error:
        raise ResolverError(f"Failed to parse XML {xml_path}: {error}") from error


def substitute_packman_tokens(value: str) -> str:
    return (
        value.replace("${platform_target_abi}", PLATFORM_TARGET_ABI)
        .replace("${platform_target}", PLATFORM)
        .replace("${config}", BUILD_CONFIG)
    )


def make_packman_url(package_name: str, version: str, archive_suffix: str) -> str:
    encoded = urllib.parse.quote(
        f"{package_name}@{version}{archive_suffix}",
        safe="._-",
    )
    return f"{PACKMAN_BASE_URL}/{encoded}"


def normalize_url(url: str) -> str:
    return url.replace("http://", "https://")


def download_and_hash(cache_root: Path, artifact_name: str, url: str) -> str:
    cache_root.mkdir(parents=True, exist_ok=True)
    url_map_path = cache_root / "resolver-url-map.json"
    url_map = read_url_map(url_map_path)
    cached_sha = url_map.get(url)
    if cached_sha:
        cached_archive = cache_root / "archives" / cached_sha / "archive"
        if cached_archive.is_file():
            print(f"Reusing {artifact_name}: {cached_archive}", flush=True)
            return cached_sha

    tmp_dir = cache_root / ".resolver-tmp"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(
        prefix=f"{artifact_name}.",
        suffix=".download",
        dir=tmp_dir,
    )
    os.close(fd)
    tmp_path = Path(tmp_name)
    digest = hashlib.sha256()
    try:
        print(f"Downloading {artifact_name}: {url}", flush=True)
        request = urllib.request.Request(url, headers={"User-Agent": "BrickSim dep resolver"})
        with urllib.request.urlopen(request) as response, tmp_path.open("wb") as output:
            while True:
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
                output.write(chunk)

        sha256 = digest.hexdigest()
        archive_dir = cache_root / "archives" / sha256
        archive_dir.mkdir(parents=True, exist_ok=True)
        archive_path = archive_dir / "archive"
        if not archive_path.exists():
            os.replace(tmp_path, archive_path)
        else:
            tmp_path.unlink()

        url_map[url] = sha256
        write_url_map(url_map_path, url_map)
        return sha256
    except Exception:
        tmp_path.unlink(missing_ok=True)
        raise


def read_url_map(url_map_path: Path) -> dict[str, str]:
    if not url_map_path.is_file():
        return {}
    try:
        data = json.loads(url_map_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ResolverError(f"Invalid resolver URL cache {url_map_path}: {error}") from error
    if not isinstance(data, dict):
        raise ResolverError(f"Invalid resolver URL cache {url_map_path}: expected object")
    result: dict[str, str] = {}
    for key, value in data.items():
        if isinstance(key, str) and isinstance(value, str):
            result[key] = value
    return result


def write_url_map(url_map_path: Path, url_map: dict[str, str]) -> None:
    url_map_path.write_text(
        json.dumps(dict(sorted(url_map.items())), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def read_required_table(
    table: dict[str, object], key: str, context: str
) -> dict[str, object]:
    value = table.get(key)
    if not isinstance(value, dict):
        raise ResolverError(f"{context} is missing table {key}")
    return value


def render_toml(specs: list[ArtifactSpec]) -> str:
    lines = [
        "[cache]",
        f'namespace = "{DEFAULT_CACHE_NAMESPACE}"',
        "",
    ]
    for index, spec in enumerate(specs):
        lines.append("[[artifacts]]")
        lines.append(f'name = "{spec.name}"')
        lines.append(f'url = "{spec.url}"')
        lines.append(f'sha256 = "{spec.sha256}"')
        if spec.patches:
            lines.append("")
            for patch in spec.patches:
                lines.append("[[artifacts.patches]]")
                lines.append(f'path = "{patch.path}"')
                lines.append(f'root = "{patch.root}"')
        if index != len(specs) - 1:
            lines.append("")
    lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ResolverError as error:
        print(f"[resolve_isaacsim_deps] {error}", file=sys.stderr)
        raise SystemExit(1)
