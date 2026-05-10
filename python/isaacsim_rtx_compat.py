"""Enable a generated Vulkan profile before application startup.

Recent NVIDIA drivers can report
``VkPhysicalDeviceMaintenance3Properties.maxMemoryAllocationSize`` as
``UINT64_MAX`` where the same GPU previously reported a finite value just below
4 GiB. Some RTX initialization paths treat that value as a practical allocation
limit when sizing acceleration-structure resources, so the sentinel value can
push closed-source renderer code into an invalid allocation path before the app
finishes startup.

This module must be imported before the target application initializes Vulkan.
It queries the local NVIDIA device, computes a conservative finite cap, downloads
and verifies the Khronos Vulkan Profiles layer into XDG cache when needed, then
generates a cache-local profile and implicit-layer manifest. The profile changes
only the reported ``maxMemoryAllocationSize`` property; renderer settings,
features, and application arguments are left unchanged. The profile is enabled
only for NVIDIA Vulkan driver versions newer than 590.48.01 that also report an
unsafe allocation limit; older or already-safe drivers are left untouched.
"""

from ctypes import (
    CDLL,
    POINTER,
    Structure,
    addressof,
    byref,
    c_char,
    c_char_p,
    c_int32,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
)
from hashlib import sha256
from json import dump
from os import environ
from pathlib import Path
from shutil import which
from subprocess import run
from sys import stderr
from urllib.request import urlretrieve

_URL = "https://geo.mirror.pkgbuild.com/extra/os/x86_64/vulkan-profiles-1.4.341.0-1-x86_64.pkg.tar.zst"
_PKG = "d52d72259b6c8911c9e3b41985e014cb76954f013b87af45f5ef1fcff1126131"
_SO = "99f20d7af6e073eac74a04308be01a3a81505629cff6dc066a7d3282b4dce533"
_NAME = "VP_RTX_driver_compat_generated"
_NVIDIA = 0x10DE
_LAST_KNOWN_GOOD_DRIVER = (590, 48, 1, 0)
_TWO_MIB = 2 * 1024 * 1024
_CAP = 4 * 1024 * 1024 * 1024 - _TWO_MIB
_CACHE = (
    Path(environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "vulkan-rtx-compat"
)


class _App(Structure):
    _fields_ = [
        ("sType", c_uint32),
        ("pNext", c_void_p),
        ("pApplicationName", c_char_p),
        ("applicationVersion", c_uint32),
        ("pEngineName", c_char_p),
        ("engineVersion", c_uint32),
        ("apiVersion", c_uint32),
    ]


class _Create(Structure):
    _fields_ = [
        ("sType", c_uint32),
        ("pNext", c_void_p),
        ("flags", c_uint32),
        ("pApplicationInfo", c_void_p),
        ("enabledLayerCount", c_uint32),
        ("ppEnabledLayerNames", c_void_p),
        ("enabledExtensionCount", c_uint32),
        ("ppEnabledExtensionNames", c_void_p),
    ]


class _Props(Structure):
    _fields_ = [
        ("apiVersion", c_uint32),
        ("driverVersion", c_uint32),
        ("vendorID", c_uint32),
        ("deviceID", c_uint32),
        ("deviceType", c_uint32),
        ("deviceName", c_char * 256),
        ("pipelineCacheUUID", c_uint8 * 16),
        ("_rest", c_uint8 * 2048),
    ]


class _Props2(Structure):
    _fields_ = [("sType", c_uint32), ("pNext", c_void_p), ("properties", _Props)]


class _Maintenance3(Structure):
    _fields_ = [
        ("sType", c_uint32),
        ("pNext", c_void_p),
        ("maxPerSetDescriptors", c_uint32),
        ("maxMemoryAllocationSize", c_uint64),
    ]


class _MemoryType(Structure):
    _fields_ = [("propertyFlags", c_uint32), ("heapIndex", c_uint32)]


class _MemoryHeap(Structure):
    _fields_ = [("size", c_uint64), ("flags", c_uint32)]


class _MemoryProps(Structure):
    _fields_ = [
        ("memoryTypeCount", c_uint32),
        ("memoryTypes", _MemoryType * 32),
        ("memoryHeapCount", c_uint32),
        ("memoryHeaps", _MemoryHeap * 16),
    ]


def _hash(path: Path) -> str:
    data = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            data.update(chunk)
    return data.hexdigest()


def _prepend(name: str, value: Path) -> None:
    text = str(value)
    parts = environ.get(name, "").split(":") if environ.get(name) else []
    if text not in parts:
        environ[name] = f"{text}:{environ[name]}" if environ.get(name) else text


def _layer() -> Path:
    layer = _CACHE / "libVkLayer_khronos_profiles.so"
    if layer.exists() and _hash(layer) == _SO:
        return layer
    force_download = layer.exists()
    if which("tar") is None:
        msg = "tar is required to extract the Vulkan profiles layer"
        raise RuntimeError(msg)
    _CACHE.mkdir(parents=True, exist_ok=True)
    package = _CACHE / "vulkan-profiles-1.4.341.0-1-x86_64.pkg.tar.zst"
    if force_download:
        package.unlink(missing_ok=True)
    if not package.exists() or _hash(package) != _PKG:
        tmp = package.with_suffix(".download")
        urlretrieve(_URL, tmp)
        if _hash(tmp) != _PKG:
            tmp.unlink(missing_ok=True)
            msg = "downloaded Vulkan profiles package failed sha256 verification"
            raise RuntimeError(msg)
        tmp.replace(package)
    extracted = run(
        [
            "tar",
            "--zstd",
            "-xOf",
            str(package),
            "usr/lib/libVkLayer_khronos_profiles.so",
        ],
        check=True,
        capture_output=True,
    ).stdout
    layer.write_bytes(extracted)
    layer.chmod(0o755)
    if _hash(layer) != _SO:
        layer.unlink(missing_ok=True)
        msg = "extracted Vulkan profiles layer failed sha256 verification"
        raise RuntimeError(msg)
    return layer


def _limit() -> int | None:
    override = environ.get("RTX_VULKAN_COMPAT_MAX_MEMORY_ALLOCATION_SIZE")
    if override is not None:
        return int(override, 0)
    vk = CDLL("libvulkan.so.1")
    create_instance = vk.vkCreateInstance
    create_instance.argtypes = [POINTER(_Create), c_void_p, POINTER(c_void_p)]
    create_instance.restype = c_int32
    enumerate_devices = vk.vkEnumeratePhysicalDevices
    enumerate_devices.argtypes = [c_void_p, POINTER(c_uint32), POINTER(c_void_p)]
    enumerate_devices.restype = c_int32
    get_props = vk.vkGetPhysicalDeviceProperties2
    get_props.argtypes = [c_void_p, POINTER(_Props2)]
    get_memory = vk.vkGetPhysicalDeviceMemoryProperties
    get_memory.argtypes = [c_void_p, POINTER(_MemoryProps)]
    destroy_instance = vk.vkDestroyInstance
    destroy_instance.argtypes = [c_void_p, c_void_p]
    app = _App(
        sType=0, pApplicationName=b"vulkan-rtx-compat", apiVersion=(1 << 22) | (1 << 12)
    )
    info = _Create(sType=1, pApplicationInfo=c_void_p(addressof(app)))
    instance = c_void_p()
    if create_instance(byref(info), None, byref(instance)) != 0:
        return None
    caps: list[int] = []
    try:
        count = c_uint32()
        if enumerate_devices(instance, byref(count), None) != 0 or count.value == 0:
            return None
        devices = (c_void_p * count.value)()
        if enumerate_devices(instance, byref(count), devices) != 0:
            return None
        for device in devices:
            m3 = _Maintenance3(sType=1000168000)
            props = _Props2(sType=1000059001, pNext=c_void_p(addressof(m3)))
            get_props(device, byref(props))
            driver = int(props.properties.driverVersion)
            if (
                props.properties.vendorID != _NVIDIA
                or (
                    driver >> 22,
                    (driver >> 14) & 0xFF,
                    (driver >> 6) & 0xFF,
                    driver & 0x3F,
                )
                <= _LAST_KNOWN_GOOD_DRIVER
                or int(m3.maxMemoryAllocationSize) <= _CAP
            ):
                continue
            mem = _MemoryProps()
            get_memory(device, byref(mem))
            heap = max(
                (
                    int(mem.memoryHeaps[i].size)
                    for i in range(mem.memoryHeapCount)
                    if mem.memoryHeaps[i].flags & 1
                ),
                default=0,
            )
            caps.append(min(_CAP, heap - _TWO_MIB) if heap > _TWO_MIB else _CAP)
    finally:
        destroy_instance(instance, None)
    return ((min(caps) // 256) * 256) if caps else None


def enable() -> bool:
    """Enable the compatibility profile when the current Vulkan device needs it.

    Returns:
        True if the profile layer was enabled, otherwise False.
    """
    if environ.get("RTX_VULKAN_COMPAT_ACTIVE") == "1":
        return True
    if environ.get("RTX_VULKAN_COMPAT_DISABLE") == "1":
        return False
    limit = _limit()
    if limit is None or limit <= 0:
        return False
    layer = _layer()
    profile_dir = _CACHE / "profiles"
    data_root = _CACHE / "xdg-data"
    manifest_dir = data_root / "vulkan" / "implicit_layer.d"
    profile_dir.mkdir(parents=True, exist_ok=True)
    manifest_dir.mkdir(parents=True, exist_ok=True)
    with (profile_dir / f"{_NAME}.json").open("w", encoding="utf-8") as stream:
        dump(
            {
                "$schema": "https://schema.khronos.org/vulkan/profiles-0.8-latest.json#",
                "capabilities": {
                    "RTX_DRIVER_COMPAT": {
                        "properties": {
                            "VkPhysicalDeviceMaintenance3Properties": {
                                "maxMemoryAllocationSize": limit
                            }
                        }
                    }
                },
                "profiles": {
                    _NAME: {
                        "version": 1,
                        "api-version": "1.1.0",
                        "label": "RTX Driver Compatibility",
                        "description": "Generated compatibility profile.",
                        "contributors": {
                            "local": {"github": "https://github.com/yushijinhun"}
                        },
                        "history": [
                            {
                                "revision": 1,
                                "date": "2026-05-10",
                                "author": "local",
                                "comment": "Generated compatibility profile.",
                            }
                        ],
                        "capabilities": ["RTX_DRIVER_COMPAT"],
                    }
                },
            },
            stream,
            indent=2,
        )
        stream.write("\n")
    with (manifest_dir / "VkLayer_KHRONOS_profiles.json").open(
        "w", encoding="utf-8"
    ) as stream:
        dump(
            {
                "file_format_version": "1.2.1",
                "layer": {
                    "name": "VK_LAYER_KHRONOS_profiles",
                    "type": "GLOBAL",
                    "library_path": str(layer),
                    "api_version": "1.4.341",
                    "implementation_version": "1",
                    "description": "Khronos Profiles layer",
                    "enable_environment": {"RTX_VULKAN_COMPAT_ENABLE_LAYER": "1"},
                    "disable_environment": {"RTX_VULKAN_COMPAT_DISABLE_LAYER": ""},
                },
            },
            stream,
            indent=2,
        )
        stream.write("\n")
    environ["RTX_VULKAN_COMPAT_ENABLE_LAYER"] = "1"
    environ["VK_KHRONOS_PROFILES_PROFILE_NAME"] = _NAME
    environ["VK_KHRONOS_PROFILES_SIMULATE_CAPABILITIES"] = "SIMULATE_PROPERTIES_BIT"
    environ["VK_KHRONOS_PROFILES_DEBUG_REPORTS"] = "DEBUG_REPORT_ERROR_BIT"
    environ["RTX_VULKAN_COMPAT_ACTIVE"] = "1"
    environ["RTX_VULKAN_COMPAT_MAX_MEMORY_ALLOCATION_SIZE_ACTIVE"] = str(limit)
    _prepend("VK_KHRONOS_PROFILES_PROFILE_DIRS", profile_dir)
    _prepend("VK_ADD_IMPLICIT_LAYER_PATH", manifest_dir)
    _prepend("XDG_DATA_DIRS", data_root)
    print(
        "RTX Vulkan compatibility profile enabled: "
        f"maxMemoryAllocationSize={limit}, cache={_CACHE}",
        file=stderr,
    )
    return True


enabled = enable()
