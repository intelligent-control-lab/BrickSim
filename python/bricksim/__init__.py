"""BrickSim Isaac Sim extension package."""

# Loads the extension when imported

try:
    import carb as _carb
except ModuleNotFoundError:
    _carb = None


def _is_bricksim_launcher_invocation() -> bool:
    import os
    import sys

    argv0 = os.path.basename(sys.argv[0]) if sys.argv else ""
    if argv0 == "bricksim" or argv0.startswith("bricksim-"):
        return True

    main_module = sys.modules.get("__main__")
    main_spec = getattr(main_module, "__spec__", None)
    main_name = getattr(main_spec, "name", None)
    if isinstance(main_name, str) and (
        main_name == "bricksim" or main_name.startswith("bricksim.")
    ):
        return True

    import inspect

    for frame in inspect.stack():
        if (
            frame.filename == "<frozen runpy>"
            and frame.function == "_run_module_as_main"
        ):
            return True
        if frame.function == "<module>" and (
            os.path.basename(frame.filename) == "bricksim"
            or os.path.basename(frame.filename).startswith("bricksim-")
        ):
            return True
    return False


def _ensure_bricksim_extension_enabled():
    import importlib.util
    from pathlib import Path

    import omni.ext
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_id = ext_manager.get_extension_id_by_module(__name__)
    if ext_id is not None and omni.ext.get_extension_name(ext_id) == "bricksim":
        # Extension is already loaded or being loaded
        return

    def _resolve_isaaclab_exts_dir() -> Path:
        spec = importlib.util.find_spec("isaaclab")
        if spec is None or spec.origin is None:
            raise RuntimeError("Required Python package not found: isaaclab")
        module_dir = Path(spec.origin).resolve().parent
        exts_dir = module_dir / "source"
        if (exts_dir / "isaaclab" / "config" / "extension.toml").is_file():
            return exts_dir.resolve()
        editable_source_root = module_dir.parent.parent
        if (editable_source_root / "isaaclab" / "config" / "extension.toml").is_file():
            return editable_source_root.resolve()
        raise RuntimeError(f"Unable to resolve Isaac Lab source root from {module_dir}")

    def _resolve_bricksim_exts_dir() -> Path:
        module_dir = Path(__file__).resolve().parent
        exts_dir = module_dir / "_exts"
        config_path = exts_dir / "bricksim" / "config" / "extension.toml"
        if not config_path.is_file():
            raise RuntimeError(
                f"Unable to resolve BrickSim extension root from {module_dir}"
            )
        return exts_dir.resolve()

    def _add_extension_path(path: Path) -> None:
        resolved_path = path.resolve()
        for folder in ext_manager.get_folders():
            folder_path = folder.get("path")
            if folder_path is not None and Path(folder_path).resolve() == resolved_path:
                return
        ext_manager.add_path(
            str(resolved_path), omni.ext.ExtensionPathType.EXT_1_FOLDER
        )

    _add_extension_path(_resolve_isaaclab_exts_dir())
    _add_extension_path(_resolve_bricksim_exts_dir())
    if not ext_manager.set_extension_enabled_immediate("bricksim", True):
        raise RuntimeError("Failed to enable BrickSim extension")


if _carb is None:
    if not _is_bricksim_launcher_invocation():
        from warnings import warn as _warn

        _warn(
            "BrickSim is loaded outside Omniverse, some features may not work "
            "properly.",
            UserWarning,
            stacklevel=2,
        )
else:
    from . import (
        core as _core,  # noqa: F401  # Load native library
    )
    from . import (
        envs as _envs,  # noqa: F401  # Register all environments
    )

    # Ensure extension class is registered with Omniverse.
    from .extension import BrickSimExtension as BrickSimExtension

    _ensure_bricksim_extension_enabled()
