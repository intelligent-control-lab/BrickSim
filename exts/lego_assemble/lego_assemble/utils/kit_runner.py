import asyncio
import importlib
import os
import runpy
from typing import Any, Awaitable, Optional, Tuple

import carb.settings
import omni.kit.async_engine as _async_engine

_current_task: Optional[asyncio.Task[Any]] = None
_current_target: Optional[str] = None

_settings = carb.settings.get_settings()
_SETTING_HAS_TARGET = "/app/lego_assemble/kit_runner/has_target"
_SETTING_TARGET = "/app/lego_assemble/kit_runner/target"


def _parse_target(target: str) -> Tuple[str, str]:
    """
    Parse a target string into (module_or_path, func_name).

    Accepts either "module_or_path" or "module_or_path:func".
    If no function is specified, "main" is assumed.
    """
    if ":" in target:
        module_or_path, func_name = target.split(":", 1)
    else:
        module_or_path, func_name = target, "main"
    return module_or_path, func_name


def _is_path(s: str) -> bool:
    """
    Heuristic check whether the given string looks like a filesystem path.
    """
    return s.endswith(".py") or os.path.isabs(s) or (os.sep in s) or ("\\" in s)


def run(target: str, *args: Any, **kwargs: Any) -> asyncio.Task[Any]:
    """
    Run (and hot-reload) a target async function inside Kit.

    Args:
        target: Module name or filesystem path, optionally with ":func" suffix,
                e.g. "demos.demo_r1lite", "demos.demo_r1lite:main",
                "/abs/path/to/demo_r1lite.py", or "/abs/path/to/demo_r1lite.py:main".
        *args: Positional arguments forwarded to the target function.
        **kwargs: Keyword arguments forwarded to the target function.

    Returns:
        The asyncio.Task created by Kit's async engine for the coroutine.
    """
    global _current_task, _current_target

    # Cancel previous run if still active.
    if _current_task is not None and not _current_task.done():
        _current_task.cancel()

    _current_target = target
    # Publish to carb settings so other components (e.g., UI) can react.
    try:
        _settings.set(_SETTING_TARGET, target)
        _settings.set(_SETTING_HAS_TARGET, True)
    except Exception:
        # Settings may not be available in all contexts (e.g., headless tools).
        pass
    module_or_path, func_name = _parse_target(target)

    # Script path case: execute via runpy and fetch the target function from globals.
    if _is_path(module_or_path):
        script_path = module_or_path
        globals_dict = runpy.run_path(script_path, run_name="__lego_kit_runner__")
        func = globals_dict.get(func_name)
        if func is None:
            raise AttributeError(
                f"Target function '{func_name}' not found in script '{script_path}'"
            )
    else:
        module = importlib.import_module(module_or_path)
        module = importlib.reload(module)
        func = getattr(module, func_name)

    coro = func(*args, **kwargs)
    if not isinstance(coro, Awaitable):
        raise TypeError(
            f"Target '{module_or_path}:{func_name}' did not return an awaitable coroutine"
        )

    async def wrapper_coro():
        try:
            await coro
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"Exception in kit_runner target '{target}':")
            import traceback
            traceback.print_exc()
    task = _async_engine.run_coroutine(wrapper_coro())
    _current_task = task
    return task


def stop() -> None:
    """
    Cancel the currently running task, if any.
    """
    global _current_task
    if _current_task is not None and not _current_task.done():
        _current_task.cancel()
    _current_task = None


def has_target() -> bool:
    """
    Return True if a target has been run via kit_runner in this process.
    """
    return _current_target is not None


def current_target() -> Optional[str]:
    """
    Return the last target string passed to run(), if any.
    """
    return _current_target


def rerun() -> asyncio.Task[Any]:
    """
    Rerun the last target, if any.

    Returns:
        The asyncio.Task created by Kit's async engine for the coroutine.

    Raises:
        RuntimeError: If no previous target has been run.
    """
    if _current_target is None:
        raise RuntimeError("kit_runner.rerun() called but no previous target is stored")
    return run(_current_target)
