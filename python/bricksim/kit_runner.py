"""Hot-reload runner for executing Python targets inside Kit."""

import asyncio
import contextlib
import importlib
import os
import runpy
import sys
from collections.abc import Awaitable, Callable, Iterator, Mapping, Sequence
from typing import Optional, TypeAlias, TypeGuard

import carb.settings
import omni.kit.async_engine as _async_engine

TargetFunction: TypeAlias = Callable[[], object]

_current_task: Optional[asyncio.Task[None]] = None
_current_target: Optional[str] = None
_current_cli_args: tuple[str, ...] = ()

_settings = carb.settings.get_settings()
_SETTING_HAS_TARGET = "/app/bricksim/kit_runner/has_target"
_SETTING_TARGET = "/app/bricksim/kit_runner/target"


def _parse_target(target: str) -> tuple[str, str]:
    """Parse a target string into (module_or_path, func_name).

    Accepts either "module_or_path" or "module_or_path:func".
    If no function is specified, "main" is assumed.

    Returns:
        tuple of module/path string and function name.
    """
    if ":" in target:
        module_or_path, func_name = target.split(":", 1)
    else:
        module_or_path, func_name = target, "main"
    return module_or_path, func_name


def _is_path(s: str) -> bool:
    """Heuristic check whether the given string looks like a filesystem path.

    Returns:
        ``True`` if the string looks like a Python file path.
    """
    return s.endswith(".py") or os.path.isabs(s) or (os.sep in s) or ("\\" in s)


@contextlib.contextmanager
def _temporary_argv(argv0: str, cli_args: Sequence[str]) -> Iterator[None]:
    old_argv = sys.argv.copy()
    sys.argv = [argv0, *cli_args]
    try:
        yield
    finally:
        sys.argv = old_argv


def _is_target_function(value: object) -> TypeGuard[TargetFunction]:
    return callable(value)


def _is_awaitable(value: object) -> TypeGuard[Awaitable[object]]:
    return isinstance(value, Awaitable)


def _is_none_task(value: object) -> TypeGuard[asyncio.Task[None]]:
    return isinstance(value, asyncio.Task)


def _resolve_target_function(
    module_or_path: str,
    func_name: str,
    cli_args: tuple[str, ...],
) -> TargetFunction:
    # Script path case: execute via runpy and fetch the target function from globals.
    func: object
    if _is_path(module_or_path):
        with _temporary_argv(module_or_path, cli_args):
            globals_dict: Mapping[str, object] = runpy.run_path(
                module_or_path, run_name="__lego_kit_runner__"
            )
        if func_name not in globals_dict:
            raise AttributeError(
                f"Target function '{func_name}' not found in script '{module_or_path}'"
            )
        func = globals_dict[func_name]
    else:
        with _temporary_argv(module_or_path, cli_args):
            module = importlib.import_module(module_or_path)
            module = importlib.reload(module)
        func = getattr(module, func_name)

    if not _is_target_function(func):
        raise TypeError(f"Target '{module_or_path}:{func_name}' is not callable")
    return func


def run(
    target: str,
    *,
    cli_args: Sequence[str] = (),
) -> asyncio.Task[None]:
    """Run (and hot-reload) a target async function inside Kit.

    Args:
        target: Module name or filesystem path, optionally with ":func" suffix,
                e.g. "demos.demo_r1lite", "demos.demo_r1lite:main",
                "/abs/path/to/demo_r1lite.py", or "/abs/path/to/demo_r1lite.py:main".
        cli_args: CLI arguments exposed to the target via sys.argv.

    Returns:
        The asyncio.Task created by Kit's async engine for the coroutine.
    """
    global _current_task, _current_target, _current_cli_args

    # Cancel previous run if still active.
    if _current_task is not None and not _current_task.done():
        _current_task.cancel()

    run_cli_args = tuple(cli_args)
    _current_target = target
    _current_cli_args = run_cli_args
    # Publish to carb settings so other components (e.g., UI) can react.
    try:
        _settings.set(_SETTING_TARGET, target)
        _settings.set(_SETTING_HAS_TARGET, True)
    except Exception:
        # Settings may not be available in all contexts (e.g., headless tools).
        pass
    module_or_path, func_name = _parse_target(target)
    func = _resolve_target_function(module_or_path, func_name, run_cli_args)

    with _temporary_argv(module_or_path, run_cli_args):
        coro = func()
    if not _is_awaitable(coro):
        raise TypeError(
            f"Target '{module_or_path}:{func_name}' did not return an "
            "awaitable coroutine"
        )

    async def wrapper_coro() -> None:
        try:
            with _temporary_argv(module_or_path, run_cli_args):
                await coro
        except asyncio.CancelledError:
            pass
        except Exception:
            print(f"Exception in kit_runner target '{target}':")
            import traceback

            traceback.print_exc()

    task: object = _async_engine.run_coroutine(wrapper_coro())
    if not _is_none_task(task):
        raise TypeError("Kit async engine did not return an asyncio.Task")

    _current_task = task
    return task


def stop() -> None:
    """Cancel the currently running task, if any."""
    global _current_task
    if _current_task is not None and not _current_task.done():
        _current_task.cancel()
    _current_task = None


def has_target() -> bool:
    """Return True if a target has been run via kit_runner in this process."""
    return _current_target is not None


def current_target() -> Optional[str]:
    """Return the last target string passed to run(), if any."""
    return _current_target


def rerun() -> asyncio.Task[None]:
    """Rerun the last target, if any.

    Returns:
        The asyncio.Task created by Kit's async engine for the coroutine.

    Raises:
        RuntimeError: If no previous target has been run.
    """
    if _current_target is None:
        raise RuntimeError("kit_runner.rerun() called but no previous target is stored")
    return run(_current_target, cli_args=_current_cli_args)
