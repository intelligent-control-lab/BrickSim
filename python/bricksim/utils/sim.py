"""Async helpers for stepping Isaac Sim worlds."""

import asyncio
from collections.abc import Awaitable
from typing import Protocol, runtime_checkable

from isaacsim.core.api.world import World
from pxr import Usd


@runtime_checkable
class _UsdContextWithStage(Protocol):
    def get_stage(self) -> Usd.Stage | None: ...


@runtime_checkable
class _AppWithNextUpdate(Protocol):
    def next_update_async(self) -> Awaitable[float]: ...


def get_current_stage() -> Usd.Stage | None:
    """Return the current USD stage."""
    import omni.usd

    context = omni.usd.get_context()
    if not isinstance(context, _UsdContextWithStage):
        raise TypeError("USD context does not expose get_stage().")
    return context.get_stage()


async def wait_for_next_update() -> float:
    """Wait for the next Kit app update.

    Returns:
        Delta time in seconds.
    """
    import omni.kit.app

    app = omni.kit.app.get_app()
    if not isinstance(app, _AppWithNextUpdate):
        raise TypeError("Kit app does not expose next_update_async().")
    return await app.next_update_async()


async def wait_for_physics_step(world: World) -> float:
    """Wait for the next physics callback.

    Returns:
        Physics step size in seconds.
    """
    loop = asyncio.get_event_loop()
    fut: asyncio.Future[float] = loop.create_future()
    callback_name = "__lego_next_step__"

    def on_step(step_size: float):
        if not fut.done():
            # resolve future on the asyncio loop thread
            loop.call_soon_threadsafe(fut.set_result, float(step_size))
        # one-shot callback
        world.remove_physics_callback(callback_name)

    world.add_physics_callback(callback_name, on_step)
    return await fut


async def wait_for_duration(world: World, duration: float) -> None:
    """Wait until physics time advances by at least ``duration`` seconds."""
    elapsed = 0.0
    while elapsed < duration:
        step_size = await wait_for_physics_step(world)
        elapsed += step_size
