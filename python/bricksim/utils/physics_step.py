"""Async helpers for waiting on Isaac Sim physics steps."""

import asyncio
from itertools import count

from isaacsim.core.api.world import World

_callback_id = count()


async def wait_for_physics_step(world: World) -> float:
    """Wait for the next physics callback.

    Returns:
        Physics step size in seconds.
    """
    loop = asyncio.get_event_loop()
    fut: asyncio.Future[float] = loop.create_future()
    callback_name = f"__bricksim_next_step_{next(_callback_id)}__"
    registered = True

    def remove_callback() -> None:
        nonlocal registered
        if registered:
            registered = False
            world.remove_physics_callback(callback_name)

    def on_step(step_size: float):
        if not fut.done():
            # resolve future on the asyncio loop thread
            loop.call_soon_threadsafe(fut.set_result, float(step_size))
        # one-shot callback
        remove_callback()

    world.add_physics_callback(callback_name, on_step)
    try:
        return await fut
    finally:
        remove_callback()
