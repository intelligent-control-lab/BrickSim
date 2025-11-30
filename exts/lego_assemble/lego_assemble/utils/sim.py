import asyncio
from isaacsim.core.api.world import World

async def wait_for_physics_step(world: World) -> float:
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
    elapsed = 0.0
    while elapsed < duration:
        step_size = await wait_for_physics_step(world)
        elapsed += step_size
