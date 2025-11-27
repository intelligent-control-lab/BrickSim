import os
import torch
import omni.kit.app # pyright: ignore
from omni.kit.async_engine import run_coroutine
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import open_stage_async

async def main():
    if World._world_initialized:
        World.clear_instance()

    stage_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../resources/r1lite_demo.usda")
    await open_stage_async(stage_path)

    world: World = World(
        backend="torch",
        device="cpu",
        physics_prim_path="/physicsScene"
    )
    await world.initialize_simulation_context_async()

    robot = SingleArticulation(prim_path="/World/Robot", name="Robot")
    world.scene.add(robot)
    await world.reset_async()
    await world.play_async()

    app = omni.kit.app.get_app()
    t = 0.0
    while t < 10.0:
        # Random actions
        q = 0.1 * (torch.rand((robot.num_dof,), device=world.device) * 2.0 - 1.0)
        robot.set_joint_positions(q)
        dt = await app.next_update_async()
        t += dt

run_coroutine(main())
