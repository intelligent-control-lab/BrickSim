import os
import torch
import omni.kit.app # pyright: ignore
from omni.kit.async_engine import run_coroutine
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import open_stage_async
from lego_assemble import allocate_brick_part, parse_color, create_connection

async def main():
    app = omni.kit.app.get_app()
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

    # Spawn the base plate
    base_plate = allocate_brick_part(
        dimensions=(20, 20, 1),
        color=parse_color("Light Gray"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.0, 0.0, 0.01),
    )
    brick_1 = allocate_brick_part(
        dimensions=(2, 4, 3),
        color=parse_color("Red"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.0, 0.0, 0.2),
    )
    conn_1 = create_connection(
        stud_path=base_plate,
        stud_if=1,
        hole_path=brick_1,
        hole_if=0,
        offset=(0, 0),
        yaw=0,
    )

    robot = SingleArticulation(prim_path="/World/Robot", name="Robot")
    world.scene.add(robot)
    await world.reset_async()
    await world.play_async()

    t = 0.0
    while t < 10.0:
        # Random actions
        q = 0.1 * (torch.rand((robot.num_dof,), device=world.device) * 2.0 - 1.0)
        robot.set_joint_positions(q)
        dt = await app.next_update_async()
        t += dt

run_coroutine(main())
