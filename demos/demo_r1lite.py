import os
import torch
import omni.kit.app # pyright: ignore
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.stage import open_stage_async, add_reference_to_stage
from lego_assemble import allocate_brick_part, parse_color, create_connection
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

async def main():
    # Initialize simulation
    app = omni.kit.app.get_app()
    if World._world_initialized:
        World.clear_instance()
    stage_path = os.path.join(SCRIPT_DIR, "../resources/demo.usda")
    await open_stage_async(stage_path)
    world: World = World(
        backend="numpy",
        device="cpu",
        physics_prim_path="/physicsScene"
    ) 
    await world.initialize_simulation_context_async()

    # Spawn the robot
    robot_usd = os.path.join(SCRIPT_DIR, "../resources/robots/r1lite/robot/robot.usd")
    robot_prim_path = "/World/Robot"
    add_reference_to_stage(usd_path=robot_usd, prim_path=robot_prim_path)

    # Set robot pose
    robot_xf = SingleXFormPrim(prim_path=robot_prim_path, name="Robot")
    robot_xf.set_world_pose(
        position=(0.0, 0.8, -0.749),
        orientation=(0.7071067811865476, 0.0, 0.0, -0.7071067811865475),
    )

    # Create robot articulation
    robot = SingleArticulation(prim_path=robot_prim_path, name="Robot")
    world.scene.add(robot)

    # Spawn some lego
    base_plate = allocate_brick_part(
        dimensions=(20, 20, 1),
        color=parse_color("Light Gray"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.0, 0.1, 0.01),
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

    # Start simulation loop
    await world.reset_async()
    await world.play_async()

    t = 0.0
    while True:

        # Random actions
        q = 0.01 * (np.random.rand(robot.num_dof) * 2.0 - 1.0)

        robot.set_joint_positions(q)
        dt = await app.next_update_async()
        t += dt
