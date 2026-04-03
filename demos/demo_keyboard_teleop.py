import asyncio
import os
import traceback
import numpy as np
import sys, tty, termios, select
import omni.kit.app
from typing import Optional
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.stage import open_stage_async, add_reference_to_stage
from bricksim import allocate_brick_part, parse_color, create_connection
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MAX_JOINT_STEP = 0.01  # rad per physics step (~3 degrees)

def getch(timeout=0.05): 
    fd = sys.stdin.fileno() 
    old = termios.tcgetattr(fd) 
    try: 
        tty.setraw(fd) 
        rlist, _, _ = select.select([sys.stdin], [], [], timeout) 
        if rlist: 
            ch = sys.stdin.read(1) 
            if ch == '\x1b': # arrow keys 
                ch += sys.stdin.read(2) 
            return ch 
        return None 
    finally: 
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def read_keyboard_action(mode):
    """Async version of your keyboard-action parser."""
    values = np.zeros(6)
    new_mode = mode

    try:
        input_key = getch()
    except Exception:
        print("Exception while reading keyboard input:")
        traceback.print_exc()
        return values, new_mode

    # Mode switching keys
    if input_key == "1":
        new_mode = "1"
    elif input_key == "2":
        new_mode = "2"
    elif input_key == "3":
        new_mode = "3"
    elif input_key == "4":
        new_mode = "4"
    elif input_key == "5":
        new_mode = "5"
    elif input_key == "6":
        new_mode = "6"
    elif input_key == "q":
        new_mode = "None"

    # Movement handling
    elif mode == "1":
        if input_key in ('\x1b[A', '\x1b[C'):
            values[0] = MAX_JOINT_STEP
        elif input_key in ('\x1b[B', '\x1b[D'):
            values[0] = -MAX_JOINT_STEP

    elif mode == "2":
        if input_key in ('\x1b[A', '\x1b[C'):
            values[1] = MAX_JOINT_STEP
        elif input_key in ('\x1b[B', '\x1b[D'):
            values[1] = -MAX_JOINT_STEP

    elif mode == "3":
        if input_key in ('\x1b[A', '\x1b[C'):
            values[2] = MAX_JOINT_STEP
        elif input_key in ('\x1b[B', '\x1b[D'):
            values[2] = -MAX_JOINT_STEP

    elif mode == "4":
        if input_key in ('\x1b[A', '\x1b[C'):
            values[3] = MAX_JOINT_STEP
        elif input_key in ('\x1b[B', '\x1b[D'):
            values[3] = -MAX_JOINT_STEP

    elif mode == "5":
        if input_key in ('\x1b[A', '\x1b[C'):
            values[4] = MAX_JOINT_STEP
        elif input_key in ('\x1b[B', '\x1b[D'):
            values[4] = -MAX_JOINT_STEP

    elif mode == "6":
        if input_key in ('\x1b[A', '\x1b[C'):
            values[5] = MAX_JOINT_STEP
        elif input_key in ('\x1b[B', '\x1b[D'):
            values[5] = -MAX_JOINT_STEP

    return values, new_mode

async def main():
    # Initialize simulation
    app = omni.kit.app.get_app()
    if World._world_initialized:
        World.clear_instance()
    stage_path = os.path.join(SCRIPT_DIR, "../resources/demo.usda")
    time.sleep(0.5)
    await open_stage_async(stage_path)
    world: World = World(
        backend="numpy",
        device="cpu",
        physics_prim_path="/physicsScene"
    ) 
    await world.initialize_simulation_context_async()

    # Spawn the robot
    robot_usd = os.path.join(SCRIPT_DIR, "../resources/robots/SO101/robot/robot.usd")
    robot_prim_path = "/World/Robot"
    add_reference_to_stage(usd_path=robot_usd, prim_path=robot_prim_path)

    # Set robot pose
    robot_xf = SingleXFormPrim(prim_path=robot_prim_path, name="Robot")
    robot_xf.set_world_pose(
        position=(-0.1, 0.1, 0.0),
        orientation=(1.0, 0.0, 0.0, 0.0),
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
        pos=(0.10, 0.1, 0.01),
    )
    brick_1 = allocate_brick_part(
        dimensions=(2, 4, 3),
        color=parse_color("Red"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.00, -0.05, 0.05),
    )
    brick_2 = allocate_brick_part(
        dimensions=(2, 4, 3),
        color=parse_color("Blue"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.05, -0.05, 0.05),
    )
    brick_3 = allocate_brick_part(
        dimensions=(2, 6, 3),
        color=parse_color("Pink"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.10, -0.05, 0.05),
    )
    brick_4 = allocate_brick_part(
        dimensions=(2, 2, 3),
        color=parse_color("Green"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.15, -0.05, 0.05),
    )

    # Start simulation loop
    await world.reset_async()
    await world.play_async()

    # Teleoperation loop
    mode = "None"
    pre_mode = mode
    print("Press 1, 2, 3, 4, 5, 6 to switch between joints. Press Up/Right or Down/Left to move.\n")
    q_target = robot.get_joint_positions()
    
    try:
        while True:
            action, mode = read_keyboard_action(mode)
            if(mode != pre_mode):
                print("Mode: Joint", mode)
                pre_mode = mode
            q_target = q_target + action  
            robot.apply_action(ArticulationAction(joint_positions=q_target))
            await app.next_update_async()

    except asyncio.CancelledError:
        # Propagate cancellation
        raise

    except Exception:
        print("Exception in teleop loop:")
        traceback.print_exc()

    finally:
        pass
