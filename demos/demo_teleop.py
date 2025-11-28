import asyncio
import os
import traceback
import numpy as np
import torch
import omni.kit.app  # pyright: ignore
from typing import Optional
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.stage import open_stage_async, add_reference_to_stage
from lerobot.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LEADER_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7C123160-if00"
LEADER_ID = "my_lerobot_leader"

def connect_leader():
    leader_config = SO101LeaderConfig(port=LEADER_PORT, id=LEADER_ID)
    leader = SO101Leader(leader_config)
    try:
        leader.connect(calibrate=False)
        return leader
    except Exception:
        return None

def disconnect_leader(leader: SO101Leader):
    if leader.is_connected:
        try:
            leader.disconnect()
        except Exception:
            print("Exception while disconnecting leader:")
            traceback.print_exc()

def read_leader_action(leader: SO101Leader) -> Optional[np.ndarray]:
    try:
        action = leader.get_action()
    except Exception:
        print("Exception while reading leader action:")
        traceback.print_exc()
        return None

    # Flatten leader action in key order
    values = [action[k] for k in action.keys()]

    # Gripper offset on last joint (degrees)
    values[-1] = values[-1] - 15.0

    # Degrees -> radians
    leader_action = np.asarray(values, dtype=np.float32) / 180.0 * np.pi
    return leader_action

async def main():
    # Initialize simulation
    app = omni.kit.app.get_app()
    if World._world_initialized:
        World.clear_instance()
    stage_path = os.path.join(SCRIPT_DIR, "../resources/demo.usda")
    await open_stage_async(stage_path)
    world: World = World(
        backend="torch",
        device="cpu",
        physics_prim_path="/physicsScene"
    ) 
    await world.initialize_simulation_context_async()

    # Spawn the robot
    robot_usd = os.path.join(SCRIPT_DIR, "../resources/robots/so101/robot.usd")
    robot_prim_path = "/World/Robot"
    add_reference_to_stage(usd_path=robot_usd, prim_path=robot_prim_path)

    # Set robot pose
    robot_xf = SingleXFormPrim(prim_path=robot_prim_path, name="Robot")
    robot_xf.set_world_pose(
        position=(-0.1, 0.0, 0.0),
        orientation=(1.0, 0.0, 0.0, 0.0),
    )

    # Create robot articulation
    robot = SingleArticulation(prim_path=robot_prim_path, name="Robot")
    world.scene.add(robot)

    # Start simulation loop
    await world.reset_async()
    await world.play_async()

    # Teleoperation loop
    leader = None
    leader_action = np.zeros(robot.num_dof, dtype=np.float32)
    try:
        while True:

            # Ensure the leader is connected; if not, try to connect once.
            if leader is None:
                leader = connect_leader()
                if leader is not None:
                    print("Leader connected.")

            # If connected, try to read the current leader action.
            if leader is not None:
                action = read_leader_action(leader)
                if action is not None:
                    leader_action = action
                else:
                    print("Lost connection to leader.")
                    disconnect_leader(leader)
                    leader = None

            q = torch.as_tensor(leader_action, device=world.device, dtype=torch.float32)
            robot.set_joint_positions(q)
            await app.next_update_async()

    except asyncio.CancelledError:
        # Propagate cancellation
        raise

    except Exception:
        print("Exception in teleop loop:")
        traceback.print_exc()

    finally:
        # Always try to cleanly close the serial port on exit
        if leader is not None:
            disconnect_leader(leader)
