import asyncio
import json
import os
import traceback
import math
import numpy as np
import omni.kit.app
import omni.timeline
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport, get_viewport_from_window_name
from typing import Any, Optional
from pxr import UsdPhysics, Usd, Gf
from isaacsim.core.api.world import World
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim, SingleGeometryPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.stage import open_stage_async, add_reference_to_stage, get_current_stage
from bricksim.assets import DEFAULT_STAGE_PATH
from bricksim.colors import parse_color
from bricksim.core import (
    allocate_brick_part,
    AssemblyThresholds,
    BreakageThresholds,
    create_connection,
    get_breakage_thresholds,
    set_assembly_thresholds,
    set_breakage_thresholds,
)

LEADER_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF288330-if00" #### ---- CHANGE THIS TO YOUR SERIAL PORT ---- ####
LEADER_ID = "my_lerobot_leader" #### ---- CHANGE THIS TO YOUR LEADER ID ---- ####

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODE = "TELEOP"  # "TELEOP", "RECORD", "REPLAY"
RECORD_FILEPATH = os.path.join(SCRIPT_DIR, "demo_teleop_record.jsonl")
SAVE_VIDEO = True
MAX_JOINT_STEP = 0.1  # rad per physics step
VALID_MODES = ("TELEOP", "RECORD", "REPLAY")
SCREENSHOTS_DIR = os.path.join(SCRIPT_DIR, "screenshots")

def connect_leader():
    from lerobot.teleoperators.so_leader import SOLeaderTeleopConfig, SOLeader

    leader_config = SOLeaderTeleopConfig(port=LEADER_PORT, id=LEADER_ID)
    leader = SOLeader(leader_config)
    try:
        leader.connect(calibrate=False)
        return leader
    except Exception:
        return None

def disconnect_leader(leader: Any):
    if leader.is_connected:
        try:
            leader.disconnect()
        except Exception:
            print("Exception while disconnecting leader:")
            traceback.print_exc()

def read_leader_action(leader: Any) -> Optional[np.ndarray]:
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

def open_record_writer(filepath: str):
    os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
    return open(filepath, "w", encoding="utf-8")

def write_record_step(record_fp, leader_action: np.ndarray):
    json.dump(leader_action.tolist(), record_fp)
    record_fp.write("\n")
    record_fp.flush()

def open_replay_reader(filepath: str):
    return open(filepath, "r", encoding="utf-8")

def read_replay_step(replay_fp, expected_num_dof: int, filepath: str, line_no: int) -> Optional[np.ndarray]:
    line = replay_fp.readline()
    if line == "":
        return None

    try:
        payload = json.loads(line)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in replay file {filepath!r} at line {line_no}") from exc

    action = np.asarray(payload, dtype=np.float32)
    if action.shape != (expected_num_dof,):
        raise ValueError(
            f"Replay action in {filepath!r} at line {line_no} has shape {action.shape}, "
            f"expected {(expected_num_dof,)}"
        )
    return action

async def main():
    if MODE not in VALID_MODES:
        raise ValueError(f"Invalid MODE {MODE!r}; expected one of {VALID_MODES}")

    # Initialize simulation
    app = omni.kit.app.get_app()
    if World._world_initialized:
        World.clear_instance()
    await open_stage_async(str(DEFAULT_STAGE_PATH))
    stage = get_current_stage()
    world: World = World(
        backend="numpy",
        device="cpu",
        physics_prim_path="/physicsScene"
    ) 
    await world.initialize_simulation_context_async()

    # Disable the cube
    stage.GetPrimAtPath("/World/Cube").SetActive(False)

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

    # Set robot params
    get_current_stage().GetPrimAtPath("/World/Robot/root_joint").GetAttribute("physxArticulation:solverPositionIterationCount").Set(64)
    stage.GetPrimAtPath("/World/Robot/upper_arm_link/collisions").SetInstanceable(False)
    stage.GetPrimAtPath("/World/Robot/moving_jaw_so101_v1_link/collisions").SetInstanceable(False)
    pad_material = PhysicsMaterial(
        prim_path="/World/PhysicsMaterials/FingerPad",
        static_friction=1.0,
        dynamic_friction=1.0,
        restitution=0.5,
    )
    SingleGeometryPrim(prim_path="/World/Robot/upper_arm_link/collisions/upper_arm_so101_v1/node_STL_BINARY_").apply_physics_material(pad_material)
    SingleGeometryPrim(prim_path="/World/Robot/moving_jaw_so101_v1_link/collisions/moving_jaw_so101_v1/node_STL_BINARY_").apply_physics_material(pad_material)
    gripper_joint_prim = stage.GetPrimAtPath("/World/Robot/joints/gripper")
    gripper_joint_prim.GetAttribute("drive:angular:physics:maxForce").Set(0.2)
    gripper_joint_prim.GetAttribute("drive:angular:physics:stiffness").Set(1.0)

    # Set assembly thresholds
    assembly_thr = AssemblyThresholds()
    assembly_thr.distance_tolerance = 0.001
    assembly_thr.max_penetration = 0.005
    assembly_thr.z_angle_tolerance = 5.0 * (math.pi / 180.0)
    assembly_thr.required_force = 1.0
    assembly_thr.yaw_tolerance = 5.0 * (math.pi / 180.0)
    assembly_thr.position_tolerance = 0.002
    set_assembly_thresholds(assembly_thr)

    # Set breakage thresholds
    breakage_thr = get_breakage_thresholds()
    breakage_thr.enabled = False
    set_breakage_thresholds(breakage_thr)

    # Spawn some lego
    base_plate = allocate_brick_part(
        dimensions=(20, 20, 1),
        color=parse_color("Light Gray"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.10, 0.10, 0.0),
    )
    base_plate_prim = stage.GetPrimAtPath(base_plate)
    UsdPhysics.RigidBodyAPI(base_plate_prim).GetKinematicEnabledAttr().Set(True)
    # brick_1 = allocate_brick_part(
        # dimensions=(2, 4, 3),
        # color=parse_color("Red"),
        # env_id=-1,
        # rot=(1.0, 0.0, 0.0, 0.0),
        # pos=(0.00, -0.05, 0.0),
    # )
    brick_2 = allocate_brick_part(
        dimensions=(2, 4, 3),
        color=parse_color("Blue"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.05, 0.22, 0.0),
    )
    brick_3 = allocate_brick_part(
        dimensions=(2, 6, 3),
        color=parse_color("Pink"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.10, 0.22, 0.0),
    )
    # brick_4 = allocate_brick_part(
    #     dimensions=(2, 2, 3),
    #     color=parse_color("Green"),
    #     env_id=-1,
    #     rot=(1.0, 0.0, 0.0, 0.0),
    #     pos=(0.15, -0.05, 0.0),
    # )

    # Set camera
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        camera = stage.GetPrimAtPath("/OmniverseKit_Persp")
        camera.GetAttribute("focalLength").Set(18.14756)
        camera.GetAttribute("xformOp:translate").Set(Gf.Vec3f(-0.2166, 0.51795, 0.15479))
        camera.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(83.29281, 0.0, -139.03314))

    # Start simulation loop
    await world.reset_async()
    await world.play_async()

    # Teleoperation loop
    leader = None
    record_fp = None
    replay_fp = None
    replay_line_no = 0
    leader_action = np.zeros(robot.num_dof, dtype=np.float32)
    video_viewport = None
    video_frame_idx = 0

    if MODE == "RECORD":
        record_fp = open_record_writer(RECORD_FILEPATH)
    elif MODE == "REPLAY":
        replay_fp = open_replay_reader(RECORD_FILEPATH)

    if SAVE_VIDEO:
        os.makedirs(SCREENSHOTS_DIR, exist_ok=True)
        video_viewport = get_active_viewport() or get_viewport_from_window_name("Viewport")
        if video_viewport is None:
            raise RuntimeError("No viewport found. Make sure a Viewport window is open.")

    try:
        while True:
            if MODE in ("TELEOP", "RECORD"):
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

                if record_fp is not None:
                    write_record_step(record_fp, leader_action)
            else:
                replay_line_no += 1
                action = read_replay_step(replay_fp, robot.num_dof, RECORD_FILEPATH, replay_line_no)
                if action is None:
                    print("Replay completed.")
                    break
                leader_action = action

            # Smooth / limit joint motion before sending it to physics
            q = robot.get_joint_positions()
            desired = leader_action
            delta = desired - q
            delta = np.clip(delta, -MAX_JOINT_STEP, MAX_JOINT_STEP)
            q_target = q + delta
            if SAVE_VIDEO:
                video_frame_idx += 1
                video_capture = capture_viewport_to_file(
                    video_viewport,
                    os.path.join(SCREENSHOTS_DIR, f"{video_frame_idx:06d}.png"),
                )
                timeline = omni.timeline.get_timeline_interface()
                timeline.pause()
                await video_capture.wait_for_result(completion_frames=0)
                timeline.play()
            robot.apply_action(ArticulationAction(joint_positions=q_target))
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
        if record_fp is not None:
            record_fp.close()
        if replay_fp is not None:
            replay_fp.close()
