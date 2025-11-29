import os
import numpy as np
import omni.kit.app # pyright: ignore
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.stage import open_stage_async, add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.interface_config_loader import load_supported_motion_policy_config

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def quat_angle_error(q_current, q_target) -> float:
    """Small helper: angle between two quaternions."""
    q_current = np.asarray(q_current, dtype=np.float64)
    q_target = np.asarray(q_target, dtype=np.float64)
    q_current /= np.linalg.norm(q_current)
    q_target /= np.linalg.norm(q_target)
    dot = float(np.clip(np.dot(q_current, q_target), -1.0, 1.0))
    # shortest rotation, ignore sign
    return 2.0 * np.arccos(abs(dot))


async def move_ee_to(
    robot: SingleArticulation,
    rmpflow: RmpFlow,
    motion_policy: ArticulationMotionPolicy,
    target_pos: np.ndarray,
    target_quat: np.ndarray,
    pos_tol: float = 0.005,   # 5 mm
    rot_tol: float = 0.10,    # ~6 deg
    vel_tol: float = 0.05,    # ~3 deg/s
    timeout: float | None = 5.0,
) -> bool:
    """
    Drive the robot with RMPflow until the end-effector reaches the target
    (within tolerance) or we hit timeout.
    """
    print(f"Moving end-effector to pos={target_pos}, quat={target_quat}")
    app = omni.kit.app.get_app()

    # Prim that tracks RMPflow's EE frame over time
    ee_prim = rmpflow.get_end_effector_as_prim()

    elapsed = 0.0
    while True:
        dt = await app.next_update_async()
        elapsed += dt

        # --- RMPflow update ---
        rmpflow.set_end_effector_target(target_pos, target_quat)
        rmpflow.update_world()
        base_pos, base_quat = robot.get_world_pose()
        rmpflow.set_robot_base_pose(base_pos, base_quat)

        action = motion_policy.get_next_articulation_action(dt)
        robot.apply_action(action)

        # --- Check "reached" condition ---
        ee_pos, ee_quat = ee_prim.get_world_pose()
        pos_err = np.linalg.norm(ee_pos - target_pos)
        ang_err = quat_angle_error(ee_quat, target_quat)
        joint_vel = robot.get_joint_velocities()
        max_vel = float(np.max(np.abs(joint_vel))) if joint_vel.size else 0.0

        if pos_err < pos_tol and ang_err < rot_tol and max_vel < vel_tol:
            print(f"Reached target: pos_err={pos_err:.4f}, ang_err={ang_err:.4f}, max_vel={max_vel:.4f}")
            return True

        if timeout is not None and elapsed > timeout:
            print(f"Timeout reached: pos_err={pos_err:.4f}, ang_err={ang_err:.4f}, max_vel={max_vel:.4f}")
            return False

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
    robot_usd = get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
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

    # Set up Lula RMPflow
    rmp_cfg = load_supported_motion_policy_config("Franka", "RMPflow")
    rmpflow = RmpFlow(**rmp_cfg)
    motion_policy = ArticulationMotionPolicy(robot, rmpflow)

    # Start simulation loop
    await world.reset_async()
    await world.play_async()

    waypoints = [
        np.array([0.45, 0.00, 0.05]),
        np.array([0.45, 0.20, 0.05]),
        np.array([0.60, 0.20, 0.05]),
        np.array([0.60, 0.00, 0.05]),
    ]
    for wp in waypoints:
        target_pos = wp
        target_quat = np.array([0.0, 0.0, 1.0, 0.0], dtype=np.float64)

        await move_ee_to(
            robot,
            rmpflow,
            motion_policy,
            target_pos,
            target_quat,
        )
