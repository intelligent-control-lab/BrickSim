import os
import numpy as np
import omni.kit.app # pyright: ignore
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.stage import open_stage_async, add_reference_to_stage, get_current_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.interface_config_loader import load_supported_motion_policy_config
from lego_assemble import allocate_brick_part, parse_color, arrange_bricks_on_table, get_brick_dimensions

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

async def set_gripper(robot: SingleArticulation, target_width: float, timeout: float = 5.0) -> bool:
    """
    Controls the Franka gripper to reach a target width.
    Includes robust grasp detection (detects velocity stall when closing).
    """
    app = omni.kit.app.get_app()

    # Identify gripper joints
    gripper_indices = [
        robot.get_dof_index("panda_finger_joint1"),
        robot.get_dof_index("panda_finger_joint2"),
    ]

    # Target position per finger
    target_pos = target_width / 2.0
    pos_tolerance = 0.003 # 1 mm tolerance
    vel_tolerance = 0.005 # 0.5 cm/s velocity threshold for grasp detection
    min_grasp_time = 0.4  # Minimum time to wait before checking grasp detection

    # Define the action to set the target positions for the gripper drives
    action = ArticulationAction(
        joint_positions=np.array([target_pos, target_pos]),
        joint_indices=gripper_indices
    )

    elapsed = 0.0
    # Store initial width to reliably detect if we are opening or closing
    initial_positions = robot.get_joint_positions()
    initial_width = initial_positions[gripper_indices[0]] + initial_positions[gripper_indices[1]]
    is_closing = target_width < initial_width

    # Apply the action once (sets the target for the PD controller)
    robot.apply_action(action)

    while True:
        dt = await app.next_update_async()
        elapsed += dt

        # Check current state
        current_positions = robot.get_joint_positions()
        current_width = current_positions[gripper_indices[0]] + current_positions[gripper_indices[1]]
        
        joint_vels = robot.get_joint_velocities()
        gripper_vels = joint_vels[gripper_indices]
        max_vel = np.max(np.abs(gripper_vels))

        # Check if reached the exact target width (important when opening)
        if abs(current_width - target_width) < pos_tolerance:
            return True

        # Check if grasping (closing and movement stopped before reaching target)
        if is_closing and max_vel < vel_tolerance and elapsed > min_grasp_time:
             # Ensure we haven't reached the target (otherwise it's not a velocity stall grasp)
             if abs(current_width - target_width) >= pos_tolerance:
                print(f"Grasp detected (velocity stall). Width: {current_width:.4f}")
                return True

        if elapsed > timeout:
            print(f"Gripper timeout. Current width: {current_width:.4f}, Target: {target_width:.4f}")
            return False

async def grasp_lego_part(
    world: World,
    robot: SingleArticulation,
    rmpflow: RmpFlow,
    motion_policy: ArticulationMotionPolicy,
    brick_prim_path: str,
):
    """
    Executes a sequence to grasp a specified lego brick and lift it.
    It uses a top-down grasp approach relative to the brick's frame, 
    preferring the shorter edge of the brick.
    """
    BRICK_UNIT_LENGTH = 0.0080 # 8.0 mm per stud
    PLATE_UNIT_HEIGHT = 0.0032 # 3.2 mm per plate
    TCP_TO_FINGER_TIP = 0.0090 # 9.0 mm from Franka TCP to finger tips
    GRASP_DEPTH       = 0.003  # 3.0 mm into the brick

    app = omni.kit.app.get_app()
    print(f"--- Attempting to grasp brick: {brick_prim_path} ---")
    stage = get_current_stage()

    # 1. Get Brick Info
    dimensions = get_brick_dimensions(brick_prim_path)
    if dimensions is None:
        return False
    
    L, W, H = dimensions

    # Use SingleXFormPrim to track the brick's pose
    view_name = f"grasp_target_view_{os.path.basename(brick_prim_path)}"
    brick_xf = SingleXFormPrim(prim_path=brick_prim_path, name=view_name)

    # # 2. Prepare RMPflow: Remove target brick from obstacles
    # # This is crucial so RMPflow allows the EE to approach the brick closely.
    # target_prim = stage.GetPrimAtPath(brick_prim_path)
    # if target_prim:
    #     # Check if it's already registered as an obstacle before removing
    #     if rmpflow.obstacle_exists(target_prim):
    #          print(f"-> Removing target brick {brick_prim_path} from RMPflow obstacles.")
    #          rmpflow.remove_obstacle_from_prim(target_prim)
    #          rmpflow.update_world()
        
    # Get current pose of the brick (WXYZ quaternion)
    brick_pos, brick_quat_wxyz = brick_xf.get_world_pose()
    
    # Rotation Matrix from World (W) to Brick (B) frame
    R_W_B = quats_to_rot_matrices(brick_quat_wxyz)

    # 3. Determine Grasp Orientation relative to Brick Frame (R_B_G)
    # Goal: Top-down grasp (Z_G aligns with -Z_B).
    # Goal: Grasp the shorter edge.
    # We assume the standard Franka TCP frame used by RMPflow: 
    # Z_G is the approach vector, Y_G is the closing direction (between fingers).

    # Brick frame convention (based on C++ specs): X_B is along L, Y_B is along W.

    if W <= L:
        # Shorter edge is W (along Y_B). Grasp across W.
        # Align Y_G (closing direction) with Y_B.
        # R_B_G derivation (Columns are axes of G expressed in B frame):
        # Z_G_B = (0, 0, -1)  (-Z_B)
        # Y_G_B = (0, 1, 0)  (Y_B)
        # X_G_B = Y_G x Z_G = (-1, 0, 0) (-X_B)
        R_B_G = np.array([
            [-1., 0., 0.],
            [ 0., 1., 0.],
            [ 0., 0., -1.]
        ])
        grasp_width = W * BRICK_UNIT_LENGTH
    else:
        # Shorter edge is L (along X_B). Grasp across L.
        # Align Y_G (closing direction) with X_B.
        # R_B_G derivation:
        # Z_G_B = (0, 0, -1) (-Z_B)
        # Y_G_B = (1, 0, 0) (X_B)
        # X_G_B = Y_G x Z_G = (0, 1, 0) (Y_B)
        R_B_G = np.array([
            [0., 1., 0.],
            [1., 0., 0.],
            [0., 0., -1.]
        ])
        grasp_width = L * BRICK_UNIT_LENGTH

    # 4. Calculate Grasp Pose in World Frame (T_W_G)
    
    # Calculate target orientation R_W_G = R_W_B @ R_B_G
    R_W_G = R_W_B @ R_B_G
    target_quat = rot_matrices_to_quats(R_W_G)

    # Calculate target position.
    # Brick origin is center bottom (based on C++ specs).
    brick_height = H * PLATE_UNIT_HEIGHT
    # T_B_G (Position of Grasp TCP relative to Brick origin)
    T_B_G = np.array([0, 0, brick_height + TCP_TO_FINGER_TIP - GRASP_DEPTH])
    
    # T_W_G = T_W_B + R_W_B @ T_B_G
    target_pos = brick_pos + R_W_B @ T_B_G

    # Calculate Pre-grasp and Post-grasp poses
    # Approach along the brick's local Z-axis (Z_B) for robustness.
    # Z_B_W = R_W_B @ (0, 0, 1) = R_W_B[:, 2]
    approach_vector_W = R_W_B[:, 2]
    
    pre_grasp_offset = 0.10 # 10 cm above
    pre_grasp_pos = target_pos + approach_vector_W * pre_grasp_offset
    
    # Lift vertically in world frame after grasp
    lift_height = 0.15
    post_grasp_pos = pre_grasp_pos.copy()
    post_grasp_pos[2] += lift_height # Lift higher in world Z

    # 5. Execute Grasp Sequence

    # 5.1 Open Gripper
    # Add safety margin (1.5cm) and clamp to max Franka opening (0.08m)
    open_width = min(grasp_width + 0.015, 0.08)
    print(f"-> Opening gripper to width: {open_width:.3f}m (Grasp width: {grasp_width:.3f}m)")
    if not await set_gripper(robot, open_width):
        return False

    # 5.2 Move to Pre-grasp Pose
    print("-> Moving to pre-grasp pose.")
    success = await move_ee_to(
        robot, rmpflow, motion_policy, pre_grasp_pos, target_quat, timeout=15.0
    )
    if not success:
        return False

    # 5.3 Move to Grasp Pose
    print("-> Moving to grasp pose.")
    # Use tighter tolerances for the final approach
    success = await move_ee_to(
        robot, rmpflow, motion_policy, target_pos, target_quat,
        pos_tol=0.002, rot_tol=0.05, timeout=8.0 # 2mm, ~3deg tolerance
    )
    if not success:
        return False

    # 5.4 Close Gripper
    print("-> Closing gripper.")
    # Command target width to 0.0 to ensure firm contact and trigger robust grasp detection
    if not await set_gripper(robot, 0.0):
         print("Warning: Gripper closing sequence reported timeout.")

    # # 5.5 Attach the object to the end-effector for RMPflow
    # # This informs RMPflow that the object is now part of the robot collision geometry
    # if target_prim:
    #     print("-> Attaching brick to RMPflow end-effector.")
    #     rmpflow.attach_object_to_end_effector(target_prim)
    #     rmpflow.update_world()

    # Wait a moment for the grasp to stabilize in physics
    for _ in range(30):
        await app.next_update_async()

    # 5.6 Move to Post-grasp Pose (Lift)
    print("-> Lifting brick.")
    success = await move_ee_to(
        robot, rmpflow, motion_policy, post_grasp_pos, target_quat, timeout=10.0
    )

    print("--- Grasp sequence finished. ---")
    return success

async def main():
    # Initialize simulation
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

    # Place base plate
    base_plate = allocate_brick_part(
        dimensions=(20, 20, 1),
        color=parse_color("Light Gray"),
        env_id=-1,
        rot=(1.0, 0.0, 0.0, 0.0),
        pos=(0.2, 0.0, 0.0),
    )

    # Place bricks
    bricks_to_place = [
        ((2, 4, 3), "Red"),
        ((2, 4, 3), "Blue"),
        ((2, 6, 3), "Pink"),
        ((2, 2, 3), "Green"),
    ]
    brick_paths = [
        allocate_brick_part(
            dimensions=dims,
            color=parse_color(color),
            env_id=-1,
        ) for dims, color in bricks_to_place
    ]
    _, not_placed = arrange_bricks_on_table(
        parts_to_arrange=brick_paths,
        parts_to_avoid=[base_plate],
        obstacles=None,
        table_xy=(0.2, -0.2, 0.5, 0.2),
        table_z=0.0,
        allow_rotation=True,
    )
    if len(not_placed) > 0:
        raise RuntimeError(f"Failed to place all bricks on table; not placed: {not_placed}")

    # Start simulation loop
    await world.reset_async()
    await world.play_async()

    # Grasp brick
    target_brick_path = str(brick_paths[0])
    success = await grasp_lego_part(
            world,
            robot,
            rmpflow,
            motion_policy,
            target_brick_path
    )
    if success:
        print("Task completed: Successfully grasped and lifted the brick.")
    else:
        print("Task failed: Could not complete the grasp sequence.")
