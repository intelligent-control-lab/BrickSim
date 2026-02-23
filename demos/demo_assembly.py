import math
import json
import os
import numpy as np
import omni.kit.app # pyright: ignore
from typing import Optional
from copy import deepcopy
from pxr import Gf
from isaacsim.core.api.world import World
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim, SingleGeometryPrim
from isaacsim.core.utils.stage import open_stage_async, add_reference_to_stage, get_current_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.interface_config_loader import load_supported_motion_policy_config
from bricksim import arrange_parts_in_workspace, import_lego, get_brick_dimensions, compute_connection_transform, set_assembly_thresholds, AssemblyThresholds, wait_for_physics_step, parse_color, Colors, are_parts_connected
from bricksim.utils.topology import bfs_sort_connections
from bricksim.importers.stabletext2brick import bricks_text_to_topology_json, is_bricks_text
from bricksim.importers.legolization import legolization_json_to_topology_json, is_legolization_json

try:
    from isaacsim.util.debug_draw import _debug_draw # pyright: ignore
    DEBUG_DRAW = _debug_draw.acquire_debug_draw_interface()
except Exception:
    print("Warning: Unable to import debug draw interface.")
    DEBUG_DRAW = None
# DEBUG_DRAW = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# STRUCTURE_PATH = os.path.join(SCRIPT_DIR, "../resources/demo_structure.json")
STRUCTURE_PATH = os.path.join(SCRIPT_DIR, "../resources/demo_structure2.txt")
PRE_PLACED_PARTS = [0,] # Indices of parts in the structure JSON that are pre-placed in the scene
PLACE_POSE = ([0.3, 0.0, 0.0], (1.0, 0.0, 0.0, 0.0)) # Position (x,y,z), Orientation (w,x,y,z)
BASEPLATE_KWARGS = {
    "include_base_plate": True,
    "base_plate_size": (20, 20),
    "base_plate_color": parse_color("Light Gray")
}

def quat_angle_error(q_current, q_target) -> float:
    """Small helper: angle between two quaternions."""
    q_current = np.asarray(q_current, dtype=np.float64)
    q_target = np.asarray(q_target, dtype=np.float64)
    q_current /= np.linalg.norm(q_current)
    q_target /= np.linalg.norm(q_target)
    dot = float(np.clip(np.dot(q_current, q_target), -1.0, 1.0))
    # shortest rotation, ignore sign
    return 2.0 * np.arccos(abs(dot))

def compose_transforms(T1_pos, T1_quat, T2_pos, T2_quat):
    """
    Composes two transforms T1 (parent) and T2 (child). T_result = T1 * T2.
    All inputs/outputs are (pos_xyz, quat_wxyz).
    """
    R1 = quats_to_rot_matrices(T1_quat)
    R2 = quats_to_rot_matrices(T2_quat)
    
    # R_result = R1 @ R2
    R_result = R1 @ R2
    
    # T_result_pos = T1_pos + R1 @ T2_pos
    T_result_pos = T1_pos + R1 @ T2_pos
    
    T_result_quat = rot_matrices_to_quats(R_result)
    return T_result_pos, T_result_quat

def inverse_transform(T_pos, T_quat):
    """
    Computes the inverse of a transform T. T_inv = inverse(T).
    All inputs/outputs are (pos_xyz, quat_wxyz).
    """
    R = quats_to_rot_matrices(T_quat)
    R_inv = R.T
    # T_inv_pos = -R_inv @ T_pos
    T_inv_pos = -R_inv @ T_pos
    T_inv_quat = rot_matrices_to_quats(R_inv)
    return T_inv_pos, T_inv_quat

async def move_ee_to(
    world: World,
    robot: SingleArticulation,
    rmpflow: RmpFlow,
    motion_policy: ArticulationMotionPolicy,
    target_pos: np.ndarray,
    target_quat: np.ndarray,
    pos_tol: float = 0.005,
    rot_tol: float = 0.100,
    vel_tol: Optional[float] = 0.100,
    timeout: Optional[float] = 10.0,
    settle_time: float = 0.0, # Time to hold the pose after reaching tolerance
) -> bool:
    """
    Drive the robot with RMPflow until the end-effector reaches the target
    (within tolerance) or we hit timeout.
    """
    print(f"Moving end-effector to pos={target_pos}, quat={target_quat}")

    # Prim that tracks RMPflow's EE frame over time
    ee_prim = rmpflow.get_end_effector_as_prim()

    elapsed = 0.0
    time_reached = None # Timestamp when tolerances were first met
    while True:
        dt = await wait_for_physics_step(world)
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

        # Check if conditions are met
        is_within_tolerance = (pos_err < pos_tol) and (ang_err < rot_tol) and (vel_tol is None or max_vel < vel_tol)

        # --- Debug draw: visualize target vs current EE ---
        if DEBUG_DRAW is not None:
            # Clear previous points so we only see the latest target/current
            DEBUG_DRAW.clear_points()
            # Target = red point
            DEBUG_DRAW.draw_points(
                [tuple(target_pos.tolist())],
                [(1.0, 0.0, 0.0, 1.0)],  # RGBA
                [10.0],                  # size in pixels
            )
            # Current EE = green point
            DEBUG_DRAW.draw_points(
                [tuple(ee_pos.tolist())],
                [(0.0, 1.0, 0.0, 1.0)],
                [10.0],
            )

        if is_within_tolerance:
            if time_reached is None:
                print(f"Reached target tolerance.")
                time_reached = elapsed
            
            # Check if settled (reached + waited settle_time)
            if elapsed - time_reached >= settle_time:
                print(f"Settled at target.")
                return True
        else:
            # Reset if tolerances are violated (e.g., pushed away during settling)
            time_reached = None

        if (timeout is not None) and (elapsed > timeout):
            print(f"Timeout reached: pos_err={pos_err:.4f}, ang_err={ang_err:.4f}, max_vel={max_vel:.4f}")
            return False

async def set_gripper(world: World, robot: SingleArticulation, target_width: Optional[float] = None, *, delta_width: Optional[float] = None, timeout: float = 3.0) -> bool:
    """
    Controls the Franka gripper to reach a target width.
    Includes robust grasp detection (detects velocity stall when closing).
    """
    # Identify gripper joints
    gripper_indices = [
        robot.get_dof_index("panda_finger_joint1"),
        robot.get_dof_index("panda_finger_joint2"),
    ]

    # Store initial width to reliably detect if we are opening or closing
    initial_positions = robot.get_joint_positions()
    initial_width = initial_positions[gripper_indices[0]] + initial_positions[gripper_indices[1]]

    if target_width is None:
        if delta_width is None:
            raise ValueError("Either target_width or delta_width must be specified.")
        target_width = initial_width + delta_width

    is_closing = target_width < initial_width

    # Target position per finger
    target_pos = target_width / 2.0
    pos_tolerance = 0.003 # 3 mm tolerance
    vel_tolerance = 0.0001 # 0.1 mm/s velocity threshold for grasp detection
    min_grasp_time = 1.0  # Minimum time to wait before checking grasp detection

    # Define the action to set the target positions for the gripper drives
    action = ArticulationAction(
        joint_positions=np.array([target_pos, target_pos]),
        joint_indices=gripper_indices
    )

    elapsed = 0.0
    stall_time = 0.0

    # Apply the action once (sets the target for the PD controller)
    robot.apply_action(action)

    while True:
        dt = await wait_for_physics_step(world)
        elapsed += dt

        # Check current state
        current_positions = robot.get_joint_positions()
        current_width = current_positions[gripper_indices[0]] + current_positions[gripper_indices[1]]
        
        joint_vels = robot.get_joint_velocities()
        gripper_vels = joint_vels[gripper_indices]
        max_vel = np.max(np.abs(gripper_vels))

        # Check if reached the exact target width (important when opening)
        if abs(current_width - target_width) < pos_tolerance:
            print(f"Gripper reached target width: {current_width:.4f}")
            return True

        # Check if grasping (closing and movement stopped before reaching target)
        if is_closing and max_vel < vel_tolerance:
            stall_time += dt
            # Ensure haven't reached the target (otherwise it's not a velocity stall grasp)
            if abs(current_width - target_width) >= pos_tolerance and stall_time > min_grasp_time:
               print(f"Grasp detected (velocity stall). Width: {current_width:.4f}")
               return True
        else:
            stall_time = 0.0

        if elapsed > timeout:
            print(f"Gripper timeout. Current width: {current_width:.4f}, Target: {target_width:.4f}")
            return False

async def grasp_lego_part(
    world: World,
    robot: SingleArticulation,
    rmpflow: RmpFlow,
    motion_policy: ArticulationMotionPolicy,
    brick_prim_path: str,
) -> bool:
    """
    Executes a sequence to grasp a specified lego brick and lift it.
    It uses a top-down grasp approach relative to the brick's frame, 
    preferring the shorter edge of the brick.
    Returns (success, T_B_G) where T_B_G is (pos, quat_wxyz) from Brick frame to Grasp TCP frame, measured after physical grasp.
    """
    BRICK_UNIT_LENGTH = 0.0080 # 8.0 mm per stud
    PLATE_UNIT_HEIGHT = 0.0032 # 3.2 mm per plate
    TCP_TO_FINGER_TIP = 0.0090 # 9.0 mm from Franka TCP to finger tips
    GRASP_DEPTH       = 0.001  # 1.0 mm into the brick

    print(f"--- Attempting to grasp brick: {brick_prim_path} ---")

    # Get Brick Info
    dimensions = get_brick_dimensions(brick_prim_path)
    if dimensions is None:
        return False
    
    L, W, H = dimensions
    if L == 1 or W == 1:
        GRASP_DEPTH = 0.002 # 2.0 mm for very small bricks

    # Use SingleXFormPrim to track the brick's pose
    view_name = f"grasp_target_view_{os.path.basename(brick_prim_path)}"
    brick_xf = SingleXFormPrim(prim_path=brick_prim_path, name=view_name)
        
    # Get current pose of the brick (WXYZ quaternion)
    brick_pos, brick_quat_wxyz = brick_xf.get_world_pose()
    
    # Rotation Matrix from World (W) to Brick (B) frame
    R_W_B = quats_to_rot_matrices(brick_quat_wxyz)

    # Determine Grasp Orientation relative to Brick Frame (R_B_G)
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

    # Calculate Grasp Pose in World Frame (T_W_G)
    
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
    lift_height = 0.10
    post_grasp_pos = pre_grasp_pos.copy()
    post_grasp_pos[2] += lift_height # Lift higher in world Z

    # Execute Grasp Sequence

    # 1. Open Gripper
    # Add safety margin (1.5cm) and clamp to max Franka opening (0.08m)
    open_width = min(grasp_width + 0.015, 0.08)
    print(f"-> Opening gripper to width: {open_width:.3f}m (Grasp width: {grasp_width:.3f}m)")
    if not await set_gripper(world, robot, open_width):
        return False

    # 2. Move to Pre-grasp Pose
    print("-> Moving to pre-grasp pose.")
    success = await move_ee_to(
        world, robot, rmpflow, motion_policy, pre_grasp_pos, target_quat,
        pos_tol=0.02,
        rot_tol=0.1,
        vel_tol=None,
        timeout=10.0,
    )
    if not success:
        return False

    # 3. Move to Grasp Pose
    print("-> Moving to grasp pose.")
    # Use tighter tolerances for the final approach
    success = await move_ee_to(
        world, robot, rmpflow, motion_policy, target_pos, target_quat,
        pos_tol=0.003,
        rot_tol=0.05,
        vel_tol=0.10,
        timeout=10.0,
    )
    if not success:
        return False

    # 4. Close Gripper
    print("-> Closing gripper.")
    # Command target width to 0.0 to ensure firm contact and trigger robust grasp detection
    if not await set_gripper(world, robot, 0.0):
         print("Warning: Gripper closing sequence reported timeout.")

    # 5. Move to Post-grasp Pose (Lift)
    print("-> Lifting brick.")
    success = await move_ee_to(
        world, robot, rmpflow, motion_policy, post_grasp_pos, target_quat,
        pos_tol=0.02,
        rot_tol=0.1,
        vel_tol=None,
        timeout=10.0,
    )

    print("--- Grasp sequence finished. ---")
    return success

async def assemble_lego_part(
    world: World,
    robot: SingleArticulation,
    rmpflow: RmpFlow,
    motion_policy: ArticulationMotionPolicy,
    stud_brick_path: str,      # The brick on the table/structure (Stud provider)
    hole_brick_path: str,      # The brick currently held by the robot (Hole provider)
    offset: tuple[int, int],   # Grid offset (stud interface frame)
    yaw_index: int,            # C4 Yaw index (0, 1, 2, or 3)
):
    """
    Assembles a grasped brick (hole) onto a target brick (stud) using 
    specified connection parameters. It relies on physical interaction (pressing)
    to trigger the automatic assembly connection detection.
    """
    press_depth = 0.000
    press_duration = 1.0

    print(f"--- Attempting to assemble {hole_brick_path} onto {stud_brick_path} ---")
    print(f"-> Offset: {offset}, Yaw Index: {yaw_index}")

    # Interface IDs for standard bricks (Defined in C++ specs)
    STUD_ID = 1
    HOLE_ID = 0
    
    # Interpret press_force into motion parameters (kinematic proxy for force control)
    # Higher force -> slightly deeper penetration target and longer pressing time
    # This ensures contact force is generated by the underlying PD controllers.
    assembly_height_offset = 0.04 # 4 cm approach height

    # 1. Calculate Required Relative Transform (T_S_H)
    # compute_connection_transform returns (WXYZ quat, pos)
    T_S_H_raw = compute_connection_transform(
        stud_path=stud_brick_path,
        stud_if=STUD_ID,
        hole_path=hole_brick_path,
        hole_if=HOLE_ID,
        offset=offset,
        yaw=yaw_index
    )
    T_S_H_quat = np.array(T_S_H_raw[0], dtype=np.float64)
    T_S_H_pos = np.array(T_S_H_raw[1], dtype=np.float64)

    # 2. Get Stud Brick Pose (T_W_S)
    stud_brick_xf = SingleXFormPrim(prim_path=stud_brick_path, name="assembly_target_stud")
        
    # T_W_S (World to Stud Brick)
    T_W_S_pos, T_W_S_quat = stud_brick_xf.get_world_pose()

    # 3. Calculate Target Gripper Pose (T_W_G_target)
    # T_W_G = T_W_S @ T_S_H @ T_H_G

    ####
    # Measure the actual relative transform after the grasp is stable.
    # T_B_G = inverse(T_W_B) @ T_W_G
    
    print("-> Calculating actual grasp transform T_B_G.")
    ee_prim = rmpflow.get_end_effector_as_prim()
    brick_xf = SingleXFormPrim(prim_path=hole_brick_path, name="assembly_grasped_hole")
    
    # Get actual poses T_W_G and T_W_B
    T_W_G_actual_pos, T_W_G_actual_quat = ee_prim.get_world_pose()
    T_W_B_actual_pos, T_W_B_actual_quat = brick_xf.get_world_pose()

    # Calculate inverse(T_W_B) => T_B_W
    T_B_W_actual_pos, T_B_W_actual_quat = inverse_transform(T_W_B_actual_pos, T_W_B_actual_quat)

    # Calculate T_B_G = T_B_W @ T_W_G
    T_B_G_actual_pos, T_B_G_actual_quat = compose_transforms(
        T_B_W_actual_pos, T_B_W_actual_quat,
        T_W_G_actual_pos, T_W_G_actual_quat
    )
    T_hole_G = (T_B_G_actual_pos, T_B_G_actual_quat)
    ####

    T_H_G_pos, T_H_G_quat = T_hole_G

    # Combine T_W_S @ T_S_H => T_W_H
    T_W_H_pos, T_W_H_quat = compose_transforms(
        T_W_S_pos, T_W_S_quat,
        T_S_H_pos, T_S_H_quat
    )
    
    # Combine T_W_H @ T_H_G => T_W_G_target
    T_W_G_target_pos, T_W_G_target_quat = compose_transforms(
        T_W_H_pos, T_W_H_quat,
        T_H_G_pos, T_H_G_quat
    )

    # 4. Define Approach/Retreat Vectors
    # Approach along the stud brick's Z-axis (the direction studs point).
    R_W_S = quats_to_rot_matrices(T_W_S_quat)
    approach_vector_W = R_W_S[:, 2]

    # 5. Define Poses
    
    # Pre-assembly pose: Above the target pose
    pre_assembly_pos = T_W_G_target_pos + approach_vector_W * assembly_height_offset
    
    # Press pose: Move slightly below the target pose along the approach vector
    # This penetration target ensures contact.
    press_assembly_pos = T_W_G_target_pos - approach_vector_W * press_depth

    # Retreat pose (same as pre-assembly or higher)
    retreat_pos = pre_assembly_pos.copy()
    retreat_pos[2] += 0.05 # Lift slightly higher in world Z after assembly

    # 6. Execute Assembly Sequence

    # 6.1 Move to Pre-assembly Pose
    print("-> Moving to pre-assembly pose.")
    success = await move_ee_to(
        world, robot, rmpflow, motion_policy, pre_assembly_pos, T_W_G_target_quat,
        pos_tol=0.02,
        rot_tol=0.10,
        vel_tol=None,
        timeout=10.0,
    )
    if not success:
        print("Failed to reach pre-assembly pose.")
        
    # 6.2 Move down to Pressing Pose and Hold
    print(f"-> Moving to assembly pose and pressing (depth={press_depth*1000:.1f}mm, duration={press_duration:.1f}s).")
    # Use tight tolerances for alignment. 
    # The settle_time ensures the command is maintained, allowing force to build up and the physics snap to occur.
    success = await move_ee_to(
        world, robot, rmpflow, motion_policy, press_assembly_pos, T_W_G_target_quat,
        pos_tol=0.003,
        rot_tol=0.050,
        vel_tol=0.30,
        timeout=3.0,
        settle_time=press_duration,
    )
    
    # The physics engine should detect the force/alignment and create the connection during the settle_time.
    if not success:
        # Failure here often means timeout during settling (which is common when pressing against a surface).
        print("Note: Pressing sequence reported timeout or tolerance failure (may be expected due to contact).")

    # 6.3 Release Gripper
    print("-> Releasing brick.")
    # Open gripper wide
    if not await set_gripper(world, robot, delta_width=+0.015):
        print("Warning: Gripper opening sequence reported timeout.")

    # 6.4 Retreat
    print("-> Retreating.")
    success = await move_ee_to(
        world, robot, rmpflow, motion_policy, retreat_pos, T_W_G_target_quat,
        pos_tol=0.02,
        rot_tol=0.2,
        vel_tol=None,
        timeout=10.0,
    )

    print("--- Assembly sequence finished. ---")
    # Returns the success status of the final retreat motion.
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

    # Unset instanceable
    stage = get_current_stage()
    stage.GetPrimAtPath("/World/Robot/panda_rightfinger/geometry").SetInstanceable(False)
    stage.GetPrimAtPath("/World/Robot/panda_leftfinger/geometry").SetInstanceable(False)

    # Set physics material for fingertip pads
    pad_material = PhysicsMaterial(
        prim_path="/World/PhysicsMaterials/FingerPad",
        static_friction=2.5,
        dynamic_friction=2.0,
        restitution=0.0,
    )
    SingleGeometryPrim(prim_path="/World/Robot/panda_leftfinger/geometry/panda_leftfinger").apply_physics_material(pad_material)
    SingleGeometryPrim(prim_path="/World/Robot/panda_rightfinger/geometry/panda_rightfinger").apply_physics_material(pad_material)

    # Adjust finger max force & stiffness & damping
    FINGER_MAX_FORCE = 15.0 # Default: 7.2
    FINGER_DAMPING = 80.0 # Default: 80.0
    FINGER_STIFFNESS = 400.0 # Default: 400.0
    joint_prim = get_current_stage().GetPrimAtPath("/World/Robot/panda_hand/panda_finger_joint1")
    joint_prim.GetAttribute("drive:linear:physics:maxForce").Set(FINGER_MAX_FORCE)
    joint_prim.GetAttribute("drive:linear:physics:damping").Set(FINGER_DAMPING)
    joint_prim.GetAttribute("drive:linear:physics:stiffness").Set(FINGER_STIFFNESS)

    # Increase position solver iterations for better assembly stability (32 -> 64)
    get_current_stage().GetPrimAtPath("/World/Robot").GetAttribute("physxArticulation:solverPositionIterationCount").Set(64)
    # get_current_stage().GetPrimAtPath("/World/Robot").GetAttribute("physxArticulation:solverVelocityIterationCount").Set(1)

    # Set physics material for tabletop
    table_material = PhysicsMaterial(
        prim_path="/World/PhysicsMaterials/Tabletop",
        static_friction=1.0,
        dynamic_friction=0.8,
        restitution=0.2,
    )
    SingleGeometryPrim(prim_path="/World/scene/roomScene/colliders/table/tableTopActor").apply_physics_material(table_material)

    # Set assembly thresholds
    thresholds = AssemblyThresholds()
    thresholds.distance_tolerance = 0.001
    thresholds.max_penetration = 0.005
    thresholds.z_angle_tolerance = 5.0 * (math.pi / 180.0)
    thresholds.required_force = 1.0
    thresholds.yaw_tolerance = 5.0 * (math.pi / 180.0)
    thresholds.position_tolerance = 0.002
    set_assembly_thresholds(thresholds)

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

    # Setup workspace
    workspace_prim = get_current_stage().GetPrimAtPath("/World/LegoWorkspace")
    workspace_prim.GetAttribute("xformOp:scale").Set(Gf.Vec3d(0.4, 0.3, 0.2))
    workspace_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.35, 0.0, 0.1))
    workspace_prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    workspace_prim.GetRelationship("lego:workspace_obstacles").AddTarget("/World/Robot/panda_link0")

    # Load structure to assemble (including base plate)
    with open(STRUCTURE_PATH, 'r') as f:
        text = f.read()
    if is_bricks_text(text):
        # StableText2Brick format
        topology = bricks_text_to_topology_json(text, color=parse_color("Pink"), **BASEPLATE_KWARGS)
        randomize_colors = True
    elif is_legolization_json(text):
        topology = legolization_json_to_topology_json(json.loads(text), color=parse_color("Pink"), **BASEPLATE_KWARGS)
        randomize_colors = True
    else:
        # Assume direct topology JSON
        topology = json.loads(text)
        randomize_colors = False
    if randomize_colors:
        # Randomize colors of non-baseplate parts for better visual distinction
        rng = np.random.default_rng(seed=42)
        color_names = list(Colors.keys())
        for part in topology['parts']:
            if part['id'] == 0:
                # Baseplate
                continue
            part['payload']['color'] = parse_color(rng.choice(color_names))

    # Place pre-placed parts
    pre_placed_topology = deepcopy(topology)
    pre_placed_topology['parts'] = [
        part
        for part in topology['parts']
        if part['id'] in PRE_PLACED_PARTS
    ]
    pre_placed, not_pre_placed = import_lego(
        json=pre_placed_topology,
        env_id=-1,
        ref_pos=PLACE_POSE[0],
        ref_rot=PLACE_POSE[1],
    )
    if len(not_pre_placed) > 0:
        raise RuntimeError(f"Failed to place pre-placed parts; not placed: {not_pre_placed}")

    # Spawn unplaced parts on table for assembly
    to_place_topology = deepcopy(topology)
    to_place_topology['parts'] = [
        part
        for part in topology['parts']
        if part['id'] not in PRE_PLACED_PARTS
    ]
    to_place_topology['connections'] = []
    to_place_topology['pose_hints'] = []
    to_place_placed, to_place_not_placed = import_lego(
        json=to_place_topology,
        env_id=-1
    )
    if len(to_place_not_placed) > 0:
        raise RuntimeError(f"Failed to place to-place parts; not placed: {to_place_not_placed}")
    arranged, not_arranged = arrange_parts_in_workspace(
        workspace_path="/World/LegoWorkspace",
        parts_to_arrange=[path for id, path in to_place_placed.items()],
    )
    if len(not_arranged) > 0:
        raise RuntimeError(f"Failed to arrange all parts in workspace; not arranged: {not_arranged}")

    # Generate a naive assembly plan
    sorted_topology = bfs_sort_connections(topology)
    def part_id_to_path(id):
        if id in pre_placed:
            return pre_placed[id]
        else:
            return to_place_placed[id]
    print("Assembly Order:")
    def format_part(id):
        if id in pre_placed:
            return f"{pre_placed[id]} (pre-placed)"
        else:
            return f"{to_place_placed[id]}"
    plan = []
    for conn in sorted_topology['connections']:
        skip = conn['stud_id'] in PRE_PLACED_PARTS and conn['hole_id'] in PRE_PLACED_PARTS
        if not skip:
            plan.append({
                'stud_path': part_id_to_path(conn['stud_id']),
                'stud_iface': conn['stud_iface'],
                'hole_path': part_id_to_path(conn['hole_id']),
                'hole_iface': conn['hole_iface'],
                'offset': conn['offset'],
                'yaw': conn['yaw'],
            })
        print(f" {'SKIP' if skip else '    '} #{conn['id']}: stud = {format_part(conn['stud_id'])} % {conn['stud_iface']}; hole = {format_part(conn['hole_id'])} % {conn['hole_iface']}; offset = {conn['offset']}, yaw = {conn['yaw']}")

    # Start simulation loop
    await world.reset_async()
    await world.play_async()

    async def assemble(stud_path: str, hole_path: str, offset: tuple[int, int], yaw: int):
        success = await grasp_lego_part(world, robot, rmpflow, motion_policy, hole_path)
        if not success:
            raise RuntimeError("Grasp failed; cannot proceed to assembly.")
        success = await assemble_lego_part(world, robot, rmpflow, motion_policy, stud_path, hole_path, offset, yaw)
        if not success:
            raise RuntimeError("Assembly failed during motion execution.")

    for step, conn in enumerate(plan):
        print(f"=== Assembly Step {step + 1} / {len(plan)} ===")
        print(f"  stud: {conn['stud_path']} % {conn['stud_iface']}; hole: {conn['hole_path']} % {conn['hole_iface']}; offset: {conn['offset']}, yaw: {conn['yaw']}")
        if are_parts_connected(conn['stud_path'], conn['hole_path']):
            print("  -> Parts are already connected; skipping this assembly step.")
            continue
        await assemble(
            stud_path=conn['stud_path'],
            hole_path=conn['hole_path'],
            offset=tuple(conn['offset']),
            yaw=conn['yaw'],
        )
