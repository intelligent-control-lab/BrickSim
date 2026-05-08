"""Official expert policy for the assemble-brick task."""

from collections.abc import Mapping

import isaaclab.utils.math as math_utils
import torch

from bricksim.units import BRICK_UNIT_LENGTH, PLATE_UNIT_HEIGHT

from .env import ARM_ACTION_SCALE

# Units: meters. TCP is the current Franka pad-center frame.
TCP_TO_FINGER_TIP = 0.0090
GRASP_DEPTH = 0.0050
ONE_WIDE_GRASP_DEPTH_EXTRA = 0.0010

PRE_GRASP_OFFSET = 0.10
LIFT_HEIGHT = 0.10
ASSEMBLY_HEIGHT_OFFSET = 0.04
PRESS_DEPTH = 0.003
ASSEMBLY_XY_TOL = 0.02
TRANSPORT_XY_THRESHOLD = 0.36
TRANSPORT_ROT_SENSITIVE_MIN = 0.9
TRANSPORT_ROT_SENSITIVE_TOL = 0.935

PRE_GRASP_POS_TOL = 0.03
PRE_GRASP_ROT_TOL = 0.75
GRASP_POS_TOL = 0.004
GRASP_ROT_TOL = 0.50
LIFT_POS_TOL = 0.03
PRE_ASSEMBLY_ROT_TOL = 0.25

GRIPPER_POS_TOL = 0.003
GRIPPER_SECURED_SPEED_TOL = 0.003
TRANSPORT_GRIPPER_COMPRESSION = 0.00012
OPEN_GRIPPER_EXTRA_WIDTH = 0.015
MAX_FRANKA_GRIPPER_WIDTH = 0.08


class AssembleBrickExpert:
    """Official memory-less scripted policy for assemble_brick task.

    The policy uses only static action-scale configuration and the current privileged
    observation passed to `compute_actions`; it stores no per-rollout state. It returns
    batched relative IK pose commands plus a binary gripper command.
    """

    def __init__(self, arm_action_scale: tuple[float, ...] = ARM_ACTION_SCALE) -> None:
        """Initialize the expert with static action configuration."""
        self.arm_action_scale = arm_action_scale

    def compute_actions(self, obs: Mapping[str, torch.Tensor]) -> torch.Tensor:
        """Return a batched action tensor from the current privileged observation.

        Args:
            obs: Non-concatenated ``observations["privileged"]`` dictionary.

        Returns:
            Action tensor with shape ``(num_envs, 7)``. The first six values are
            relative IK pose commands and the final value is the binary gripper
            command, where ``-1`` closes and ``+1`` opens.
        """
        ee_pose = obs["ee_frame_pose"]
        ee_pos = ee_pose[:, :3]
        ee_quat = ee_pose[:, 3:7]
        brick_pose = obs["brick_pose"]
        brick_pos = brick_pose[:, :3]
        brick_quat = brick_pose[:, 3:7]
        target_pose = obs["target_pose"]
        target_pos = target_pose[:, :3]
        target_quat = target_pose[:, 3:7]
        dimensions = obs["brick_dimensions"].to(dtype=brick_pose.dtype)

        (
            grasp_width,
            grasp_pos,
            grasp_quat,
            pre_grasp_pos,
            grasp_pos_h,
            grasp_quat_h,
        ) = self._compute_grasp_targets(
            ee_quat=ee_quat,
            brick_pos=brick_pos,
            brick_quat=brick_quat,
            dimensions=dimensions,
        )
        (
            lateral_distance,
            rot_error,
            pre_assembly_pos,
            press_pos,
            assembly_cmd_quat,
        ) = self._compute_assembly_state(
            target_pos=target_pos,
            target_quat=target_quat,
            brick_pos=brick_pos,
            brick_quat=brick_quat,
            ee_quat=ee_quat,
            grasp_pos_h=grasp_pos_h,
            grasp_quat_h=grasp_quat_h,
        )
        lateral_aligned = lateral_distance < ASSEMBLY_XY_TOL
        rot_aligned = rot_error < PRE_ASSEMBLY_ROT_TOL

        lift_pos = ee_pos.clone()
        lift_pos[:, 2] = self._transport_height(target_pos, dimensions)

        gripper_width = obs["gripper_width"].squeeze(-1)
        gripper_speed = obs["gripper_speed"].squeeze(-1)
        connection_created = obs["connection_created"]
        if connection_created.ndim == 2:
            connection_created = connection_created.squeeze(-1)
        connection_created = connection_created.to(dtype=torch.bool)
        gripper_open = gripper_width >= MAX_FRANKA_GRIPPER_WIDTH - GRIPPER_POS_TOL
        open_width = torch.minimum(
            grasp_width + OPEN_GRIPPER_EXTRA_WIDTH,
            torch.full_like(grasp_width, MAX_FRANKA_GRIPPER_WIDTH),
        )
        brick_grasped = obs["brick_grasped"]
        if brick_grasped.ndim == 2:
            brick_grasped = brick_grasped.squeeze(-1)
        brick_grasped = brick_grasped.to(dtype=torch.bool)
        ready_to_descend = self._aligned_for_descent(
            ee_pos,
            ee_quat,
            pre_grasp_pos,
            grasp_quat,
            PRE_GRASP_POS_TOL,
            PRE_GRASP_ROT_TOL,
        )
        grasp_reached = self._pose_reached(
            ee_pos,
            ee_quat,
            grasp_pos,
            grasp_quat,
            GRASP_POS_TOL,
            GRASP_ROT_TOL,
        )
        width_near_grasp = torch.abs(gripper_width - grasp_width) <= GRIPPER_POS_TOL
        brick_lifted = brick_pos[:, 2] > target_pos[:, 2] + 0.02
        lifting_from_grasp = (ee_pos[:, 2] > grasp_pos[:, 2] + GRASP_POS_TOL) & (
            ee_pos[:, 2] < lift_pos[:, 2] - LIFT_POS_TOL
        )
        gripper_stalled = gripper_speed < GRIPPER_SECURED_SPEED_TOL
        grasp_secured = self._grasp_secured(
            ee_pos=ee_pos,
            brick_pos=brick_pos,
            lateral_distance=lateral_distance,
            rot_error=rot_error,
            gripper_width=gripper_width,
            grasp_width=grasp_width,
            width_near_grasp=width_near_grasp,
            brick_lifted=brick_lifted,
            lifting_from_grasp=lifting_from_grasp,
            gripper_stalled=gripper_stalled,
            brick_grasped=brick_grasped,
            grasp_reached=grasp_reached,
            near_target=lateral_aligned & rot_aligned,
        )
        partly_closed = gripper_width < MAX_FRANKA_GRIPPER_WIDTH - GRIPPER_POS_TOL
        failed_lift_without_brick = (
            width_near_grasp
            & ~brick_grasped
            & ~brick_lifted
            & (ee_pos[:, 2] >= lift_pos[:, 2] - LIFT_POS_TOL)
        )
        too_closed_without_brick = (
            ~brick_grasped
            & ~brick_lifted
            & (
                failed_lift_without_brick
                | (~width_near_grasp & (gripper_width < grasp_width - GRIPPER_POS_TOL))
            )
        )

        target_cmd_pos = ee_pos.clone()
        target_cmd_quat = ee_quat.clone()
        close_gripper = torch.zeros_like(connection_created)

        active_mask = ~connection_created
        not_grasped_mask = active_mask & ~grasp_secured
        needs_open_mask = not_grasped_mask & (
            too_closed_without_brick
            | (~partly_closed & (gripper_width < open_width - GRIPPER_POS_TOL))
        )
        move_pre_grasp_mask = (
            not_grasped_mask & ~needs_open_mask & ~partly_closed & ~ready_to_descend
        )
        move_grasp_mask = (
            not_grasped_mask
            & ~needs_open_mask
            & (ready_to_descend | partly_closed)
            & ~grasp_reached
        )
        close_at_grasp_mask = (
            not_grasped_mask
            & ~needs_open_mask
            & (ready_to_descend | partly_closed)
            & grasp_reached
        )

        target_cmd_pos[move_pre_grasp_mask] = pre_grasp_pos[move_pre_grasp_mask]
        target_cmd_quat[move_pre_grasp_mask] = grasp_quat[move_pre_grasp_mask]
        target_cmd_pos[move_grasp_mask] = grasp_pos[move_grasp_mask]
        target_cmd_quat[move_grasp_mask] = grasp_quat[move_grasp_mask]
        target_cmd_pos[close_at_grasp_mask] = grasp_pos[close_at_grasp_mask]
        target_cmd_quat[close_at_grasp_mask] = grasp_quat[close_at_grasp_mask]
        close_gripper |= close_at_grasp_mask
        close_gripper |= move_grasp_mask & partly_closed

        assembly_mask = active_mask & grasp_secured
        needs_lift = (
            assembly_mask
            & ~brick_lifted
            & (lateral_distance > ASSEMBLY_XY_TOL)
            & (ee_pos[:, 2] < lift_pos[:, 2] - LIFT_POS_TOL)
        )
        press_mask = assembly_mask & lateral_aligned & rot_aligned
        move_pre_assembly_mask = assembly_mask & ~needs_lift & ~press_mask
        target_cmd_pos[needs_lift] = lift_pos[needs_lift]
        target_cmd_quat[needs_lift] = ee_quat[needs_lift]
        target_cmd_pos[move_pre_assembly_mask] = pre_assembly_pos[
            move_pre_assembly_mask
        ]
        target_cmd_quat[move_pre_assembly_mask] = assembly_cmd_quat[
            move_pre_assembly_mask
        ]
        target_cmd_pos[press_mask] = press_pos[press_mask]
        target_cmd_quat[press_mask] = assembly_cmd_quat[press_mask]
        close_gripper |= assembly_mask

        pose_action = self._actions_to_target(
            ee_pos,
            ee_quat,
            target_cmd_pos,
            target_cmd_quat,
        )
        gripper_action = torch.where(
            close_gripper,
            torch.full_like(gripper_width, -1.0),
            torch.ones_like(gripper_width),
        ).unsqueeze(-1)
        actions = torch.cat((pose_action, gripper_action), dim=1)
        release_mask = connection_created & ~gripper_open
        idle_mask = connection_created & gripper_open
        actions[idle_mask] = 0.0
        actions[release_mask, :6] = 0.0
        return actions

    def _compute_grasp_targets(
        self,
        *,
        ee_quat: torch.Tensor,
        brick_pos: torch.Tensor,
        brick_quat: torch.Tensor,
        dimensions: torch.Tensor,
    ) -> tuple[
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
    ]:
        length = dimensions[:, 0]
        width = dimensions[:, 1]
        height = dimensions[:, 2]
        grasp_depth = self._grasp_depth(dimensions)
        grasp_width = torch.minimum(length, width) * BRICK_UNIT_LENGTH
        grasp_pos_h = torch.zeros_like(brick_pos)
        grasp_pos_h[:, 2] = height * PLATE_UNIT_HEIGHT + TCP_TO_FINGER_TIP - grasp_depth

        grasp_quat_h = self._grasp_quat_from_dimensions(dimensions)
        flip_about_tool_z = torch.tensor(
            [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
            device=brick_pos.device,
            dtype=brick_pos.dtype,
        )
        grasp_rot_h = math_utils.matrix_from_quat(grasp_quat_h)
        flipped_grasp_quat_h = math_utils.quat_from_matrix(
            grasp_rot_h @ flip_about_tool_z
        )
        grasp_pos, nominal_grasp_quat = math_utils.combine_frame_transforms(
            brick_pos,
            brick_quat,
            grasp_pos_h,
            grasp_quat_h,
        )
        _, flipped_grasp_quat = math_utils.combine_frame_transforms(
            brick_pos,
            brick_quat,
            grasp_pos_h,
            flipped_grasp_quat_h,
        )
        nominal_ang_err = math_utils.quat_error_magnitude(ee_quat, nominal_grasp_quat)
        flipped_ang_err = math_utils.quat_error_magnitude(ee_quat, flipped_grasp_quat)
        use_nominal = (nominal_ang_err <= flipped_ang_err).unsqueeze(-1)
        grasp_quat = torch.where(use_nominal, nominal_grasp_quat, flipped_grasp_quat)
        selected_grasp_quat_h = torch.where(
            use_nominal,
            grasp_quat_h,
            flipped_grasp_quat_h,
        )

        unit_z = torch.zeros_like(brick_pos)
        unit_z[:, 2] = 1.0
        approach_vector = math_utils.quat_apply(brick_quat, unit_z)
        pre_grasp_pos = grasp_pos + PRE_GRASP_OFFSET * approach_vector
        return (
            grasp_width,
            grasp_pos,
            grasp_quat,
            pre_grasp_pos,
            grasp_pos_h,
            selected_grasp_quat_h,
        )

    def _compute_assembly_state(
        self,
        *,
        target_pos: torch.Tensor,
        target_quat: torch.Tensor,
        brick_pos: torch.Tensor,
        brick_quat: torch.Tensor,
        ee_quat: torch.Tensor,
        grasp_pos_h: torch.Tensor,
        grasp_quat_h: torch.Tensor,
    ) -> tuple[
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
    ]:
        _, brick_to_target_quat = math_utils.subtract_frame_transforms(
            target_pos,
            target_quat,
            brick_pos,
            brick_quat,
        )
        brick_to_target_quat = math_utils.quat_unique(brick_to_target_quat)
        identity_quat = torch.zeros_like(brick_to_target_quat)
        identity_quat[:, 0] = 1.0
        rot_error = math_utils.quat_error_magnitude(brick_to_target_quat, identity_quat)

        target_approach_axis = torch.zeros_like(target_pos)
        target_approach_axis[:, 2] = 1.0
        target_approach_axis = math_utils.quat_apply(target_quat, target_approach_axis)
        target_delta = target_pos - brick_pos
        target_axial_delta = (
            torch.sum(target_delta * target_approach_axis, dim=1, keepdim=True)
            * target_approach_axis
        )
        target_lateral_delta = target_delta - target_axial_delta
        lateral_distance = torch.linalg.vector_norm(target_lateral_delta, dim=1)

        assembly_pos, assembly_quat = math_utils.combine_frame_transforms(
            target_pos,
            target_quat,
            grasp_pos_h,
            grasp_quat_h,
        )
        target_from_brick_quat = math_utils.quat_mul(
            target_quat, math_utils.quat_inv(brick_quat)
        )
        assembly_cmd_quat = math_utils.quat_mul(target_from_brick_quat, ee_quat)
        assembly_cmd_quat = torch.where(
            (lateral_distance < ASSEMBLY_XY_TOL).unsqueeze(-1),
            assembly_cmd_quat,
            assembly_quat,
        )
        pre_assembly_pos = assembly_pos + ASSEMBLY_HEIGHT_OFFSET * target_approach_axis
        press_pos = (
            assembly_pos - PRESS_DEPTH * target_approach_axis + target_lateral_delta
        )
        return (
            lateral_distance,
            rot_error,
            pre_assembly_pos,
            press_pos,
            assembly_cmd_quat,
        )

    def _grasp_secured(
        self,
        *,
        ee_pos: torch.Tensor,
        brick_pos: torch.Tensor,
        lateral_distance: torch.Tensor,
        rot_error: torch.Tensor,
        gripper_width: torch.Tensor,
        grasp_width: torch.Tensor,
        width_near_grasp: torch.Tensor,
        brick_lifted: torch.Tensor,
        lifting_from_grasp: torch.Tensor,
        gripper_stalled: torch.Tensor,
        brick_grasped: torch.Tensor,
        grasp_reached: torch.Tensor,
        near_target: torch.Tensor,
    ) -> torch.Tensor:
        horizontal_error = torch.linalg.vector_norm(
            ee_pos[:, :2] - brick_pos[:, :2], dim=1
        )
        transport = lateral_distance > TRANSPORT_XY_THRESHOLD
        rotation_sensitive_transport = (
            transport
            & (rot_error > TRANSPORT_ROT_SENSITIVE_MIN)
            & (rot_error < TRANSPORT_ROT_SENSITIVE_TOL)
        )
        deeply_compressed = gripper_width <= grasp_width - TRANSPORT_GRIPPER_COMPRESSION
        secure_enough_for_transport = ~rotation_sensitive_transport | deeply_compressed
        lost_brick_far_from_gripper = (
            ~brick_grasped & ~brick_lifted & (horizontal_error > GRASP_POS_TOL * 3.0)
        )
        stalled_contact = (
            width_near_grasp
            & gripper_stalled
            & (brick_grasped | grasp_reached)
            & secure_enough_for_transport
        )
        inferred_grasp = (
            (brick_grasped & (brick_lifted | lifting_from_grasp))
            | (width_near_grasp & brick_lifted)
            | stalled_contact
            | (near_target & width_near_grasp & brick_grasped)
        )
        return inferred_grasp & ~lost_brick_far_from_gripper

    def _grasp_depth(self, dimensions: torch.Tensor) -> torch.Tensor:
        length = dimensions[:, 0]
        width = dimensions[:, 1]
        return torch.where(
            (length == 1.0) | (width == 1.0),
            torch.full_like(length, GRASP_DEPTH + ONE_WIDE_GRASP_DEPTH_EXTRA),
            torch.full_like(length, GRASP_DEPTH),
        )

    def _transport_height(
        self, target_pos: torch.Tensor, dimensions: torch.Tensor
    ) -> torch.Tensor:
        height = dimensions[:, 2]
        return (
            target_pos[:, 2]
            + height * PLATE_UNIT_HEIGHT
            + TCP_TO_FINGER_TIP
            - self._grasp_depth(dimensions)
            + PRE_GRASP_OFFSET
            + LIFT_HEIGHT
        )

    def _grasp_quat_from_dimensions(self, dimensions: torch.Tensor) -> torch.Tensor:
        length = dimensions[:, 0]
        width = dimensions[:, 1]
        nominal_for_width = torch.tensor(
            [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
            device=dimensions.device,
            dtype=dimensions.dtype,
        )
        nominal_for_length = torch.tensor(
            [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
            device=dimensions.device,
            dtype=dimensions.dtype,
        )
        use_width = (width <= length).reshape(-1, 1, 1)
        grasp_rot_h = torch.where(use_width, nominal_for_width, nominal_for_length)
        return math_utils.quat_from_matrix(grasp_rot_h)

    def _pose_reached(
        self,
        source_pos: torch.Tensor,
        source_quat: torch.Tensor,
        target_pos: torch.Tensor,
        target_quat: torch.Tensor,
        pos_tol: float,
        rot_tol: float,
    ) -> torch.Tensor:
        pos_error = torch.linalg.vector_norm(source_pos - target_pos, dim=1)
        rot_error = math_utils.quat_error_magnitude(source_quat, target_quat)
        return (pos_error < pos_tol) & (rot_error < rot_tol)

    def _aligned_for_descent(
        self,
        source_pos: torch.Tensor,
        source_quat: torch.Tensor,
        target_pos: torch.Tensor,
        target_quat: torch.Tensor,
        horizontal_tol: float,
        rot_tol: float,
    ) -> torch.Tensor:
        horizontal_error = torch.linalg.vector_norm(
            source_pos[:, :2] - target_pos[:, :2], dim=1
        )
        rot_error = math_utils.quat_error_magnitude(source_quat, target_quat)
        return (horizontal_error < horizontal_tol) & (rot_error < rot_tol)

    def _actions_to_target(
        self,
        ee_pos: torch.Tensor,
        ee_quat: torch.Tensor,
        target_pos: torch.Tensor,
        target_quat: torch.Tensor,
    ) -> torch.Tensor:
        pos_error, rot_error = math_utils.compute_pose_error(
            ee_pos,
            ee_quat,
            target_pos,
            target_quat,
            rot_error_type="axis_angle",
        )
        delta_pose = torch.cat((pos_error, rot_error), dim=1)
        action_scale = delta_pose.new_tensor(self.arm_action_scale).unsqueeze(0)
        return torch.clamp(delta_pose / action_scale, -1.0, 1.0)
