"""Reward terms for the assemble-brick MDP."""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer
from isaaclab.utils.math import quat_error_magnitude
from isaaclab_tasks.manager_based.manipulation.place.mdp.observations import (
    object_grasped,
)

from .commands import (
    assembly_check_connection_formed,
    assembly_goal_target_pose,
    assembly_moving_brick_name,
)
from .common import gripper_is_open


def _object_command_pose_alignment(
    env: ManagerBasedRLEnv,
    command_name: str,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Compute moving-brick pose error relative to a command target.

    Args:
        env: Environment with a command manager and scene assets.
        command_name: Name of the assemble-brick command term.

    Returns:
        Tuple of position delta with shape ``(num_envs, 3)`` in meters,
        XY distance with shape ``(num_envs,)`` in meters, and quaternion
        angular error with shape ``(num_envs,)`` in radians. Position delta
        is moving-brick world position minus command target world position.
    """
    moving_brick = env.scene[assembly_moving_brick_name(env, command_name)]
    target_pos_w, target_quat_w = assembly_goal_target_pose(env, command_name)
    pos_delta = moving_brick.data.root_pos_w - target_pos_w
    xy_dist = torch.linalg.vector_norm(pos_delta[:, :2], dim=1)
    rot_error = quat_error_magnitude(moving_brick.data.root_quat_w, target_quat_w)
    return pos_delta, xy_dist, rot_error


def reward_reach_brick(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    std: float = 0.25,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward reaching the command moving brick with the end effector.

    Args:
        env: Environment with a command manager and scene assets.
        command_name: Name of the assemble-brick command term.
        std: Distance scale for the tanh reward kernel, in meters.
        ee_frame_cfg: Scene entity for the end-effector frame.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    moving_brick = env.scene[assembly_moving_brick_name(env, command_name)]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_pos_w = ee_frame.data.target_pos_w[..., 0, :]
    distance = torch.linalg.vector_norm(moving_brick.data.root_pos_w - ee_pos_w, dim=1)
    return 1.0 - torch.tanh(distance / std)


def reward_grasp_bonus(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
) -> torch.Tensor:
    """Return a bonus when the object is grasped.

    Args:
        env: Environment with scene assets.
        command_name: Name of the assemble-brick command term.
        ee_frame_cfg: Scene entity for the end-effector frame.
        robot_cfg: Scene entity for the robot articulation.
        diff_threshold: Object-to-end-effector grasp distance threshold in meters.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    return object_grasped(
        env,
        robot_cfg=robot_cfg,
        ee_frame_cfg=ee_frame_cfg,
        object_cfg=SceneEntityCfg(assembly_moving_brick_name(env, command_name)),
        diff_threshold=diff_threshold,
    ).to(torch.float32)


def reward_lift_bonus(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    lift_height: float = 0.03,
) -> torch.Tensor:
    """Return a bonus when the object is lifted above the target.

    Args:
        env: Environment with a command manager and scene assets.
        command_name: Name of the assemble-brick command term.
        lift_height: Required height above the command target in meters.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    moving_brick = env.scene[assembly_moving_brick_name(env, command_name)]
    target_pos_w, _ = assembly_goal_target_pose(env, command_name)
    return (moving_brick.data.root_pos_w[:, 2] > (target_pos_w[:, 2] + lift_height)).to(
        torch.float32
    )


def reward_transport_xy(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    std: float = 0.04,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
) -> torch.Tensor:
    """Return a grasp-gated XY transport reward.

    Args:
        env: Environment with a command manager and scene assets.
        command_name: Name of the assemble-brick command term.
        std: Distance scale for the tanh reward kernel, in meters.
        ee_frame_cfg: Scene entity for the end-effector frame.
        robot_cfg: Scene entity for the robot articulation.
        diff_threshold: Object-to-end-effector grasp distance threshold in meters.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    _, xy_dist, _ = _object_command_pose_alignment(env, command_name)
    grasped = object_grasped(
        env,
        robot_cfg=robot_cfg,
        ee_frame_cfg=ee_frame_cfg,
        object_cfg=SceneEntityCfg(assembly_moving_brick_name(env, command_name)),
        diff_threshold=diff_threshold,
    )
    return grasped.to(torch.float32) * (1.0 - torch.tanh(xy_dist / std))


def reward_yaw_align(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    std: float = 0.20,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
) -> torch.Tensor:
    """Return a grasp-gated yaw-alignment reward.

    Args:
        env: Environment with a command manager and scene assets.
        command_name: Name of the assemble-brick command term.
        std: Angular scale for the tanh reward kernel, in radians.
        ee_frame_cfg: Scene entity for the end-effector frame.
        robot_cfg: Scene entity for the robot articulation.
        diff_threshold: Object-to-end-effector grasp distance threshold in meters.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    _, _, rot_error = _object_command_pose_alignment(env, command_name)
    grasped = object_grasped(
        env,
        robot_cfg=robot_cfg,
        ee_frame_cfg=ee_frame_cfg,
        object_cfg=SceneEntityCfg(assembly_moving_brick_name(env, command_name)),
        diff_threshold=diff_threshold,
    )
    return grasped.to(torch.float32) * (1.0 - torch.tanh(rot_error / std))


def reward_pre_insert_height(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    std: float = 0.01,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
    target_height_offset: float = 0.02,
    loose_xy_threshold: float = 0.03,
    loose_rot_threshold: float = 0.25,
) -> torch.Tensor:
    """Return a reward for reaching pre-insertion height near the target.

    Args:
        env: Environment with a command manager and scene assets.
        command_name: Name of the assemble-brick command term.
        std: Height-error scale for the tanh reward kernel, in meters.
        ee_frame_cfg: Scene entity for the end-effector frame.
        robot_cfg: Scene entity for the robot articulation.
        diff_threshold: Object-to-end-effector grasp distance threshold in meters.
        target_height_offset: Pre-insertion height above the command target in meters.
        loose_xy_threshold: Maximum XY error for enabling this reward, in meters.
        loose_rot_threshold: Maximum angular error for enabling this reward, in radians.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    pos_delta, xy_dist, rot_error = _object_command_pose_alignment(env, command_name)
    z_err = torch.abs(pos_delta[:, 2] - target_height_offset)
    gate = torch.logical_and(
        xy_dist < loose_xy_threshold, rot_error < loose_rot_threshold
    )
    gate = torch.logical_and(
        gate,
        object_grasped(
            env,
            robot_cfg=robot_cfg,
            ee_frame_cfg=ee_frame_cfg,
            object_cfg=SceneEntityCfg(assembly_moving_brick_name(env, command_name)),
            diff_threshold=diff_threshold,
        ),
    )
    return gate.to(torch.float32) * (1.0 - torch.tanh(z_err / std))


def reward_insert_z(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    std: float = 0.006,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
    tight_xy_threshold: float = 0.012,
    tight_rot_threshold: float = 0.12,
) -> torch.Tensor:
    """Return a reward for vertical insertion alignment.

    Args:
        env: Environment with a command manager and scene assets.
        command_name: Name of the assemble-brick command term.
        std: Height-error scale for the tanh reward kernel, in meters.
        ee_frame_cfg: Scene entity for the end-effector frame.
        robot_cfg: Scene entity for the robot articulation.
        diff_threshold: Object-to-end-effector grasp distance threshold in meters.
        tight_xy_threshold: Maximum XY error for enabling this reward, in meters.
        tight_rot_threshold: Maximum angular error for enabling this reward, in radians.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    pos_delta, xy_dist, rot_error = _object_command_pose_alignment(env, command_name)
    z_err = torch.abs(pos_delta[:, 2])
    gate = torch.logical_and(
        xy_dist < tight_xy_threshold, rot_error < tight_rot_threshold
    )
    gate = torch.logical_and(
        gate,
        object_grasped(
            env,
            robot_cfg=robot_cfg,
            ee_frame_cfg=ee_frame_cfg,
            object_cfg=SceneEntityCfg(assembly_moving_brick_name(env, command_name)),
            diff_threshold=diff_threshold,
        ),
    )
    return gate.to(torch.float32) * (1.0 - torch.tanh(z_err / std))


def reward_success_bonus(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    action_term_name: str = "gripper_action",
    open_position_threshold: float = 0.005,
) -> torch.Tensor:
    """Return a sparse bonus when the target connection is formed and gripper is open.

    Args:
        env: Environment with a command manager and scene assets.
        command_name: Name of the assemble-brick command term.
        action_term_name: Name of the binary gripper action term.
        open_position_threshold: Joint-position tolerance around the action
            term's open command.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    return assembly_check_connection_formed(env, command_name) & gripper_is_open(
        env,
        action_term_name=action_term_name,
        open_position_threshold=open_position_threshold,
    )
