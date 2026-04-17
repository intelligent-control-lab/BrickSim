"""Reward terms for the assemble-brick MDP."""

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab_tasks.manager_based.manipulation.place.mdp.observations import (
    object_grasped,
)

from .common import (
    assemble_brick_goal_satisfied,
    marker_pose_w,
    object_marker_pose_alignment,
)


def grasp_bonus_from_object_grasped(
    env,
    object_cfg: SceneEntityCfg,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
) -> torch.Tensor:
    """Return a bonus when the object is grasped.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    return object_grasped(
        env,
        robot_cfg=robot_cfg,
        ee_frame_cfg=ee_frame_cfg,
        object_cfg=object_cfg,
        diff_threshold=diff_threshold,
    ).to(torch.float32)


def lift_bonus_relative_to_target(
    env,
    object_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    lift_height: float = 0.03,
) -> torch.Tensor:
    """Return a bonus when the object is lifted above the target.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    object = env.scene[object_cfg.name]
    target_pos_w, _ = marker_pose_w(env, target_cfg)
    return (object.data.root_pos_w[:, 2] > (target_pos_w[:, 2] + lift_height)).to(
        torch.float32
    )


def object_transport_xy(
    env,
    std: float,
    object_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
) -> torch.Tensor:
    """Return a grasp-gated XY transport reward.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    _, xy_dist, _ = object_marker_pose_alignment(env, object_cfg, target_cfg)
    grasped = object_grasped(
        env,
        robot_cfg=robot_cfg,
        ee_frame_cfg=ee_frame_cfg,
        object_cfg=object_cfg,
        diff_threshold=diff_threshold,
    )
    return grasped.to(torch.float32) * (1.0 - torch.tanh(xy_dist / std))


def object_yaw_align(
    env,
    std: float,
    object_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
) -> torch.Tensor:
    """Return a grasp-gated yaw-alignment reward.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    _, _, rot_error = object_marker_pose_alignment(env, object_cfg, target_cfg)
    grasped = object_grasped(
        env,
        robot_cfg=robot_cfg,
        ee_frame_cfg=ee_frame_cfg,
        object_cfg=object_cfg,
        diff_threshold=diff_threshold,
    )
    return grasped.to(torch.float32) * (1.0 - torch.tanh(rot_error / std))


def object_pre_insert_height(
    env,
    std: float,
    object_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
    target_height_offset: float = 0.02,
    loose_xy_threshold: float = 0.03,
    loose_rot_threshold: float = 0.25,
) -> torch.Tensor:
    """Return a reward for reaching pre-insertion height near the target.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    object = env.scene[object_cfg.name]
    target_pos_w, _ = marker_pose_w(env, target_cfg)
    _, xy_dist, rot_error = object_marker_pose_alignment(env, object_cfg, target_cfg)
    desired_z = target_pos_w[:, 2] + target_height_offset
    z_err = torch.abs(object.data.root_pos_w[:, 2] - desired_z)
    gate = torch.logical_and(
        xy_dist < loose_xy_threshold, rot_error < loose_rot_threshold
    )
    gate = torch.logical_and(
        gate,
        object_grasped(
            env,
            robot_cfg=robot_cfg,
            ee_frame_cfg=ee_frame_cfg,
            object_cfg=object_cfg,
            diff_threshold=diff_threshold,
        ),
    )
    return gate.to(torch.float32) * (1.0 - torch.tanh(z_err / std))


def object_insert_z(
    env,
    std: float,
    object_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
    tight_xy_threshold: float = 0.012,
    tight_rot_threshold: float = 0.12,
) -> torch.Tensor:
    """Return a reward for vertical insertion alignment.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    object = env.scene[object_cfg.name]
    target_pos_w, _ = marker_pose_w(env, target_cfg)
    _, xy_dist, rot_error = object_marker_pose_alignment(env, object_cfg, target_cfg)
    z_err = torch.abs(object.data.root_pos_w[:, 2] - target_pos_w[:, 2])
    gate = torch.logical_and(
        xy_dist < tight_xy_threshold, rot_error < tight_rot_threshold
    )
    gate = torch.logical_and(
        gate,
        object_grasped(
            env,
            robot_cfg=robot_cfg,
            ee_frame_cfg=ee_frame_cfg,
            object_cfg=object_cfg,
            diff_threshold=diff_threshold,
        ),
    )
    return gate.to(torch.float32) * (1.0 - torch.tanh(z_err / std))


def assemble_brick_success_bonus(
    env,
    stud_if: int,
    hole_if: int,
    target_offset: tuple[int, int],
    target_yaw: int,
    object_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    stud_cfg: SceneEntityCfg = SceneEntityCfg("lego_baseplate"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    pos_tol: float = 0.002,
    rot_tol: float = 0.08726646259971647,
) -> torch.Tensor:
    """Return a sparse success bonus for the target assembly.

    Returns:
        Float tensor with shape ``(num_envs,)``.
    """
    return assemble_brick_goal_satisfied(
        env,
        stud_if=stud_if,
        hole_if=hole_if,
        target_offset=target_offset,
        target_yaw=target_yaw,
        object_cfg=object_cfg,
        target_cfg=target_cfg,
        stud_cfg=stud_cfg,
        robot_cfg=robot_cfg,
        pos_tol=pos_tol,
        rot_tol=rot_tol,
    ).to(torch.float32)
