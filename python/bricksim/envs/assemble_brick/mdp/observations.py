"""Observation terms for the assemble-brick MDP."""

from typing import Literal

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import (
    quat_apply_inverse,
    quat_unique,
    subtract_frame_transforms,
)
from isaaclab_tasks.manager_based.manipulation.place.mdp.observations import (
    object_grasped,
)

from .common import (
    connection_target_match,
    gripper_is_open,
    marker_pose_w,
    wrong_connection_to_target,
)


def object_grasped_obs(
    env,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
) -> torch.Tensor:
    """Return the object-grasped indicator as a float observation.

    Returns:
        Float tensor with shape ``(num_envs, 1)``.
    """
    return (
        object_grasped(
            env,
            robot_cfg=robot_cfg,
            ee_frame_cfg=ee_frame_cfg,
            object_cfg=object_cfg,
            diff_threshold=diff_threshold,
        )
        .to(torch.float32)
        .unsqueeze(-1)
    )


def marker_pose_in_robot_root_frame(
    env,
    marker_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    return_key: Literal["pos", "quat", None] = None,
) -> torch.Tensor:
    """Return marker pose expressed in the robot root frame.

    Returns:
        Position, quaternion, or concatenated pose tensor depending on
        ``return_key``.
    """
    robot_asset = env.scene[robot_cfg.name]
    marker_pos_w, marker_quat_w = marker_pose_w(env, marker_cfg)
    marker_pos_b, marker_quat_b = subtract_frame_transforms(
        robot_asset.data.root_pos_w,
        robot_asset.data.root_quat_w,
        marker_pos_w,
        marker_quat_w,
    )
    if return_key == "pos":
        return marker_pos_b
    if return_key == "quat":
        return marker_quat_b
    return torch.cat((marker_pos_b, marker_quat_b), dim=1)


def rigid_object_velocity_in_robot_root_frame(
    env,
    asset_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    return_key: Literal["lin", "ang", None] = None,
) -> torch.Tensor:
    """Return rigid-object velocity expressed in the robot root frame.

    Returns:
        Linear velocity, angular velocity, or concatenated velocity tensor
        depending on ``return_key``.
    """
    robot_asset = env.scene[robot_cfg.name]
    asset = env.scene[asset_cfg.name]
    lin_vel_b = quat_apply_inverse(
        robot_asset.data.root_quat_w, asset.data.root_lin_vel_w[:, :3]
    )
    ang_vel_b = quat_apply_inverse(
        robot_asset.data.root_quat_w, asset.data.root_ang_vel_w[:, :3]
    )
    if return_key == "lin":
        return lin_vel_b
    if return_key == "ang":
        return ang_vel_b
    return torch.cat((lin_vel_b, ang_vel_b), dim=1)


def object_marker_pose_error(
    env,
    object_cfg: SceneEntityCfg,
    marker_cfg: SceneEntityCfg,
    return_key: Literal["pos", "quat", None] = None,
) -> torch.Tensor:
    """Return object pose error expressed in the marker frame.

    Returns:
        Position error, quaternion error, or concatenated error tensor
        depending on ``return_key``.
    """
    object = env.scene[object_cfg.name]
    target_pos_w, target_quat_w = marker_pose_w(env, marker_cfg)
    delta_pos_t, delta_quat_t = subtract_frame_transforms(
        target_pos_w, target_quat_w, object.data.root_pos_w, object.data.root_quat_w
    )
    delta_quat_t = quat_unique(delta_quat_t)
    if return_key == "pos":
        return delta_pos_t
    if return_key == "quat":
        return delta_quat_t
    return torch.cat((delta_pos_t, delta_quat_t), dim=1)


def goal_target_match_obs(
    env,
    stud_if: int,
    hole_if: int,
    target_offset: tuple[int, int],
    target_yaw: int,
    object_cfg: SceneEntityCfg,
    stud_cfg: SceneEntityCfg = SceneEntityCfg("lego_baseplate"),
) -> torch.Tensor:
    """Return the target-connection match indicator as a float observation.

    Returns:
        Float tensor with shape ``(num_envs, 1)``.
    """
    return (
        connection_target_match(
            env,
            stud_if=stud_if,
            hole_if=hole_if,
            target_offset=target_offset,
            target_yaw=target_yaw,
            object_cfg=object_cfg,
            stud_cfg=stud_cfg,
        )
        .to(torch.float32)
        .unsqueeze(-1)
    )


def wrong_connection_obs(
    env,
    stud_if: int,
    hole_if: int,
    target_offset: tuple[int, int],
    target_yaw: int,
    object_cfg: SceneEntityCfg,
    stud_cfg: SceneEntityCfg = SceneEntityCfg("lego_baseplate"),
) -> torch.Tensor:
    """Return the wrong-connection indicator as a float observation.

    Returns:
        Float tensor with shape ``(num_envs, 1)``.
    """
    return (
        wrong_connection_to_target(
            env,
            stud_if=stud_if,
            hole_if=hole_if,
            target_offset=target_offset,
            target_yaw=target_yaw,
            object_cfg=object_cfg,
            stud_cfg=stud_cfg,
        )
        .to(torch.float32)
        .unsqueeze(-1)
    )


def gripper_is_open_obs(
    env,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Return the gripper-open indicator as a float observation.

    Returns:
        Float tensor with shape ``(num_envs, 1)``.
    """
    return gripper_is_open(env, robot_cfg).to(torch.float32).unsqueeze(-1)


def captured_hole_to_eef_obs(
    env,
    return_key: Literal["pos", "quat", None] = None,
) -> torch.Tensor:
    """Return the expert-captured hole-to-end-effector transform.

    Returns:
        Position, quaternion, or concatenated transform tensor depending on
        ``return_key``.
    """
    dtype = env.scene.env_origins.dtype
    expert = getattr(env, "_expert", None)
    pos = getattr(expert, "_captured_hole_to_eef_pos", None)
    quat = getattr(expert, "_captured_hole_to_eef_quat", None)
    valid = getattr(expert, "_captured_valid", None)

    if pos is None or quat is None or valid is None:
        pos = torch.zeros((env.num_envs, 3), device=env.device, dtype=dtype)
        quat = torch.zeros((env.num_envs, 4), device=env.device, dtype=dtype)
    else:
        pos = torch.where(valid.unsqueeze(-1), pos, torch.zeros_like(pos))
        quat = torch.where(valid.unsqueeze(-1), quat, torch.zeros_like(quat))

    if return_key == "pos":
        return pos
    if return_key == "quat":
        return quat
    return torch.cat((pos, quat), dim=1)
