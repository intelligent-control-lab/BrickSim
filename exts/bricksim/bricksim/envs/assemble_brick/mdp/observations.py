from typing import Literal

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply_inverse, quat_unique, subtract_frame_transforms
from isaaclab_tasks.manager_based.manipulation.place.mdp.observations import object_grasped

from .common import marker_pose_w


def object_grasped_obs(
    env,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
) -> torch.Tensor:
    return object_grasped(
        env,
        robot_cfg=robot_cfg,
        ee_frame_cfg=ee_frame_cfg,
        object_cfg=object_cfg,
        diff_threshold=diff_threshold,
    ).to(torch.float32).unsqueeze(-1)


def marker_pose_in_robot_root_frame(
    env,
    marker_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    return_key: Literal["pos", "quat", None] = None,
) -> torch.Tensor:
    robot_asset = env.scene[robot_cfg.name]
    marker_pos_w, marker_quat_w = marker_pose_w(env, marker_cfg)
    marker_pos_b, marker_quat_b = subtract_frame_transforms(
        robot_asset.data.root_pos_w, robot_asset.data.root_quat_w, marker_pos_w, marker_quat_w
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
    robot_asset = env.scene[robot_cfg.name]
    asset = env.scene[asset_cfg.name]
    lin_vel_b = quat_apply_inverse(robot_asset.data.root_quat_w, asset.data.root_lin_vel_w[:, :3])
    ang_vel_b = quat_apply_inverse(robot_asset.data.root_quat_w, asset.data.root_ang_vel_w[:, :3])
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
