"""Shared MDP helpers for assemble-brick observations, rewards, and terms."""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.envs.mdp.actions.binary_joint_actions import BinaryJointPositionAction
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_error_magnitude

from bricksim.mdp.connection_state import (
    InterfacePairConnectionQuery,
    interface_pair_connection_state,
)


def marker_pose_w(
    env: ManagerBasedRLEnv,
    marker_cfg: SceneEntityCfg = SceneEntityCfg("marker_brick"),
) -> tuple[torch.Tensor, torch.Tensor]:
    """Return marker world poses as tensors.

    Returns:
        Tuple of world positions ``(num_envs, 3)`` and quaternions
        ``(num_envs, 4)``.
    """
    marker = env.scene[marker_cfg.name]
    pos_w, quat_w = marker.get_world_poses(indices=list(range(env.num_envs)))
    dtype = env.scene.env_origins.dtype
    return (
        torch.as_tensor(pos_w, device=env.device, dtype=dtype),
        torch.as_tensor(quat_w, device=env.device, dtype=dtype),
    )


def gripper_is_open(
    env: ManagerBasedRLEnv,
    action_term_name: str = "gripper_action",
    open_position_threshold: float = 0.005,
) -> torch.Tensor:
    """Return a mask indicating whether the gripper is open.

    Args:
        env: Manager-based RL environment.
        action_term_name: Name of the binary gripper action term.
        open_position_threshold: Joint-position tolerance around the action
            term's open command.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    action_term = env.action_manager.get_term(action_term_name)
    assert isinstance(action_term, BinaryJointPositionAction)
    joint_pos = action_term._asset.data.joint_pos[:, action_term._joint_ids]
    open_command = action_term._open_command.to(dtype=joint_pos.dtype)
    return torch.all(
        torch.abs(joint_pos - open_command) <= open_position_threshold, dim=1
    )


def object_marker_pose_alignment(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Compute object pose error relative to the marker target.

    Returns:
        Position delta, XY distance, and quaternion angular error.
    """
    object = env.scene[object_cfg.name]
    target_pos_w, target_quat_w = marker_pose_w(env, target_cfg)
    pos_delta = object.data.root_pos_w - target_pos_w
    xy_dist = torch.linalg.vector_norm(pos_delta[:, :2], dim=1)
    rot_error = quat_error_magnitude(object.data.root_quat_w, target_quat_w)
    return pos_delta, xy_dist, rot_error


def connection_target_match(
    env: ManagerBasedRLEnv,
    stud_if: int,
    hole_if: int,
    target_offset: tuple[int, int],
    target_yaw: int,
    object_cfg: SceneEntityCfg,
    stud_cfg: SceneEntityCfg = SceneEntityCfg("lego_baseplate"),
) -> torch.Tensor:
    """Return whether the queried connection matches the target placement.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    connection_state = interface_pair_connection_state(
        env,
        InterfacePairConnectionQuery.make(
            stud_cfg=stud_cfg,
            hole_cfg=object_cfg,
            stud_if=stud_if,
            hole_if=hole_if,
        ),
    )
    return (
        connection_state.connected
        & (connection_state.offsets[:, 0] == target_offset[0])
        & (connection_state.offsets[:, 1] == target_offset[1])
        & (connection_state.yaws == target_yaw)
    )


def wrong_connection_to_target(
    env: ManagerBasedRLEnv,
    stud_if: int,
    hole_if: int,
    target_offset: tuple[int, int],
    target_yaw: int,
    object_cfg: SceneEntityCfg,
    stud_cfg: SceneEntityCfg = SceneEntityCfg("lego_baseplate"),
) -> torch.Tensor:
    """Return whether the object is connected at a non-target placement.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    connection_state = interface_pair_connection_state(
        env,
        InterfacePairConnectionQuery.make(
            stud_cfg=stud_cfg,
            hole_cfg=object_cfg,
            stud_if=stud_if,
            hole_if=hole_if,
        ),
    )
    target_match = (
        (connection_state.offsets[:, 0] == target_offset[0])
        & (connection_state.offsets[:, 1] == target_offset[1])
        & (connection_state.yaws == target_yaw)
    )
    return connection_state.connected & ~target_match


def assemble_brick_goal_satisfied(
    env: ManagerBasedRLEnv,
    stud_if: int,
    hole_if: int,
    target_offset: tuple[int, int],
    target_yaw: int,
    object_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    stud_cfg: SceneEntityCfg = SceneEntityCfg("lego_baseplate"),
    action_term_name: str = "gripper_action",
    open_position_threshold: float = 0.005,
    pos_tol: float = 0.002,
    rot_tol: float = 0.08726646259971647,
) -> torch.Tensor:
    """Return whether the target assembly goal is satisfied.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    target_match = connection_target_match(
        env,
        stud_if=stud_if,
        hole_if=hole_if,
        target_offset=target_offset,
        target_yaw=target_yaw,
        object_cfg=object_cfg,
        stud_cfg=stud_cfg,
    )
    pos_delta, _, rot_error = object_marker_pose_alignment(env, object_cfg, target_cfg)
    pose_close = torch.linalg.vector_norm(pos_delta, dim=1) < pos_tol
    rot_close = rot_error < rot_tol
    gripper_open = gripper_is_open(
        env,
        action_term_name=action_term_name,
        open_position_threshold=open_position_threshold,
    )
    return target_match & pose_close & rot_close & gripper_open
