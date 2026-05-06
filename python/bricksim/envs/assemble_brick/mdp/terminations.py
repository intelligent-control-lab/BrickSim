"""Termination terms for the assemble-brick MDP."""

import torch
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg

from .commands import assembly_check_connection_formed
from .common import gripper_is_open


def target_connection_formed(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
) -> torch.Tensor:
    """Check if the command connection is formed at the target placement.

    Args:
        env: Environment with a command manager.
        command_name: Name of the assemble-brick command term.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    return assembly_check_connection_formed(env, command_name)


def non_target_connection_formed(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
) -> torch.Tensor:
    """Check if the command connection is formed at a non-target placement.

    This refers to connections that match the command's interface pair but
    do not match the target placement.

    Args:
        env: Environment with a command manager.
        command_name: Name of the assemble-brick command term.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    return assembly_check_connection_formed(env, command_name, invert=True)


def target_connection_formed_and_gripper_open(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    action_term_name: str = "gripper_action",
    open_position_threshold: float = 0.005,
) -> torch.Tensor:
    """Return whether the target connection is formed and gripper is open.

    Args:
        env: Environment with a command manager.
        command_name: Name of the assemble-brick command term.
        action_term_name: Name of the binary gripper action term.
        open_position_threshold: Joint-position tolerance around the action
            term's open command.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    return target_connection_formed(env, command_name) & gripper_is_open(
        env,
        action_term_name=action_term_name,
        open_position_threshold=open_position_threshold,
    )


def brick_height_below_threshold(
    env: ManagerBasedRLEnv,
    minimum_height: float = -0.02,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("lego_brick"),
) -> torch.Tensor:
    """Check if the brick root height is below the drop threshold.

    Args:
        env: Environment with scene assets.
        minimum_height: Minimum allowed root height in world meters.
        asset_cfg: Scene entity for the brick to check.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    brick: RigidObject = env.scene[asset_cfg.name]
    return brick.data.root_pos_w[:, 2] < minimum_height
