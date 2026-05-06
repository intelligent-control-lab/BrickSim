"""Shared MDP helpers for assemble-brick observations, rewards, and terms."""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.envs.mdp.actions.binary_joint_actions import BinaryJointPositionAction


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
