"""Typed shims for Isaac Lab APIs used by BrickSim MDP terms."""

from collections.abc import Sequence

import torch
from isaaclab.assets import Articulation, DeformableObject, RigidObject

IsaacLabEnvIds = torch.Tensor | Sequence[int] | None


def write_asset_root_pose_to_sim(
    asset: Articulation | RigidObject,
    root_pose: torch.Tensor,
    env_ids: IsaacLabEnvIds,
) -> None:
    """Write asset root poses while preserving tensor environment indices."""
    # Isaac Lab 2.3.2 annotates env_ids as Sequence[int] | None, but its
    # implementation accepts torch.Tensor and uses it for tensor indexing.
    asset.write_root_pose_to_sim(root_pose, env_ids=env_ids)  # ty: ignore[invalid-argument-type]


def write_asset_root_velocity_to_sim(
    asset: Articulation | RigidObject,
    root_velocity: torch.Tensor,
    env_ids: IsaacLabEnvIds,
) -> None:
    """Write asset root velocities while preserving tensor environment indices."""
    # See write_asset_root_pose_to_sim.
    asset.write_root_velocity_to_sim(root_velocity, env_ids=env_ids)  # ty: ignore[invalid-argument-type]


def write_articulation_joint_state_to_sim(
    asset: Articulation,
    joint_pos: torch.Tensor,
    joint_vel: torch.Tensor,
    env_ids: IsaacLabEnvIds,
) -> None:
    """Write articulation joint state while preserving tensor environment indices."""
    # See write_asset_root_pose_to_sim.
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)  # ty: ignore[invalid-argument-type]


def set_articulation_joint_position_target(
    asset: Articulation,
    joint_pos: torch.Tensor,
    env_ids: IsaacLabEnvIds,
) -> None:
    """Set articulation joint-position targets with tensor environment indices."""
    # See write_asset_root_pose_to_sim.
    asset.set_joint_position_target(joint_pos, env_ids=env_ids)  # ty: ignore[invalid-argument-type]


def set_articulation_joint_velocity_target(
    asset: Articulation,
    joint_vel: torch.Tensor,
    env_ids: IsaacLabEnvIds,
) -> None:
    """Set articulation joint-velocity targets with tensor environment indices."""
    # See write_asset_root_pose_to_sim.
    asset.set_joint_velocity_target(joint_vel, env_ids=env_ids)  # ty: ignore[invalid-argument-type]


def write_deformable_nodal_state_to_sim(
    asset: DeformableObject,
    nodal_state: torch.Tensor,
    env_ids: IsaacLabEnvIds,
) -> None:
    """Write deformable nodal state while preserving tensor environment indices."""
    # See write_asset_root_pose_to_sim.
    asset.write_nodal_state_to_sim(nodal_state, env_ids=env_ids)  # ty: ignore[invalid-argument-type]
