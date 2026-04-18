"""Utility helpers shared by BrickSim MDP terms."""

from collections.abc import Sequence
from dataclasses import MISSING as _DATACLASS_MISSING
from typing import Never

import torch
from isaaclab.assets import Articulation, DeformableObject, RigidObject
from isaaclab.envs import ManagerBasedEnv

MISSING: Never = _DATACLASS_MISSING  # ty: ignore[invalid-assignment]
IsaacLabEnvIds = torch.Tensor | Sequence[int] | None
BrickDimensions = tuple[int, int, int]


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


def brick_dimensions_from_spawn(spawn_cfg: object) -> BrickDimensions:
    """Return BrickSim brick dimensions from a physical or marker spawn config.

    Returns:
        Brick dimensions as ``(length, width, height)`` in BrickSim units.
    """
    from .spawn import BrickPartCfg, MarkerBrickPartCfg

    if not isinstance(spawn_cfg, (BrickPartCfg, MarkerBrickPartCfg)):
        raise TypeError(f"Expected BrickSim brick spawn cfg, got {type(spawn_cfg)}")
    return spawn_cfg.dimensions


def scene_entity_brick_dimensions(
    env: ManagerBasedEnv,
    entity_name: str,
) -> BrickDimensions:
    """Return BrickSim brick dimensions from a scene entity spawn config.

    Returns:
        Brick dimensions as ``(length, width, height)`` in BrickSim units.
    """
    entity_cfg = getattr(env.scene.cfg, entity_name)
    return brick_dimensions_from_spawn(getattr(entity_cfg, "spawn"))


def resolve_brick_rigid_object(env: ManagerBasedEnv, entity_name: str) -> RigidObject:
    """Resolve a BrickSim brick scene entity to its runtime Isaac Lab asset.

    Args:
        env: The manager-based Isaac Lab environment.
        entity_name: Name of the scene entity in ``env.scene``.

    Returns:
        The resolved ``RigidObject`` instance.

    Raises:
        TypeError: If the named scene entity is not a ``RigidObject`` or was
            not spawned with ``BrickPartCfg``.

    This helper is intentionally narrow. It only supports connection-capable
    BrickSim bricks, which in the current design are represented as
    ``RigidObject`` instances spawned with ``BrickPartCfg``. Marker bricks and
    other non-physical assets are excluded.
    """
    from .spawn import BrickPartCfg

    asset = env.scene[entity_name]
    if not isinstance(asset, RigidObject):
        raise TypeError(
            f"Scene entity '{entity_name}' must resolve to RigidObject, got "
            f"{type(asset)}"
        )
    if not isinstance(asset.cfg.spawn, BrickPartCfg):
        raise TypeError(
            f"Scene entity '{entity_name}' must be spawned with BrickPartCfg, "
            f"got {type(asset.cfg.spawn)}"
        )
    return asset
