"""Utility helpers shared by BrickSim MDP terms."""

from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedEnv

from .spawn import BrickPartCfg


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
