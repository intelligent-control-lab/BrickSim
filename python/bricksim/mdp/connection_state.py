"""Current connection-state queries for BrickSim MDP terms."""

from dataclasses import dataclass, field
from typing import cast

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg

from bricksim.core import lookup_physics_connection

from .cache_token import ResetAwareCacheToken
from .utils import resolve_brick_rigid_object


@dataclass(frozen=True, slots=True)
class InterfacePairConnectionQuery:
    """Identity of one stud/hole interface-pair query.

    This object describes *what* is being queried, not the per-environment
    resolved USD prims used to answer the query.

    Attributes:
        stud_name: Name of the stud-side scene entity in ``env.scene``.
            This is the Isaac Lab scene key for a runtime asset accepted by
            :func:`bricksim.mdp.utils.resolve_brick_rigid_object`, such as
            ``"lego_baseplate"``, not a USD prim path.
        hole_name: Name of the hole-side scene entity in ``env.scene``.
            This is the Isaac Lab scene key for a runtime asset accepted by
            :func:`bricksim.mdp.utils.resolve_brick_rigid_object`, such as
            ``"lego_brick"``, not a USD prim path.
        stud_if: BrickSim stud interface index on the scene entity selected
            by ``stud_name``.
        hole_if: BrickSim hole interface index on the scene entity selected
            by ``hole_name``.

    At query time, ``stud_name`` and ``hole_name`` are resolved through
    ``env.scene[...]`` and must resolve to runtime brick assets accepted by
    :func:`bricksim.mdp.utils.resolve_brick_rigid_object`. Those assets are
    then mapped to per-environment USD prim paths via their PhysX views. The
    resolved prim paths are not stored in this dataclass because they vary
    across environments.
    """

    stud_name: str
    hole_name: str
    stud_if: int
    hole_if: int

    @classmethod
    def make(
        cls,
        stud_cfg: SceneEntityCfg,
        hole_cfg: SceneEntityCfg,
        stud_if: int,
        hole_if: int,
    ) -> "InterfacePairConnectionQuery":
        """Construct a query from Isaac Lab scene-entity configs.

        Returns:
            Query keyed by scene entity names and interface ids.
        """
        return cls(
            stud_name=stud_cfg.name,
            hole_name=hole_cfg.name,
            stud_if=stud_if,
            hole_if=hole_if,
        )


@dataclass(slots=True)
class InterfacePairConnectionState:
    """Current physics state for one fixed brick-interface query.

    This object represents the result of querying a single interface pair
    ``query`` against the current physics state of a vectorized environment.
    The state is batched across all environments.

    The query endpoints are scene entities that must resolve to BrickSim brick
    rigid objects, i.e. runtime assets accepted by
    :func:`bricksim.mdp.utils.resolve_brick_rigid_object`. Non-physical marker
    bricks and other asset kinds are not supported by this API.

    All tensors are indexed by environment id along dimension 0, so entry
    ``i`` corresponds to environment ``i``. The leading dimension is always
    ``env.num_envs``.

    Attributes:
        query: The fixed stud/hole query that produced this state object.
        connected: Boolean tensor of shape ``(num_envs,)``. ``True`` when the
            queried stud/hole interface pair is currently connected in that
            environment.
        offsets: Integer tensor of shape ``(num_envs, 2)``. When
            ``connected[i]`` is true, ``offsets[i]`` is the BrickSim grid
            offset ``(dx, dy)`` of that connection. When ``connected[i]`` is
            false, the value is unspecified and is currently zero-filled.
        yaws: Integer tensor of shape ``(num_envs,)``. When ``connected[i]``
            is true, ``yaws[i]`` is the BrickSim discrete yaw index for that
            connection. When ``connected[i]`` is false, the value is ``-1``.

    Since this query is for one fixed interface pair, each environment
    contributes at most one connection state to the result.
    """

    query: InterfacePairConnectionQuery
    connected: torch.Tensor
    offsets: torch.Tensor
    yaws: torch.Tensor


@dataclass(slots=True)
class _ConnectionStateCache:
    """Per-environment cache for interface-pair connection-state queries."""

    token: ResetAwareCacheToken | None = None
    by_query: dict[InterfacePairConnectionQuery, InterfacePairConnectionState] = field(
        default_factory=dict
    )


def _connection_state_cache(env: ManagerBasedRLEnv) -> _ConnectionStateCache:
    cache = getattr(env, "_bricksim_connection_state_cache", None)
    if cache is None:
        cache = _ConnectionStateCache()
        setattr(env, "_bricksim_connection_state_cache", cache)
    return cast(_ConnectionStateCache, cache)


def interface_pair_connection_state(
    env: ManagerBasedRLEnv,
    query: InterfacePairConnectionQuery,
) -> InterfacePairConnectionState:
    """Query and cache current physics state for one brick interface pair.

    The returned state is batched over all vectorized environments. For a
    given query, the result is cached until either:

    1. ``env.common_step_counter`` changes, or
    2. Isaac Lab resets any sub-environment within the same step.

    The second condition is detected through a reset-sensitive cache token
    based on a private reset-generation counter. The counter is maintained by
    lazily instrumenting ``env._reset_idx`` through
    :class:`ResetAwareCacheToken`. This is slightly awkward, but it avoids
    requiring every task env to wire explicit cache invalidation events.

    Returns:
        Current connection state for the queried interface pair.
    """
    cache = _connection_state_cache(env)
    if cache.token is None or not cache.token.matches_env(env):
        cache.token = ResetAwareCacheToken.from_env(env)
        cache.by_query.clear()

    if query in cache.by_query:
        return cache.by_query[query]

    stud_asset = resolve_brick_rigid_object(env, query.stud_name)
    hole_asset = resolve_brick_rigid_object(env, query.hole_name)
    stud_paths = stud_asset.root_physx_view.prim_paths
    hole_paths = hole_asset.root_physx_view.prim_paths
    connected = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    offsets = torch.zeros((env.num_envs, 2), device=env.device, dtype=torch.long)
    yaws = torch.full((env.num_envs,), -1, device=env.device, dtype=torch.long)

    for env_id, (stud_path, hole_path) in enumerate(
        zip(stud_paths, hole_paths, strict=True)
    ):
        info = lookup_physics_connection(
            stud_path=stud_path,
            stud_if=query.stud_if,
            hole_path=hole_path,
            hole_if=query.hole_if,
        )
        if info is None:
            continue
        connected[env_id] = True
        offsets[env_id] = torch.tensor(info.offset, device=env.device, dtype=torch.long)
        yaws[env_id] = info.yaw

    state = InterfacePairConnectionState(
        query=query, connected=connected, offsets=offsets, yaws=yaws
    )
    cache.by_query[query] = state
    return state
