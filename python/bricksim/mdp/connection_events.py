"""Connection-event stream projections for BrickSim MDP terms."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol, TypeGuard

import torch
from isaaclab.envs import ManagerBasedRLEnv

from bricksim.core import ConnectionInfo
from bricksim.core import get_assembled_connections as _native_get_assembled_connections
from bricksim.core import (
    get_disassembled_connections as _native_get_disassembled_connections,
)

from .brick_part import resolve_brick_rigid_object
from .cache_token import ResetAwareCacheToken
from .connection_state import InterfacePairConnectionQuery


class ConnectionEventStream(Enum):
    """Native connection-event stream selector."""

    ASSEMBLED = "assembled"
    DISASSEMBLED = "disassembled"


@dataclass(slots=True)
class InterfacePairConnectionEvents:
    """Matching connection events for one fixed brick-interface query.

    This object summarizes one event stream in the current step, either
    assembled or disassembled, after filtering it to a single fixed
    interface-pair query.

    The query endpoints are scene entities that must resolve to BrickSim brick
    rigid objects, i.e. runtime assets accepted by
    :func:`bricksim.mdp.brick_part.resolve_brick_rigid_object`. Non-physical marker
    bricks and other asset kinds are not supported by this API.

    All tensors are indexed by environment id along dimension 0, so entry
    ``i`` corresponds to environment ``i``. The leading dimension is always
    ``env.num_envs``.

    Attributes:
        query: The fixed stud/hole query that produced this event summary.
        stream: Which raw event stream was summarized.
        occurred: Boolean tensor of shape ``(num_envs,)``. ``True`` when at
            least one matching event occurred in that environment during the
            current simulation step.
        counts: Integer tensor of shape ``(num_envs,)`` containing the number
            of matching events observed in each environment during the current
            step.
        offsets: Integer tensor of shape ``(num_envs, 2)``. When
            ``occurred[i]`` is true, ``offsets[i]`` stores the BrickSim grid
            offset ``(dx, dy)`` from the last matching event seen for
            environment ``i`` in the current step. When ``occurred[i]`` is
            false, the value is unspecified and is currently zero-filled.
        yaws: Integer tensor of shape ``(num_envs,)``. When ``occurred[i]`` is
            true, ``yaws[i]`` stores the BrickSim discrete yaw index from the
            last matching event seen for environment ``i`` in the current
            step. When ``occurred[i]`` is false, the value is ``-1``.

    The payload tensors summarize the last matching event per environment.
    Use ``counts`` to detect whether that summary was unique.
    """

    query: InterfacePairConnectionQuery
    stream: ConnectionEventStream
    occurred: torch.Tensor
    counts: torch.Tensor
    offsets: torch.Tensor
    yaws: torch.Tensor


@dataclass(slots=True)
class _ProjectedConnectionEventsCache:
    token: ResetAwareCacheToken | None = None
    raw_events: list[ConnectionInfo] = field(default_factory=list)
    by_query: dict[InterfacePairConnectionQuery, InterfacePairConnectionEvents] = field(
        default_factory=dict
    )


@dataclass(slots=True)
class _ConnectionEventsCache:
    assembled: _ProjectedConnectionEventsCache = field(
        default_factory=_ProjectedConnectionEventsCache
    )
    disassembled: _ProjectedConnectionEventsCache = field(
        default_factory=_ProjectedConnectionEventsCache
    )


class _ConnectionEventsCachedEnv(Protocol):
    """Isaac Lab env with BrickSim connection-events cache attached."""

    _bricksim_connection_events_cache: _ConnectionEventsCache


def _has_connection_events_cache(
    env: ManagerBasedRLEnv,
) -> TypeGuard[_ConnectionEventsCachedEnv]:
    return hasattr(env, "_bricksim_connection_events_cache")


def _connection_events_cache(env: ManagerBasedRLEnv) -> _ConnectionEventsCache:
    if _has_connection_events_cache(env):
        return env._bricksim_connection_events_cache
    cache = _ConnectionEventsCache()
    setattr(env, "_bricksim_connection_events_cache", cache)
    return cache


def _stream_cache(
    env: ManagerBasedRLEnv, stream: ConnectionEventStream
) -> _ProjectedConnectionEventsCache:
    cache = _connection_events_cache(env)
    if stream is ConnectionEventStream.ASSEMBLED:
        return cache.assembled
    return cache.disassembled


def _read_native_stream(stream: ConnectionEventStream) -> list[ConnectionInfo]:
    """Read and clear one native connection-event stream for the current step.

    Returns:
        Native connection events from the selected stream.
    """
    if stream is ConnectionEventStream.ASSEMBLED:
        return _native_get_assembled_connections(clear=True)
    return _native_get_disassembled_connections(clear=True)


def _raw_stream_events(
    env: ManagerBasedRLEnv, stream: ConnectionEventStream
) -> list[ConnectionInfo]:
    """Return the cached raw event stream for the current env state.

    The stream is refreshed when the reset-aware cache token changes. If the
    token change came from a same-step reset, the previously read raw stream is
    kept and only the projected per-query cache is invalidated, so non-reset
    envs do not lose valid same-step events.

    Returns:
        Cached raw events for the selected stream.
    """
    stream_cache = _stream_cache(env, stream)
    if stream_cache.token is not None and stream_cache.token.matches_env(env):
        return stream_cache.raw_events

    same_step_reset = (
        stream_cache.token is not None
        and stream_cache.token.invalidated_by_same_step_reset(env)
    )
    if not same_step_reset:
        stream_cache.raw_events = _read_native_stream(stream)

    stream_cache.token = ResetAwareCacheToken.from_env(env)
    stream_cache.by_query.clear()
    return stream_cache.raw_events


def _interface_pair_events_from_stream(
    env: ManagerBasedRLEnv,
    stream: ConnectionEventStream,
    query: InterfacePairConnectionQuery,
) -> InterfacePairConnectionEvents:
    """Project one raw event stream into an env-major tensor summary.

    This function intentionally ignores environments whose
    ``env.episode_length_buf`` is currently zero. In Isaac Lab, that means the
    sub-environment was reset in the current step. For those environments, the
    raw native event stream is ambiguous because it may contain pre-reset
    events and/or reset-generated artifacts from scene mutation.

    We keep the raw step-level stream intact so that valid same-step events for
    non-reset envs are preserved. The MDP-facing summary then masks reset envs
    so downstream reward/observation logic sees only trustworthy per-env
    events.

    Returns:
        Env-major summary for the requested interface-pair event stream.
    """
    raw_events = _raw_stream_events(env, stream)
    stream_cache = _stream_cache(env, stream)
    if query in stream_cache.by_query:
        return stream_cache.by_query[query]

    stud_asset = resolve_brick_rigid_object(env, query.stud_name)
    hole_asset = resolve_brick_rigid_object(env, query.hole_name)
    stud_paths = stud_asset.root_physx_view.prim_paths
    hole_paths = hole_asset.root_physx_view.prim_paths

    occurred = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    counts = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)
    offsets = torch.zeros((env.num_envs, 2), device=env.device, dtype=torch.long)
    yaws = torch.full((env.num_envs,), -1, device=env.device, dtype=torch.long)
    reset_env_mask = env.episode_length_buf == 0

    for info in raw_events:
        env_id = info.env_id
        if not (0 <= env_id < env.num_envs):
            continue
        if reset_env_mask[env_id]:
            continue
        if info.stud_ifid != query.stud_if or info.hole_ifid != query.hole_if:
            continue
        if info.stud_path != stud_paths[env_id] or info.hole_path != hole_paths[env_id]:
            continue
        occurred[env_id] = True
        counts[env_id] += 1
        offsets[env_id] = torch.tensor(info.offset, device=env.device, dtype=torch.long)
        yaws[env_id] = info.yaw

    result = InterfacePairConnectionEvents(
        query=query,
        stream=stream,
        occurred=occurred,
        counts=counts,
        offsets=offsets,
        yaws=yaws,
    )
    stream_cache.by_query[query] = result
    return result


def get_assembled_connections(env: ManagerBasedRLEnv) -> list[ConnectionInfo]:
    """Return the raw assembled-connection event list for the current step.

    The native event list is cached on the environment and refreshed at most
    once per step/reset-sensitive cache token.

    This is the low-level raw stream API. If Isaac Lab resets some
    sub-environments within the current step, this raw list may still contain
    events for those reset envs. The higher-level projected MDP API masks those
    envs explicitly instead of discarding the whole raw stream.

    Returns:
        Cached raw assembled-connection events.
    """
    return _raw_stream_events(env, ConnectionEventStream.ASSEMBLED)


def get_disassembled_connections(env: ManagerBasedRLEnv) -> list[ConnectionInfo]:
    """Return the raw disassembled-connection event list for the current step.

    The native event list is cached on the environment and refreshed at most
    once per step/reset-sensitive cache token.

    This is the low-level raw stream API. If Isaac Lab resets some
    sub-environments within the current step, this raw list may still contain
    events for those reset envs. The higher-level projected MDP API masks those
    envs explicitly instead of discarding the whole raw stream.

    Returns:
        Cached raw disassembled-connection events.
    """
    return _raw_stream_events(env, ConnectionEventStream.DISASSEMBLED)


def interface_pair_assembled_events(
    env: ManagerBasedRLEnv,
    query: InterfacePairConnectionQuery,
) -> InterfacePairConnectionEvents:
    """Return assembled events for one brick interface-pair query in the current step.

    The query is expressed in terms of scene-entity names, not USD prim paths.
    The named scene entities must resolve to runtime brick assets accepted by
    :func:`bricksim.mdp.brick_part.resolve_brick_rigid_object`. Per-environment
    prim paths are resolved internally from those assets when evaluating the
    query.

    Cache freshness is reset-sensitive. If Isaac Lab resets any sub-environment
    within the current step, this function masks those reset envs while still
    preserving valid same-step events for non-reset envs.

    Returns:
        Env-major assembled-event summary for the query.
    """
    return _interface_pair_events_from_stream(
        env,
        ConnectionEventStream.ASSEMBLED,
        query,
    )


def interface_pair_disassembled_events(
    env: ManagerBasedRLEnv,
    query: InterfacePairConnectionQuery,
) -> InterfacePairConnectionEvents:
    """Return disassembled events for one brick interface-pair query.

    This is scoped to the current step. The query is expressed in terms of
    scene-entity names, not USD prim paths. The named scene entities must
    resolve to runtime brick assets accepted by
    :func:`bricksim.mdp.brick_part.resolve_brick_rigid_object`. Per-environment prim
    paths are resolved internally from those assets when evaluating the query.

    Cache freshness is reset-sensitive. If Isaac Lab resets any sub-environment
    within the current step, this function masks those reset envs while still
    preserving valid same-step events for non-reset envs.

    Returns:
        Env-major disassembled-event summary for the query.
    """
    return _interface_pair_events_from_stream(
        env,
        ConnectionEventStream.DISASSEMBLED,
        query,
    )
