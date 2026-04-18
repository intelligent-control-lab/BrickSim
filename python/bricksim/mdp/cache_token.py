"""Reset-aware cache tokens for BrickSim MDP helpers."""

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol, TypeGuard

from isaaclab.envs import ManagerBasedRLEnv

_ResetIdx = Callable[[Sequence[int]], None]


class _ResetGenerationTrackedEnv(Protocol):
    """Isaac Lab env after BrickSim reset-generation tracking is installed."""

    _bricksim_original_reset_idx: _ResetIdx
    _bricksim_reset_generation: int
    _bricksim_reset_generation_installed: bool


def _has_reset_generation_tracking(
    env: ManagerBasedRLEnv,
) -> TypeGuard[_ResetGenerationTrackedEnv]:
    return (
        hasattr(env, "_bricksim_original_reset_idx")
        and hasattr(env, "_bricksim_reset_generation")
        and hasattr(env, "_bricksim_reset_generation_installed")
    )


def _require_reset_generation_tracking(
    env: ManagerBasedRLEnv,
) -> _ResetGenerationTrackedEnv:
    if not _has_reset_generation_tracking(env):
        raise RuntimeError("BrickSim reset-generation tracking is not installed.")
    return env


@dataclass(slots=True)
class ResetAwareCacheToken:
    """Reset-sensitive freshness token for cached env-derived data.

    Isaac Lab increments ``env.common_step_counter`` only during stepping, but
    it can reset sub-environments and recompute observations within the same
    step. Therefore ``common_step_counter`` alone is not sufficient to decide
    whether a cached env-derived snapshot is still valid after reset.

    To make cache invalidation seamless, we lazily instrument ``env._reset_idx``
    and maintain a private ``env._bricksim_reset_generation`` counter. The
    token then tracks the pair ``(common_step_counter, reset_generation)``.
    This catches:

    1. normal stepping, where ``common_step_counter`` changes, and
    2. any explicit or automatic reset path that goes through ``_reset_idx``,
       even if it happens within the same common step.

    This approach is a little invasive because it monkey-patches a private env
    method, but it avoids requiring every task/env to register BrickSim-
    specific reset invalidation events manually.
    """

    step: int
    reset_generation: int

    @staticmethod
    def _reset_tracked_env(env: ManagerBasedRLEnv) -> _ResetGenerationTrackedEnv:
        """Install lazy reset-generation tracking on the env if needed.

        Isaac Lab does not expose a built-in reset generation counter. To make
        cache invalidation seamless, we wrap ``env._reset_idx`` once and bump a
        private ``env._bricksim_reset_generation`` counter after every reset.

        This relies on Isaac Lab reset paths flowing through ``_reset_idx``.
        The wrapper is installed per env instance and is never stacked twice.

        Returns:
            Env narrowed to the BrickSim reset-generation fields.
        """
        if _has_reset_generation_tracking(env):
            return env

        original_reset_idx: _ResetIdx = env._reset_idx

        def wrapped_reset_idx(env_ids: Sequence[int]) -> None:
            # Increment after delegating so the generation reflects the newly
            # reset scene state that subsequent queries will observe.
            original_reset_idx(env_ids)
            tracked_env = _require_reset_generation_tracking(env)
            tracked_env._bricksim_reset_generation += 1

        setattr(env, "_bricksim_original_reset_idx", original_reset_idx)
        setattr(env, "_bricksim_reset_generation", 0)
        env._reset_idx: _ResetIdx = wrapped_reset_idx
        setattr(env, "_bricksim_reset_generation_installed", True)
        return _require_reset_generation_tracking(env)

    @classmethod
    def from_env(cls, env: ManagerBasedRLEnv) -> "ResetAwareCacheToken":
        """Create a reset-sensitive freshness token for the current env state.

        Returns:
            Token matching the environment's current step/reset generation.
        """
        tracked_env = cls._reset_tracked_env(env)
        return cls(
            step=env.common_step_counter,
            reset_generation=tracked_env._bricksim_reset_generation,
        )

    def matches_env(self, env: ManagerBasedRLEnv) -> bool:
        """Return whether this token still matches the current env state.

        Returns:
            ``True`` when the environment has not stepped or reset since the
            token was created.
        """
        tracked_env = self._reset_tracked_env(env)
        return (
            self.step == env.common_step_counter
            and self.reset_generation == tracked_env._bricksim_reset_generation
        )

    def invalidated_by_same_step_reset(self, env: ManagerBasedRLEnv) -> bool:
        """Return whether this token was invalidated by a reset within the same step.

        This is the awkward case for event streams: pre-reset events and
        reset-time native artifacts would otherwise leak into post-reset queries
        without a new ``common_step_counter`` value.

        Returns:
            ``True`` when only the reset generation changed.
        """
        tracked_env = self._reset_tracked_env(env)
        return (
            self.step == env.common_step_counter
            and self.reset_generation != tracked_env._bricksim_reset_generation
        )
