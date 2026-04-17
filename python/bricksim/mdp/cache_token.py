"""Reset-aware cache tokens for BrickSim MDP helpers."""

from dataclasses import dataclass

from isaaclab.envs import ManagerBasedRLEnv


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
    def _ensure_reset_generation_tracking(env: ManagerBasedRLEnv) -> None:
        """Install lazy reset-generation tracking on the env if needed.

        Isaac Lab does not expose a built-in reset generation counter. To make
        cache invalidation seamless, we wrap ``env._reset_idx`` once and bump a
        private ``env._bricksim_reset_generation`` counter after every reset.

        This relies on Isaac Lab reset paths flowing through ``_reset_idx``.
        The wrapper is installed per env instance and is never stacked twice.
        """
        if getattr(env, "_bricksim_reset_generation_installed", False):
            return

        env._bricksim_reset_generation = 0
        original_reset_idx = env._reset_idx

        def wrapped_reset_idx(env_ids):
            # Increment after delegating so the generation reflects the newly
            # reset scene state that subsequent queries will observe.
            result = original_reset_idx(env_ids)
            env._bricksim_reset_generation += 1
            return result

        env._bricksim_original_reset_idx = original_reset_idx
        env._reset_idx = wrapped_reset_idx
        env._bricksim_reset_generation_installed = True

    @classmethod
    def from_env(cls, env: ManagerBasedRLEnv) -> "ResetAwareCacheToken":
        """Create a reset-sensitive freshness token for the current env state.

        Returns:
            Token matching the environment's current step/reset generation.
        """
        cls._ensure_reset_generation_tracking(env)
        return cls(
            step=env.common_step_counter,
            reset_generation=env._bricksim_reset_generation,
        )

    def matches_env(self, env: ManagerBasedRLEnv) -> bool:
        """Return whether this token still matches the current env state.

        Returns:
            ``True`` when the environment has not stepped or reset since the
            token was created.
        """
        self._ensure_reset_generation_tracking(env)
        return (
            self.step == env.common_step_counter
            and self.reset_generation == env._bricksim_reset_generation
        )

    def invalidated_by_same_step_reset(self, env: ManagerBasedRLEnv) -> bool:
        """Return whether this token was invalidated by a reset within the same step.

        This is the awkward case for event streams: pre-reset events and
        reset-time native artifacts would otherwise leak into post-reset queries
        without a new ``common_step_counter`` value.

        Returns:
            ``True`` when only the reset generation changed.
        """
        self._ensure_reset_generation_tracking(env)
        return (
            self.step == env.common_step_counter
            and self.reset_generation != env._bricksim_reset_generation
        )
