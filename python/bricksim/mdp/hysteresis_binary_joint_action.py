"""Binary joint action terms with hysteresis."""

from collections.abc import Sequence

import torch
from isaaclab.envs import ManagerBasedEnv
from isaaclab.envs.mdp.actions.actions_cfg import BinaryJointPositionActionCfg
from isaaclab.envs.mdp.actions.binary_joint_actions import BinaryJointPositionAction
from isaaclab.managers import ActionTerm
from isaaclab.utils import configclass

from .utils import MISSING


class HysteresisBinaryJointAction(BinaryJointPositionAction):
    """Binary joint position action with open/close hysteresis."""

    cfg: "HysteresisBinaryJointActionCfg"

    def __init__(
        self,
        cfg: "HysteresisBinaryJointActionCfg",
        env: ManagerBasedEnv,
    ) -> None:
        """Initialize the action term and action latch.

        Args:
            cfg: Binary joint position action configuration.
            env: Manager-based environment that owns this action term.
        """
        super().__init__(cfg, env)
        self._processed_actions[:] = self.initial_command

    @property
    def initial_command(self) -> torch.Tensor:
        """Initial command based on the configuration's initial state."""
        return self._close_command if self.cfg.initial_closed else self._open_command

    def process_actions(self, actions: torch.Tensor) -> None:
        """Map raw joint commands to open/close joint targets with hysteresis.

        Args:
            actions: Raw joint action tensor with shape ``(num_envs, 1)``.
        """
        self._raw_actions[:] = actions
        if actions.dtype == torch.bool:
            self._processed_actions = torch.where(
                actions == 0, self._close_command, self._open_command
            )
        else:
            self._processed_actions[actions[:, 0] < self.cfg.close_thresholds] = (
                self._close_command
            )
            self._processed_actions[actions[:, 0] > self.cfg.open_thresholds] = (
                self._open_command
            )

        if self.cfg.clip is not None:
            self._processed_actions = torch.clamp(
                self._processed_actions,
                min=self._clip[:, :, 0],
                max=self._clip[:, :, 1],
            )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        """Reset actions to the initial command for the specified environment slots.

        Args:
            env_ids: Environment slots to reset. If ``None``, every slot is reset.
        """
        super().reset(env_ids)
        self._processed_actions[env_ids] = self.initial_command


@configclass
class HysteresisBinaryJointActionCfg(BinaryJointPositionActionCfg):
    """Configuration for the binary joint action with hysteresis."""

    class_type: type[ActionTerm] = HysteresisBinaryJointAction

    open_thresholds: float = MISSING
    """Threshold for entering the open state."""

    close_thresholds: float = MISSING
    """Threshold for entering the closed state."""

    initial_closed: bool = MISSING
    """Initial joint state, where ``True`` means closed and ``False`` means open."""
