"""Command terms for brick assembly goals."""

from collections.abc import Sequence
from typing import Literal

import torch
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import (
    combine_frame_transforms,
    subtract_frame_transforms,
)

from bricksim.mdp.brick_part import scene_entity_brick_part_dimensions
from bricksim.mdp.connection_state import (
    InterfacePairConnectionQuery,
    InterfacePairConnectionState,
    interface_pair_connection_state,
)
from bricksim.mdp.utils import MISSING, enumerate_env_ids
from bricksim.utils.connection_geometry import (
    connection_brick_transforms,
    enumerate_all_possible_connections,
)


class _GoalLayout:
    """Column layout of the goal tensor."""

    OFFSET = slice(0, 2)
    YAW = 2
    TARGET_POS = slice(3, 6)
    TARGET_QUAT = slice(6, 10)
    DIM = 10


class AssembleBrickCommand(CommandTerm):
    """Command term that samples brick assembly goals on reset."""

    cfg: "AssembleBrickCommandCfg"

    def __init__(
        self,
        cfg: "AssembleBrickCommandCfg",
        env: ManagerBasedRLEnv,
    ) -> None:
        """Initialize the command with resolved goal sampling buffer.

        Args:
            cfg: Command configuration with reference and placed brick names.
            env: Manager-based RL environment that provides scene tensors.
        """
        super().__init__(cfg, env)
        self._command = torch.zeros((env.num_envs, _GoalLayout.DIM), device=env.device)
        self.stud_dims = scene_entity_brick_part_dimensions(env, cfg.stud_brick)
        self.hole_dims = scene_entity_brick_part_dimensions(env, cfg.hole_brick)
        goal_table = torch.as_tensor(cfg.goals, device=env.device, dtype=torch.long)
        if goal_table.numel() > 0:
            if goal_table.ndim != 2 or goal_table.shape[1] != 3:
                raise ValueError("goals must have shape (num_goals, 3)")
            self._goal_table = goal_table
        else:
            self._goal_table = enumerate_all_possible_connections(
                stud_dimensions=self.stud_dims[:2],
                hole_dimensions=self.hole_dims[:2],
                device=env.device,
            )

    def __str__(self) -> str:
        """Return a short command-term summary.

        Returns:
            Human-readable command summary string.
        """
        msg = "AssembleBrickCommand:\n"
        msg += f"\tStud: {self.cfg.stud_brick} #{self.cfg.stud_brick_iface}\n"
        msg += f"\tHole: {self.cfg.hole_brick} #{self.cfg.hole_brick_iface}\n"
        msg += f"\tMoving: {self.cfg.moving_brick_type}\n"
        msg += f"\tGoal count: {self._goal_table.shape[0]}\n"
        return msg

    @property
    def command(self) -> torch.Tensor:
        """Return the sampled command tensor.

        Returns:
            Tensor with shape ``(num_envs, 10)``. Columns are grid offset
            ``(x, y)``, C4 yaw, world target position in meters, and WXYZ
            target quaternion for the moving brick.
        """
        return self._command

    @property
    def goal_table(self) -> torch.Tensor:
        """Return the resolved discrete goal table.

        Returns:
            Long tensor with shape ``(num_rows, 3)``. Each row is
            ``(offset_x, offset_y, yaw)`` in grid units and C4 yaw.
        """
        return self._goal_table

    @property
    def query(self) -> InterfacePairConnectionQuery:
        """Return the configured stud/hole interface-pair query.

        Returns:
            Query describing the command's configured stud/hole interface pair.
        """
        return InterfacePairConnectionQuery(
            stud_name=self.cfg.stud_brick,
            hole_name=self.cfg.hole_brick,
            stud_if=self.cfg.stud_brick_iface,
            hole_if=self.cfg.hole_brick_iface,
        )

    def compute(self, dt: float) -> None:
        """Refresh target poses without timer-based resampling.

        Args:
            dt: Elapsed simulation time in seconds; ignored because rows
                resample only on reset.
        """
        del dt
        self._update_metrics()
        self._update_command()

    def _update_metrics(self) -> None:
        """Leave command metrics empty for this command term."""
        pass

    def _resample_command(
        self, env_ids: torch.Tensor | Sequence[int] | slice | None
    ) -> None:
        """Sample goal rows for reset environments.

        Args:
            env_ids: Environment ids whose command rows should change. ``None``
                selects all environments.
        """
        env_ids = enumerate_env_ids(self.num_envs, env_ids, device=self.device)
        if env_ids.numel() == 0:
            return

        sampled_ids = torch.randint(
            low=0,
            high=self._goal_table.shape[0],
            size=(env_ids.numel(),),
            device=self.device,
        )
        sampled = self._goal_table[sampled_ids]
        self._command[env_ids, _GoalLayout.OFFSET] = sampled[:, :2].to(
            self._command.dtype
        )
        self._command[env_ids, _GoalLayout.YAW] = sampled[:, 2].to(self._command.dtype)
        self._update_command(env_ids)

    def _update_command(
        self, env_ids: Sequence[int] | torch.Tensor | slice | None = None
    ) -> None:
        """Refresh target poses from the current fixed-side brick poses."""
        env_ids = enumerate_env_ids(self.num_envs, env_ids, device=self.device)
        if env_ids.numel() == 0:
            return

        reference_brick: RigidObject
        if self.cfg.moving_brick_type == "hole":
            reference_brick = self._env.scene[self.cfg.stud_brick]
        elif self.cfg.moving_brick_type == "stud":
            reference_brick = self._env.scene[self.cfg.hole_brick]
        else:
            raise ValueError(f"invalid moving_brick_type: {self.cfg.moving_brick_type}")
        reference_pos_w: torch.Tensor = reference_brick.data.root_pos_w[env_ids, :3]
        reference_quat_w: torch.Tensor = reference_brick.data.root_quat_w[env_ids]

        dtype = reference_pos_w.dtype
        rel_pos, rel_quat = connection_brick_transforms(
            stud_dimensions=self.stud_dims,
            hole_dimensions=self.hole_dims,
            offsets=self._command[env_ids, _GoalLayout.OFFSET],
            yaws=self._command[env_ids, _GoalLayout.YAW],
            dtype=dtype,
        )
        if self.cfg.moving_brick_type == "stud":
            rel_pos, rel_quat = subtract_frame_transforms(rel_pos, rel_quat)

        target_pos_w, target_quat_w = combine_frame_transforms(
            reference_pos_w, reference_quat_w, rel_pos, rel_quat
        )
        self._command[env_ids, _GoalLayout.TARGET_POS] = target_pos_w.to(
            self._command.dtype
        )
        self._command[env_ids, _GoalLayout.TARGET_QUAT] = target_quat_w.to(
            self._command.dtype
        )


@configclass
class AssembleBrickCommandCfg(CommandTermCfg):
    """Configuration for AssembleBrickCommand.

    Attributes:
        class_type: AssembleBrickCommand class.
        stud_brick: Asset name for the stud brick.
        stud_brick_iface: Interface id for the stud brick.
        hole_brick: Asset name for the hole brick.
        hole_brick_iface: Interface id for the hole brick.
        moving_brick_type: Is the moving brick (whose pose is generated) the hole or
            the stud brick?
        goals: Goal candidates (offset_x, offset_y, yaw_index) in shape (num_goals, 3).
            empty enumerates all possible connections.
        resampling_time_range: Unused.
    """

    class_type: type[CommandTerm] = AssembleBrickCommand
    stud_brick: str = MISSING
    stud_brick_iface: int = 1
    hole_brick: str = MISSING
    hole_brick_iface: int = 0
    moving_brick_type: Literal["hole", "stud"] = "hole"
    goals: torch.Tensor | Sequence[tuple[int, int, int]] = ()
    resampling_time_range: tuple[float, float] = (0.0, 0.0)


def assembly_goal_offsets(
    env: ManagerBasedRLEnv,
    command_name: str,
) -> torch.Tensor:
    """Return the goal offsets of the command.

    Args:
        env: Environment with a command manager.
        command_name: Name of the AssembleBrickCommand command term.

    Returns:
        Long ``(num_envs, 2)`` tensor of ``(x, y)`` offsets in grid units.
    """
    command = env.command_manager.get_command(command_name)
    return command[:, _GoalLayout.OFFSET].to(torch.long)


def assembly_goal_yaws(
    env: ManagerBasedRLEnv,
    command_name: str,
) -> torch.Tensor:
    """Return the goal C4 yaws of the command.

    Args:
        env: Environment with a command manager.
        command_name: Name of the AssembleBrickCommand command term.

    Returns:
        Long ``(num_envs,)`` tensor of quarter-turn yaw indices.
    """
    command = env.command_manager.get_command(command_name)
    return command[:, _GoalLayout.YAW].to(torch.long)


def assembly_goal_target_pose(
    env: ManagerBasedRLEnv,
    command_name: str,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Return the goal target poses of the command.

    Args:
        env: Environment with a command manager.
        command_name: Name of the AssembleBrickCommand command term.

    Returns:
        Target positions with shape ``(num_envs, 3)`` in meters and target
        quaternions with shape ``(num_envs, 4)`` in WXYZ order.
    """
    command = env.command_manager.get_command(command_name)
    return (
        command[:, _GoalLayout.TARGET_POS],
        command[:, _GoalLayout.TARGET_QUAT],
    )


def assembly_moving_brick_name(
    env: ManagerBasedRLEnv,
    command_name: str,
) -> str:
    """Return the scene name of the command's moving brick.

    Args:
        env: Environment with a command manager.
        command_name: Name of the AssembleBrickCommand command term.

    Returns:
        Scene entity name for the brick whose target pose is generated by the
        command.
    """
    command_term = env.command_manager.get_term(command_name)
    if not isinstance(command_term, AssembleBrickCommand):
        raise TypeError(f"command term {command_name} is not an AssembleBrickCommand")
    if command_term.cfg.moving_brick_type == "hole":
        return command_term.cfg.hole_brick
    if command_term.cfg.moving_brick_type == "stud":
        return command_term.cfg.stud_brick
    raise ValueError(f"invalid moving_brick_type: {command_term.cfg.moving_brick_type}")


def assembly_query_connection_state(
    env: ManagerBasedRLEnv,
    command_name: str,
) -> InterfacePairConnectionState:
    """Return the current connection state for the interface pair in the command.

    Args:
        env: Environment with a command manager.
        command_name: Name of the AssembleBrickCommand command term.

    Returns:
        Batched current connection state for the command's configured
        stud/hole interface pair.
    """
    command_term = env.command_manager.get_term(command_name)
    if not isinstance(command_term, AssembleBrickCommand):
        raise TypeError(f"command term {command_name} is not an AssembleBrickCommand")
    return interface_pair_connection_state(env, command_term.query)


def assembly_check_connection_formed(
    env: ManagerBasedRLEnv,
    command_name: str,
    invert: bool = False,
) -> torch.Tensor:
    """Check if the connections between the command's interface pair match the goal.

    Args:
        env: Environment with a command manager.
        command_name: Name of the AssembleBrickCommand command term.
        invert: If ``True``, check for connections that are formed but do not match the
            goal.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    connection_state = assembly_query_connection_state(env, command_name)
    target_offsets = assembly_goal_offsets(env, command_name)
    target_yaws = assembly_goal_yaws(env, command_name)
    target_match = (
        (connection_state.offsets[:, 0] == target_offsets[:, 0])
        & (connection_state.offsets[:, 1] == target_offsets[:, 1])
        & (connection_state.yaws == target_yaws)
    )
    if invert:
        target_match = ~target_match
    return connection_state.connected & target_match
