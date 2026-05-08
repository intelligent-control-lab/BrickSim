#!/usr/bin/env python3
"""Run the assemble-brick expert policy."""

# ruff: noqa: E402

import argparse

from isaaclab.app import AppLauncher


def parse_goal_rows(goals: str) -> tuple[tuple[int, int, int], ...]:
    """Parse command goal rows from a semicolon-separated CLI string.

    Args:
        goals: String formatted as ``"x,y,yaw;..."``.

    Returns:
        Tuple of goal rows, where each row is ``(offset_x, offset_y, yaw)``.

    Raises:
        argparse.ArgumentTypeError: If any row does not contain three integers.
    """
    parsed_goal_rows: list[tuple[int, int, int]] = []
    for row_index, goal_text in enumerate(goals.split(";")):
        goal_values = goal_text.strip().split(",")
        if len(goal_values) != 3:
            raise argparse.ArgumentTypeError(
                f"goal row {row_index} must be formatted as 'x,y,yaw'"
            )
        try:
            x_text, y_text, yaw_text = goal_values
            parsed_goal_rows.append(
                (int(x_text.strip()), int(y_text.strip()), int(yaw_text.strip()))
            )
        except ValueError as error:
            raise argparse.ArgumentTypeError(
                f"goal row {row_index} must contain integer values"
            ) from error
    return tuple(parsed_goal_rows)


parser = argparse.ArgumentParser(
    description="Expert-policy rollout for the assemble-brick task."
)
parser.add_argument("--task", type=str, default="Lego-AssembleBrick-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--seed", type=int, default=None, help="Optional environment seed.")
parser.add_argument(
    "--goals",
    type=parse_goal_rows,
    default=None,
    help=(
        "Semicolon-separated command goal rows 'x,y,yaw;...'. "
        "If omitted, sample all valid goals."
    ),
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from isaaclab.envs.manager_based_rl_env import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg

import bricksim  # noqa: F401
from bricksim.envs.assemble_brick.env import AssembleBrickEnvCfg, CommandsCfg
from bricksim.envs.assemble_brick.expert import AssembleBrickExpert


def main() -> None:
    """Run the expert rollout until the simulation app exits.

    Returns:
        None.
    """
    env_cfg = parse_env_cfg(
        args_cli.task,
        num_envs=args_cli.num_envs,
        use_fabric=False,
        device="cpu",
    )
    assert isinstance(env_cfg, AssembleBrickEnvCfg)
    if args_cli.seed is not None:
        env_cfg.seed = args_cli.seed
    if args_cli.goals is not None:
        commands_cfg = env_cfg.commands
        assert isinstance(commands_cfg, CommandsCfg)
        commands_cfg.assembly_goal.goals = args_cli.goals
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    assert isinstance(env, ManagerBasedRLEnv)
    expert = AssembleBrickExpert()

    if args_cli.seed is not None:
        print(f"[INFO]: Using seed: {args_cli.seed}")
    obs, _ = env.reset()
    step = 0

    while simulation_app.is_running():
        with torch.inference_mode():
            privileged_obs = obs["privileged"]
            # Should be un-concatenated dict[str, torch.Tensor]
            assert not isinstance(privileged_obs, torch.Tensor)
            actions = expert.compute_actions(privileged_obs)
            obs, rewards, terminated, truncated, _ = env.step(actions)
            if bool((terminated | truncated).any()):
                print(
                    f"step={step} "
                    f"rewards={rewards.detach().cpu().tolist()} "
                    f"terminated={terminated.detach().cpu().tolist()} "
                    f"truncated={truncated.detach().cpu().tolist()}"
                )

            step += 1


if __name__ == "__main__":
    main()
