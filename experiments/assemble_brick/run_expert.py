#!/usr/bin/env python3
"""Run the assemble-brick expert policy."""

# ruff: noqa: E402

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description="Expert-policy rollout for the assemble-brick task."
)
parser.add_argument("--task", type=str, default="Lego-AssembleBrick-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--seed", type=int, default=None, help="Optional environment seed.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from isaaclab.envs.manager_based_rl_env import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg

import bricksim  # noqa: F401
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
    if args_cli.seed is not None:
        env_cfg.seed = args_cli.seed
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
