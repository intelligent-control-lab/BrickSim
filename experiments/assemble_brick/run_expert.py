#!/usr/bin/env python3

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Expert-policy rollout for BrickSim assemble-brick.")
parser.add_argument("--task", type=str, default="Lego-AssembleBrick-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--seed", type=int, default=None, help="Optional environment seed.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import bricksim
from isaaclab_tasks.utils import parse_env_cfg


def main():
    env_cfg = parse_env_cfg(
        args_cli.task,
        num_envs=args_cli.num_envs,
        use_fabric=False,
        device="cpu",
    )
    if args_cli.seed is not None:
        env_cfg.seed = args_cli.seed
    env = gym.make(args_cli.task, cfg=env_cfg)

    try:
        if args_cli.seed is not None:
            print(f"[INFO]: Using seed: {args_cli.seed}")
        env.reset()
        step = 0

        while simulation_app.is_running():
            with torch.inference_mode():
                actions = env.unwrapped.compute_expert_actions()
                observations, rewards, terminated, truncated, info = env.step(actions)

                if bool((terminated | truncated).any()):
                    print(
                        f"step={step} "
                        f"rewards={rewards.detach().cpu().tolist()} "
                        f"terminated={terminated.detach().cpu().tolist()} "
                        f"truncated={truncated.detach().cpu().tolist()}"
                    )

                step += 1
    finally:
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
