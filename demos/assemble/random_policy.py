#!/usr/bin/env python3

import argparse

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Random-policy rollout for the BrickSim assemble-brick task.")
parser.add_argument("--task", type=str, default="Lego-AssembleBrick-v0", help="Gym task id.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of parallel environments.")
parser.add_argument(
    "--mode",
    type=str,
    choices=("random", "gripper_test"),
    default="random",
    help="Rollout mode. 'gripper_test' holds the arm fixed and alternates the gripper open/close command.",
)
parser.add_argument(
    "--arm_action_range",
    type=float,
    default=0.1,
    help="Uniform half-range for the 6-D relative arm IK action.",
)
parser.add_argument(
    "--gripper_hold_steps",
    type=int,
    default=100,
    help="Number of steps to hold each gripper command in gripper_test mode.",
)
parser.add_argument(
    "--print_every",
    type=int,
    default=20,
    help="Print gripper observation every N steps in gripper_test mode.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import bricksim.envs
from isaaclab_tasks.utils import parse_env_cfg


def get_concatenated_term_slice(env, group_name: str, term_name: str) -> slice:
    group_terms = env.unwrapped.observation_manager.active_terms[group_name]
    group_term_dims = env.unwrapped.observation_manager.group_obs_term_dim[group_name]

    start = 0
    for active_term_name, term_dim in zip(group_terms, group_term_dims, strict=True):
        term_width = int(torch.tensor(term_dim).prod().item())
        if active_term_name == term_name:
            return slice(start, start + term_width)
        start += term_width

    raise KeyError(f"Observation term '{term_name}' not found in group '{group_name}'.")


def main():
    env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs, use_fabric=False, device="cpu")
    env = gym.make(args_cli.task, cfg=env_cfg)
    gripper_obs_slice = get_concatenated_term_slice(env, group_name="policy", term_name="gripper_pos")

    try:
        print(f"[INFO]: task: {args_cli.task}")
        print(f"[INFO]: observation space: {env.observation_space}")
        print(f"[INFO]: action space: {env.action_space}")
        print(f"[INFO]: mode: {args_cli.mode}")

        env.reset()
        step_count = 0

        while simulation_app.is_running():
            with torch.inference_mode():
                num_envs = env.unwrapped.num_envs
                device = env.unwrapped.device
                action_dim = env.unwrapped.action_manager.total_action_dim

                actions = torch.zeros((num_envs, action_dim), device=device)
                if args_cli.mode == "random":
                    actions[:, :6] = args_cli.arm_action_range * (2.0 * torch.rand((num_envs, 6), device=device) - 1.0)
                    actions[:, 6] = torch.where(
                        torch.rand((num_envs,), device=device) < 0.5,
                        torch.full((num_envs,), -1.0, device=device),
                        torch.full((num_envs,), 1.0, device=device),
                    )
                else:
                    is_open_phase = (step_count // args_cli.gripper_hold_steps) % 2 == 0
                    actions[:, 6] = 1.0 if is_open_phase else -1.0

                observations, *_ = env.step(actions)

                if args_cli.mode == "gripper_test" and step_count % args_cli.print_every == 0:
                    gripper_obs = observations["policy"][0, gripper_obs_slice].detach().cpu().tolist()
                    gripper_cmd = "open" if actions[0, 6].item() > 0.0 else "close"
                    print(f"[INFO]: step={step_count} gripper_cmd={gripper_cmd} gripper_pos={gripper_obs}")

                step_count += 1
    finally:
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
