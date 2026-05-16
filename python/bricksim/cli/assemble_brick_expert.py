"""CLI for running the assemble-brick expert policy."""

import argparse


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


def build_argument_parser() -> argparse.ArgumentParser:
    """Build the argument parser for the assemble-brick expert CLI.

    Returns:
        Configured argument parser.
    """
    parser = argparse.ArgumentParser(
        description="Expert-policy rollout for the assemble-brick task."
    )
    parser.add_argument("--num_envs", type=int, default=1)
    parser.add_argument(
        "--seed", type=int, default=None, help="Optional environment seed."
    )
    parser.add_argument(
        "--goals",
        type=parse_goal_rows,
        default=None,
        help=(
            "Semicolon-separated command goal rows 'x,y,yaw;...'. "
            "If omitted, sample all valid goals."
        ),
    )
    return parser


def main() -> int:
    """Run the assemble-brick expert policy.

    Returns:
        Process exit code.
    """
    import isaacsim_rtx_compat  # noqa: F401, I001

    from isaaclab.app import AppLauncher

    parser = build_argument_parser()
    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    import torch
    from isaaclab.envs import ManagerBasedRLEnv

    from bricksim.envs.assemble_brick.env import (
        AssembleBrickEnvCfg,
        CommandsCfg,
    )
    from bricksim.envs.assemble_brick.expert import AssembleBrickExpert

    env_cfg = AssembleBrickEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.render_interval = 1
    if args_cli.seed is not None:
        env_cfg.seed = args_cli.seed
    if args_cli.goals is not None:
        commands_cfg = env_cfg.commands
        assert isinstance(commands_cfg, CommandsCfg)
        commands_cfg.assembly_goal.goals = args_cli.goals
    env = ManagerBasedRLEnv(cfg=env_cfg)
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

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
