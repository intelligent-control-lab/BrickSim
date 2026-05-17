"""CLI for running the assemble-brick expert policy."""

import argparse
from pathlib import Path

import cv2
import torch
from PIL import Image


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
    parser.add_argument(
        "--obs-mode",
        choices=("state", "rgb", "rgbd"),
        default="state",
        help="Observation mode to run.",
    )
    parser.add_argument(
        "--save-camera-dir",
        type=Path,
        default=None,
        help="Directory for saving captured camera images.",
    )
    parser.add_argument(
        "--save-camera-every",
        type=int,
        default=30,
        help="Save one camera frame every N environment steps.",
    )
    parser.add_argument(
        "--show-camera",
        action="store_true",
        help="Display captured camera images with OpenCV.",
    )
    parser.add_argument(
        "--show-goals",
        action="store_true",
        help="Show assembly goal marker visualizer.",
    )
    return parser


def _save_camera_images(
    images_obs: dict[str, torch.Tensor],
    save_camera_dir: Path,
    step: int,
) -> None:
    """Save camera observation images for one rollout step.

    Args:
        images_obs: Non-concatenated image observation group keyed by term name.
        save_camera_dir: Directory where PNG images are written.
        step: Rollout step index used in output filenames.

    Returns:
        None.
    """
    for obs_name, image_tensor in images_obs.items():
        image = image_tensor[0].detach().cpu()
        output_path = save_camera_dir / f"step_{step:06d}_{obs_name}.png"
        output_image = image.contiguous().numpy()
        Image.fromarray(output_image).save(output_path)


def _show_camera_images(images_obs: dict[str, torch.Tensor]) -> None:
    """Display camera observation images for one rollout step.

    Args:
        images_obs: Non-concatenated image observation group keyed by term name.

    Returns:
        None.
    """
    for obs_name, image_tensor in images_obs.items():
        image = image_tensor[0].detach().cpu()
        if image.shape[-1] == 3:
            display_image = cv2.cvtColor(image.contiguous().numpy(), cv2.COLOR_RGB2BGR)
        else:
            depth = torch.nan_to_num(image, nan=0.0, posinf=0.0, neginf=0.0)
            valid_depth = depth > 0
            if bool(valid_depth.any()):
                min_depth = depth[valid_depth].min()
                max_depth = depth[valid_depth].quantile(0.95)
                depth_vis = (depth - min_depth) / (max_depth - min_depth).clamp_min(
                    1e-6
                )
                depth_vis = depth_vis.clamp(0.0, 1.0)
            else:
                depth_vis = torch.zeros_like(depth)

            display_image = cv2.applyColorMap(
                (depth_vis * 255).to(torch.uint8).contiguous().numpy(),
                cv2.COLORMAP_TURBO,
            )

        cv2.imshow(obs_name, display_image)
    cv2.waitKey(1)


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
    if args_cli.save_camera_every <= 0:
        parser.error("--save-camera-every must be positive")
    if args_cli.obs_mode != "state":
        args_cli.enable_cameras = True
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    from isaaclab.envs import ManagerBasedRLEnv

    from bricksim.envs.assemble_brick.env import (
        AssembleBrickBaseEnvCfg,
        AssembleBrickEnvCfg,
        AssembleBrickRGBDEnvCfg,
        AssembleBrickRGBEnvCfg,
    )
    from bricksim.envs.assemble_brick.expert import AssembleBrickExpert

    env_cfg: AssembleBrickBaseEnvCfg
    if args_cli.obs_mode == "state":
        env_cfg = AssembleBrickEnvCfg()
    elif args_cli.obs_mode == "rgb":
        env_cfg = AssembleBrickRGBEnvCfg()
    else:
        env_cfg = AssembleBrickRGBDEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.render_interval = 1
    if args_cli.seed is not None:
        env_cfg.seed = args_cli.seed
    if args_cli.goals is not None:
        env_cfg.commands.assembly_goal.goals = args_cli.goals
    if args_cli.show_goals:
        env_cfg.commands.assembly_goal.debug_vis = True
    env = ManagerBasedRLEnv(cfg=env_cfg)
    expert = AssembleBrickExpert()

    if args_cli.seed is not None:
        print(f"[INFO]: Using seed: {args_cli.seed}")
    obs, _ = env.reset()
    if args_cli.obs_mode != "state":
        images_obs = obs["images"]
        assert not isinstance(images_obs, torch.Tensor)

        for obs_name, image_obs in images_obs.items():
            print(
                f"[INFO]: image obs '{obs_name}': shape={tuple(image_obs.shape)} "
                f"dtype={image_obs.dtype} device={image_obs.device}"
            )
    if args_cli.save_camera_dir is not None:
        args_cli.save_camera_dir.mkdir(parents=True, exist_ok=True)
    step = 0

    while simulation_app.is_running():
        with torch.inference_mode():
            privileged_obs = obs["privileged"]
            # Should be un-concatenated dict[str, torch.Tensor]
            assert not isinstance(privileged_obs, torch.Tensor)
            actions = expert.compute_actions(privileged_obs)
            obs, rewards, terminated, truncated, _ = env.step(actions)
            if (
                args_cli.save_camera_dir is not None
                and args_cli.obs_mode != "state"
                and step % args_cli.save_camera_every == 0
            ):
                images_obs = obs["images"]
                assert not isinstance(images_obs, torch.Tensor)
                _save_camera_images(images_obs, args_cli.save_camera_dir, step)
            if args_cli.show_camera and args_cli.obs_mode != "state":
                images_obs = obs["images"]
                assert not isinstance(images_obs, torch.Tensor)
                _show_camera_images(images_obs)
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
