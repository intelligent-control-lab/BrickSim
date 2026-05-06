"""Observation terms for the assemble-brick MDP."""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import subtract_frame_transforms
from isaaclab_tasks.manager_based.manipulation.place.mdp.observations import (
    object_grasped,
)

from bricksim.mdp.brick_part import scene_entity_brick_part_dimensions

from .commands import (
    assembly_goal_target_pose,
    assembly_moving_brick_name,
    assembly_query_connection_state,
)


def franka_gripper_width(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    joint_names: tuple[str, ...] = ("panda_finger_.*",),
) -> torch.Tensor:
    """Return the Franka gripper opening width.

    Args:
        env: Manager-based RL environment.
        robot_cfg: Scene entity for the robot articulation.
        joint_names: Joint-name patterns for the two finger joints.

    Returns:
        Float tensor with shape ``(num_envs, 1)`` containing the sum of the
        selected finger joint positions in meters.
    """
    robot = env.scene[robot_cfg.name]
    joint_ids, _ = robot.find_joints(joint_names)
    return torch.sum(robot.data.joint_pos[:, joint_ids], dim=1, keepdim=True)


def franka_gripper_speed(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    joint_names: tuple[str, ...] = ("panda_finger_.*",),
) -> torch.Tensor:
    """Return the maximum absolute Franka finger joint velocity.

    Args:
        env: Manager-based RL environment.
        robot_cfg: Scene entity for the robot articulation.
        joint_names: Joint-name patterns for the two finger joints.

    Returns:
        Float tensor with shape ``(num_envs, 1)`` containing the largest
        absolute selected finger joint velocity in meters per second.
    """
    robot = env.scene[robot_cfg.name]
    joint_ids, _ = robot.find_joints(joint_names)
    speed = torch.max(torch.abs(robot.data.joint_vel[:, joint_ids]), dim=1).values
    return speed.unsqueeze(-1)


def obs_command_connection_created(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
) -> torch.Tensor:
    """Return whether the commanded interface pair is connected.

    This returns ``True`` no matter whether the connection uses the correct or
    wrong placement.

    Args:
        env: Manager-based RL environment.
        command_name: Name of the assemble-brick command term.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    return assembly_query_connection_state(env, command_name).connected


def obs_moving_brick_dimensions(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
) -> torch.Tensor:
    """Return the dimensions of the command moving brick.

    Args:
        env: Manager-based RL environment.
        command_name: Name of the assemble-brick command term.

    Returns:
        Long tensor with shape ``(num_envs, 3)`` ordered as ``(L, W, H)``.
    """
    dimensions = torch.tensor(
        scene_entity_brick_part_dimensions(
            env, assembly_moving_brick_name(env, command_name)
        ),
        device=env.device,
        dtype=torch.long,
    )
    return dimensions.unsqueeze(0).repeat(env.num_envs, 1)


def obs_moving_brick_pose(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Return the pose of the command moving brick in the robot root frame.

    Args:
        env: Manager-based RL environment.
        command_name: Name of the assemble-brick command term.
        robot_cfg: Scene entity for the robot articulation.

    Returns:
        Pose tensor with shape ``(num_envs, 7)`` containing position in meters
        and quaternion in WXYZ order.
    """
    moving_brick = env.scene[assembly_moving_brick_name(env, command_name)]
    robot = env.scene[robot_cfg.name]
    brick_pos_b, brick_quat_b = subtract_frame_transforms(
        robot.data.root_pos_w,
        robot.data.root_quat_w,
        moving_brick.data.root_pos_w,
        moving_brick.data.root_quat_w,
    )
    return torch.cat((brick_pos_b, brick_quat_b), dim=1)


def obs_moving_brick_grasped(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    diff_threshold: float = 0.04,
    force_threshold: float = 1.0,
) -> torch.Tensor:
    """Return whether the command moving brick is grasped.

    Args:
        env: Manager-based RL environment.
        command_name: Name of the assemble-brick command term.
        ee_frame_cfg: Scene entity for the end-effector frame transformer.
        robot_cfg: Scene entity for the robot articulation.
        diff_threshold: Maximum object-root to end-effector distance in meters.
        force_threshold: Minimum per-finger contact-force magnitude in newtons
            when contact-grasp sensors are present.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    return object_grasped(
        env,
        robot_cfg=robot_cfg,
        ee_frame_cfg=ee_frame_cfg,
        object_cfg=SceneEntityCfg(assembly_moving_brick_name(env, command_name)),
        diff_threshold=diff_threshold,
        force_threshold=force_threshold,
    )


def obs_command_target_pose(
    env: ManagerBasedRLEnv,
    command_name: str = "assembly_goal",
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Return the sampled command target pose in the robot root frame.

    Args:
        env: Manager-based RL environment.
        command_name: Name of the assemble-brick command term.
        robot_cfg: Scene entity for the robot articulation.

    Returns:
        Pose tensor with shape ``(num_envs, 7)`` containing position in meters
        and quaternion in WXYZ order.
    """
    robot_asset = env.scene[robot_cfg.name]
    target_pos_w, target_quat_w = assembly_goal_target_pose(env, command_name)
    target_pos_b, target_quat_b = subtract_frame_transforms(
        robot_asset.data.root_pos_w,
        robot_asset.data.root_quat_w,
        target_pos_w,
        target_quat_w,
    )
    return torch.cat((target_pos_b, target_quat_b), dim=1)
