import math
import torch
from isaaclab.sensors import FrameTransformer
import isaaclab.utils.math as math_utils
from enum import IntEnum
from pxr import Gf
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from . import get_brick_physics_interface

class TrackedBrick(IntEnum):
    TO_GRASP = 0

def get_brick_pose(env: ManagerBasedEnv, tracked_brick: TrackedBrick) -> torch.Tensor:
    iface = get_brick_physics_interface()
    tracker = iface.get_tracker(num_envs=env.num_envs, num_trackings=len(TrackedBrick))
    return torch.from_numpy(tracker.get_transforms(tracked_brick))

def get_brick_pos_quat(env: ManagerBasedEnv, tracked_brick: TrackedBrick) -> tuple[torch.Tensor, torch.Tensor]:
    pose = get_brick_pose(env, tracked_brick)
    pos = pose[:, :3]
    quat = math_utils.convert_quat(pose[:, 3:7], to="wxyz")
    return pos, quat

def reset_and_spawn_brick(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    dimensions: list[tuple[int,int,int]],
    colors: list[str],
    pos_range: tuple[tuple[float, float, float], tuple[float, float, float]],
    yaw_range: tuple[float, float] = (-math.pi, math.pi),
):
    dim_choices = torch.randint(0, len(dimensions), (len(env_ids),), device="cpu")
    color_choices = torch.randint(0, len(colors), (len(env_ids),), device="cpu")
    pos = math_utils.sample_uniform(
        lower=torch.tensor(pos_range[0], device="cpu"),
        upper=torch.tensor(pos_range[1], device="cpu"),
        size=(len(env_ids), 3),
        device="cpu",
    )
    quat = math_utils.quat_from_angle_axis(
        angle=math_utils.sample_uniform(
            lower=yaw_range[0],
            upper=yaw_range[1],
            size=(len(env_ids),),
            device="cpu",
        ),
        axis=torch.tensor([0., 0., 1.], device="cpu")
    )

    iface = get_brick_physics_interface()

    for env_id in env_ids:
        iface.reset_env(int(env_id))

    new_brick_ids = []
    for env_id in env_ids:
        brick_xform, brick_id = iface.create_brick(
            dimensions=dimensions[dim_choices[env_id]],
            color_name=colors[color_choices[env_id]],
            env_id=int(env_id),
            pos=Gf.Vec3f(pos[env_id].tolist()),
            quat=Gf.Quatf(*quat[env_id].tolist()),
        )
        new_brick_ids.append(brick_id)

    tracker = iface.get_tracker(num_envs=env.num_envs, num_trackings=len(TrackedBrick))
    tracker.set_tracked_bricks(TrackedBrick.TO_GRASP, env_ids, new_brick_ids)

# From isaaclab_tasks/manager_based/manipulation/lift/mdp/observations.py
def brick_pose_in_robot_root_frame(
    env: ManagerBasedEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    robot: RigidObject = env.scene[robot_cfg.name]
    robot_t = robot.data.root_state_w[:, :3]
    robot_q = robot.data.root_state_w[:, 3:7]
    brick_t, brick_q = get_brick_pos_quat(env, TrackedBrick.TO_GRASP)
    brick_rel_t, brick_rel_q = math_utils.subtract_frame_transforms(
        robot_t, robot_q,
        brick_t, brick_q
    )
    return torch.cat((brick_rel_t, brick_rel_q), dim=-1)

# From isaaclab_tasks/manager_based/manipulation/lift/mdp/rewards.py
def brick_is_lifted(
    env: ManagerBasedRLEnv,
    minimal_height: float,
    tracked_brick: TrackedBrick,
) -> torch.Tensor:
    """Reward the agent for lifting the object above the minimal height."""
    brick_pose = get_brick_pose(env, tracked_brick)
    return torch.where(brick_pose[:, 2] > minimal_height, 1.0, 0.0)

# From isaaclab_tasks/manager_based/manipulation/lift/mdp/rewards.py
def brick_ee_distance(
    env: ManagerBasedRLEnv,
    std: float,
    tracked_brick: TrackedBrick,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward the agent for reaching the object using tanh-kernel."""
    # extract the used quantities (to enable type-hinting)
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    # Target object position: (num_envs, 3)
    cube_pos_w = get_brick_pose(env, tracked_brick)[:, :3]
    # End-effector position: (num_envs, 3)
    ee_w = ee_frame.data.target_pos_w[..., 0, :]
    # Distance of the end-effector to the object: (num_envs,)
    object_ee_distance = torch.norm(cube_pos_w - ee_w, dim=1)

    return 1 - torch.tanh(object_ee_distance / std)

# From isaaclab_tasks/manager_based/manipulation/lift/mdp/rewards.py
def brick_goal_distance(
    env: ManagerBasedRLEnv,
    std: float,
    minimal_height: float,
    command_name: str,
    tracked_brick: TrackedBrick,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward the agent for tracking the goal pose using tanh-kernel."""
    # extract the used quantities (to enable type-hinting)
    robot: RigidObject = env.scene[robot_cfg.name]
    brick_pose = get_brick_pose(env, tracked_brick)
    command = env.command_manager.get_command(command_name)
    # compute the desired position in the world frame
    des_pos_b = command[:, :3]
    des_pos_w, _ = math_utils.combine_frame_transforms(robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], des_pos_b)
    # distance of the end-effector to the object: (num_envs,)
    distance = torch.norm(des_pos_w - brick_pose[:, :3], dim=1)
    # rewarded if the object is lifted above the threshold
    return (brick_pose[:, 2] > minimal_height) * (1 - torch.tanh(distance / std))

# From isaaclab/envs/mdp/terminations.py
def brick_height_below_minimum(
    env: ManagerBasedRLEnv,
    minimum_height: float,
    tracked_brick: TrackedBrick,
) -> torch.Tensor:
    """Terminate when the asset's root height is below the minimum height.

    Note:
        This is currently only supported for flat terrains, i.e. the minimum height is in the world frame.
    """
    brick_pose = get_brick_pose(env, tracked_brick)
    return brick_pose[:, 2] < minimum_height
