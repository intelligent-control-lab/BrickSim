import torch
from isaaclab.sensors import FrameTransformer
import isaaclab.utils.math as math_utils
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from .tracking import TrackedBrick, get_brick_pose

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

def brick_upright(
    env: ManagerBasedRLEnv,
    minimal_height: float,
    tracked_brick: TrackedBrick,
) -> torch.Tensor:
    brick_pose = get_brick_pose(env, tracked_brick)
    brick_q = brick_pose[:, 3:7] # xyzw

    qx = brick_q[:,0]
    qy = brick_q[:,1]
    cos_angle = 1 - 2*(qx*qx + qy*qy) # [-1, 1]

    return (brick_pose[:, 2] > minimal_height) * torch.clamp(cos_angle, 0, 1) # never reward upside down

@torch.jit.script
def _cos_quat_diff(q1: torch.Tensor, q2: torch.Tensor): # wxyz
    dq = math_utils.quat_mul(q1, math_utils.quat_conjugate(q2))
    dq = math_utils.normalize(dq)
    qw = dq[:, 0]; qx = dq[:, 1]; qy = dq[:, 2]; qz = dq[:, 3]
    theta = 2 * torch.atan2(torch.sqrt(qx*qx + qy*qy + qz*qz), qw)
    return torch.cos(theta)

def brick_goal_aligned(
    env: ManagerBasedRLEnv,
    minimal_height: float,
    command_name: str,
    tracked_brick: TrackedBrick,
) -> torch.Tensor:
    brick_pose = get_brick_pose(env, tracked_brick)
    brick_q = math_utils.convert_quat(
        brick_pose[:, 3:7], # xyzw
        to="wxyz"
    )
    command = env.command_manager.get_command(command_name)
    des_q = command[:, 3:7] # wxyz
    orientation_diff = _cos_quat_diff(brick_q, des_q) # [-1, 1]
    height_gate = brick_pose[:, 2] > minimal_height
    return height_gate * torch.clamp(orientation_diff, 0, 1)

# @torch.jit.script
# def _cos_yaw_diff(q1: torch.Tensor, q2: torch.Tensor) -> torch.Tensor: # wxyz
#     dq = math_utils.quat_mul(q1, math_utils.quat_conjugate(q2))
#     qw = dq[:, 0]; qx = dq[:, 1]; qy = dq[:, 2]; qz = dq[:, 3]
#     yaw = torch.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
#     cos_yaw = torch.cos(yaw)
#     return cos_yaw

# def brick_yaw_tracking(
#     env: ManagerBasedRLEnv,
#     minimal_height: float,
#     command_name: str,
#     tracked_brick: TrackedBrick,
# ) -> torch.Tensor:
#     brick_pose = get_brick_pose(env, tracked_brick)
#     brick_q = math_utils.convert_quat(
#         brick_pose[:, 3:7], # xyzw
#         to="wxyz"
#     )
#     command = env.command_manager.get_command(command_name)
#     des_q = command[:, 3:7] # wxyz
#     cos_yaw_diff = _cos_yaw_diff(brick_q, des_q) # [-1, 1]
#     height_gate = brick_pose[:, 2] > minimal_height
#     return height_gate * (1 + cos_yaw_diff) / 2
