import torch
import isaaclab.utils.math as math_utils
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg
from .tracking import TrackedBrick, get_brick_pos_quat

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
