import torch
import isaaclab.utils.math as math_utils
from enum import IntEnum
from isaaclab.envs import ManagerBasedEnv
from lego_assemble.physics.interface import get_brick_physics_interface

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
