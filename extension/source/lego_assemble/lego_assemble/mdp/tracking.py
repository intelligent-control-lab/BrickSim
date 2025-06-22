import torch
import numpy as np
import isaaclab.utils.math as math_utils
from enum import IntEnum
from typing import Union
from isaaclab.envs import ManagerBasedEnv
from lego_assemble.physics.interface import BrickTracker, get_brick_physics_interface

class TrackedBrick(IntEnum):
    TO_GRASP = 0

def get_tracker(env: ManagerBasedEnv) -> BrickTracker:
    return get_brick_physics_interface().get_tracker(num_envs=env.num_envs, num_trackings=len(TrackedBrick))

def get_brick_pose(env: ManagerBasedEnv, tracked_brick: TrackedBrick) -> torch.Tensor:
    return get_tracker(env).get_transforms(tracked_brick)

def get_brick_pos_quat(env: ManagerBasedEnv, tracked_brick: TrackedBrick) -> tuple[torch.Tensor, torch.Tensor]:
    pose = get_brick_pose(env, tracked_brick)
    pos = pose[:, :3]
    quat = math_utils.convert_quat(pose[:, 3:7], to="wxyz")
    return pos, quat

def set_tracked_bricks(env: ManagerBasedEnv, tracking_id: int, env_ids: torch.Tensor, brick_ids: Union[torch.Tensor, np.ndarray, list[int]]):
    get_tracker(env).set_tracked_bricks(tracking_id, env_ids, torch.as_tensor(brick_ids, device=env.device))
