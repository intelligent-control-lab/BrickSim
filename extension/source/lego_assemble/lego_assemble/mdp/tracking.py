import torch
import numpy as np
import isaaclab.utils.math as math_utils
from enum import IntEnum
from typing import Union
from isaaclab.envs import ManagerBasedEnv
from lego_assemble.physics.interface import NumpyBrickTracker, TorchBrickTracker, get_brick_physics_interface

class TrackedBrick(IntEnum):
    TO_GRASP = 0

def get_tracker(env: ManagerBasedEnv) -> Union[NumpyBrickTracker, TorchBrickTracker]:
    return get_brick_physics_interface().get_tracker(num_envs=env.num_envs, num_trackings=len(TrackedBrick))

def get_brick_pose(env: ManagerBasedEnv, tracked_brick: TrackedBrick) -> torch.Tensor:
    result = get_tracker(env).get_transforms(tracked_brick)
    if isinstance(result, np.ndarray):
        result = torch.from_numpy(result).to(env.device)
    return result

def get_brick_pos_quat(env: ManagerBasedEnv, tracked_brick: TrackedBrick) -> tuple[torch.Tensor, torch.Tensor]:
    pose = get_brick_pose(env, tracked_brick)
    pos = pose[:, :3]
    quat = math_utils.convert_quat(pose[:, 3:7], to="wxyz")
    return pos, quat

def set_tracked_bricks(env: ManagerBasedEnv, tracking_id: int, env_ids: torch.Tensor, brick_ids: Union[torch.Tensor, np.ndarray, list[int]]):
    tracker = get_tracker(env)
    if isinstance(tracker, NumpyBrickTracker):
        if isinstance(brick_ids, torch.Tensor):
            brick_ids = brick_ids.cpu().numpy()
        tracker.set_tracked_bricks(tracking_id, env_ids, brick_ids)
    elif isinstance(tracker, TorchBrickTracker):
        tracker.set_tracked_bricks(tracking_id, env_ids, torch.as_tensor(brick_ids, device=env.device))
