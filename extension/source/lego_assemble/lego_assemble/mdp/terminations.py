import torch
from isaaclab.envs import ManagerBasedRLEnv
from .tracking import TrackedBrick, get_brick_pose

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
