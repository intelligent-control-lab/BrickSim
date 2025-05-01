import time
import math
import torch
import isaaclab.utils.math as math_utils
from enum import IntEnum
from pxr import Gf
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg
from . import get_brick_physics_interface

class TrackedBrick(IntEnum):
    TO_GRASP = 0

def spawn_random_brick(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    dimensions: list[tuple[int,int,int]],
    colors: list[str],
    pos_range: tuple[tuple[float, float, float], tuple[float, float, float]],
    yaw_range: tuple[float, float] = (-math.pi, math.pi),
):
    t0 = time.perf_counter()

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

    t1 = time.perf_counter()
    print(f"Reset {len(env_ids)} envs in {(t1 - t0)*1e3:.2f} ms")

def brick_pose_in_robot_root_frame(
    env: ManagerBasedEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    iface = get_brick_physics_interface()
    tracker = iface.get_tracker(num_envs=env.num_envs, num_trackings=len(TrackedBrick))
    # x, y, z, qx, qy, qz, qw
    brick_poses = torch.from_numpy(tracker.get_transforms(TrackedBrick.TO_GRASP))

    robot: RigidObject = env.scene[robot_cfg.name]
    robot_t = robot.data.root_state_w[:, :3]
    robot_q = robot.data.root_state_w[:, 3:7]
    brick_t = brick_poses[:, :3]
    brick_q = math_utils.convert_quat(brick_poses[:, 3:7], to="wxyz")
    brick_rel_t, brick_rel_q = math_utils.subtract_frame_transforms(
        robot_t, robot_q,
        brick_t, brick_q
    )
    return torch.cat((brick_rel_t, brick_rel_q), dim=-1)
