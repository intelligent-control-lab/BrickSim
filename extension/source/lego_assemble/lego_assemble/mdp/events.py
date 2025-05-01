import math
import torch
import isaaclab.utils.math as math_utils
from pxr import Gf
from isaaclab.envs import ManagerBasedEnv
from lego_assemble.physics.interface import get_brick_physics_interface
from .tracking import TrackedBrick

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
    quat: torch.Tensor = math_utils.quat_from_angle_axis(
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
    for env_id, dim_choice, color_choice, p, q in zip(env_ids, dim_choices, color_choices, pos, quat):
        brick_xform, brick_id = iface.create_brick(
            dimensions=dimensions[dim_choice],
            color_name=colors[color_choice],
            env_id=int(env_id),
            pos=Gf.Vec3f(p.tolist()),
            quat=Gf.Quatf(*q.tolist()),
        )
        new_brick_ids.append(brick_id)

    tracker = iface.get_tracker(num_envs=env.num_envs, num_trackings=len(TrackedBrick))
    tracker.set_tracked_bricks(TrackedBrick.TO_GRASP, env_ids, new_brick_ids)
