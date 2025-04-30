"""Launch Isaac Sim Simulator first."""

import os
import sys
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Enable lego_assemble extension
plugin_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "source"))
sys.argv += ["--ext-folder", plugin_folder, "--enable", "lego_assemble"]

# Verbose output
sys.argv += ["-v"]

app_launcher = AppLauncher(args_cli)
from isaacsim.simulation_app import SimulationApp
simulation_app: SimulationApp = app_launcher.app

"""Rest everything follows."""

import time
import math
import torch
import isaaclab.envs.mdp as mdp
import isaaclab.utils.math as math_utils
from enum import IntEnum
from pxr import Gf
from isaaclab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.sim import GroundPlaneCfg, DomeLightCfg, UsdFileCfg
from isaaclab_assets import FRANKA_PANDA_CFG, ISAAC_NUCLEUS_DIR
from isaaclab.assets import AssetBaseCfg, ArticulationCfg, RigidObject
from isaaclab.scene import InteractiveSceneCfg
import lego_assemble

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

    iface = lego_assemble.get_brick_physics_interface()

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
    iface = lego_assemble.get_brick_physics_interface()
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

@configclass
class FrankaSceneCfg(InteractiveSceneCfg):

    # lego_assemble doesn't support replicate_physics
    replicate_physics = False

    robot: ArticulationCfg = FRANKA_PANDA_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[0.707, 0, 0, 0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )

    ground_plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.05]),
        spawn=GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

@configclass
class ActionsCfg:
    joint_efforts = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=["panda_joint.*"],
    )

@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)
        brick_pose = ObsTerm(func=brick_pose_in_robot_root_frame)

    policy: PolicyCfg = PolicyCfg()

@configclass
class EventCfg:
    reset_arm_pose = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["panda_joint.*"]),
            "position_range": (-0.125 * math.pi, 0.125 * math.pi),
            "velocity_range": (-0.1, 0.1),
        },
    )
    spawn_brick = EventTerm(
        func=spawn_random_brick,
        mode="reset",
        params={
            "dimensions": [
                (4, 2, 3),
                (2, 2, 3),
            ],
            "colors": [
                "Pink",
                "Light Blue",
            ],
            "pos_range": (
                (0.1, -0.5, 0.0),
                (0.6, 0.5, 0.0),
            )
        }
    )

@configclass
class FrankaEnvCfg(ManagerBasedEnvCfg):
    scene = FrankaSceneCfg(num_envs=16, env_spacing=2.5)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
        self.sim.device = "cpu" # lego_assemble supports cpu only
        self.viewer.eye = (4.5, 0.0, 6.0)
        self.viewer.lookat = (0.0, 0.0, 2.0)
        self.decimation = 4
        self.sim.dt = 0.005

def main():
    env_cfg = FrankaEnvCfg()
    env = ManagerBasedEnv(cfg=env_cfg)

    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            if count % 300 == 0:
                count = 0
                env.reset()
            joint_efforts = torch.randn_like(env.action_manager.action)
            obs, _ = env.step(joint_efforts)
            count += 1
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
