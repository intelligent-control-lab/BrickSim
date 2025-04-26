import os
import sys
import math
import torch
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Franka Panda Demo")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = "isaaclab.python.rendering.kit"          # same as before
ext_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "source"))
sys.argv += ["--ext-folder", ext_folder, "--enable", "lego_assemble", "-v"]
app_launcher = AppLauncher(args_cli)

from isaacsim.simulation_app import SimulationApp
simulation_app: SimulationApp = app_launcher.app

import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.sim import GroundPlaneCfg, DomeLightCfg
from isaaclab_assets import FRANKA_PANDA_CFG
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.scene import InteractiveSceneCfg

@configclass
class FrankaSceneCfg(InteractiveSceneCfg):

    # lego_assemble doesn't support replicate_physics
    replicate_physics = False

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=GroundPlaneCfg(size=(100.0, 100.0)),
    )
    robot: ArticulationCfg = FRANKA_PANDA_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=DomeLightCfg(intensity=500.0, color=(0.9, 0.9, 0.9)),
    )

@configclass
class ActionsCfg:
    joint_efforts = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=["panda_joint.*"],
        scale=5.0,
    )

@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

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
                print("-" * 80)
                print("[INFO]: Resetting environment...")
            # sample random torques
            joint_efforts = torch.randn_like(env.action_manager.action)
            obs, _ = env.step(joint_efforts)
            # print shoulder pitch of env 0 (panda_joint2)
            print("[Env 0]: panda_joint2 = ", obs["policy"][0][1].item())
            count += 1
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
