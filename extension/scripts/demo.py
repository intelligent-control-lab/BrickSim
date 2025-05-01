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

import math
import torch
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from isaaclab.envs.mdp import BinaryJointPositionActionCfg, JointEffortActionCfg, action_rate_l2, generated_commands, joint_vel_l2, last_action, reset_joints_by_offset, joint_pos_rel, joint_vel_rel, time_out
from isaaclab.managers import EventTermCfg, ObservationGroupCfg, ObservationTermCfg, RewardTermCfg, SceneEntityCfg, TerminationTermCfg
from isaaclab.utils import configclass
from isaaclab.sim import GroundPlaneCfg, DomeLightCfg, UsdFileCfg
from isaaclab_assets import FRANKA_PANDA_CFG, ISAAC_NUCLEUS_DIR
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg, OffsetCfg
from lego_assemble.brick_mdp import TrackedBrick, brick_ee_distance, brick_goal_distance, brick_height_below_minimum, brick_is_lifted, reset_and_spawn_brick, brick_pose_in_robot_root_frame
from lego_assemble.brick_pose_command import BrickUniformPoseCommandCfg

@configclass
class FrankaSceneCfg(InteractiveSceneCfg):

    # lego_assemble doesn't support replicate_physics
    replicate_physics = False

    robot: ArticulationCfg = FRANKA_PANDA_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )

    ee_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
        debug_vis=False,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
                name="end_effector",
                offset=OffsetCfg(
                    pos=[0.0, 0.0, 0.1034],
                ),
            ),
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger",
                name="tool_rightfinger",
                offset=OffsetCfg(
                    pos=(0.0, 0.0, 0.046),
                ),
            ),
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/panda_leftfinger",
                name="tool_leftfinger",
                offset=OffsetCfg(
                    pos=(0.0, 0.0, 0.046),
                ),
            ),
        ],
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
    joint_efforts = JointEffortActionCfg(
        asset_name="robot",
        joint_names=["panda_joint.*"],
    )

    gripper_action = BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["panda_finger.*"],
        open_command_expr={"panda_finger_.*": 0.04},
        close_command_expr={"panda_finger_.*": 0.0},
    )

@configclass
class CommandsCfg:
    goal_pose = BrickUniformPoseCommandCfg(
        asset_name="robot",
        tracked_brick=TrackedBrick.TO_GRASP,
        resampling_time_range=(1e6, 1e6),
        debug_vis=True,
        ranges=BrickUniformPoseCommandCfg.Ranges(
            pos_x=(0.1, 0.6),
            pos_y=(-0.25, 0.25),
            pos_z=(0.25, 0.5),
            roll=(0.0, 0.0),
            pitch=(0.0, 0.0),
            yaw=(-math.pi, math.pi)
        ),
    )

@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        joint_pos_rel = ObservationTermCfg(
            func=joint_pos_rel
        )

        joint_vel_rel = ObservationTermCfg(
            func=joint_vel_rel
        )

        brick_pose = ObservationTermCfg(
            func=brick_pose_in_robot_root_frame
        )

        target_object_position = ObservationTermCfg(
            func=generated_commands,
            params={
                "command_name": "goal_pose"
            }
        )

        actions = ObservationTermCfg(
            func=last_action,
        )

    policy: PolicyCfg = PolicyCfg()

@configclass
class EventCfg:
    reset_arm_pose = EventTermCfg(
        func=reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["panda_joint.*"]),
            "position_range": (-0.125 * math.pi, 0.125 * math.pi),
            "velocity_range": (-0.1, 0.1),
        },
    )
    spawn_brick = EventTermCfg(
        func=reset_and_spawn_brick,
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
                (0.1, -0.25, 0.0),
                (0.6, 0.25, 0.0),
            )
        }
    )

@configclass
class RewardsCfg:
    reaching_object = RewardTermCfg(
        func=brick_ee_distance,
        params={
            "std": 0.1,
            "tracked_brick": TrackedBrick.TO_GRASP,
        },
        weight=1.0,
    )

    lifting_object = RewardTermCfg(
        func=brick_is_lifted,
        params={
            "minimal_height": 0.04,
            "tracked_brick": TrackedBrick.TO_GRASP,
        },
        weight=15.0,
    )

    object_goal_tracking = RewardTermCfg(
        func=brick_goal_distance,
        params={
            "std": 0.3,
            "minimal_height": 0.04,
            "command_name": "goal_pose",
            "tracked_brick": TrackedBrick.TO_GRASP,
        },
        weight=16.0,
    )

    object_goal_tracking_fine_grained = RewardTermCfg(
        func=brick_goal_distance,
        params={
            "std": 0.05,
            "minimal_height": 0.04,
            "command_name": "goal_pose",
            "tracked_brick": TrackedBrick.TO_GRASP,
        },
        weight=5.0,
    )

    # action penalty
    action_rate = RewardTermCfg(
        func=action_rate_l2,
        weight=-1e-4,
    )

    joint_vel = RewardTermCfg(
        func=joint_vel_l2,
        weight=-1e-4,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

@configclass
class TerminationsCfg:
    time_out = TerminationTermCfg(
        func=time_out,
        time_out=True,
    )

    object_dropping = TerminationTermCfg(
        func=brick_height_below_minimum,
        params={
            "minimum_height": -0.05,
            "tracked_brick": TrackedBrick.TO_GRASP,
        },
    )

@configclass
class FrankaEnvCfg(ManagerBasedRLEnvCfg):
    scene = FrankaSceneCfg(num_envs=16, env_spacing=2.5)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    commands = CommandsCfg()
    events = EventCfg()
    rewards = RewardsCfg()
    terminations= TerminationsCfg()

    def __post_init__(self):
        self.sim.device = "cpu" # lego_assemble supports cpu only
        self.viewer.eye = (4.5, 0.0, 6.0)
        self.viewer.lookat = (0.0, 0.0, 2.0)
        self.decimation = 4
        self.sim.dt = 0.005
        self.episode_length_s = 5.0

def main():
    env_cfg = FrankaEnvCfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)

    while simulation_app.is_running():
        with torch.inference_mode():
            joint_efforts = torch.randn_like(env.action_manager.action)
            env.step(joint_efforts)
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
