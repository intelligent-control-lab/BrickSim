import math
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp import BinaryJointPositionActionCfg, JointEffortActionCfg, JointPositionActionCfg, action_rate_l2, generated_commands, joint_vel_l2, last_action, modify_reward_weight, reset_joints_by_offset, joint_pos_rel, joint_vel_rel, time_out
from isaaclab.managers import CurriculumTermCfg, EventTermCfg, ObservationGroupCfg, ObservationTermCfg, RewardTermCfg, SceneEntityCfg, TerminationTermCfg
from isaaclab.utils import configclass
from isaaclab.sim import GroundPlaneCfg, DomeLightCfg, UsdFileCfg
from isaaclab_assets import FRANKA_PANDA_CFG, ISAAC_NUCLEUS_DIR
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg, OffsetCfg
from lego_assemble.mdp.events import reset_and_spawn_brick
from lego_assemble.mdp.observations import brick_pose_in_robot_root_frame
from lego_assemble.mdp.rewards import brick_ee_distance, brick_goal_distance, brick_is_lifted, brick_upright
from lego_assemble.mdp.terminations import brick_height_below_minimum
from lego_assemble.mdp.tracking import TrackedBrick
from lego_assemble.mdp.pose_command import BrickUniformPoseCommandCfg

# Reference: isaaclab_tasks/manager_based/manipulation/lift/lift_env_cfg.py

@configclass
class SceneCfg(InteractiveSceneCfg):

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
    # joint_efforts = JointEffortActionCfg(
    #     asset_name="robot",
    #     joint_names=["panda_joint.*"],
    # )

    arm_action = JointPositionActionCfg(
        asset_name="robot",
        joint_names=["panda_joint.*"],
        scale=0.5,
        use_default_offset=True,
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
            func=brick_pose_in_robot_root_frame,
            params={
                "tracked_brick": TrackedBrick.TO_GRASP,
            }
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
            "minimal_height": 0.01,
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

    brick_upright = RewardTermCfg(
        func=brick_upright,
        params={
            "minimal_height": 0.01,
            "tracked_brick": TrackedBrick.TO_GRASP,
        },
        weight=1.0,
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
class CurriculumCfg:
    action_rate = CurriculumTermCfg(
        func=modify_reward_weight,
        params={
            "term_name": "action_rate",
            "weight": -1e-1,
            "num_steps": 10000,
        },
    )

    joint_vel = CurriculumTermCfg(
        func=modify_reward_weight,
        params={
            "term_name": "joint_vel",
            "weight": -1e-1,
            "num_steps": 10000,
        },
    )

@configclass
class LiftBrickEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene = SceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations = ObservationsCfg()
    actions = ActionsCfg()
    commands = CommandsCfg()
    # MDP settings
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
    events = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        self.sim.device = "cpu" # lego_assemble supports cpu only

        # general settings
        self.decimation = 2
        self.episode_length_s = 5.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = self.decimation

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
