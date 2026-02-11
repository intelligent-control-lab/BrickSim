from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp import (BinaryJointPositionActionCfg,
                               JointPositionActionCfg, joint_pos_rel,
                               joint_vel_rel, last_action, time_out)
from isaaclab.managers import (ObservationGroupCfg, ObservationTermCfg,
                               TerminationTermCfg)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg, OffsetCfg
from isaaclab.sim import (DomeLightCfg, GroundPlaneCfg, RigidBodyPropertiesCfg,
                          UsdFileCfg)
from isaaclab.utils import configclass
from isaaclab_assets import FRANKA_PANDA_CFG, ISAAC_NUCLEUS_DIR
from isaaclab_tasks.manager_based.manipulation.stack.mdp import (ee_frame_pos,
                                                                 ee_frame_quat)
from lego_assemble.mdp.spawn import BrickPartCfg


@configclass
class SceneCfg(InteractiveSceneCfg):

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
        visualizer_cfg=None,
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

    lego_baseplate: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Baseplate",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=BrickPartCfg(
            dimensions=[32, 32, 1],
            color="Dark Gray",
            rigid_props=RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
        ),
    )

    brick: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Brick",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.08), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=BrickPartCfg(
            dimensions=[2, 4, 3],
            color="Pink",
            rigid_props=RigidBodyPropertiesCfg(kinematic_enabled=False, disable_gravity=False),
        ),
    )

@configclass
class ActionsCfg:

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
    pass

@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        joint_pos_rel = ObservationTermCfg(
            func=joint_pos_rel,
        )

        joint_vel_rel = ObservationTermCfg(
            func=joint_vel_rel,
        )

        eef_pos = ObservationTermCfg(
            func=ee_frame_pos,
        )

        eef_quat = ObservationTermCfg(
            func=ee_frame_quat,
        )

        last_actions = ObservationTermCfg(
            func=last_action,
        )

    policy: PolicyCfg = PolicyCfg()

@configclass
class EventCfg:
    pass

@configclass
class RewardsCfg:
    pass

@configclass
class TerminationsCfg:
    time_out = TerminationTermCfg(
        func=time_out,
        time_out=True,
    )

@configclass
class CurriculumCfg:
    pass

@configclass
class AssembleBrickEnvCfg(ManagerBasedRLEnvCfg):
    scene = SceneCfg(num_envs=16, env_spacing=2.5)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    commands = CommandsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
    events = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        self.sim.device = "cpu"
        self.decimation = 2
        self.episode_length_s = 5.0
        self.sim.dt = 0.01
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
