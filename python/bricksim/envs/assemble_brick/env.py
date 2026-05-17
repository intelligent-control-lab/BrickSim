"""Isaac Lab manager-based environment for the assemble-brick task."""

import math

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp import (
    action_rate_l2,
    image,
    joint_vel_l2,
    reset_root_state_uniform,
    time_out,
)
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.managers import (
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    SceneEntityCfg,
    TerminationTermCfg,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg, OffsetCfg, TiledCameraCfg
from isaaclab.sim import (
    DomeLightCfg,
    GroundPlaneCfg,
    PinholeCameraCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from isaaclab.utils import configclass
from isaaclab_assets import ISAAC_NUCLEUS_DIR
from isaaclab_tasks.manager_based.manipulation.stack.mdp.franka_stack_events import (
    randomize_joint_by_gaussian_offset,
)
from isaaclab_tasks.manager_based.manipulation.stack.mdp.observations import (
    ee_frame_pose_in_base_frame,
)

from bricksim.assets import FR3_ROBOT_USD_PATH
from bricksim.mdp.brick_part import BrickPartCfg
from bricksim.mdp.connection_thresholds import (
    configure_assembly_thresholds,
    configure_breakage_thresholds,
)
from bricksim.mdp.events import (
    reset_bricksim_managed,
    reset_scene_to_default_no_kinematic_vel,
)
from bricksim.mdp.hysteresis_binary_joint_action import HysteresisBinaryJointActionCfg

from .mdp.commands import AssembleBrickCommandCfg
from .mdp.observations import (
    franka_gripper_speed,
    franka_gripper_width,
    obs_command_connection_created,
    obs_command_target_pose,
    obs_moving_brick_dimensions,
    obs_moving_brick_grasped,
    obs_moving_brick_pose,
)
from .mdp.rewards import (
    reward_grasp_bonus,
    reward_insert_z,
    reward_lift_bonus,
    reward_pre_insert_height,
    reward_reach_brick,
    reward_success_bonus,
    reward_transport_xy,
    reward_yaw_align,
)
from .mdp.terminations import (
    brick_height_below_threshold,
    non_target_connection_formed,
    target_connection_formed_and_gripper_open,
)

# FR3 TCP / pad-center offset relative to fr3_hand.
# Units: meters. Quaternion storage: wxyz.
FR3_HAND_TCP_OFFSET_POS = (0.0, 0.0, 0.1034)
FR3_HAND_TCP_OFFSET_ROT = (1.0, 0.0, 0.0, 0.0)

ARM_ACTION_SCALE = (0.2, 0.2, 0.2, 1.0, 1.0, 1.0)


@configclass
class SceneCfg(InteractiveSceneCfg):
    """Scene assets for the assemble-brick task."""

    replicate_physics = False

    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=UsdFileCfg(usd_path=str(FR3_ROBOT_USD_PATH)),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "fr3_joint1": 0.0444,
                "fr3_joint2": -0.1894,
                "fr3_joint3": -0.1107,
                "fr3_joint4": -2.5148,
                "fr3_joint5": 0.0044,
                "fr3_joint6": 2.3775,
                "fr3_joint7": 0.6952,
                "fr3_finger_joint.*": 0.04,
            },
        ),
        actuators={
            "fr3_arm": ImplicitActuatorCfg(
                joint_names_expr=["fr3_joint[1-7]"],
                stiffness=None,
                damping=None,
            ),
            "fr3_hand": ImplicitActuatorCfg(
                joint_names_expr=["fr3_finger_joint.*"],
                effort_limit_sim=15.0,
                stiffness=None,
                damping=None,
            ),
        },
    )

    ee_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/fr3_link0",
        debug_vis=False,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/fr3_hand",
                name="end_effector",
                offset=OffsetCfg(
                    pos=FR3_HAND_TCP_OFFSET_POS,
                    rot=FR3_HAND_TCP_OFFSET_ROT,
                ),
            ),
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/fr3_rightfinger",
                name="tool_rightfinger",
                offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
            ),
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/fr3_leftfinger",
                name="tool_leftfinger",
                offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
            ),
        ],
    )

    hand_camera: TiledCameraCfg | None = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/D435/camera_color_optical_frame/color_camera",
        height=720,
        width=1280,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=PinholeCameraCfg.from_intrinsic_matrix(
            # Nominal D435 color intrinsics at 1280x720.
            # Storage: row-major 3x3 pinhole matrix.
            intrinsic_matrix=[
                924.277380,
                0.0,
                640.0,
                0.0,
                925.738464,
                360.0,
                0.0,
                0.0,
                1.0,
            ],
            width=1280,
            height=720,
            clipping_range=(0.001, 3.0),
        ),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.5, 0, 0.003), rot=(0.707, 0, 0, 0.707)
        ),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
        ),
    )

    ground_plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, -1.037)),
        spawn=GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=DomeLightCfg(color=(1.0, 1.0, 1.0), intensity=3000.0),
    )

    lego_baseplate: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Baseplate",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.5, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0)
        ),
        spawn=BrickPartCfg(
            dimensions=(32, 32, 1),
            color="Dark Gray",
            rigid_props=RigidBodyPropertiesCfg(
                kinematic_enabled=True, disable_gravity=True
            ),
        ),
    )

    lego_brick: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Brick",
        spawn=BrickPartCfg(
            dimensions=(2, 4, 3),
            color="Pink",
            rigid_props=RigidBodyPropertiesCfg(
                kinematic_enabled=False, disable_gravity=False
            ),
        ),
    )


@configclass
class ActionsCfg:
    """Action terms for normalized FR3 arm and gripper commands.

    Callers should send finite actions in ``[-1, 1]`` and are responsible for
    clipping policy outputs before calling ``env.step()``.
    """

    arm_action = DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["fr3_joint[1-7]"],
        body_name="fr3_hand",
        controller=DifferentialIKControllerCfg(
            command_type="pose", use_relative_mode=True, ik_method="dls"
        ),
        scale=ARM_ACTION_SCALE,
        body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(
            pos=FR3_HAND_TCP_OFFSET_POS,
            rot=FR3_HAND_TCP_OFFSET_ROT,
        ),
    )

    gripper_action = HysteresisBinaryJointActionCfg(
        asset_name="robot",
        joint_names=["fr3_finger_joint.*"],
        open_command_expr={"fr3_finger_joint.*": 0.04},
        close_command_expr={"fr3_finger_joint.*": 0.0},
        open_thresholds=0.2,
        close_thresholds=-0.2,
        initial_closed=False,
    )


@configclass
class CommandsCfg:
    """Command terms for sampled assembly goals."""

    assembly_goal: AssembleBrickCommandCfg = AssembleBrickCommandCfg(
        stud_brick="lego_baseplate",
        stud_brick_iface=1,
        hole_brick="lego_brick",
        hole_brick_iface=0,
        moving_brick_type="hole",
        goal_marker_visualizer_prim_path="/Visuals/Command/assembly_goal",
        goal_marker_color="Red",
        debug_vis=False,
    )


@configclass
class ObservationsCfg:
    """Policy and privileged observation groups for assemble-brick training."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observation terms exposed to the policy."""

        ee_frame_pose = ObservationTermCfg(func=ee_frame_pose_in_base_frame)
        gripper_width = ObservationTermCfg(func=franka_gripper_width)
        gripper_speed = ObservationTermCfg(func=franka_gripper_speed)
        brick_pose = ObservationTermCfg(func=obs_moving_brick_pose)
        target_pose = ObservationTermCfg(func=obs_command_target_pose)

    @configclass
    class PrivilegedCfg(PolicyCfg):
        """Privileged observation terms exposed to training."""

        concatenate_terms = False

        brick_grasped = ObservationTermCfg(func=obs_moving_brick_grasped)
        connection_created = ObservationTermCfg(func=obs_command_connection_created)
        brick_dimensions = ObservationTermCfg(func=obs_moving_brick_dimensions)

    @configclass
    class ImagesCfg(ObservationGroupCfg):
        """Rendered D435 camera observation terms."""

        concatenate_terms = False

        hand_color: ObservationTermCfg = ObservationTermCfg(
            func=image,
            params={
                "sensor_cfg": SceneEntityCfg("hand_camera"),
                "data_type": "rgb",
                "normalize": False,
            },
        )
        hand_depth: ObservationTermCfg | None = ObservationTermCfg(
            func=image,
            params={
                "sensor_cfg": SceneEntityCfg("hand_camera"),
                "data_type": "distance_to_image_plane",
                "normalize": False,
            },
        )

    policy: PolicyCfg = PolicyCfg()
    privileged: PrivilegedCfg = PrivilegedCfg()
    images: ImagesCfg | None = ImagesCfg()


@configclass
class EventCfg:
    """Event terms for resets and startup initialization."""

    reset_all = EventTermCfg(
        func=reset_scene_to_default_no_kinematic_vel,
        mode="reset",
    )

    clear_connections = EventTermCfg(
        func=reset_bricksim_managed,
        mode="reset",
    )

    randomize_fr3_joint_state = EventTermCfg(
        func=randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.02,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    reset_brick_pose = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("lego_brick"),
            "pose_range": {
                "x": (0.36, 0.56),
                "y": (0.16, 0.32),
                "yaw": (0.0, math.tau),
            },
            "velocity_range": {},
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the assemble-brick task."""

    reach_brick = RewardTermCfg(func=reward_reach_brick, weight=5.0)
    grasp_bonus = RewardTermCfg(func=reward_grasp_bonus, weight=2.0)
    lift_bonus = RewardTermCfg(func=reward_lift_bonus, weight=2.0)
    transport_xy = RewardTermCfg(func=reward_transport_xy, weight=4.0)
    yaw_align = RewardTermCfg(func=reward_yaw_align, weight=2.0)
    pre_insert_height = RewardTermCfg(func=reward_pre_insert_height, weight=3.0)
    insert_z = RewardTermCfg(func=reward_insert_z, weight=4.0)
    success_bonus = RewardTermCfg(func=reward_success_bonus, weight=1000.0)
    action_rate = RewardTermCfg(func=action_rate_l2, weight=-1e-5)
    joint_vel = RewardTermCfg(func=joint_vel_l2, weight=-1e-5)


@configclass
class TerminationsCfg:
    """Termination terms for success, timeout, and failure states."""

    timeout = TerminationTermCfg(func=time_out, time_out=True)
    success = TerminationTermCfg(func=target_connection_formed_and_gripper_open)
    wrong_connection = TerminationTermCfg(func=non_target_connection_formed)
    brick_dropped = TerminationTermCfg(func=brick_height_below_threshold)


@configclass
class AssembleBrickBaseEnvCfg(ManagerBasedRLEnvCfg):
    """Base environment config for the assemble-brick task. Do not use this directly."""

    scene: SceneCfg = SceneCfg(num_envs=1, env_spacing=5.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    # The following gripper parameters are used by
    # isaaclab_tasks.manager_based.manipulation.place.mdp.observations.object_grasped
    gripper_joint_names = ["fr3_finger_joint.*"]
    gripper_open_val = 0.04
    gripper_threshold = 0.005

    def __post_init__(self):
        """Finalize simulation, viewer, and gripper task settings."""
        configure_assembly_thresholds(enabled=True)
        configure_breakage_thresholds(enabled=False)
        self.sim.device = "cpu"
        self.sim.use_fabric = False
        self.decimation = 6
        self.episode_length_s = 12.0
        self.sim.dt = 1 / 60
        self.sim.render_interval = self.decimation
        self.num_rerenders_on_reset = 3
        self.viewer.eye = (0.86535, 0.47963, 0.24637)
        self.viewer.lookat = (
            0.21471784579495856,
            -0.2155713921141228,
            -0.05919967178876398,
        )


@configclass
class AssembleBrickEnvCfg(AssembleBrickBaseEnvCfg):
    """State-based environment config for the assemble-brick task."""

    def __post_init__(self):
        """Remove image observations."""
        super().__post_init__()
        self.scene.hand_camera = None
        self.observations.images = None
        self.num_rerenders_on_reset = 0


@configclass
class AssembleBrickRGBDEnvCfg(AssembleBrickBaseEnvCfg):
    """Environment config for the assemble-brick task with RGB-D observation."""

    pass


@configclass
class AssembleBrickRGBEnvCfg(AssembleBrickBaseEnvCfg):
    """Environment config for the assemble-brick task with RGB observation."""

    def __post_init__(self):
        """Remove depth observations."""
        super().__post_init__()

        hand_camera = self.scene.hand_camera
        assert hand_camera is not None
        hand_camera.data_types = ["rgb"]

        images = self.observations.images
        assert images is not None
        images.hand_depth = None
