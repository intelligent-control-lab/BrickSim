"""Isaac Lab manager-based environment for the assemble-brick task."""

import math
from collections.abc import Sequence

import torch
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from isaaclab.envs.mdp import (
    action_rate_l2,
    joint_pos_rel,
    joint_vel_l2,
    joint_vel_rel,
    last_action,
    reset_root_state_uniform,
    root_height_below_minimum,
    time_out,
)
from isaaclab.envs.mdp.actions.actions_cfg import (
    BinaryJointPositionActionCfg,
    DifferentialInverseKinematicsActionCfg,
)
from isaaclab.managers import (
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    SceneEntityCfg,
    TerminationTermCfg,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg, OffsetCfg
from isaaclab.sim import (
    DomeLightCfg,
    GroundPlaneCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from isaaclab.utils import configclass
from isaaclab_assets import ISAAC_NUCLEUS_DIR
from isaaclab_assets.robots.franka import FRANKA_PANDA_HIGH_PD_CFG
from isaaclab_tasks.manager_based.manipulation.lift.mdp.rewards import (
    object_ee_distance,
)
from isaaclab_tasks.manager_based.manipulation.place.mdp.observations import (
    object_poses_in_base_frame,
)
from isaaclab_tasks.manager_based.manipulation.stack.mdp import franka_stack_events
from isaaclab_tasks.manager_based.manipulation.stack.mdp.observations import (
    ee_frame_pose_in_base_frame,
    gripper_pos,
)

from bricksim.assets import FRANKA_ROBOT_USD_PATH
from bricksim.mdp.connection_thresholds import (
    configure_assembly_thresholds,
    configure_breakage_thresholds,
)
from bricksim.mdp.events import (
    reset_bricksim_managed,
    reset_scene_to_default_no_kinematic_vel,
    reset_to_connected_pose,
)
from bricksim.mdp.spawn import BrickPartCfg, MarkerBrickPartCfg

from .expert import AssembleBrickExpert
from .mdp.common import assemble_brick_goal_satisfied, wrong_connection_to_target
from .mdp.goal import AssembleBrickGoal
from .mdp.observations import (
    captured_hole_to_eef_obs,
    goal_target_match_obs,
    gripper_is_open_obs,
    marker_pose_in_robot_root_frame,
    object_grasped_obs,
    object_marker_pose_error,
    rigid_object_velocity_in_robot_root_frame,
    wrong_connection_obs,
)
from .mdp.rewards import (
    assemble_brick_success_bonus,
    grasp_bonus_from_object_grasped,
    lift_bonus_relative_to_target,
    object_insert_z,
    object_pre_insert_height,
    object_transport_xy,
    object_yaw_align,
)

GOAL = AssembleBrickGoal(
    stud_if=1,
    hole_if=0,
    offset=(5, 5),
    yaw=1,
    pos_tol=0.002,
    rot_tol=math.radians(5.0),
)


@configclass
class SceneCfg(InteractiveSceneCfg):
    """Scene assets for the assemble-brick task."""

    replicate_physics = False

    robot: ArticulationCfg = FRANKA_PANDA_HIGH_PD_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )
    robot.spawn.usd_path = str(FRANKA_ROBOT_USD_PATH)
    robot.spawn.variants = {"Physics": "Assemble"}
    robot.spawn.articulation_props.solver_position_iteration_count = 64
    robot.spawn.articulation_props.solver_velocity_iteration_count = 1
    robot.actuators["panda_hand"].effort_limit_sim = 15.0
    robot.actuators["panda_hand"].stiffness = 400.0
    robot.actuators["panda_hand"].damping = 80.0

    ee_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
        debug_vis=False,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
                name="end_effector",
                offset=OffsetCfg(pos=[0.0, 0.0, 0.1034]),
            ),
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger",
                name="tool_rightfinger",
                offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
            ),
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/panda_leftfinger",
                name="tool_leftfinger",
                offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
            ),
        ],
        visualizer_cfg=None,
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.5, 0, 0.003], rot=[0.707, 0, 0, 0.707]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
        ),
    )

    ground_plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.037]),
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
            dimensions=[32, 32, 1],
            color="Dark Gray",
            rigid_props=RigidBodyPropertiesCfg(
                kinematic_enabled=True, disable_gravity=True
            ),
        ),
    )

    lego_brick: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Brick",
        spawn=BrickPartCfg(
            dimensions=[2, 4, 3],
            color="Pink",
            rigid_props=RigidBodyPropertiesCfg(
                kinematic_enabled=False, disable_gravity=False
            ),
        ),
    )

    marker_brick: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/MarkerBrick",
        spawn=MarkerBrickPartCfg(
            dimensions=[2, 4, 3],
            color="Red",
        ),
    )


@configclass
class ActionsCfg:
    """Action terms for controlling the Franka arm and gripper."""

    arm_action = DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["panda_joint.*"],
        body_name="panda_hand",
        controller=DifferentialIKControllerCfg(
            command_type="pose", use_relative_mode=True, ik_method="dls"
        ),
        scale=1.0,
        body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(
            pos=[0.0, 0.0, 0.100]
        ),
    )

    gripper_action = BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["panda_finger.*"],
        open_command_expr={"panda_finger_.*": 0.04},
        close_command_expr={"panda_finger_.*": 0.0},
    )


@configclass
class ObservationsCfg:
    """Policy and critic observation groups for assemble-brick training."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observation terms exposed to the policy."""

        joint_pos = ObservationTermCfg(func=joint_pos_rel)
        joint_vel = ObservationTermCfg(func=joint_vel_rel)
        eef_pos = ObservationTermCfg(
            func=ee_frame_pose_in_base_frame, params={"return_key": "pos"}
        )
        eef_quat = ObservationTermCfg(
            func=ee_frame_pose_in_base_frame, params={"return_key": "quat"}
        )
        gripper_pos = ObservationTermCfg(func=gripper_pos)
        brick_pos = ObservationTermCfg(
            func=object_poses_in_base_frame,
            params={"object_cfg": SceneEntityCfg("lego_brick"), "return_key": "pos"},
        )
        brick_quat = ObservationTermCfg(
            func=object_poses_in_base_frame,
            params={"object_cfg": SceneEntityCfg("lego_brick"), "return_key": "quat"},
        )
        brick_lin_vel = ObservationTermCfg(
            func=rigid_object_velocity_in_robot_root_frame,
            params={"asset_cfg": SceneEntityCfg("lego_brick"), "return_key": "lin"},
        )
        brick_ang_vel = ObservationTermCfg(
            func=rigid_object_velocity_in_robot_root_frame,
            params={"asset_cfg": SceneEntityCfg("lego_brick"), "return_key": "ang"},
        )
        target_pos = ObservationTermCfg(
            func=marker_pose_in_robot_root_frame,
            params={"marker_cfg": SceneEntityCfg("marker_brick"), "return_key": "pos"},
        )
        target_quat = ObservationTermCfg(
            func=marker_pose_in_robot_root_frame,
            params={"marker_cfg": SceneEntityCfg("marker_brick"), "return_key": "quat"},
        )
        brick_to_target_pos = ObservationTermCfg(
            func=object_marker_pose_error,
            params={
                "object_cfg": SceneEntityCfg("lego_brick"),
                "marker_cfg": SceneEntityCfg("marker_brick"),
                "return_key": "pos",
            },
        )
        brick_to_target_quat = ObservationTermCfg(
            func=object_marker_pose_error,
            params={
                "object_cfg": SceneEntityCfg("lego_brick"),
                "marker_cfg": SceneEntityCfg("marker_brick"),
                "return_key": "quat",
            },
        )
        brick_grasped = ObservationTermCfg(
            func=object_grasped_obs,
            params={"object_cfg": SceneEntityCfg("lego_brick"), "diff_threshold": 0.04},
        )
        last_actions = ObservationTermCfg(func=last_action)

        def __post_init__(self):
            """Finalize policy observation-group settings."""
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObservationGroupCfg):
        """Observation terms exposed to the critic."""

        joint_pos = ObservationTermCfg(func=joint_pos_rel)
        joint_vel = ObservationTermCfg(func=joint_vel_rel)
        eef_pos = ObservationTermCfg(
            func=ee_frame_pose_in_base_frame, params={"return_key": "pos"}
        )
        eef_quat = ObservationTermCfg(
            func=ee_frame_pose_in_base_frame, params={"return_key": "quat"}
        )
        gripper_pos = ObservationTermCfg(func=gripper_pos)
        brick_pos = ObservationTermCfg(
            func=object_poses_in_base_frame,
            params={"object_cfg": SceneEntityCfg("lego_brick"), "return_key": "pos"},
        )
        brick_quat = ObservationTermCfg(
            func=object_poses_in_base_frame,
            params={"object_cfg": SceneEntityCfg("lego_brick"), "return_key": "quat"},
        )
        brick_lin_vel = ObservationTermCfg(
            func=rigid_object_velocity_in_robot_root_frame,
            params={"asset_cfg": SceneEntityCfg("lego_brick"), "return_key": "lin"},
        )
        brick_ang_vel = ObservationTermCfg(
            func=rigid_object_velocity_in_robot_root_frame,
            params={"asset_cfg": SceneEntityCfg("lego_brick"), "return_key": "ang"},
        )
        target_pos = ObservationTermCfg(
            func=marker_pose_in_robot_root_frame,
            params={"marker_cfg": SceneEntityCfg("marker_brick"), "return_key": "pos"},
        )
        target_quat = ObservationTermCfg(
            func=marker_pose_in_robot_root_frame,
            params={"marker_cfg": SceneEntityCfg("marker_brick"), "return_key": "quat"},
        )
        brick_to_target_pos = ObservationTermCfg(
            func=object_marker_pose_error,
            params={
                "object_cfg": SceneEntityCfg("lego_brick"),
                "marker_cfg": SceneEntityCfg("marker_brick"),
                "return_key": "pos",
            },
        )
        brick_to_target_quat = ObservationTermCfg(
            func=object_marker_pose_error,
            params={
                "object_cfg": SceneEntityCfg("lego_brick"),
                "marker_cfg": SceneEntityCfg("marker_brick"),
                "return_key": "quat",
            },
        )
        brick_grasped = ObservationTermCfg(
            func=object_grasped_obs,
            params={"object_cfg": SceneEntityCfg("lego_brick"), "diff_threshold": 0.04},
        )
        last_actions = ObservationTermCfg(func=last_action)
        goal_target_match = ObservationTermCfg(
            func=goal_target_match_obs,
            params={
                "stud_if": GOAL.stud_if,
                "hole_if": GOAL.hole_if,
                "target_offset": GOAL.offset,
                "target_yaw": GOAL.yaw,
                "object_cfg": SceneEntityCfg("lego_brick"),
            },
        )
        wrong_connection = ObservationTermCfg(
            func=wrong_connection_obs,
            params={
                "stud_if": GOAL.stud_if,
                "hole_if": GOAL.hole_if,
                "target_offset": GOAL.offset,
                "target_yaw": GOAL.yaw,
                "object_cfg": SceneEntityCfg("lego_brick"),
            },
        )
        gripper_open = ObservationTermCfg(func=gripper_is_open_obs)
        captured_hole_to_eef_pos = ObservationTermCfg(
            func=captured_hole_to_eef_obs, params={"return_key": "pos"}
        )
        captured_hole_to_eef_quat = ObservationTermCfg(
            func=captured_hole_to_eef_obs, params={"return_key": "quat"}
        )

        def __post_init__(self):
            """Finalize critic observation-group settings."""
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
    """Event terms for resets and startup initialization."""

    reset_all = EventTermCfg(
        func=reset_scene_to_default_no_kinematic_vel,
        mode="reset",
    )

    init_franka_arm_pose = EventTermCfg(
        func=franka_stack_events.set_default_joint_pose,
        mode="startup",
        params={
            "default_pose": [
                0.0444,
                -0.1894,
                -0.1107,
                -2.5148,
                0.0044,
                2.3775,
                0.6952,
                0.0400,
                0.0400,
            ],
        },
    )

    reset_bricksim_managed = EventTermCfg(
        func=reset_bricksim_managed,
        mode="reset",
    )

    randomize_franka_joint_state = EventTermCfg(
        func=franka_stack_events.randomize_joint_by_gaussian_offset,
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

    reset_marker_brick_pose = EventTermCfg(
        func=reset_to_connected_pose,
        mode="reset",
        params={
            "moved_cfg": SceneEntityCfg("marker_brick"),
            "reference_brick_cfg": SceneEntityCfg("lego_baseplate"),
            "moved_side": "hole",
            "stud_if": GOAL.stud_if,
            "hole_if": GOAL.hole_if,
            "offset": GOAL.offset,
            "yaw": GOAL.yaw,
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the assemble-brick task."""

    reach_brick = RewardTermCfg(
        func=object_ee_distance,
        params={"std": 0.08, "object_cfg": SceneEntityCfg("lego_brick")},
        weight=1.0,
    )

    grasp_bonus = RewardTermCfg(
        func=grasp_bonus_from_object_grasped,
        params={"object_cfg": SceneEntityCfg("lego_brick"), "diff_threshold": 0.04},
        weight=2.0,
    )

    lift_bonus = RewardTermCfg(
        func=lift_bonus_relative_to_target,
        params={
            "object_cfg": SceneEntityCfg("lego_brick"),
            "target_cfg": SceneEntityCfg("marker_brick"),
            "lift_height": 0.03,
        },
        weight=2.0,
    )

    transport_xy = RewardTermCfg(
        func=object_transport_xy,
        params={
            "std": 0.04,
            "object_cfg": SceneEntityCfg("lego_brick"),
            "target_cfg": SceneEntityCfg("marker_brick"),
        },
        weight=4.0,
    )

    yaw_align = RewardTermCfg(
        func=object_yaw_align,
        params={
            "std": 0.20,
            "object_cfg": SceneEntityCfg("lego_brick"),
            "target_cfg": SceneEntityCfg("marker_brick"),
        },
        weight=2.0,
    )

    pre_insert_height = RewardTermCfg(
        func=object_pre_insert_height,
        params={
            "std": 0.01,
            "object_cfg": SceneEntityCfg("lego_brick"),
            "target_cfg": SceneEntityCfg("marker_brick"),
            "target_height_offset": 0.02,
            "loose_xy_threshold": 0.03,
            "loose_rot_threshold": 0.25,
        },
        weight=3.0,
    )

    insert_z = RewardTermCfg(
        func=object_insert_z,
        params={
            "std": 0.006,
            "object_cfg": SceneEntityCfg("lego_brick"),
            "target_cfg": SceneEntityCfg("marker_brick"),
            "tight_xy_threshold": 0.012,
            "tight_rot_threshold": 0.12,
        },
        weight=4.0,
    )

    success_bonus = RewardTermCfg(
        func=assemble_brick_success_bonus,
        params={
            "stud_if": GOAL.stud_if,
            "hole_if": GOAL.hole_if,
            "target_offset": GOAL.offset,
            "target_yaw": GOAL.yaw,
            "object_cfg": SceneEntityCfg("lego_brick"),
            "target_cfg": SceneEntityCfg("marker_brick"),
            "pos_tol": GOAL.pos_tol,
            "rot_tol": GOAL.rot_tol,
        },
        weight=1000.0,
    )

    action_rate = RewardTermCfg(
        func=action_rate_l2,
        weight=-1e-4,
    )

    joint_vel = RewardTermCfg(
        func=joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    """Termination terms for success, timeout, and failure states."""

    time_out = TerminationTermCfg(func=time_out, time_out=True)

    success = TerminationTermCfg(
        func=assemble_brick_goal_satisfied,
        params={
            "stud_if": GOAL.stud_if,
            "hole_if": GOAL.hole_if,
            "target_offset": GOAL.offset,
            "target_yaw": GOAL.yaw,
            "object_cfg": SceneEntityCfg("lego_brick"),
            "target_cfg": SceneEntityCfg("marker_brick"),
            "pos_tol": GOAL.pos_tol,
            "rot_tol": GOAL.rot_tol,
        },
    )

    wrong_connection = TerminationTermCfg(
        func=wrong_connection_to_target,
        params={
            "stud_if": GOAL.stud_if,
            "hole_if": GOAL.hole_if,
            "target_offset": GOAL.offset,
            "target_yaw": GOAL.yaw,
            "object_cfg": SceneEntityCfg("lego_brick"),
        },
    )

    brick_dropped = TerminationTermCfg(
        func=root_height_below_minimum,
        params={"minimum_height": -0.02, "asset_cfg": SceneEntityCfg("lego_brick")},
    )


@configclass
class AssembleBrickEnvCfg(ManagerBasedRLEnvCfg):
    """Full Isaac Lab environment config for the assemble-brick task."""

    scene = SceneCfg(num_envs=16, env_spacing=2.5)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    commands = None
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
    events = EventCfg()
    curriculum = None

    def __post_init__(self):
        """Finalize simulation, viewer, and gripper task settings."""
        configure_assembly_thresholds(enabled=True)
        configure_breakage_thresholds(enabled=False)
        self.sim.device = "cpu"
        self.sim.use_fabric = False
        self.decimation = 1
        self.episode_length_s = 12.0
        self.sim.dt = 0.01
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625

        # IsaacLab viewer defaults for the assemble-brick task.
        # Units: meters in world frame. The look-at point is inferred from the
        # saved viewport pose.
        self.viewer.eye = (0.86535, 0.47963, 0.24637)
        self.viewer.lookat = (
            0.21471784579495856,
            -0.2155713921141228,
            -0.05919967178876398,
        )

        self.gripper_joint_names = ["panda_finger_.*"]
        self.gripper_open_val = 0.04
        self.gripper_threshold = 0.005


class AssembleBrickEnv(ManagerBasedRLEnv):
    """Manager-based RL environment with an attached scripted expert."""

    cfg: AssembleBrickEnvCfg

    def __init__(
        self, cfg: AssembleBrickEnvCfg, render_mode: str | None = None, **kwargs
    ):
        """Initialize the environment and attach the assemble-brick expert."""
        super().__init__(cfg=cfg, render_mode=render_mode, **kwargs)
        self._expert = AssembleBrickExpert(
            self,
            stud_if=GOAL.stud_if,
            hole_if=GOAL.hole_if,
            target_offset=GOAL.offset,
            target_yaw=GOAL.yaw,
        )

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        self._expert.reset(env_ids)

    def compute_expert_actions(self) -> torch.Tensor:
        """Compute one batched action tensor from the scripted expert.

        Returns:
            Expert actions with shape ``(num_envs, action_dim)``.
        """
        return self._expert.compute_actions()
