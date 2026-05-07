"""Isaac Lab manager-based environment for the assemble-brick task."""

import math

import torch
from gymnasium.spaces import Box
from gymnasium.vector.utils import batch_space
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg, VecEnvStepReturn
from isaaclab.envs.mdp import (
    action_rate_l2,
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
from isaaclab_tasks.manager_based.manipulation.stack.mdp import franka_stack_events
from isaaclab_tasks.manager_based.manipulation.stack.mdp.observations import (
    ee_frame_pose_in_base_frame,
)

from bricksim.assets import FRANKA_ROBOT_USD_PATH
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

# Franka TCP / pad-center offset relative to panda_hand.
# Units: meters. Quaternion storage: wxyz.
FRANKA_HAND_TCP_OFFSET_POS = (0.0, 0.0, 0.1034)
FRANKA_HAND_TCP_OFFSET_ROT = (1.0, 0.0, 0.0, 0.0)


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
                offset=OffsetCfg(
                    pos=FRANKA_HAND_TCP_OFFSET_POS,
                    rot=FRANKA_HAND_TCP_OFFSET_ROT,
                ),
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
    """Action terms for controlling the Franka arm and gripper."""

    arm_action = DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["panda_joint.*"],
        body_name="panda_hand",
        controller=DifferentialIKControllerCfg(
            command_type="pose", use_relative_mode=True, ik_method="dls"
        ),
        scale=(0.2, 0.2, 0.2, 1.0, 1.0, 1.0),
        body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(
            pos=FRANKA_HAND_TCP_OFFSET_POS,
            rot=FRANKA_HAND_TCP_OFFSET_ROT,
        ),
    )

    gripper_action = HysteresisBinaryJointActionCfg(
        asset_name="robot",
        joint_names=["panda_finger.*"],
        open_command_expr={"panda_finger_.*": 0.04},
        close_command_expr={"panda_finger_.*": 0.0},
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
        goals=((5, 5, 1),),
        goal_marker_visualizer_prim_path="/Visuals/Command/assembly_goal",
        goal_marker_color="Red",
        debug_vis=True,
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

    policy: PolicyCfg = PolicyCfg()
    privileged: PrivilegedCfg = PrivilegedCfg()


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
class AssembleBrickEnvCfg(ManagerBasedRLEnvCfg):
    """Full Isaac Lab environment config for the assemble-brick task."""

    scene = SceneCfg(num_envs=16, env_spacing=2.5)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    commands = CommandsCfg()
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

        # Used by saaclab_tasks/manager_based/manipulation/stack/mdp/observations.py:282
        self.gripper_joint_names = ["panda_finger_.*"]
        self.gripper_open_val = 0.04
        self.gripper_threshold = 0.005


class AssembleBrickEnv(ManagerBasedRLEnv):
    """Manager-based RL environment for the assemble-brick task."""

    cfg: AssembleBrickEnvCfg

    def __init__(
        self,
        cfg: AssembleBrickEnvCfg,
        render_mode: str | None = None,
        **kwargs: object,
    ) -> None:
        """Initialize the environment and expose normalized action bounds.

        Args:
            cfg: Environment configuration.
            render_mode: Render mode forwarded to Isaac Lab.
            **kwargs: Additional Isaac Lab environment constructor arguments.
        """
        super().__init__(cfg=cfg, render_mode=render_mode, **kwargs)
        assert self.single_action_space.dtype is not None
        self.single_action_space = Box(
            low=-1.0,
            high=1.0,
            shape=self.single_action_space.shape,
            dtype=self.single_action_space.dtype.type,
        )
        self.action_space = batch_space(self.single_action_space, self.num_envs)

    def step(self, action: torch.Tensor) -> VecEnvStepReturn:
        """Clamp finite actions before stepping the Isaac Lab environment.

        Args:
            action: Batched action tensor with shape ``(num_envs, action_dim)``.

        Returns:
            Observations, rewards, terminated flags, truncated flags, and extras.

        Raises:
            ValueError: If any action element is NaN or infinite.
        """
        finite_mask = torch.isfinite(action)
        if not finite_mask.all():
            invalid_count = int((~finite_mask).sum().item())
            raise ValueError(
                f"Non-finite values detected in actions: "
                f"{invalid_count} / {action.numel()} elements are invalid."
            )
        action_low = torch.as_tensor(
            self.single_action_space.low,
            device=action.device,
            dtype=action.dtype,
        )
        action_high = torch.as_tensor(
            self.single_action_space.high,
            device=action.device,
            dtype=action.dtype,
        )
        clamped_action = torch.clamp(action, min=action_low, max=action_high)
        return super().step(clamped_action)

    def compute_expert_actions(self) -> torch.Tensor:
        """Raise until the command-aware expert is implemented.

        Raises:
            NotImplementedError: Always, because the fixed-goal expert was
                removed and the command-aware expert has not been added yet.
        """
        raise NotImplementedError(
            "Assemble-brick expert was removed; command-aware expert actions "
            "will be added back later."
        )
