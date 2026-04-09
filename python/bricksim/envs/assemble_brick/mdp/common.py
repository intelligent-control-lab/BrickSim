import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_error_magnitude

from bricksim.mdp.connection_state import InterfacePairConnectionQuery, interface_pair_connection_state


def marker_pose_w(
    env,
    marker_cfg: SceneEntityCfg = SceneEntityCfg("marker_brick"),
) -> tuple[torch.Tensor, torch.Tensor]:
    marker = env.scene[marker_cfg.name]
    pos_w, quat_w = marker.get_world_poses(indices=list(range(env.num_envs)))
    dtype = env.scene.env_origins.dtype
    return (
        torch.as_tensor(pos_w, device=env.device, dtype=dtype),
        torch.as_tensor(quat_w, device=env.device, dtype=dtype),
    )


def gripper_is_open(env, robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    robot = env.scene[robot_cfg.name]
    joint_ids, _ = robot.find_joints(env.cfg.gripper_joint_names)
    joint_pos = robot.data.joint_pos[:, joint_ids]
    open_val = torch.tensor(env.cfg.gripper_open_val, device=env.device, dtype=joint_pos.dtype)
    return torch.all(torch.abs(joint_pos - open_val) <= env.cfg.gripper_threshold, dim=1)


def object_marker_pose_alignment(
    env,
    object_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    object = env.scene[object_cfg.name]
    target_pos_w, target_quat_w = marker_pose_w(env, target_cfg)
    pos_delta = object.data.root_pos_w - target_pos_w
    xy_dist = torch.linalg.vector_norm(pos_delta[:, :2], dim=1)
    rot_error = quat_error_magnitude(object.data.root_quat_w, target_quat_w)
    return pos_delta, xy_dist, rot_error


def connection_target_match(
    env,
    stud_if: int,
    hole_if: int,
    target_offset: tuple[int, int],
    target_yaw: int,
    object_cfg: SceneEntityCfg,
    stud_cfg: SceneEntityCfg = SceneEntityCfg("lego_baseplate"),
) -> torch.Tensor:
    connection_state = interface_pair_connection_state(
        env,
        InterfacePairConnectionQuery.make(
            stud_cfg=stud_cfg,
            hole_cfg=object_cfg,
            stud_if=stud_if,
            hole_if=hole_if,
        ),
    )
    return (
        connection_state.connected
        & (connection_state.offsets[:, 0] == target_offset[0])
        & (connection_state.offsets[:, 1] == target_offset[1])
        & (connection_state.yaws == target_yaw)
    )


def wrong_connection_to_target(
    env,
    stud_if: int,
    hole_if: int,
    target_offset: tuple[int, int],
    target_yaw: int,
    object_cfg: SceneEntityCfg,
    stud_cfg: SceneEntityCfg = SceneEntityCfg("lego_baseplate"),
) -> torch.Tensor:
    connection_state = interface_pair_connection_state(
        env,
        InterfacePairConnectionQuery.make(
            stud_cfg=stud_cfg,
            hole_cfg=object_cfg,
            stud_if=stud_if,
            hole_if=hole_if,
        ),
    )
    target_match = (
        (connection_state.offsets[:, 0] == target_offset[0])
        & (connection_state.offsets[:, 1] == target_offset[1])
        & (connection_state.yaws == target_yaw)
    )
    return connection_state.connected & ~target_match


def assemble_brick_goal_satisfied(
    env,
    stud_if: int,
    hole_if: int,
    target_offset: tuple[int, int],
    target_yaw: int,
    object_cfg: SceneEntityCfg,
    target_cfg: SceneEntityCfg,
    stud_cfg: SceneEntityCfg = SceneEntityCfg("lego_baseplate"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    pos_tol: float = 0.002,
    rot_tol: float = 0.08726646259971647,
) -> torch.Tensor:
    target_match = connection_target_match(
        env,
        stud_if=stud_if,
        hole_if=hole_if,
        target_offset=target_offset,
        target_yaw=target_yaw,
        object_cfg=object_cfg,
        stud_cfg=stud_cfg,
    )
    pos_delta, _, rot_error = object_marker_pose_alignment(env, object_cfg, target_cfg)
    pose_close = torch.linalg.vector_norm(pos_delta, dim=1) < pos_tol
    rot_close = rot_error < rot_tol
    return target_match & pose_close & rot_close & gripper_is_open(env, robot_cfg)
