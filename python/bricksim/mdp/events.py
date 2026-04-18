"""Event terms for BrickSim-managed Isaac Lab environments."""

from typing import Literal

import isaaclab.utils.math as math_utils
import torch
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.sim.views import XformPrimView

from bricksim.core import compute_connection_transform, deallocate_all_managed

from .brick_part import (
    resolve_brick_rigid_object,
    scene_entity_brick_part_dimensions,
)
from .isaaclab_shims import (
    set_articulation_joint_position_target,
    set_articulation_joint_velocity_target,
    write_articulation_joint_state_to_sim,
    write_asset_root_pose_to_sim,
    write_asset_root_velocity_to_sim,
    write_deformable_nodal_state_to_sim,
)


# TODO: clean up _interface_pose & _compute_visual_connection_transform
def _interface_pose(
    dimensions: tuple[int, int, int],
    side: Literal["stud", "hole"],
    *,
    device: torch.device,
    dtype: torch.dtype,
) -> tuple[torch.Tensor, torch.Tensor]:
    length = float(dimensions[0]) * 0.008
    width = float(dimensions[1]) * 0.008
    height = float(dimensions[2]) * 0.0032
    z = height if side == "stud" else 0.0
    pos = torch.tensor([[-length / 2.0, -width / 2.0, z]], device=device, dtype=dtype)
    quat = torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=device, dtype=dtype)
    return pos, quat


def _compute_visual_connection_transform(
    stud_dimensions: tuple[int, int, int],
    hole_dimensions: tuple[int, int, int],
    offset: tuple[int, int],
    yaw: int,
    *,
    device: torch.device,
    dtype: torch.dtype,
) -> tuple[torch.Tensor, torch.Tensor]:
    stud_if_pos, stud_if_quat = _interface_pose(
        stud_dimensions, "stud", device=device, dtype=dtype
    )
    hole_if_pos, hole_if_quat = _interface_pose(
        hole_dimensions, "hole", device=device, dtype=dtype
    )

    yaw_angle = torch.tensor(
        [float(yaw) * (torch.pi / 2.0)], device=device, dtype=dtype
    )
    zero = torch.zeros_like(yaw_angle)
    si_hi_quat = math_utils.quat_from_euler_xyz(zero, zero, yaw_angle)
    si_hi_pos = torch.tensor(
        [[float(offset[0]) * 0.008, float(offset[1]) * 0.008, 0.0]],
        device=device,
        dtype=dtype,
    )

    hole_inv_pos, hole_inv_quat = math_utils.subtract_frame_transforms(
        hole_if_pos, hole_if_quat
    )
    stud_hi_pos, stud_hi_quat = math_utils.combine_frame_transforms(
        stud_if_pos, stud_if_quat, si_hi_pos, si_hi_quat
    )
    return math_utils.combine_frame_transforms(
        stud_hi_pos, stud_hi_quat, hole_inv_pos, hole_inv_quat
    )


def _rigid_object_is_kinematic(rigid_object: RigidObject) -> bool:
    spawn_cfg = rigid_object.cfg.spawn
    rigid_props = getattr(spawn_cfg, "rigid_props", None)
    return rigid_props is not None and rigid_props.kinematic_enabled is True


def _set_kinematic_root_velocity_cache(
    rigid_object: RigidObject,
    root_velocity: torch.Tensor,
    env_ids: torch.Tensor | None,
) -> None:
    if env_ids is None:
        local_env_ids = slice(None)
    else:
        local_env_ids = env_ids

    data = rigid_object._data
    sim_timestamp = data._sim_timestamp

    if data._root_com_vel_w.data is not None:
        data._root_com_vel_w.data[local_env_ids] = root_velocity
        data._root_com_vel_w.timestamp = sim_timestamp
    if data._root_link_vel_w.data is not None:
        data._root_link_vel_w.data[local_env_ids] = root_velocity
        data._root_link_vel_w.timestamp = sim_timestamp
    if data._root_com_state_w.data is not None:
        data._root_com_state_w.data[local_env_ids, 7:] = root_velocity
        data._root_com_state_w.timestamp = sim_timestamp
    if data._root_link_state_w.data is not None:
        data._root_link_state_w.data[local_env_ids, 7:] = root_velocity
        data._root_link_state_w.timestamp = sim_timestamp
    if data._root_state_w.data is not None:
        data._root_state_w.data[local_env_ids, 7:] = root_velocity
        data._root_state_w.timestamp = sim_timestamp
    if data._body_com_acc_w.data is not None:
        data._body_com_acc_w.data[local_env_ids] = 0.0
        data._body_com_acc_w.timestamp = sim_timestamp


def reset_scene_to_default_no_kinematic_vel(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    reset_joint_targets: bool = False,
) -> None:
    """Reset the scene to defaults.

    Avoid writing PhysX velocities for kinematic rigid objects.
    """
    for rigid_object in env.scene.rigid_objects.values():
        default_root_state = rigid_object.data.default_root_state[env_ids].clone()
        default_root_state[:, 0:3] += env.scene.env_origins[env_ids]
        write_asset_root_pose_to_sim(
            rigid_object, default_root_state[:, :7], env_ids=env_ids
        )
        if _rigid_object_is_kinematic(rigid_object):
            _set_kinematic_root_velocity_cache(
                rigid_object, default_root_state[:, 7:], env_ids=env_ids
            )
        else:
            write_asset_root_velocity_to_sim(
                rigid_object, default_root_state[:, 7:], env_ids=env_ids
            )

    for articulation_asset in env.scene.articulations.values():
        default_root_state = articulation_asset.data.default_root_state[env_ids].clone()
        default_root_state[:, 0:3] += env.scene.env_origins[env_ids]
        write_asset_root_pose_to_sim(
            articulation_asset, default_root_state[:, :7], env_ids=env_ids
        )
        write_asset_root_velocity_to_sim(
            articulation_asset, default_root_state[:, 7:], env_ids=env_ids
        )

        default_joint_pos = articulation_asset.data.default_joint_pos[env_ids].clone()
        default_joint_vel = articulation_asset.data.default_joint_vel[env_ids].clone()
        write_articulation_joint_state_to_sim(
            articulation_asset, default_joint_pos, default_joint_vel, env_ids=env_ids
        )
        if reset_joint_targets:
            set_articulation_joint_position_target(
                articulation_asset, default_joint_pos, env_ids=env_ids
            )
            set_articulation_joint_velocity_target(
                articulation_asset, default_joint_vel, env_ids=env_ids
            )

    for deformable_object in env.scene.deformable_objects.values():
        nodal_state = deformable_object.data.default_nodal_state_w[env_ids].clone()
        write_deformable_nodal_state_to_sim(
            deformable_object, nodal_state, env_ids=env_ids
        )


def reset_bricksim_managed(env: ManagerBasedEnv, env_ids: torch.Tensor) -> None:
    """Deallocate all BrickSim-managed runtime objects for the selected environments.

    BrickSim maintains a native allocator for managed LEGO parts and
    connections keyed by environment id. This event clears all such managed
    objects for each selected environment id.

    Args:
        env: The Isaac Lab environment. This parameter is unused by the
            implementation and exists only to match the Isaac Lab event-term
            calling convention.
        env_ids: Environment ids whose managed BrickSim state should be
            cleared.
    """
    del env
    if env_ids.device.type != "cpu":
        env_ids = env_ids.detach().cpu()
    for env_id in env_ids:
        deallocate_all_managed(env_id.item())


def reset_to_connected_pose(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    moved_cfg: SceneEntityCfg,
    reference_brick_cfg: SceneEntityCfg,
    moved_side: Literal["stud", "hole"],
    stud_if: int = 1,
    hole_if: int = 0,
    offset: tuple[int, int] = (0, 0),
    yaw: int = 0,
) -> None:
    """Reset one asset to the world pose implied by a BrickSim connection specification.

    This function does not create a BrickSim connection. It only computes the
    world pose that the moved entity would have if it were connected to the
    reference brick according to the given stud/hole interface specification,
    then writes that pose into the scene.

    Args:
        env: The Isaac Lab environment.
        env_ids: The environment ids to reset.
        moved_cfg: Scene entry for the entity whose pose will be overwritten.
            The moved entity may be either a physical ``RigidObject`` or an
            ``XFormPrim`` extra such as a marker brick.
        reference_brick_cfg: Scene entry for the BrickSim brick that stays
            fixed and defines the target connection frame. It must resolve to a
            runtime brick accepted by
            :func:`bricksim.mdp.brick_part.resolve_brick_rigid_object`.
        moved_side: Which side of the connection is contributed by
            ``moved_cfg``. Use ``"hole"`` when the moved entity should be
            placed as the hole-side part connected to the reference stud, and
            ``"stud"`` when the moved entity should be placed as the stud-side
            part connected to the reference hole.
        stud_if: BrickSim stud interface index.
        hole_if: BrickSim hole interface index.
        offset: BrickSim grid offset ``(dx, dy)`` in stud units, not meters.
        yaw: BrickSim discrete yaw index, not radians.
    """
    if moved_side not in ("stud", "hole"):
        raise ValueError(
            f"Unsupported moved_side '{moved_side}'. Expected 'stud' or 'hole'."
        )

    if env_ids.device.type != "cpu":
        env_id_values = [int(env_id) for env_id in env_ids.detach().cpu().tolist()]
    else:
        env_id_values = [int(env_id) for env_id in env_ids.tolist()]
    if len(env_id_values) == 0:
        return

    moved: object = env.scene[moved_cfg.name]
    reference_brick = resolve_brick_rigid_object(env, reference_brick_cfg.name)
    reference_pos_w = reference_brick.data.root_pos_w[env_ids, :3]
    reference_quat_w = reference_brick.data.root_quat_w[env_ids]

    if isinstance(moved, RigidObject):
        stud_paths = (
            reference_brick.root_physx_view.prim_paths
            if moved_side == "hole"
            else moved.root_physx_view.prim_paths
        )
        hole_paths = (
            moved.root_physx_view.prim_paths
            if moved_side == "hole"
            else reference_brick.root_physx_view.prim_paths
        )
        rel_quat_pos = [
            compute_connection_transform(
                stud_path=stud_paths[env_id],
                stud_if=stud_if,
                hole_path=hole_paths[env_id],
                hole_if=hole_if,
                offset=offset,
                yaw=yaw,
            )
            for env_id in env_id_values
        ]
        rel_quat = torch.tensor(
            [quat_wxyz for quat_wxyz, _ in rel_quat_pos],
            device=reference_quat_w.device,
            dtype=reference_quat_w.dtype,
        )
        rel_pos = torch.tensor(
            [pos_xyz for _, pos_xyz in rel_quat_pos],
            device=reference_pos_w.device,
            dtype=reference_pos_w.dtype,
        )
    elif isinstance(moved, XformPrimView):
        moved_dimensions = scene_entity_brick_part_dimensions(env, moved_cfg.name)
        reference_dimensions = scene_entity_brick_part_dimensions(
            env, reference_brick_cfg.name
        )
        stud_dimensions, hole_dimensions = (
            (reference_dimensions, moved_dimensions)
            if moved_side == "hole"
            else (moved_dimensions, reference_dimensions)
        )
        rel_pos, rel_quat = _compute_visual_connection_transform(
            stud_dimensions,
            hole_dimensions,
            offset,
            yaw,
            device=reference_pos_w.device,
            dtype=reference_pos_w.dtype,
        )
        rel_pos = rel_pos.expand(len(env_id_values), -1)
        rel_quat = rel_quat.expand(len(env_id_values), -1)
    else:
        raise TypeError(
            f"Scene entity '{moved_cfg.name}' must resolve to a RigidObject "
            f"or Xform-like view, got {type(moved)}"
        )

    if moved_side == "hole":
        target_pos_w, target_quat_w = math_utils.combine_frame_transforms(
            reference_pos_w, reference_quat_w, rel_pos, rel_quat
        )
    else:
        rel_inv_pos, rel_inv_quat = math_utils.subtract_frame_transforms(
            rel_pos, rel_quat
        )
        target_pos_w, target_quat_w = math_utils.combine_frame_transforms(
            reference_pos_w, reference_quat_w, rel_inv_pos, rel_inv_quat
        )

    if isinstance(moved, RigidObject):
        root_pose = torch.cat([target_pos_w, target_quat_w], dim=-1)
        write_asset_root_pose_to_sim(moved, root_pose, env_ids=env_ids)
        zero_vel = torch.zeros(
            (root_pose.shape[0], 6), device=root_pose.device, dtype=root_pose.dtype
        )
        write_asset_root_velocity_to_sim(moved, zero_vel, env_ids=env_ids)
    else:
        moved.set_world_poses(
            positions=target_pos_w, orientations=target_quat_w, indices=env_id_values
        )
