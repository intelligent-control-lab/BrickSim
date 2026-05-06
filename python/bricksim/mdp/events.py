"""Event terms for BrickSim-managed Isaac Lab environments."""

import torch
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedEnv

from bricksim.core import deallocate_all_managed

from .isaaclab_shims import (
    set_articulation_joint_position_target,
    set_articulation_joint_velocity_target,
    write_articulation_joint_state_to_sim,
    write_asset_root_pose_to_sim,
    write_asset_root_velocity_to_sim,
    write_deformable_nodal_state_to_sim,
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
