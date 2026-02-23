# TODO: rewrite this function, codex produces shit code

from __future__ import annotations

import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg
from bricksim._native import compute_connection_transform, deallocate_all_managed

def reset_managed_lego(env, env_ids: torch.Tensor) -> None:
    if env_ids.device.type != "cpu":
        env_ids = env_ids.detach().cpu()
    for env_id in env_ids:
        deallocate_all_managed(env_id.item())


def _env_id_list(env_ids: torch.Tensor) -> list[int]:
    if env_ids.device.type != "cpu":
        env_ids = env_ids.detach().cpu()
    return [int(env_id) for env_id in env_ids.tolist()]


def _prim_paths(entity) -> list[str]:
    if hasattr(entity, "root_physx_view"):
        return entity.root_physx_view.prim_paths
    if hasattr(entity, "prim_paths"):
        return entity.prim_paths
    raise TypeError(f"Unsupported scene entity type for prim-path lookup: {type(entity)}")


def _world_pose_w(entity, env_ids: torch.Tensor, env_id_list: list[int]) -> tuple[torch.Tensor, torch.Tensor]:
    if hasattr(entity, "data") and hasattr(entity.data, "root_pos_w") and hasattr(entity.data, "root_quat_w"):
        return entity.data.root_pos_w[env_ids], entity.data.root_quat_w[env_ids]
    if hasattr(entity, "get_world_poses"):
        return entity.get_world_poses(indices=env_id_list)
    raise TypeError(f"Unsupported scene entity type for world-pose read: {type(entity)}")


def _set_world_pose_w(
    entity,
    env_ids: torch.Tensor,
    env_id_list: list[int],
    pos_w: torch.Tensor,
    quat_w: torch.Tensor,
) -> None:
    if hasattr(entity, "write_root_pose_to_sim"):
        root_pose = torch.cat([pos_w, quat_w], dim=-1)
        entity.write_root_pose_to_sim(root_pose, env_ids=env_ids)
        if hasattr(entity, "write_root_velocity_to_sim"):
            zero_vel = torch.zeros((root_pose.shape[0], 6), device=root_pose.device, dtype=root_pose.dtype)
            entity.write_root_velocity_to_sim(zero_vel, env_ids=env_ids)
        return
    if hasattr(entity, "set_world_poses"):
        entity.set_world_poses(positions=pos_w, orientations=quat_w, indices=env_id_list)
        return
    raise TypeError(f"Unsupported scene entity type for world-pose write: {type(entity)}")

def reset_to_assembled_pose(
    env,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg,
    stud: SceneEntityCfg | None = None,
    hole: SceneEntityCfg | None = None,
    stud_if: int = 1,
    hole_if: int = 0,
    offset: tuple[int, int] = (0, 0),
    yaw: int = 0,
):
    if (stud is None) == (hole is None):
        raise ValueError("Exactly one of `stud` or `hole` must be specified.")

    env_id_list = _env_id_list(env_ids)
    if len(env_id_list) == 0:
        return

    asset = env.scene[asset_cfg.name]
    asset_paths = _prim_paths(asset)

    if stud is not None:
        other = env.scene[stud.name]
        other_paths = _prim_paths(other)

        other_pos_w, other_quat_w = _world_pose_w(other, env_ids, env_id_list)

        rel_pos_values = []
        rel_quat_values = []
        for env_id in env_id_list:
            quat_wxyz, pos_xyz = compute_connection_transform(
                stud_path=other_paths[env_id],
                stud_if=stud_if,
                hole_path=asset_paths[env_id],
                hole_if=hole_if,
                offset=offset,
                yaw=yaw,
            )
            rel_pos_values.append(pos_xyz)
            rel_quat_values.append(quat_wxyz)

        rel_pos = torch.tensor(rel_pos_values, device=other_pos_w.device, dtype=other_pos_w.dtype)
        rel_quat = torch.tensor(rel_quat_values, device=other_quat_w.device, dtype=other_quat_w.dtype)
        target_pos_w, target_quat_w = math_utils.combine_frame_transforms(other_pos_w, other_quat_w, rel_pos, rel_quat)

    else:
        other = env.scene[hole.name]
        other_paths = _prim_paths(other)

        other_pos_w, other_quat_w = _world_pose_w(other, env_ids, env_id_list)

        rel_pos_values = []
        rel_quat_values = []
        for env_id in env_id_list:
            quat_wxyz, pos_xyz = compute_connection_transform(
                stud_path=asset_paths[env_id],
                stud_if=stud_if,
                hole_path=other_paths[env_id],
                hole_if=hole_if,
                offset=offset,
                yaw=yaw,
            )
            rel_pos_values.append(pos_xyz)
            rel_quat_values.append(quat_wxyz)

        rel_pos = torch.tensor(rel_pos_values, device=other_pos_w.device, dtype=other_pos_w.dtype)
        rel_quat = torch.tensor(rel_quat_values, device=other_quat_w.device, dtype=other_quat_w.dtype)

        rel_inv_pos, rel_inv_quat = math_utils.subtract_frame_transforms(rel_pos, rel_quat)
        target_pos_w, target_quat_w = math_utils.combine_frame_transforms(
            other_pos_w, other_quat_w, rel_inv_pos, rel_inv_quat
        )

    _set_world_pose_w(asset, env_ids, env_id_list, target_pos_w, target_quat_w)
