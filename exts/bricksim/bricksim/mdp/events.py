import torch
from typing import Literal

import isaaclab.utils.math as math_utils
from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg
from isaacsim.core.prims import XFormPrim
from bricksim._native import compute_connection_transform, deallocate_all_managed
from bricksim.mdp.utils import resolve_brick_rigid_object


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
    env,
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
            :func:`bricksim.mdp.utils.resolve_brick_rigid_object`.
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
        raise ValueError(f"Unsupported moved_side '{moved_side}'. Expected 'stud' or 'hole'.")

    if env_ids.device.type != "cpu":
        env_id_values = [int(env_id) for env_id in env_ids.detach().cpu().tolist()]
    else:
        env_id_values = [int(env_id) for env_id in env_ids.tolist()]
    if len(env_id_values) == 0:
        return

    moved = env.scene[moved_cfg.name]
    if isinstance(moved, RigidObject):
        moved_paths = moved.root_physx_view.prim_paths
    elif isinstance(moved, XFormPrim):
        moved_paths = moved.prim_paths
    else:
        raise TypeError(f"Scene entity '{moved_cfg.name}' must resolve to RigidObject or XFormPrim, got {type(moved)}")

    reference_brick = resolve_brick_rigid_object(env, reference_brick_cfg.name)
    reference_paths = reference_brick.root_physx_view.prim_paths
    reference_pos_w = reference_brick.data.root_pos_w[env_ids, :3]
    reference_quat_w = reference_brick.data.root_quat_w[env_ids]

    rel_pos_values = []
    rel_quat_values = []
    for env_id in env_id_values:
        if moved_side == "hole":
            quat_wxyz, pos_xyz = compute_connection_transform(
                stud_path=reference_paths[env_id],
                stud_if=stud_if,
                hole_path=moved_paths[env_id],
                hole_if=hole_if,
                offset=offset,
                yaw=yaw,
            )
        else:
            quat_wxyz, pos_xyz = compute_connection_transform(
                stud_path=moved_paths[env_id],
                stud_if=stud_if,
                hole_path=reference_paths[env_id],
                hole_if=hole_if,
                offset=offset,
                yaw=yaw,
            )
        rel_pos_values.append(pos_xyz)
        rel_quat_values.append(quat_wxyz)

    rel_pos = torch.tensor(rel_pos_values, device=reference_pos_w.device, dtype=reference_pos_w.dtype)
    rel_quat = torch.tensor(rel_quat_values, device=reference_quat_w.device, dtype=reference_quat_w.dtype)

    if moved_side == "hole":
        target_pos_w, target_quat_w = math_utils.combine_frame_transforms(
            reference_pos_w, reference_quat_w, rel_pos, rel_quat
        )
    else:
        rel_inv_pos, rel_inv_quat = math_utils.subtract_frame_transforms(rel_pos, rel_quat)
        target_pos_w, target_quat_w = math_utils.combine_frame_transforms(
            reference_pos_w, reference_quat_w, rel_inv_pos, rel_inv_quat
        )

    if isinstance(moved, RigidObject):
        root_pose = torch.cat([target_pos_w, target_quat_w], dim=-1)
        moved.write_root_pose_to_sim(root_pose, env_ids=env_ids)
        zero_vel = torch.zeros((root_pose.shape[0], 6), device=root_pose.device, dtype=root_pose.dtype)
        moved.write_root_velocity_to_sim(zero_vel, env_ids=env_ids)
    else:
        moved.set_world_poses(positions=target_pos_w, orientations=target_quat_w, indices=env_id_values)
