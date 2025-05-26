import omni.usd
import omni.physx
import numpy as np
from typing import Optional, Union
from pxr import Gf, Usd
from pxr.PhysicsSchemaTools._physicsSchemaTools import sdfPathToInt
from omni.physx.bindings._physx import ContactEventType, ContactEventHeaderVector, ContactDataVector
from omni.physics.tensors.impl.api import create_simulation_view, RigidBodyView
from .lego_schemes import BrickLength, PlateHeight, StudHeight
from .assembler import DistanceTolerance, MaxPenetration, ZAngleTolerance, RequiredForce, YawTolerance, PositionTolerance, AssemblyEvent, assemble_bricks, path_for_brick
from .physx_c import buffer_from_ContactDataVector, buffer_from_ContactEventHeaderVector
from .utils import get_physics_scene, pose7_to_mat44_batch, inv_se3_batch

class VectorizedAssemblyDetector:
    def __init__(self):
        self.physx_sim_interface = omni.physx.get_physx_simulation_interface()
        self.stage: Usd.Stage = omni.usd.get_context().get_stage()

        physx_scene = get_physics_scene(self.stage)
        if physx_scene is None:
            raise RuntimeError("No PhysicsScene found in the stage.")
        self.dt = 1.0 / physx_scene.GetTimeStepsPerSecondAttr().Get()

        if self.stage.GetPrimAtPath("/World/envs").IsValid():
            brick_filter = "/World/envs/env_*/Brick_*"
        else:
            brick_filter = "/World/Brick_*"

        self.sim_view = create_simulation_view("numpy")
        self.rigid_body_view: RigidBodyView = self.sim_view.create_rigid_body_view(brick_filter)
        if self.rigid_body_view._backend is None:
            # There is no rigid body matching the filter
            self.sim_view = None
            self.rigid_body_view = None
            return
        self.prim_paths = self.rigid_body_view.prim_paths

        keys = np.array([sdfPathToInt(p) for p in self.prim_paths], dtype=int)
        values = np.arange(len(keys), dtype=int)
        order = np.argsort(keys)
        self.table_path_id = keys[order]
        self.table_id = values[order]

        self.lego_dims = np.array([self._get_lego_dimensions(p) for p in self.prim_paths], dtype=int)

    def check(self) -> bool:
        current_stage = omni.usd.get_context().get_stage()
        if current_stage != self.stage:
            return False
        if (self.rigid_body_view is not None) and (not self.rigid_body_view.check()):
            return False
        return True

    def _path_id_to_id(self, sdf_id: np.ndarray) -> np.ndarray:
        """
        Converts SDF Path ids to brick indicies.
        Uses a sorted search table prepared in initialize().
        Unknown actors get value -1.
        """
        out = np.full(sdf_id.shape, -1, dtype=int)
        if self.rigid_body_view is not None:
            idx = np.searchsorted(self.table_path_id, sdf_id, side="left")
            idx = np.minimum(idx, self.table_path_id.size - 1)
            valid = self.table_path_id[idx] == sdf_id
            out[valid] = self.table_id[idx[valid]]
        return out

    def _get_lego_dimensions(self, path: str) -> Optional[Gf.Vec3i]:
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None
        dim_attr = prim.GetAttribute("lego_dimensions")
        if not dim_attr.IsValid():
            return None
        return dim_attr.Get()

    def handle_assembly_contacts(self, _contacts, _contact_data) -> list[AssemblyEvent]:
        if self.rigid_body_view is None:
            return []

        if len(_contacts) == 0 or len(_contact_data) == 0:
            return []
        contacts = buffer_from_ContactEventHeaderVector(_contacts)
        contact_data = buffer_from_ContactDataVector(_contact_data)

        found_mask = (contacts["type"] != ContactEventType.CONTACT_LOST.value)
        contacts_f = contacts[found_mask]

        brick0_idx = self._path_id_to_id(contacts_f["actor0"])
        brick1_idx = self._path_id_to_id(contacts_f["actor1"])
        is_brick_mask = (brick0_idx >= 0) & (brick1_idx >= 0)
        contacts_f = contacts_f[is_brick_mask]
        brick0_idx = brick0_idx[is_brick_mask]
        brick1_idx = brick1_idx[is_brick_mask]

        if contacts_f.size == 0:
            return []

        pose: np.ndarray = self.rigid_body_view.get_transforms()
        pose0 = pose7_to_mat44_batch(pose[brick0_idx])
        pose1 = pose7_to_mat44_batch(pose[brick1_idx])

        dim0 = self.lego_dims[brick0_idx]
        dim1 = self.lego_dims[brick1_idx]

        rel_pose = np.matmul(inv_se3_batch(pose0), pose1)

        # Swap bricks where z<0 so brick0 always offers studs
        swap_mask = rel_pose[:,2,3] < 0
        if np.any(swap_mask):
            pose0_sw, pose1_sw = pose0[swap_mask].copy(), pose1[swap_mask].copy()
            dim0_sw, dim1_sw = dim0[swap_mask].copy(), dim1[swap_mask].copy()
            brick0_idx[swap_mask], brick1_idx[swap_mask] = brick1_idx[swap_mask], brick0_idx[swap_mask]
            pose0[swap_mask], pose1[swap_mask] = pose1_sw, pose0_sw
            dim0[swap_mask], dim1[swap_mask] = dim1_sw, dim0_sw
            rel_pose[swap_mask] = np.matmul(inv_se3_batch(pose0[swap_mask]), pose1[swap_mask])

        height0 = dim0[:,2] * PlateHeight
        rel_distance = rel_pose[:,2,3] - (height0 + StudHeight)

        # Reason: exceeding distance tolerance
        keep = (rel_distance <= DistanceTolerance)

        # Reason: penetrating too much
        keep &= (rel_distance >= -MaxPenetration)

        # Reason: exceeding z-angle tolerance
        keep &= (rel_pose[:,2,2] > np.cos(ZAngleTolerance))

        # Calculate force along local z
        idx0 = contacts_f["contact_data_offset"]
        ncd = contacts_f["num_contact_data"]
        imp = contact_data["impulse"]
        cumsum = np.empty((imp.shape[0] + 1, 3))
        cumsum[0] = 0.0
        np.cumsum(imp, axis=0, out=cumsum[1:])
        sum_imp = cumsum[idx0 + ncd] - cumsum[idx0]
        frc = (np.abs(np.einsum("ij,ij->i", sum_imp, pose0[:, :3, 2])) / self.dt)

        # Reason: insufficient force
        keep &= (frc >= RequiredForce)

        rel_yaw = np.arctan2(rel_pose[:,1,0], rel_pose[:,0,0])
        snapped_yaw = np.round(rel_yaw / (np.pi/2)) * (np.pi/2)
        yaw_err = rel_yaw - snapped_yaw

        # Reason: exceeding yaw tolerance
        keep &= (np.abs(yaw_err) <= YawTolerance)

        p0 = (rel_pose[:,:2,3] / BrickLength + (dim0[:,:2] - np.einsum("bij,bj->bi", rel_pose[:,:2,:2], dim1[:,:2])) / 2)
        p0_snapped = np.round(p0).astype(int)
        R_snapped = np.stack([np.cos(snapped_yaw), -np.sin(snapped_yaw), np.sin(snapped_yaw),  np.cos(snapped_yaw)], axis=1).reshape(-1,2,2)
        p1_snapped = np.round(p0_snapped + np.einsum("bij,bj->bi", R_snapped, dim1[:,:2])).astype(int)
        p_err = np.linalg.norm(p0 - p0_snapped, axis=1) * BrickLength

        # Reason: exceeding position tolerance
        keep &= (p_err <= PositionTolerance)

        overlap_x = np.maximum(0, np.minimum(dim0[:,0], np.maximum(p0_snapped[:,0], p1_snapped[:,0])) - np.maximum(0, np.minimum(p0_snapped[:,0], p1_snapped[:,0])))
        overlap_y = np.maximum(0, np.minimum(dim0[:,1], np.maximum(p0_snapped[:,1], p1_snapped[:,1])) - np.maximum(0, np.minimum(p0_snapped[:,1], p1_snapped[:,1])))

        # Reason: no overlap
        keep &= (overlap_x * overlap_y) > 0

        assembly_events: list[AssemblyEvent] = []
        for k in np.flatnonzero(keep):
            event = assemble_bricks(
                stage=self.stage,
                prim0=self.stage.GetPrimAtPath(self.prim_paths[brick0_idx[k]]),
                prim1=self.stage.GetPrimAtPath(self.prim_paths[brick1_idx[k]]),
                dim0=dim0[k],
                dim1=dim1[k],
                pose0=pose0[k],
                p0_snapped=p0_snapped[k],
                p1_snapped=p1_snapped[k],
                R_snapped=R_snapped[k],
                height0=height0[k],
                snapped_yaw=snapped_yaw[k],
            )
            if event is not None:
                assembly_events.append(event)
        return assembly_events

    def destroy(self):
        if self.rigid_body_view is not None:
            self.rigid_body_view = None
            self.sim_view.invalidate()
            self.sim_view = None

class BrickTracker:
    def __init__(self, num_envs: int, num_trackings: int):
        self.num_envs = num_envs
        self.num_trackings = num_trackings
        self.tracked_brick_ids = np.full((num_envs, num_trackings), -1, dtype=int)
        self.tracked_rigid_body_ids = np.full((num_envs, num_trackings), -1, dtype=int)
        self.backend: VectorizedAssemblyDetector = None

    def set_backend(self, backend: VectorizedAssemblyDetector):
        self.backend = backend
        if self.backend is None:
            self.tracked_rigid_body_ids.fill(-1)
            return

        sdf_ids = np.array([
            sdfPathToInt(path_for_brick(brick_id=brick_id, env_id=env_id)) if brick_id >= 0 else -1
            for (env_id, tracking_id), brick_id in np.ndenumerate(self.tracked_brick_ids)
        ], dtype=int)
        self.tracked_rigid_body_ids[:] = self.backend._path_id_to_id(sdf_ids).reshape(self.tracked_brick_ids.shape)

    def set_tracked_bricks(self, tracking_id: int, env_ids: Union[np.ndarray, list[int]], brick_ids: Union[np.ndarray, list[int]]):
        self.tracked_brick_ids[env_ids, tracking_id] = brick_ids
        if self.backend is not None:
            sdf_ids = np.array([
                sdfPathToInt(path_for_brick(brick_id=b, env_id=e)) if b >= 0 else -1
                for e, b in zip(env_ids, brick_ids)
            ], dtype=int)
            self.tracked_rigid_body_ids[env_ids, tracking_id] = self.backend._path_id_to_id(sdf_ids)

    @property
    def view(self) -> Optional[RigidBodyView]:
        if self.backend is None:
            return None
        return self.backend.rigid_body_view

    def get_transforms(self, tracking_id: int) -> np.ndarray:
        result = np.zeros((self.num_envs, 7), dtype=np.float32)
        if (view := self.view) is not None:
            mask = self.tracked_rigid_body_ids[:, tracking_id] >= 0
            result[mask] = view.get_transforms()[self.tracked_rigid_body_ids[mask, tracking_id]]
        return result
