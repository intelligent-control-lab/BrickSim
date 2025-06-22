import math
import torch
import omni.usd
import omni.physx
from typing import Optional
from pxr import Gf, Usd
from pxr.PhysicsSchemaTools._physicsSchemaTools import sdfPathToInt
from omni.physx.bindings._physx import ContactEventType, ContactEventHeaderVector, ContactDataVector
from omni.physics.tensors.impl.api import create_simulation_view, RigidBodyView
from .lego_schemes import BrickLength, PlateHeight, StudHeight
from .assembler import DistanceTolerance, MaxPenetration, ZAngleTolerance, RequiredForce, YawTolerance, PositionTolerance, AssemblyEvent, assemble_bricks, path_for_brick
from .physx_c import buffer_from_ContactDataVector, buffer_from_ContactEventHeaderVector
from .utils import get_physics_scene
from .utils_torch import pose7_to_mat44_batch, inv_se3_batch

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

        self.sim_view = create_simulation_view("torch")
        self.device = self.sim_view.device
        self.rigid_body_view: RigidBodyView = self.sim_view.create_rigid_body_view(brick_filter)
        if self.rigid_body_view._backend is None:
            # There is no rigid body matching the filter
            self.sim_view = None
            self.rigid_body_view = None
            return
        self.prim_paths = self.rigid_body_view.prim_paths

        keys = torch.tensor([sdfPathToInt(p) for p in self.prim_paths], dtype=torch.int64, device=self.device)
        values = torch.arange(len(keys), dtype=torch.int64, device=self.device)
        order = torch.argsort(keys)
        self.table_path_id = keys[order]
        self.table_id = values[order]

        self.lego_dims = torch.tensor([self._get_lego_dimensions(p) for p in self.prim_paths], dtype=torch.int64, device=self.device)

    def check(self) -> bool:
        current_stage = omni.usd.get_context().get_stage()
        if current_stage != self.stage:
            return False
        if (self.rigid_body_view is not None) and (not self.rigid_body_view.check()):
            return False
        return True

    def _path_id_to_id(self, sdf_id: torch.Tensor) -> torch.Tensor:
        """
        Converts SDF Path ids to brick indicies.
        Uses a sorted search table prepared in initialize().
        Unknown actors get value -1.
        """
        out = torch.full(sdf_id.shape, -1, dtype=torch.int64, device=self.device)
        if self.rigid_body_view is not None:
            idx = torch.searchsorted(self.table_path_id, sdf_id, side="left")
            idx = torch.clamp(idx, max=len(self.table_path_id) - 1)
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

    def handle_assembly_contacts(self, _contacts: ContactEventHeaderVector, _contact_data: ContactDataVector) -> list[AssemblyEvent]:
        if self.rigid_body_view is None:
            return []

        if len(_contacts) == 0 or len(_contact_data) == 0:
            return []

        np_contacts = buffer_from_ContactEventHeaderVector(_contacts)
        contacts_type = torch.from_numpy(np_contacts["type"]).to(self.device)
        contacts_actor0 = torch.from_numpy(np_contacts["actor0"]).to(dtype=torch.int64, device=self.device)
        contacts_actor1 = torch.from_numpy(np_contacts["actor1"]).to(dtype=torch.int64, device=self.device)

        mask = (contacts_type != ContactEventType.CONTACT_LOST.value)

        brick0_idx = self._path_id_to_id(contacts_actor0)
        brick1_idx = self._path_id_to_id(contacts_actor1)
        mask &= (brick0_idx >= 0) & (brick1_idx >= 0)

        if not mask.any():
            return []

        pose: torch.Tensor = self.rigid_body_view.get_transforms()
        pose0 = pose7_to_mat44_batch(pose[brick0_idx])
        pose1 = pose7_to_mat44_batch(pose[brick1_idx])

        dim0 = self.lego_dims[brick0_idx]
        dim1 = self.lego_dims[brick1_idx]

        rel_pose = torch.matmul(inv_se3_batch(pose0), pose1)

        # Swap bricks where z<0 so brick0 always offers studs
        swap_mask = rel_pose[:,2,3] < 0
        if swap_mask.any():
            brick0_idx_sw, brick1_idx_sw = brick0_idx[swap_mask].clone(), brick1_idx[swap_mask].clone()
            pose0_sw, pose1_sw = pose0[swap_mask].clone(), pose1[swap_mask].clone()
            dim0_sw, dim1_sw = dim0[swap_mask].clone(), dim1[swap_mask].clone()
            brick0_idx[swap_mask], brick1_idx[swap_mask] = brick1_idx_sw, brick0_idx_sw
            pose0[swap_mask], pose1[swap_mask] = pose1_sw, pose0_sw
            dim0[swap_mask], dim1[swap_mask] = dim1_sw, dim0_sw
            rel_pose[swap_mask] = torch.matmul(inv_se3_batch(pose0[swap_mask]), pose1[swap_mask])

        height0 = dim0[:,2] * PlateHeight
        rel_distance = rel_pose[:,2,3] - (height0 + StudHeight)

        # Reason: exceeding distance tolerance
        keep = (rel_distance <= DistanceTolerance)

        # Reason: penetrating too much
        keep &= (rel_distance >= -MaxPenetration)

        # Reason: exceeding z-angle tolerance
        keep &= (rel_pose[:,2,2] > math.cos(ZAngleTolerance))

        # Calculate force along local z
        idx0 = torch.from_numpy(np_contacts["contact_data_offset"]).to(dtype=torch.int64, device=self.device)
        ncd = torch.from_numpy(np_contacts["num_contact_data"]).to(dtype=torch.int64, device=self.device)
        np_contact_data = buffer_from_ContactDataVector(_contact_data)
        imp = torch.from_numpy(np_contact_data["impulse"]).to(self.device)
        cumsum = torch.empty((imp.shape[0] + 1, 3))
        cumsum[0] = 0.0
        torch.cumsum(imp, dim=0, out=cumsum[1:])
        sum_imp = cumsum[idx0 + ncd] - cumsum[idx0]
        frc = (torch.abs(torch.einsum("ij,ij->i", sum_imp, pose0[:, :3, 2])) / self.dt)

        # Reason: insufficient force
        keep &= (frc >= RequiredForce)

        rel_yaw = torch.arctan2(rel_pose[:,1,0], rel_pose[:,0,0])
        snapped_yaw = torch.round(rel_yaw / (torch.pi/2)) * (torch.pi/2)
        yaw_err = rel_yaw - snapped_yaw

        # Reason: exceeding yaw tolerance
        keep &= (torch.abs(yaw_err) <= YawTolerance)

        p0 = (rel_pose[:,:2,3] / BrickLength + (dim0[:,:2] - torch.einsum("bij,bj->bi", rel_pose[:,:2,:2], dim1[:,:2].float())) / 2)
        p0_snapped = torch.round(p0).to(torch.int64)
        R_snapped = torch.stack([
            torch.cos(snapped_yaw), -torch.sin(snapped_yaw),
            torch.sin(snapped_yaw),  torch.cos(snapped_yaw)
        ], dim=1).reshape(-1,2,2)
        p1_snapped = torch.round(p0_snapped + torch.einsum("bij,bj->bi", R_snapped, dim1[:,:2].float())).to(torch.int64)
        p_err = torch.norm(p0 - p0_snapped, dim=1) * BrickLength

        # Reason: exceeding position tolerance
        keep &= (p_err <= PositionTolerance)

        overlap_x = torch.clamp(torch.clamp(torch.maximum(p0_snapped[:,0], p1_snapped[:,0]), max=dim0[:,0]) - torch.clamp(torch.minimum(p0_snapped[:,0], p1_snapped[:,0]), min=0), min=0)
        overlap_y = torch.clamp(torch.clamp(torch.maximum(p0_snapped[:,1], p1_snapped[:,1]), max=dim0[:,1]) - torch.clamp(torch.minimum(p0_snapped[:,1], p1_snapped[:,1]), min=0), min=0)

        # Reason: no overlap
        keep &= (overlap_x * overlap_y) > 0

        assembly_events: list[AssemblyEvent] = []
        for k in torch.nonzero(keep, as_tuple=True)[0]:
            event = assemble_bricks(
                stage=self.stage,
                prim0=self.stage.GetPrimAtPath(self.prim_paths[brick0_idx[k]]),
                prim1=self.stage.GetPrimAtPath(self.prim_paths[brick1_idx[k]]),
                dim0=dim0[k].numpy(),
                dim1=dim1[k].numpy(),
                pose0=pose0[k].numpy(),
                p0_snapped=p0_snapped[k].numpy(),
                p1_snapped=p1_snapped[k].numpy(),
                R_snapped=R_snapped[k].numpy(),
                height0=height0[k].numpy(),
                snapped_yaw=snapped_yaw[k].numpy(),
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

        sim_view = create_simulation_view("torch")
        self.device = sim_view.device
        sim_view.invalidate()

        self.tracked_brick_ids = torch.full((num_envs, num_trackings), -1, dtype=torch.int64, device=self.device)
        self.tracked_rigid_body_ids = torch.full((num_envs, num_trackings), -1, dtype=torch.int64, device=self.device)
        self.backend: VectorizedAssemblyDetector = None

    def set_backend(self, backend: VectorizedAssemblyDetector):
        self.backend = backend
        self.tracked_rigid_body_ids.fill_(-1)
        if self.backend is None:
            return

        mask = self.tracked_brick_ids >= 0
        env_ids, tracking_ids = torch.nonzero(mask, as_tuple=True)
        brick_ids = self.tracked_brick_ids[env_ids, tracking_ids]

        sdf_ids = torch.tensor([
            sdfPathToInt(path_for_brick(brick_id=brick_id, env_id=env_id))
            for env_id, brick_id in zip(env_ids, brick_ids)
        ], dtype=torch.int64, device=self.device)

        self.tracked_rigid_body_ids[env_ids, tracking_ids] = self.backend._path_id_to_id(sdf_ids)

    def set_tracked_bricks(self, tracking_id: int, env_ids: torch.Tensor, brick_ids: torch.Tensor):
        self.tracked_brick_ids[env_ids, tracking_id] = brick_ids
        if self.backend is not None:
            sdf_ids = torch.tensor([
                sdfPathToInt(path_for_brick(brick_id=brick_id, env_id=env_id)) if brick_id >= 0 else -1
                for env_id, brick_id in zip(env_ids, brick_ids)
            ], dtype=torch.int64, device=self.device)
            self.tracked_rigid_body_ids[env_ids, tracking_id] = self.backend._path_id_to_id(sdf_ids)

    @property
    def view(self) -> Optional[RigidBodyView]:
        if self.backend is None:
            return None
        return self.backend.rigid_body_view

    def get_transforms(self, tracking_id: int) -> torch.Tensor:
        result = torch.zeros((self.num_envs, 7), dtype=torch.float32, device=self.device)
        if (view := self.view) is not None:
            mask = self.tracked_rigid_body_ids[:, tracking_id] >= 0
            result[mask] = view.get_transforms()[self.tracked_rigid_body_ids[mask, tracking_id]]
        return result
