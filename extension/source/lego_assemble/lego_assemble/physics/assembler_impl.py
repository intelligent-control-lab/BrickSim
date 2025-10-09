import math
import torch
import omni.usd
import omni.physx
from typing import Optional
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from pxr.PhysicsSchemaTools._physicsSchemaTools import sdfPathToInt, intToSdfPath
from omni.physx.bindings._physx import ContactEventType, ContactEventHeaderVector, ContactDataVector
from omni.physics.tensors.impl.api import create_simulation_view, RigidBodyView
from .lego_schemes import BrickLength, PlateHeight, StudHeight
from .assembler import Thresholds, parse_brick_path, path_for_brick, path_for_conn
from .physx_c import buffer_from_ContactDataVector, buffer_from_ContactEventHeaderVector
from .utils import get_physics_scene
from .math_utils import pose_to_se3, inv_se3

class AssemblyDetector:
    def __init__(self):
        self.physx_sim_interface = omni.physx.get_physx_simulation_interface()
        self.stage: Usd.Stage = omni.usd.get_context().get_stage()

        self.mpu = UsdGeom.GetStageMetersPerUnit(self.stage)
        self.kpu = UsdPhysics.GetStageKilogramsPerUnit(self.stage)

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
        self.rb_view: RigidBodyView = self.sim_view.create_rigid_body_view(brick_filter)
        if self.rb_view._backend is None:
            # There is no rigid body matching the filter
            self.sim_view = None
            self.rb_view = None
            return
        self.rb_paths = self.rb_view.prim_paths

        sdf2rb_keys_ = torch.tensor([sdfPathToInt(p) for p in self.rb_paths], dtype=torch.int64, device=self.device)
        sdf2rb_values_ = torch.arange(len(sdf2rb_keys_), dtype=torch.int64, device=self.device)
        sdf2rb_order_ = torch.argsort(sdf2rb_keys_)
        self.sdf2rb_keys = sdf2rb_keys_[sdf2rb_order_]
        self.sdf2rb_values = sdf2rb_values_[sdf2rb_order_]

        self.body_collider_id = torch.tensor([sdfPathToInt(p + "/BodyCollider") for p in self.rb_paths], dtype=torch.int64, device=self.device)
        self.top_collider_id = torch.tensor([sdfPathToInt(p + "/TopCollider") for p in self.rb_paths], dtype=torch.int64, device=self.device)

        self.lego_dims = torch.tensor([self._get_brick_dimensions(p) for p in self.rb_paths], dtype=torch.int64, device=self.device)

    def check(self) -> bool:
        current_stage = omni.usd.get_context().get_stage()
        if current_stage != self.stage:
            return False
        if (self.rb_view is not None) and (not self.rb_view.check()):
            return False
        return True

    def _get_brick_dimensions(self, path: str) -> Optional[Gf.Vec3i]:
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None
        dim_attr = prim.GetAttribute("lego_brick:dimensions")
        if not dim_attr.IsValid():
            return None
        return dim_attr.Get()

    def _sdf2rb_lookup(self, sdf_id: torch.Tensor) -> torch.Tensor:
        """
        Converts SDF path ids to rigid body indicies.
        Uses a prepared sorted search table.
        Unknown rigid bodies get value -1.
        """
        out = torch.full(sdf_id.shape, -1, dtype=torch.int64, device=self.device)
        if self.rb_view is not None:
            idx = torch.searchsorted(self.sdf2rb_keys, sdf_id, side="left")
            idx = torch.clamp(idx, max=len(self.sdf2rb_keys) - 1)
            valid = self.sdf2rb_keys[idx] == sdf_id
            out[valid] = self.sdf2rb_values[idx[valid]]
        return out

    def extract_brick_contacts(self, contacts: ContactEventHeaderVector, contact_data: ContactDataVector) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """ Extracts brick rigid body indicies and contact impulse vectors from a contact report.
            Note same brick pair appears at most once in the output. The pair is ordered, and (rb0, rb1) and (rb1, rb0) may appear at the same time.

            Returns:
                tuple[torch.Tensor, torch.Tensor, torch.Tensor]: A tuple containing:
                - rb0_id (torch.Tensor): Rigid body indicies of the 1st brick in the contact pair in (N,) of type torch.int64. This is the lower brick, offering TopCollider.
                - rb1_id (torch.Tensor): Rigid body indicies of the 2nd brick in the contact pair in (N,) of type torch.int64. This is the upper brick, offering BodyCollider.
                - impulse (torch.Tensor): Contact impulse vectors between the two bricks in (N, 3) of type torch.float32.
        """
        if len(contacts) == 0 or len(contact_data) == 0:
            return torch.empty(0, dtype=torch.int64, device=self.device), \
                   torch.empty(0, dtype=torch.int64, device=self.device), \
                   torch.empty(0, 3, dtype=torch.float32, device=self.device)

        np_contacts = buffer_from_ContactEventHeaderVector(contacts)
        contacts_type = torch.from_numpy(np_contacts["type"]).to(self.device)
        contacts_actor0 = torch.from_numpy(np_contacts["actor0"]).to(dtype=torch.int64, device=self.device)
        contacts_actor1 = torch.from_numpy(np_contacts["actor1"]).to(dtype=torch.int64, device=self.device)
        contacts_collider0 = torch.from_numpy(np_contacts["collider0"]).to(dtype=torch.int64, device=self.device)
        contacts_collider1 = torch.from_numpy(np_contacts["collider1"]).to(dtype=torch.int64, device=self.device)
        rb0_id = self._sdf2rb_lookup(contacts_actor0)
        rb1_id = self._sdf2rb_lookup(contacts_actor1)

        np_contact_data = buffer_from_ContactDataVector(contact_data)
        cd_impulse = torch.from_numpy(np_contact_data["impulse"]).to(self.device)
        cd_offset = torch.from_numpy(np_contacts["contact_data_offset"]).to(dtype=torch.int64, device=self.device)
        cd_len = torch.from_numpy(np_contacts["num_contact_data"]).to(dtype=torch.int64, device=self.device)
        cd_impulse_cumsum = torch.empty((cd_impulse.shape[0] + 1, 3))
        cd_impulse_cumsum[0] = 0.0
        torch.cumsum(cd_impulse, dim=0, out=cd_impulse_cumsum[1:])
        impulse = cd_impulse_cumsum[cd_offset + cd_len] - cd_impulse_cumsum[cd_offset]

        mask = (rb0_id >= 0) & (rb1_id >= 0) & (contacts_type != ContactEventType.CONTACT_LOST.value)

        f_rb0_id = rb0_id[mask]
        f_rb1_id = rb1_id[mask]
        f_impulse = impulse[mask]
        f_contacts_collider0 = contacts_collider0[mask]
        f_contacts_collider1 = contacts_collider1[mask]
        f_is_body0 = (f_contacts_collider0 == self.body_collider_id[f_rb0_id])
        f_is_body1 = (f_contacts_collider1 == self.body_collider_id[f_rb1_id])
        f_is_top0 = (f_contacts_collider0 == self.top_collider_id[f_rb0_id])
        f_is_top1 = (f_contacts_collider1 == self.top_collider_id[f_rb1_id])
        f_order_ok = f_is_top0 & f_is_body1
        f_order_swap = f_is_top1 & f_is_body0
        f_to_keep = f_order_ok | f_order_swap

        f_rb0_id_sw = torch.where(f_order_swap, f_rb1_id, f_rb0_id)
        f_rb1_id_sw = torch.where(f_order_swap, f_rb0_id, f_rb1_id)
        f_impulse_sw = torch.where(f_order_swap[:, None], -f_impulse, f_impulse)
        return f_rb0_id_sw[f_to_keep], f_rb1_id_sw[f_to_keep], f_impulse_sw[f_to_keep]

    def handle_brick_contacts(self, rb0_id: torch.Tensor, rb1_id: torch.Tensor, frc: torch.Tensor):
        """ Filters brick contacts and process assembly events.

            Args:
                rb0_id (torch.Tensor): Rigid body indicies of the lower brick in the contact pair in (N,).
                rb1_id (torch.Tensor): Rigid body indicies of the upper brick in the contact pair in (N,).
                frc (torch.Tensor): Contact force vectors between the two bricks in (N, 3).
        """
        #### Prepare inputs

        # Get poses of bricks
        pose: torch.Tensor = self.rb_view.get_transforms()
        pose0 = pose_to_se3(pose[rb0_id])
        pose1 = pose_to_se3(pose[rb1_id])
        rel_pose = torch.matmul(inv_se3(pose0), pose1)

        # Get dimensions of bricks
        dim0 = self.lego_dims[rb0_id].to(pose.dtype)
        dim1 = self.lego_dims[rb1_id].to(pose.dtype)

        #### Calculate metrics

        # Calculate relative distance
        brick0_height = dim0[:,2] * (PlateHeight / self.mpu)
        rel_distance = rel_pose[:,2,3] - (brick0_height + (StudHeight / self.mpu))

        # Calculate projected force along brick0's z-axis
        frc_z = torch.abs(torch.einsum("ij,ij->i", frc, pose0[:, :3, 2]))

        # Calculate snapped yaw & yaw error
        rel_yaw = torch.arctan2(rel_pose[:,1,0], rel_pose[:,0,0])
        yaw_snap = torch.round(rel_yaw / (torch.pi/2)) * (torch.pi/2)
        R_snap = torch.stack([
            torch.cos(yaw_snap), -torch.sin(yaw_snap),
            torch.sin(yaw_snap),  torch.cos(yaw_snap),
        ], dim=1).reshape(-1,2,2)
        yaw_err = rel_yaw - yaw_snap

        # Calculate snapped position in grid coordinates & position error
        p0 = rel_pose[:,:2,3] / (BrickLength / self.mpu) + (dim0[:,:2] - torch.einsum("bij,bj->bi", rel_pose[:,:2,:2], dim1[:,:2])) / 2
        p0_snap = torch.round(p0)
        p1_snap = torch.round(p0_snap + torch.einsum("bij,bj->bi", R_snap, dim1[:,:2]))
        p_err = torch.norm(p0 - p0_snap, dim=1) * (BrickLength / self.mpu)

        # Calculate overlap
        ov_start1 = torch.zeros_like(p0_snap)
        ov_end1 = dim0[:, :2]
        ov_start2 = torch.minimum(p0_snap, p1_snap)
        ov_end2 = torch.maximum(p0_snap, p1_snap)
        ov_start = torch.maximum(ov_start1, ov_start2)
        ov_end = torch.minimum(ov_end1, ov_end2)
        ov_mid = (ov_start + ov_end) / 2
        overlap = torch.clamp(ov_end - ov_start, min=0)

        # Calculate joint local transforms
        T_parent_local = torch.zeros_like(rel_pose)
        T_parent_local[:, :3, :3] = torch.eye(3, device=self.device)
        T_parent_local[:, 3, 3] = 1.0
        T_parent_local[:, :2, 3] = (ov_mid - dim0[:, :2] / 2) * (BrickLength / self.mpu)
        T_parent_local[:, 2, 3] = brick0_height

        T_child_local = torch.zeros_like(rel_pose)
        R_snap_T = R_snap.transpose(1,2)
        T_child_local[:, :2, :2] = R_snap_T
        T_child_local[:, 2, 2] = 1.0
        T_child_local[:, 3, 3] = 1.0
        T_child_local[:, :2, 3] = torch.einsum("bij,bj->bi",
            R_snap_T,
            ov_mid - (ov_start2 + ov_end2) / 2
        ) * (BrickLength / self.mpu)

        #### Filtering
        as_flag = \
            (rel_distance <= (Thresholds.DistanceTolerance / self.mpu)) & \
            (rel_distance >= -(Thresholds.MaxPenetration / self.mpu)) & \
            (rel_pose[:,2,2] > math.cos(Thresholds.ZAngleTolerance)) & \
            (frc_z >= (Thresholds.RequiredForce / self.mpu / self.kpu)) & \
            (torch.abs(yaw_err) <= Thresholds.YawTolerance) & \
            (p_err <= (Thresholds.PositionTolerance / self.mpu)) & \
            ((overlap[:,0] > 0) & (overlap[:,1] > 0))

        # Create joints
        for as_rb0, as_rb1, as_tf0, as_tf1, as_ov in zip(
            rb0_id[as_flag].cpu(),
            rb1_id[as_flag].cpu(),
            T_parent_local[as_flag].transpose(1,2).cpu(),
            T_child_local[as_flag].transpose(1,2).cpu(),
            (overlap[as_flag] * (BrickLength / self.mpu)).cpu(),
        ):
            path0 = self.rb_paths[as_rb0]
            path1 = self.rb_paths[as_rb1]
            brick_id0, env_id0 = parse_brick_path(path0)
            brick_id1, env_id1 = parse_brick_path(path1)
            if env_id0 != env_id1:
                raise ValueError(f"Bricks are in different environments: {path0} and {path1}")

            joint_path = path_for_conn(brick_id0, brick_id1, env_id0)
            bond_prim: Usd.Prim = self.stage.GetPrimAtPath(joint_path)
            if bond_prim.IsValid():
                bond_prim.GetAttribute("lego_conn:enabled").Set(True)
            else:
                bond_prim = UsdGeom.Xform.Define(self.stage, joint_path).GetPrim()
                bond_prim.CreateRelationship("lego_conn:body0").AddTarget(path0)
                bond_prim.CreateRelationship("lego_conn:body1").AddTarget(path1)
                M0 = Gf.Matrix4f(as_tf0.tolist())
                M1 = Gf.Matrix4f(as_tf1.tolist())
                bond_prim.CreateAttribute("lego_conn:pos0", Sdf.ValueTypeNames.Float3).Set(M0.ExtractTranslation())
                bond_prim.CreateAttribute("lego_conn:rot0", Sdf.ValueTypeNames.Quatf).Set(M0.ExtractRotationQuat())
                bond_prim.CreateAttribute("lego_conn:pos1", Sdf.ValueTypeNames.Float3).Set(M1.ExtractTranslation())
                bond_prim.CreateAttribute("lego_conn:rot1", Sdf.ValueTypeNames.Quatf).Set(M1.ExtractRotationQuat())
                bond_prim.CreateAttribute("lego_conn:overlap_xy", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(as_ov.tolist()))
                bond_prim.CreateAttribute("lego_conn:enabled", Sdf.ValueTypeNames.Bool).Set(True)

    def handle_contact_report(self, _contacts: ContactEventHeaderVector, _contact_data: ContactDataVector):
        if self.rb_view is None:
            return

        rb0_id, rb1_id, impulse = self.extract_brick_contacts(_contacts, _contact_data)
        if rb0_id.numel() == 0:
            return

        frc = impulse / self.dt
        self.handle_brick_contacts(rb0_id, rb1_id, frc)

    def destroy(self):
        if self.rb_view is not None:
            self.rb_view = None
            self.sim_view.invalidate()
            self.sim_view = None

class BrickTracker:
    def __init__(self, num_envs: int, num_types: int):
        self.num_envs = num_envs
        self.num_types = num_types

        sim_view = create_simulation_view("torch")
        self.device = sim_view.device
        sim_view.invalidate()

        self.brick_ids = torch.full((num_envs, num_types), -1, dtype=torch.int64, device=self.device) # (E,T)
        self.rb_ids = torch.full((num_envs, num_types), -1, dtype=torch.int64, device=self.device) # (E,T)
        self.backend: AssemblyDetector = None

    def set_backend(self, backend: AssemblyDetector):
        self.rb_ids.fill_(-1)

        self.backend = backend
        if self.backend is None:
            return

        mask = (self.brick_ids >= 0) # (E,T)
        env_ids, type_ids = torch.nonzero(mask, as_tuple=True) # (N,), (N,)
        brick_ids = self.brick_ids[env_ids, type_ids] # (N,)

        sdf_ids = torch.tensor([
            sdfPathToInt(path_for_brick(brick_id=brick_id, env_id=env_id))
            for env_id, brick_id in zip(env_ids.cpu(), brick_ids.cpu())
        ], dtype=torch.int64, device=self.device) # (N,)

        self.rb_ids[env_ids, type_ids] = self.backend._sdf2rb_lookup(sdf_ids)

    def set_tracked_bricks(self, type_id: int, env_ids: torch.Tensor, brick_ids: torch.Tensor):
        """ Sets the tracked brick ids for a specific type in the specified environments.

            -1 is used to indicate that no brick is tracked in that environment.

            Args:
                type_id (int): The tracking type id.
                env_ids (torch.Tensor): Indices of environments to set the brick ids for, shape (N,).
                brick_ids (torch.Tensor): Brick ids to track in the specified environments, shape (N,).
        """
        self.brick_ids[env_ids, type_id] = brick_ids

        if self.backend is None:
            return

        sdf_ids = torch.tensor([
            sdfPathToInt(path_for_brick(brick_id=brick_id, env_id=env_id))
            if brick_id >= 0 else -1
            for env_id, brick_id in zip(env_ids.cpu(), brick_ids.cpu())
        ], dtype=torch.int64, device=self.device)
        self.rb_ids[env_ids, type_id] = self.backend._sdf2rb_lookup(sdf_ids)

    @property
    def view(self) -> Optional[RigidBodyView]:
        if self.backend is None:
            return None
        return self.backend.rb_view

    def get_transforms(self, tracking_id: int) -> torch.Tensor:
        """ Returns the transforms of the tracked bricks for a specific tracking type.

            Args:
                tracking_id (int): The tracking type id.

            Returns:
                torch.Tensor: A tensor of shape (E, 7) containing the transforms of the tracked bricks in all environments in the format (x, y, z, qx, qy, qz, qw).
        """
        result = torch.zeros((self.num_envs, 7), dtype=torch.float32, device=self.device) # (E,7)
        if (view := self.view) is not None:
            mask = (self.rb_ids[:, tracking_id] >= 0) # (E,)
            valid_rb_ids = self.rb_ids[mask, tracking_id] # (N,)
            result[mask] = view.get_transforms()[valid_rb_ids] # (N,7)
        return result
