import logging
import carb.events
import omni.usd
import omni.physx
import omni.kit.app
import numpy as np
import omni.physx.scripts.physicsUtils as physicsUtils
from typing import Optional, Union
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from pxr.PhysicsSchemaTools._physicsSchemaTools import sdfPathToInt
from omni.physx.bindings._physx import ContactEventType
from omni.physics.tensors.impl.api import create_simulation_view, RigidBodyView
from .lego_schemes import BrickLength, PlateHeight, StudHeight
from .physx_c import buffer_from_ContactDataVector, buffer_from_ContactEventHeaderVector
from .brick_physics import DistanceTolerance, MaxPenetration, ZAngleTolerance, RequiredForce, YawTolerance, PositionTolerance, _get_physics_scene

_logger = logging.getLogger(__name__)

def _quat_to_rot_batch(q: np.ndarray) -> np.ndarray:
    """
    Convert (N,4) [qx,qy,qz,qw] to (N,3,3) rotation matrices.
    """
    qx, qy, qz, qw = q.T
    xx = 1 - 2*(qy*qy + qz*qz)
    yy = 1 - 2*(qx*qx + qz*qz)
    zz = 1 - 2*(qx*qx + qy*qy)

    xy = 2*(qx*qy);  xz = 2*(qx*qz);  yz = 2*(qy*qz)
    wx = 2*(qw*qx);  wy = 2*(qw*qy);  wz = 2*(qw*qz)

    R = np.empty((q.shape[0], 3, 3), dtype=q.dtype)
    R[:,0,0] = xx;   R[:,0,1] = xy - wz; R[:,0,2] = xz + wy
    R[:,1,0] = xy + wz; R[:,1,1] = yy;   R[:,1,2] = yz - wx
    R[:,2,0] = xz - wy; R[:,2,1] = yz + wx; R[:,2,2] = zz
    return R

def _pose7_to_mat44_batch(pose7: np.ndarray) -> np.ndarray:
    """
    (N,7) [x,y,z,qx,qy,qz,qw] -> (N,4,4) SE(3) matrices.
    """
    R = _quat_to_rot_batch(pose7[:,3:])
    T = np.zeros((pose7.shape[0], 4, 4), dtype=pose7.dtype)
    T[:,:3,:3] = R
    T[:,:3, 3] = pose7[:,:3]
    T[:,3,3]   = 1.0
    return T

def _inv_se3_batch(T: np.ndarray) -> np.ndarray:
    R = np.transpose(T[:,:3,:3], (0,2,1))
    p = -np.einsum("bij,bj->bi", R, T[:,:3,3])
    out = np.empty_like(T)
    out[:,:3,:3] = R
    out[:,:3,3]  = p
    out[:,3,:]   = np.array([0,0,0,1])
    return out

class AssemblyDetector:
    def __init__(self, brick_filter: Union[str, list[str]]):
        self.brick_filter = brick_filter

        self.physx_sim_interface = omni.physx.get_physx_simulation_interface()

        self.stage: Usd.Stage = omni.usd.get_context().get_stage()

        physx_scene = _get_physics_scene(self.stage)
        if physx_scene is None:
            raise RuntimeError("No PhysicsScene found in the stage.")
        self.dt = 1.0 / physx_scene.GetTimeStepsPerSecondAttr().Get()

        self.sim_view = create_simulation_view("numpy")
        self.rigid_body_view: RigidBodyView = self.sim_view.create_rigid_body_view(self.brick_filter)
        if self.rigid_body_view._backend is None:
            # There is no rigid body matching the filter
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
        if not self.sim_view.check():
            return False
        if self.rigid_body_view._backend and not self.rigid_body_view.check():
            return False
        return True

    def _path_id_to_id(self, actors_u64: np.ndarray) -> np.ndarray:
        """
        Converts SDF Path ids to brick indicies.
        Uses a sorted search table prepared in initialize().
        Unknown actors get value -1.
        """
        idx = np.searchsorted(self.table_path_id, actors_u64, side="left")
        idx = np.minimum(idx, self.table_path_id.size - 1)
        valid = self.table_path_id[idx] == actors_u64
        out = np.full(idx.shape, -1, dtype=int)
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

    def handle_assembly_contacts(self):
        if not self.rigid_body_view._backend:
            return []

        _contacts, _contact_data = self.physx_sim_interface.get_contact_report()
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

        pose = self.rigid_body_view.get_transforms()
        pose0 = _pose7_to_mat44_batch(pose[brick0_idx])
        pose1 = _pose7_to_mat44_batch(pose[brick1_idx])

        dim0 = self.lego_dims[brick0_idx]
        dim1 = self.lego_dims[brick1_idx]

        rel_pose = np.matmul(_inv_se3_batch(pose0), pose1)

        # Swap bricks where z<0 so brick0 always offers studs
        swap_mask = rel_pose[:,2,3] < 0
        if np.any(swap_mask):
            pose0_sw, pose1_sw = pose0[swap_mask].copy(), pose1[swap_mask].copy()
            dim0_sw, dim1_sw = dim0[swap_mask].copy(), dim1[swap_mask].copy()
            brick0_idx[swap_mask], brick1_idx[swap_mask] = brick1_idx[swap_mask], brick0_idx[swap_mask]
            pose0[swap_mask], pose1[swap_mask] = pose1_sw, pose0_sw
            dim0[swap_mask], dim1[swap_mask] = dim1_sw, dim0_sw
            rel_pose[swap_mask] = np.matmul(_inv_se3_batch(pose0[swap_mask]), pose1[swap_mask])

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
        p1_snapped = np.round(p0 + np.einsum("bij,bj->bi", R_snapped, dim1[:,:2])).astype(int)
        p_err = np.linalg.norm(p0 - p0_snapped, axis=1) * BrickLength

        # Reason: exceeding position tolerance
        keep &= (p_err <= PositionTolerance)

        overlap_x = np.maximum(0, np.minimum(dim0[:,0], np.maximum(p0_snapped[:,0], p1_snapped[:,0])) - np.maximum(0, np.minimum(p0_snapped[:,0], p1_snapped[:,0])))
        overlap_y = np.maximum(0, np.minimum(dim0[:,1], np.maximum(p0_snapped[:,1], p1_snapped[:,1])) - np.maximum(0, np.minimum(p0_snapped[:,1], p1_snapped[:,1])))

        # Reason: no overlap
        keep &= (overlap_x * overlap_y) > 0

        assembly_events = []
        for k in np.flatnonzero(keep):
            path0 = self.prim_paths[brick0_idx[k]]
            path1 = self.prim_paths[brick1_idx[k]]
            prim0 = self.stage.GetPrimAtPath(path0)
            prim1 = self.stage.GetPrimAtPath(path1)

            joint_path = f"{path0}_Conn_{prim1.GetPath().name}"
            if self.stage.GetPrimAtPath(joint_path).IsValid():
                # Reason: already assembled
                continue

            R_sn = R_snapped[k]
            assemble_xy = (p0_snapped[k] + (R_sn @ dim1[k, :2] - dim0[k, :2]) / 2) * BrickLength
            assemble_tr = np.array([
                [R_sn[0,0], R_sn[0,1],  0, assemble_xy[0]   ],
                [R_sn[1,0], R_sn[1,1],  0, assemble_xy[1]   ],
                [0,         0,          1, height0[k]       ],
                [0,         0,          0, 1                ],
            ])
            assemble_tr_gf = Gf.Matrix4f(assemble_tr.T)

            xformable1 = UsdGeom.Xformable(prim1)
            parent_pose1 = np.array(xformable1.ComputeParentToWorldTransform(Usd.TimeCode.Default())).T
            assemble_rel_pose1 = _inv_se3_batch(parent_pose1[None])[0] @ pose0[k] @ assemble_tr
            assemble_rel_pose1_gf = Gf.Matrix4d(assemble_rel_pose1.T)

            physicsUtils.set_or_add_translate_op(xformable1, assemble_rel_pose1_gf.ExtractTranslation())
            physicsUtils.set_or_add_orient_op(xformable1, assemble_rel_pose1_gf.ExtractRotationQuat())

            joint = UsdPhysics.FixedJoint.Define(self.stage, joint_path)
            joint.CreateBody0Rel().AddTarget(prim0.GetPath())
            joint.CreateBody1Rel().AddTarget(prim1.GetPath())
            joint.CreateLocalPos0Attr().Set(assemble_tr_gf.ExtractTranslation())
            joint.CreateLocalRot0Attr().Set(assemble_tr_gf.ExtractRotationQuat())

            filtered_pairs1 = UsdPhysics.FilteredPairsAPI.Apply(prim1)
            filtered_pairs1.CreateFilteredPairsRel().AddTarget(prim0.GetPath())

            assembly_events.append({
                "brick0": prim0.GetPath().pathString,
                "brick1": prim1.GetPath().pathString,
                "joint": joint_path,
                "p0": p0_snapped[k].tolist(),
                "p1": p1_snapped[k].tolist(),
                "yaw": float(snapped_yaw[k]),
            })

        return assembly_events

class BrickPhysicsInterface:
    def __init__(self, brick_filter: Union[str, list[str]] = "/World/Brick_*"):
        self.brick_filter = brick_filter
        self.brick_physics = None
        self.dirty = False
        update_bus: carb.events.IEventStream = omni.kit.app.get_app().get_update_event_stream()
        self.update_sub = update_bus.create_subscription_to_push(self._on_update)

    def mark_dirty(self):
        self.dirty = True

    def _on_update(self, event: carb.events.IEvent):
        if not omni.physx.get_physx_interface().is_running():
            return
        current_stage = omni.usd.get_context().get_stage()
        if self.brick_physics is None or not self.brick_physics.check() or self.dirty:
            if current_stage is None or _get_physics_scene(current_stage) is None:
                return
            _logger.info("Initializing AssemblyDetector")
            omni.physx.get_physx_interface().force_load_physics_from_usd()
            self.brick_physics = AssemblyDetector(self.brick_filter)
            self.dirty = False

        assembly_events = self.brick_physics.handle_assembly_contacts()
        for event in assembly_events:
            _logger.info(
                f"Assembly event: {event['brick0']} and {event['brick1']}, "
                f"joint={event['joint']}, "
                f"p_0=({event['p0'][0]}, {event['p0'][1]}), "
                f"p_1=({event['p1'][0]}, {event['p1'][1]}), "
                f"yaw={np.degrees(event['yaw'])}"
            )

    def destroy(self):
        self.update_sub.unsubscribe()
