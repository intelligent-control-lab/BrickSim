import carb
import logging
import carb.events
import omni.usd
import omni.physx
import omni.kit.app
import numpy as np
import omni.physx.scripts.physicsUtils as physicsUtils
from typing import Optional
from functools import cache
from scipy.spatial.transform import Rotation
from pxr import Gf, Usd, UsdGeom, UsdPhysics
import pxr.PhysxSchema as PhysxSchema
from pxr.PhysicsSchemaTools._physicsSchemaTools import intToSdfPath
from omni.physx.bindings._physx import ContactEventHeader, ContactEventType, ContactData
from . import lego_schemes

DistanceTolerance = 0.001           # Maximum distance between bricks (m)
MaxPenetration = 0.005              # Maximum penetration between bricks (m), penetration can happen due to simulation inaccuracies
ZAngleTolerance = np.radians(5)     # Maximum angle between z-axis of bricks (rad)
RequiredForce = 1.0                 # Minimum clutch power (N)
YawTolerance = np.radians(5)        # Maximum yaw error (rad)
PositionTolerance = 0.002           # Maximum position error (m)

logger = logging.getLogger(__name__)

def get_physics_scene(stage: Usd.Stage) -> Optional[PhysxSchema.PhysxSceneAPI]:
    prim: Usd.Prim
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            return PhysxSchema.PhysxSceneAPI(prim)
    return None

def get_rigidbody_transform(physx: omni.physx.PhysX, path: str) -> Optional[np.ndarray]:
    result = physx.get_rigidbody_transformation(path)
    if not result["ret_val"]:
        return None
    position = result["position"]
    rotation = result["rotation"]
    transform = np.eye(4)
    transform[:3, 3] = position
    transform[:3, :3] = Rotation.from_quat(rotation).as_matrix()
    return transform

def inv_se3(mat: np.ndarray) -> np.ndarray:
    inv_mat = np.eye(4)
    inv_mat[:3, :3] = mat[:3, :3].T
    inv_mat[:3, 3] = -inv_mat[:3, :3] @ mat[:3, 3]
    return inv_mat

def handle_assembly_contacts():
    physx = omni.physx.get_physx_interface()
    if not physx.is_running():
        return []

    stage: Usd.Stage = omni.usd.get_context().get_stage()
    physics_scene = get_physics_scene(stage)
    if physics_scene is None:
        return []
    sim_timestep = 1.0 / physics_scene.GetTimeStepsPerSecondAttr().Get()

    @cache
    def get_brick_data(actor: int):
        path = intToSdfPath(actor).pathString
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None
        dim_attr = prim.GetAttribute("lego_dimensions")
        if not dim_attr.IsValid():
            return None
        dim = np.array(dim_attr.Get(), dtype=int)
        pose = get_rigidbody_transform(physx, path)
        if pose is None:
            return None
        return prim, dim, pose

    assembly_events = []
    contact_headers, contact_data = omni.physx.get_physx_simulation_interface().get_contact_report()
    contact: ContactEventHeader
    for contact in contact_headers:
        if contact.type == ContactEventType.CONTACT_LOST:
            continue
        brick_data0 = get_brick_data(contact.actor0)
        brick_data1 = get_brick_data(contact.actor1)
        if brick_data0 is None or brick_data1 is None:
            continue
        prim0, dim0, pose0 = brick_data0
        prim1, dim1, pose1 = brick_data1

        rel_pose = inv_se3(pose0) @ pose1
        rel_z = rel_pose[2,3]

        # Swap the order of the bricks, so prim0 is always the one offering studs
        if rel_z < 0:
            prim0, prim1 = prim1, prim0
            dim0, dim1 = dim1, dim0
            pose0, pose1 = pose1, pose0
            rel_pose = inv_se3(pose0) @ pose1
            rel_z = rel_pose[2,3]
        # Delete other variables so we don't accidentally use them
        del brick_data0, brick_data1

        # The height of the first brick
        height0 = dim0[2] * lego_schemes.PlateHeight

        # The distance between the bricks, must be within a threshold to triger assembly
        # Can be negative if penetration occurs
        rel_distance = rel_z - (height0 + lego_schemes.StudHeight)
        if rel_distance > DistanceTolerance:
            # Reason: exceeding distance tolerance
            continue
        elif rel_distance < -MaxPenetration:
            # Reason: penetrating too much
            continue

        # The angle between the z-axis of the first brick and the z-axis of the second brick
        # Must be within a threshold to triger assembly
        rel_z_angle = np.arccos(rel_pose[2,2])
        if rel_z_angle > ZAngleTolerance:
            # Reason: exceeding z-angle tolerance
            continue

        impulse = np.zeros(3)
        contact_pt: ContactData
        for contact_pt in contact_data[contact.contact_data_offset:contact.contact_data_offset+contact.num_contact_data]:
            impulse += contact_pt.impulse
        # The contact force along the z-axis of the first brick
        frc_prj = np.abs(np.dot(impulse, pose0[:3,2])) / sim_timestep
        if frc_prj < RequiredForce:
            # Reason: insufficient force
            continue

        # Angle between the x-axis of the first brick and the x-axis of the second brick
        rel_yaw = np.arctan2(rel_pose[1,0], rel_pose[0,0])
        # Expected angle after assembly, can be -pi, -pi/2, 0, pi/2, pi
        snapped_yaw = np.round(rel_yaw / (np.pi/2)) * (np.pi/2)
        # Error withing [-pi/4, pi/4]
        yaw_err = rel_yaw - snapped_yaw
        if abs(yaw_err) > YawTolerance:
            # Reason: exceeding yaw tolerance
            continue

        # Relative position of the second brick in studs
        p0 = rel_pose[:2,3] / lego_schemes.BrickLength + (dim0[:2] - rel_pose[:2,:2] @ dim1[:2]) / 2 
        p0_snapped = np.round(p0).astype(int)
        R_snapped = np.array([[np.cos(snapped_yaw), -np.sin(snapped_yaw)], [np.sin(snapped_yaw), np.cos(snapped_yaw)]])
        p1_snapped = np.round(p0 + R_snapped @ dim1[:2]).astype(int)
        p_err = p0 - p0_snapped
        if np.linalg.norm(p_err) * lego_schemes.BrickLength > PositionTolerance:
            # Reason: exceeding position tolerance
            continue

        overlap_x = max(0, min(dim0[0], max(p0_snapped[0], p1_snapped[0])) - max(0, min(p0_snapped[0], p1_snapped[0])))
        overlap_y = max(0, min(dim0[1], max(p0_snapped[1], p1_snapped[1])) - max(0, min(p0_snapped[1], p1_snapped[1])))
        overlap_area = overlap_x * overlap_y
        if overlap_area <= 0:
            # Reason: no overlap
            continue

        joint_path = f"{prim0.GetPath()}_Conn_{prim1.GetPath().name}"
        if stage.GetPrimAtPath(joint_path).IsValid():
            # Reason: already assembled
            continue

        assemble_xy = (p0_snapped + (R_snapped @ dim1[:2] - dim0[:2]) / 2) * lego_schemes.BrickLength
        assemble_tr = np.array([
            [R_snapped[0,0], R_snapped[0,1], 0, assemble_xy[0]  ],
            [R_snapped[1,0], R_snapped[1,1], 0, assemble_xy[1]  ],
            [0,              0,              1, height0         ],
            [0,              0,              0, 1               ],
        ])
        assemble_tr_gf = Gf.Matrix4f(assemble_tr.T)

        xformable1 = UsdGeom.Xformable(prim1)
        parent_pose1 = np.array(xformable1.ComputeParentToWorldTransform(Usd.TimeCode.Default())).T
        assemble_rel_pose1 = inv_se3(parent_pose1) @ pose0 @ assemble_tr
        assemble_rel_pose1_gf = Gf.Matrix4d(assemble_rel_pose1.T)

        physicsUtils.set_or_add_translate_op(xformable1, assemble_rel_pose1_gf.ExtractTranslation())
        physicsUtils.set_or_add_orient_op(xformable1, assemble_rel_pose1_gf.ExtractRotationQuat())

        joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
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
            "p0": p0_snapped,
            "p1": p1_snapped,
            "yaw": snapped_yaw,
        })

        # logger.info(
        #     f"Contact between {prim0.GetPath()} ({dim0[0]}x{dim0[1]}x{dim0[2]}) "
        #     f"and {prim1.GetPath()} ({dim1[0]}x{dim1[1]}x{dim1[2]}), "
        #     f"distance={rel_distance:.3f} m, "
        #     f"z-angle={np.degrees(rel_z_angle):.3f} deg, "
        #     f"force={frc_prj:.3f} N, "
        #     f"p_0=({p0_snapped[0]}, {p0_snapped[1]}), "
        #     f"p_1=({p1_snapped[0]}, {p1_snapped[1]}), "
        #     f"p_err=({p_err[0]:.3f}, {p_err[1]:.3f}), "
        #     f"rel_yaw={np.degrees(snapped_yaw):.3f} ({np.degrees(yaw_err):.3f}) deg, "
        #     f"overlap={overlap_x}x{overlap_y}, "
        #     f"status: {status}"
        # )

    return assembly_events

class LegoPhysicsCallback:
    def __init__(self):
        update_bus: carb.events.IEventStream = omni.kit.app.get_app().get_update_event_stream()
        self.update_sub = update_bus.create_subscription_to_push(self.on_update)

    def on_update(self, event: carb.events.IEvent):
        assembly_events = handle_assembly_contacts()
        for event in assembly_events:
            logger.info(
                f"Assembly event: {event['brick0']} and {event['brick1']}, "
                f"joint={event['joint']}, "
                f"p_0=({event['p0'][0]}, {event['p0'][1]}), "
                f"p_1=({event['p1'][0]}, {event['p1'][1]}), "
                f"yaw={np.degrees(event['yaw'])}"
            )

    def unsubscribe(self):
        self.update_sub.unsubscribe()
