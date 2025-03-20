import logging
import omni.usd
import omni.physx
import numpy as np
import omni.physx.scripts.physicsUtils as physicsUtils
from typing import Optional
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools
from omni.physx.bindings._physx import ContactEventHeaderVector, ContactDataVector, ContactEventHeader, ContactEventType, ContactData
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

def contact_report_event_handler(contact_headers: ContactEventHeaderVector, contact_data: ContactDataVector):
    # Prevent crashing pybind
    try:
        _contact_report_event_handler(contact_headers, contact_data)
    except Exception as e:
        logger.critical(f"Error in contact_report_event_handler: {e}")

def _contact_report_event_handler(contact_headers: ContactEventHeaderVector, contact_data: ContactDataVector):
    stage: Usd.Stage = omni.usd.get_context().get_stage()
    if stage is None:
        return

    physics_scene = get_physics_scene(stage)
    if physics_scene is None:
        logger.warning("No PhysxScene found")
        return
    sim_timestep = 1.0 / physics_scene.GetTimeStepsPerSecondAttr().Get()

    contact: ContactEventHeader
    for contact in contact_headers:
        if contact.type == ContactEventType.CONTACT_LOST:
            continue
        actor0: Sdf.Path = PhysicsSchemaTools.intToSdfPath(contact.actor0)
        actor1: Sdf.Path = PhysicsSchemaTools.intToSdfPath(contact.actor1)
        prim0 = stage.GetPrimAtPath(actor0.GetPrimPath())
        prim1 = stage.GetPrimAtPath(actor1.GetPrimPath())
        if prim0 is None or prim1 is None:
            continue
        dim_attr0 = prim0.GetAttribute("lego_dimensions")
        dim_attr1 = prim1.GetAttribute("lego_dimensions")
        if not dim_attr0 or not dim_attr1:
            continue
        dim0 = np.array(dim_attr0.Get(), dtype=int)
        dim1 = np.array(dim_attr1.Get(), dtype=int)

        pose0 = np.array(omni.usd.get_world_transform_matrix(prim0)).T
        pose1 = np.array(omni.usd.get_world_transform_matrix(prim1)).T
        rel_pose = np.linalg.inv(pose0) @ pose1
        rel_z = rel_pose[2,3]

        # Swap the order of the bricks, so prim0 is always the one offering studs
        if rel_z < 0:
            prim0, prim1 = prim1, prim0
            dim0, dim1 = dim1, dim0
            pose0, pose1 = pose1, pose0
            rel_pose = np.linalg.inv(pose0) @ pose1
            rel_z = rel_pose[2,3]
        # Delete other variables so we don't accidentally use them
        del actor0, actor1, dim_attr0, dim_attr1

        # The height of the first brick
        height0 = dim0[2] * lego_schemes.PlateHeight

        # The distance between the bricks, must be within a threshold to triger assembly
        # Can be negative if penetration occurs
        rel_distance = rel_z - height0

        # The angle between the z-axis of the first brick and the z-axis of the second brick
        # Must be within a threshold to triger assembly
        rel_z_angle = np.arccos(rel_pose[2,2])

        impulse = np.zeros(3)
        contact_pt: ContactData
        for contact_pt in contact_data[contact.contact_data_offset:contact.contact_data_offset+contact.num_contact_data]:
            impulse += contact_pt.impulse
        # The contact force along the z-axis of the first brick
        frc_prj = np.abs(np.dot(impulse, pose0[:3,2])) / sim_timestep

        # Angle between the x-axis of the first brick and the x-axis of the second brick
        rel_yaw = np.arctan2(rel_pose[1,0], rel_pose[0,0])
        # Expected angle after assembly, can be -pi, -pi/2, 0, pi/2, pi
        snapped_yaw = np.round(rel_yaw / (np.pi/2)) * (np.pi/2)
        # Error withing [-pi/4, pi/4]
        yaw_err = rel_yaw - snapped_yaw

        # Relative position of the second brick in studs
        p0 = rel_pose[:2,3] / lego_schemes.BrickLength + (dim0[:2] - rel_pose[:2,:2] @ dim1[:2]) / 2 
        p0_snapped = np.round(p0).astype(int)
        R_snapped = np.array([[np.cos(snapped_yaw), -np.sin(snapped_yaw)], [np.sin(snapped_yaw), np.cos(snapped_yaw)]])
        p1_snapped = np.round(p0 + R_snapped @ dim1[:2]).astype(int)
        p_err = p0 - p0_snapped

        overlap_x = max(0, min(dim0[0], max(p0_snapped[0], p1_snapped[0])) - max(0, min(p0_snapped[0], p1_snapped[0])))
        overlap_y = max(0, min(dim0[1], max(p0_snapped[1], p1_snapped[1])) - max(0, min(p0_snapped[1], p1_snapped[1])))
        overlap_area = overlap_x * overlap_y

        assemble_xy = (p0_snapped + (R_snapped @ dim1[:2] - dim0[:2]) / 2) * lego_schemes.BrickLength
        assemble_tr = np.array([
            [R_snapped[0,0], R_snapped[0,1], 0, assemble_xy[0]  ],
            [R_snapped[1,0], R_snapped[1,1], 0, assemble_xy[1]  ],
            [0,              0,              1, height0         ],
            [0,              0,              0, 1               ],
        ])
        parent_pose1= np.array(UsdGeom.Xformable(prim1).ComputeParentToWorldTransform(Usd.TimeCode.Default())).T
        desired_tr = np.linalg.inv(parent_pose1) @ pose0 @ assemble_tr

        if rel_distance > DistanceTolerance:
            status = "exceeding distance tolerance"
        elif rel_distance < -MaxPenetration:
            status = "penetrating too much"
        elif rel_z_angle > ZAngleTolerance:
            status = "exceeding z-angle tolerance"
        elif abs(yaw_err) > YawTolerance:
            status = "exceeding yaw tolerance"
        elif np.linalg.norm(p_err) * lego_schemes.BrickLength > PositionTolerance:
            status = "exceeding position tolerance"
        elif overlap_area <= 0:
            status = "no overlap"
        elif frc_prj < RequiredForce:
            status = "insufficient force"
        else:
            physicsUtils.set_or_add_translate_op(prim1, desired_tr[:3,3].tolist())
            physicsUtils.set_or_add_orient_op(prim1, Gf.Matrix3d(desired_tr[:3,:3].T).ExtractRotation().GetQuat())
            joint_path = f"{prim0.GetPath()}_Conn_{prim1.GetPath().name}"
            if stage.GetPrimAtPath(joint_path).IsValid():
                status = "already assembled"
            else:
                joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
                joint.CreateBody0Rel().AddTarget(prim0.GetPath())
                joint.CreateBody1Rel().AddTarget(prim1.GetPath())
                status = "assembly"

        logger.info(
            f"Contact between {prim0.GetPath()} ({dim0[0]}x{dim0[1]}x{dim0[2]}) "
            f"and {prim1.GetPath()} ({dim1[0]}x{dim1[1]}x{dim1[2]}), "
            f"distance={rel_distance:.3f} m, "
            f"z-angle={np.degrees(rel_z_angle):.3f} deg, "
            f"force={frc_prj:.3f} N, "
            f"p_0=({p0_snapped[0]}, {p0_snapped[1]}), "
            f"p_1=({p1_snapped[0]}, {p1_snapped[1]}), "
            f"p_err=({p_err[0]:.3f}, {p_err[1]:.3f}), "
            f"rel_yaw={np.degrees(snapped_yaw):.3f} ({np.degrees(yaw_err):.3f}) deg, "
            f"overlap={overlap_x}x{overlap_y}, "
            f"status: {status}"
        )

class LegoPhysicsCallback:
    def __init__(self):
        self.contact_report_event_sub = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact_report_event_handler)

    def unsubscribe(self):
        self.contact_report_event_sub.unsubscribe()
