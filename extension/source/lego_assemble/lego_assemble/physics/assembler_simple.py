import omni.usd
import omni.physx
import numpy as np
from typing import Optional
from functools import cache
from scipy.spatial.transform import Rotation
from pxr import Usd
from pxr.PhysicsSchemaTools._physicsSchemaTools import intToSdfPath
from omni.physx.bindings._physx import ContactEventHeader, ContactEventType, ContactData
from .lego_schemes import BrickLength, PlateHeight, StudHeight
from .assembler import DistanceTolerance, MaxPenetration, ZAngleTolerance, RequiredForce, YawTolerance, PositionTolerance, AssemblyEvent, assemble_bricks
from .utils import get_physics_scene, inv_se3

def _get_rigidbody_transform(physx: omni.physx.PhysX, path: str) -> Optional[np.ndarray]:
    result = physx.get_rigidbody_transformation(path)
    if not result["ret_val"]:
        return None
    position = result["position"]
    rotation = result["rotation"]
    transform = np.eye(4)
    transform[:3, 3] = position
    transform[:3, :3] = Rotation.from_quat(rotation).as_matrix()
    return transform

def handle_assembly_contacts() -> list[AssemblyEvent]:
    physx = omni.physx.get_physx_interface()
    if not physx.is_running():
        return []

    stage: Usd.Stage = omni.usd.get_context().get_stage()
    if stage is None:
        return []
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
        pose = _get_rigidbody_transform(physx, path)
        if pose is None:
            return None
        return prim, dim, pose

    assembly_events: list[AssemblyEvent] = []
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
        height0 = dim0[2] * PlateHeight

        # The distance between the bricks, must be within a threshold to triger assembly
        # Can be negative if penetration occurs
        rel_distance = rel_z - (height0 + StudHeight)
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
        # Calculate p0: the 2D position (x,y) of brick1's "min-corner" (e.g., bottom-left)
        # relative to brick0's "min-corner", expressed in stud units.
        # The origin of this p0 coordinate system is the min-corner of brick0.
        #
        # Breakdown:
        #   rel_pose[:2,3]: XY translation of brick1's center relative to brick0's center (in physics units).
        #   BrickLength: Conversion factor from physics units to stud units.
        #   rel_pose[:2,3] / BrickLength: XY translation of brick1's center relative to brick0's center (in stud units).
        #   dim0[:2]: [width, depth] of brick0 in stud units.
        #   dim1[:2]: [width, depth] of brick1 in stud units.
        #   rel_pose[:2,:2]: 2D rotation matrix of brick1 relative to brick0.
        #   rel_pose[:2,:2] @ dim1[:2]: Dimensions of brick1 rotated into brick0's frame.
        #   (dim0[:2] - rel_pose[:2,:2] @ dim1[:2]) / 2: This term converts the center-relative position
        #                                                to a min-corner-relative position.
        #                                                It's equivalent to: (dim0[:2]/2) - (rotated_dim1[:2]/2)
        #                                                which is (offset_from_brick0_center_to_its_min_corner) - (offset_from_brick1_center_to_its_min_corner_in_brick0_frame)
        p0 = rel_pose[:2,3] / BrickLength + (dim0[:2] - rel_pose[:2,:2] @ dim1[:2]) / 2

        # Snap p0 (the calculated min-corner position of brick1) to the nearest stud grid point.
        # p0_snapped represents the integer grid coordinates of brick1's min-corner.
        p0_snapped = np.round(p0).astype(int)

        # Define the 2D rotation matrix for the "snapped" yaw.
        # snapped_yaw is the relative yaw angle between bricks, rounded to the nearest 90 degrees (pi/2 radians).
        R_snapped = np.array([[np.cos(snapped_yaw), -np.sin(snapped_yaw)], [np.sin(snapped_yaw), np.cos(snapped_yaw)]])

        # Calculate p1_snapped: the 2D position of brick1's "max-corner" (e.g., top-right)
        # after snapping, in brick0's stud grid (where brick0's min-corner is the origin).
        # This is found by taking an initial corner position (p0 - note: uses unsnapped p0),
        # adding brick1's dimensions rotated by the snapped_yaw, and then rounding to the grid.
        # p0: unsnapped min-corner of brick1 relative to min-corner of brick0.
        # R_snapped @ dim1[:2]: brick1's dimensions [width, depth] rotated by the snapped angle.
        #                     This vector points from brick1's min-corner to its max-corner in the snapped orientation.
        p1_snapped = np.round(p0 + R_snapped @ dim1[:2]).astype(int)
        p_err = p0 - p0_snapped
        if np.linalg.norm(p_err) * BrickLength > PositionTolerance:
            # Reason: exceeding position tolerance
            continue

        # --- Overlap Calculation ---
        # The goal is to find the overlapping area in studs between brick0 and the snapped brick1.
        # Brick0's extent in its own stud grid (min-corner at origin):
        #   x-axis: [0, dim0[0]]
        #   y-axis: [0, dim0[1]]
        # Brick1's extent in brick0's stud grid (after snapping):
        #   x-axis: [min(p0_snapped[0], p1_snapped[0]), max(p0_snapped[0], p1_snapped[0])]
        #   y-axis: [min(p0_snapped[1], p1_snapped[1]), max(p0_snapped[1], p1_snapped[1])]
        # The formula used is the standard 1D interval overlap: max(0, min(end1, end2) - max(start1, start2))
        overlap_x = max(0, min(dim0[0], max(p0_snapped[0], p1_snapped[0])) - max(0, min(p0_snapped[0], p1_snapped[0])))
        overlap_y = max(0, min(dim0[1], max(p0_snapped[1], p1_snapped[1])) - max(0, min(p0_snapped[1], p1_snapped[1])))
        overlap_area = overlap_x * overlap_y
        if overlap_area <= 0:
            # Reason: no overlap
            continue

        event = assemble_bricks(
            stage=stage,
            prim0=prim0, prim1=prim1,
            dim0=dim0, dim1=dim1,
            pose0=pose0,
            p0_snapped=p0_snapped, p1_snapped=p1_snapped,
            R_snapped=R_snapped,
            height0=height0,
            snapped_yaw=snapped_yaw,
        )
        if event is not None:
            assembly_events.append(event)

    return assembly_events
