import math
import numpy as np
import omni.physx.scripts.physicsUtils as physicsUtils
from dataclasses import dataclass
from typing import Optional
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from .lego_schemes import BrickLength
from .utils import inv_se3

DistanceTolerance = 0.001           # Maximum distance between bricks (m)
MaxPenetration = 0.005              # Maximum penetration between bricks (m), penetration can happen due to simulation inaccuracies
ZAngleTolerance = math.radians(5)   # Maximum angle between z-axis of bricks (rad)
RequiredForce = 1.0                 # Minimum clutch power (N)
YawTolerance = math.radians(5)      # Maximum yaw error (rad)
PositionTolerance = 0.002           # Maximum position error (m)

@dataclass
class AssemblyEvent:
    env_id: Optional[int]
    brick0: int
    brick1: int
    p0: tuple[int, int]
    p1: tuple[int, int]
    yaw: float

def path_for_brick(brick_id: int, env_id: Optional[int] = None) -> str:
    if env_id is None:
        return f"/World/Brick_{brick_id}"
    else:
        return f"/World/envs/env_{env_id}/Brick_{brick_id}"

def parse_brick_path(path: str) -> tuple[int, Optional[int]]:
    if path.startswith("/World/Brick_"):
        brick_id = int(path[len("/World/Brick_"):])
        env_id = None
    elif path.startswith("/World/envs/env_"):
        env_id = int(path[len("/World/envs/env_"):path.index("/", len("/World/envs/env_"))])
        brick_id = int(path[path.index("Brick_")+len("Brick_"):])
    else:
        raise ValueError(f"Invalid brick path: {path}")
    return brick_id, env_id

def path_for_conn(brick0: int, brick1: int, env_id: Optional[int] = None) -> str:
    if env_id is None:
        return f"/World/Conn_{brick0}_{brick1}"
    else:
        return f"/World/envs/env_{env_id}/Conn_{brick0}_{brick1}"

def parse_conn_path(path: str) -> tuple[int, int, Optional[int]]:
    if path.startswith("/World/Conn_"):
        env_id = None
        brick0, brick1 = map(int, path[len("/World/Conn_"):].split("_"))
    elif path.startswith("/World/envs/env_"):
        env_id = int(path[len("/World/envs/env_"):path.index("/", len("/World/envs/env_"))])
        brick0, brick1 = map(int, path[path.index("Conn_")+len("Conn_"):].split("_"))
    else:
        raise ValueError(f"Invalid connection path: {path}")
    return brick0, brick1, env_id

def assemble_bricks(
    stage: Usd.Stage,
    prim0: Usd.Prim, prim1: Usd.Prim,
    dim0: np.ndarray, dim1: np.ndarray,
    pose0: np.ndarray,
    p0_snapped: np.ndarray, p1_snapped: np.ndarray,
    R_snapped: np.ndarray,
    height0: float,
    snapped_yaw: float,
) -> Optional[AssemblyEvent]:

    brick_id0, env_id0 = parse_brick_path(prim0.GetPath().pathString)
    brick_id1, env_id1 = parse_brick_path(prim1.GetPath().pathString)
    if env_id0 != env_id1:
        raise ValueError(f"Bricks are in different environments: {prim0.GetPath()} and {prim1.GetPath()}")

    joint_path = path_for_conn(brick_id0, brick_id1, env_id0)
    if stage.GetPrimAtPath(joint_path).IsValid():
        # Already assembled
        return None

    assemble_xy = (p0_snapped + (R_snapped @ dim1[:2] - dim0[:2]) / 2) * BrickLength
    assemble_tr = np.array([
        [R_snapped[0,0],    R_snapped[0,1], 0, assemble_xy[0]   ],
        [R_snapped[1,0],    R_snapped[1,1], 0, assemble_xy[1]   ],
        [0,                 0,              1, height0          ],
        [0,                 0,              0, 1                ],
    ])
    assemble_tr_gf = Gf.Matrix4f(assemble_tr.T)

    xformable1 = UsdGeom.Xformable(prim1)
    parent_pose1 = np.array(xformable1.ComputeParentToWorldTransform(Usd.TimeCode.Default())).T
    assemble_rel_pose1 = inv_se3(parent_pose1) @ pose0 @ assemble_tr
    assemble_rel_pose1_gf = Gf.Matrix4d(assemble_rel_pose1.T)

    physicsUtils.set_or_add_translate_op(xformable1, assemble_rel_pose1_gf.ExtractTranslation())
    physicsUtils.set_or_add_orient_op(xformable1, assemble_rel_pose1_gf.ExtractRotationQuat())

    joint: UsdPhysics.FixedJoint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().AddTarget(prim0.GetPath())
    joint.CreateBody1Rel().AddTarget(prim1.GetPath())
    joint.CreateLocalPos0Attr().Set(assemble_tr_gf.ExtractTranslation())
    joint.CreateLocalRot0Attr().Set(assemble_tr_gf.ExtractRotationQuat())

    filtered_pairs1: UsdPhysics.FilteredPairsAPI = UsdPhysics.FilteredPairsAPI.Apply(prim1)
    filtered_pairs1.CreateFilteredPairsRel().AddTarget(prim0.GetPath())

    return AssemblyEvent(
        env_id=env_id0,
        brick0=brick_id0,
        brick1=brick_id1,
        p0=(p0_snapped[0], p0_snapped[1]),
        p1=(p1_snapped[0], p1_snapped[1]),
        yaw=float(snapped_yaw),
    )
