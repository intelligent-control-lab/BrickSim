import numpy as np
import pxr.PhysxSchema as PhysxSchema
from typing import Optional
from pxr import Usd

def get_physics_scene(stage: Usd.Stage) -> Optional[PhysxSchema.PhysxSceneAPI]:
    prim: Usd.Prim
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            return PhysxSchema.PhysxSceneAPI(prim)
    return None

def inv_se3(mat: np.ndarray) -> np.ndarray:
    inv_mat = np.eye(4)
    inv_mat[:3, :3] = mat[:3, :3].T
    inv_mat[:3, 3] = -inv_mat[:3, :3] @ mat[:3, 3]
    return inv_mat

def quat_to_rot_batch(q: np.ndarray) -> np.ndarray:
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

def pose7_to_mat44_batch(pose7: np.ndarray) -> np.ndarray:
    R = quat_to_rot_batch(pose7[:,3:])
    T = np.zeros((pose7.shape[0], 4, 4), dtype=pose7.dtype)
    T[:,:3,:3] = R
    T[:,:3, 3] = pose7[:,:3]
    T[:,3,3]   = 1.0
    return T

def inv_se3_batch(T: np.ndarray) -> np.ndarray:
    R = np.transpose(T[:,:3,:3], (0,2,1))
    p = -np.einsum("bij,bj->bi", R, T[:,:3,3])
    out = np.empty_like(T)
    out[:,:3,:3] = R
    out[:,:3,3]  = p
    out[:,3,:]   = np.array([0,0,0,1])
    return out
