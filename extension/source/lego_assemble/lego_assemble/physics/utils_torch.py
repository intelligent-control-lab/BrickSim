import torch

@torch.jit.script
def quat_to_rot_batch(q: torch.Tensor) -> torch.Tensor:
    qx = q[:,0]; qy = q[:,1];  qz = q[:,2];  qw = q[:,3]
    xx = 1 - 2*(qy*qy + qz*qz)
    yy = 1 - 2*(qx*qx + qz*qz)
    zz = 1 - 2*(qx*qx + qy*qy)

    xy = 2*(qx*qy);  xz = 2*(qx*qz);  yz = 2*(qy*qz)
    wx = 2*(qw*qx);  wy = 2*(qw*qy);  wz = 2*(qw*qz)

    R = torch.empty((q.shape[0], 3, 3), dtype=q.dtype, device=q.device)
    R[:,0,0] = xx;      R[:,0,1] = xy - wz; R[:,0,2] = xz + wy
    R[:,1,0] = xy + wz; R[:,1,1] = yy;      R[:,1,2] = yz - wx
    R[:,2,0] = xz - wy; R[:,2,1] = yz + wx; R[:,2,2] = zz
    return R

@torch.jit.script
def pose7_to_mat44_batch(pose7: torch.Tensor) -> torch.Tensor:
    R = quat_to_rot_batch(pose7[:,3:])
    T = torch.zeros((pose7.shape[0], 4, 4), dtype=pose7.dtype, device=pose7.device)
    T[:,:3,:3] = R
    T[:,:3, 3] = pose7[:,:3]
    T[:,3,3]   = 1.0
    return T

@torch.jit.script
def inv_se3_batch(T: torch.Tensor) -> torch.Tensor:
    R = T[:,:3,:3].transpose(1,2)
    p = -torch.einsum("bij,bj->bi", R, T[:,:3,3])
    out = torch.empty_like(T)
    out[:,:3,:3] = R
    out[:,:3,3]  = p
    out[:,3,:]   = torch.tensor([0,0,0,1], dtype=T.dtype, device=T.device)
    return out

@torch.jit.script
def quat_z_cos_batch(q: torch.Tensor) -> torch.Tensor:
    qx = q[:,0]; qy = q[:,1]
    return 1 - 2*(qx*qx + qy*qy)
