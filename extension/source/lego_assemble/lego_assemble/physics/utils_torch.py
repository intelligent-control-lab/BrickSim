import torch

@torch.jit.script
def quat_to_rotmat(q: torch.Tensor) -> torch.Tensor:
    """
    Converts a batch of quaternions (xyzw) to a batch of 3x3 rotation matrices.
    Args:
        q: A tensor of shape (N, 4) with quaternions in xyzw format.
    Returns:
        A tensor of shape (N, 3, 3) representing the rotation matrices.
    """
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
def pose_to_se3(pose: torch.Tensor) -> torch.Tensor:
    """
    Converts a batch of 7-element poses (3 translation, 4 quaternion) to 
    a batch of 4x4 SE(3) transformation matrices.
    Args:
        pose: A tensor of shape (N, 7) with poses (x, y, z, qx, qy, qz, qw).
    Returns:
        A tensor of shape (N, 4, 4) representing the SE(3) matrices.
    """
    R = quat_to_rotmat(pose[:,3:])
    T = torch.zeros((pose.shape[0], 4, 4), dtype=pose.dtype, device=pose.device)
    T[:,:3,:3] = R
    T[:,:3, 3] = pose[:,:3]
    T[:, 3, 3] = 1.0
    return T

@torch.jit.script
def inv_se3(T: torch.Tensor) -> torch.Tensor:
    """
    Computes the inverse of a batch of 4x4 SE(3) transformation matrices.
    Args:
        T: A tensor of shape (N, 4, 4) representing the SE(3) matrices.
    Returns:
        A tensor of shape (N, 4, 4) representing the inverse SE(3) matrices.
    """
    R = T[:,:3,:3].transpose(1,2)
    p = -torch.einsum("bij,bj->bi", R, T[:,:3,3])
    out = torch.empty_like(T)
    out[:,:3,:3] = R
    out[:,:3,3]  = p
    out[:,3,:]   = torch.tensor([0,0,0,1], dtype=T.dtype, device=T.device)
    return out

@torch.jit.script
def rotmat_to_quat(R: torch.Tensor) -> torch.Tensor:
    """
    Converts a batch of 3x3 rotation matrices to a batch of quaternions (xyzw).
    This implementation is numerically stable and handles all cases.
    Args:
        R: A tensor of shape (N, 3, 3) representing the rotation matrices.
    Returns:
        A tensor of shape (N, 4) with quaternions in xyzw format.
    """
    # Unpack diagonal and off-diagonal elements
    R00, R01, R02 = R[:, 0, 0], R[:, 0, 1], R[:, 0, 2]
    R10, R11, R12 = R[:, 1, 0], R[:, 1, 1], R[:, 1, 2]
    R20, R21, R22 = R[:, 2, 0], R[:, 2, 1], R[:, 2, 2]

    # Calculate the trace of the matrix
    trace = R00 + R11 + R22
    
    # Pre-calculate values for the four cases
    # Case 1: trace > 0
    s1 = torch.sqrt(torch.clamp(trace + 1.0, min=1e-8)) * 2.0
    qw1 = 0.25 * s1
    qx1 = (R21 - R12) / s1
    qy1 = (R02 - R20) / s1
    qz1 = (R10 - R01) / s1
    q1 = torch.stack([qx1, qy1, qz1, qw1], dim=1)

    # Case 2: R00 is the largest diagonal element
    s2 = torch.sqrt(torch.clamp(1.0 + R00 - R11 - R22, min=1e-8)) * 2.0
    qw2 = (R21 - R12) / s2
    qx2 = 0.25 * s2
    qy2 = (R01 + R10) / s2
    qz2 = (R02 + R20) / s2
    q2 = torch.stack([qx2, qy2, qz2, qw2], dim=1)

    # Case 3: R11 is the largest diagonal element
    s3 = torch.sqrt(torch.clamp(1.0 + R11 - R00 - R22, min=1e-8)) * 2.0
    qw3 = (R02 - R20) / s3
    qx3 = (R01 + R10) / s3
    qy3 = 0.25 * s3
    qz3 = (R12 + R21) / s3
    q3 = torch.stack([qx3, qy3, qz3, qw3], dim=1)
    
    # Case 4: R22 is the largest diagonal element
    s4 = torch.sqrt(torch.clamp(1.0 + R22 - R00 - R11, min=1e-8)) * 2.0
    qw4 = (R10 - R01) / s4
    qx4 = (R02 + R20) / s4
    qy4 = (R12 + R21) / s4
    qz4 = 0.25 * s4
    q4 = torch.stack([qx4, qy4, qz4, qw4], dim=1)

    # Define conditions for vectorized selection
    cond1 = trace > 0
    cond2 = (R00 > R11) & (R00 > R22)
    cond3 = R11 > R22

    # Use torch.where for nested selection based on conditions
    # This selects the appropriate calculation for each matrix in the batch
    q_234 = torch.where(cond3.unsqueeze(-1), q3, q4)
    q_1234 = torch.where(cond2.unsqueeze(-1), q2, q_234)
    final_q = torch.where(cond1.unsqueeze(-1), q1, q_1234)

    return final_q

@torch.jit.script
def se3_to_pose(T: torch.Tensor) -> torch.Tensor:
    """
    Converts a batch of 4x4 SE(3) transformation matrices to a batch of 
    7-element poses (3 translation, 4 quaternion).
    Args:
        T: A tensor of shape (N, 4, 4) representing the SE(3) matrices.
    Returns:
        A tensor of shape (N, 7) with poses (x, y, z, qx, qy, qz, qw).
    """
    R = T[:, :3, :3]
    p = T[:, :3, 3]
    q = rotmat_to_quat(R)
    pose7 = torch.cat([p, q], dim=1)
    return pose7
