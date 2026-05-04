"""Vectorized C4 grid-rotation helpers."""

import torch


def c4_rotate_2d(vectors: torch.Tensor, yaws: torch.Tensor) -> torch.Tensor:
    """Rotate 2D vectors by discrete C4 yaw indices.

    Args:
        vectors: Tensor with shape ``(..., 2)``.
        yaws: Integer yaw tensor broadcastable to ``vectors.shape[:-1]``.

    Returns:
        Tensor with shape ``broadcast(vectors.shape[:-1], yaws.shape) + (2,)``.
    """
    if vectors.shape[-1] != 2:
        raise ValueError("vectors must have shape (..., 2)")

    yaw_indices = yaws.to(device=vectors.device, dtype=torch.long).remainder(4)
    x, yaw_indices = torch.broadcast_tensors(vectors[..., 0], yaw_indices)
    y = torch.broadcast_to(vectors[..., 1], x.shape)

    rotated_0 = torch.stack((x, y), dim=-1)
    rotated_1 = torch.stack((-y, x), dim=-1)
    rotated_2 = torch.stack((-x, -y), dim=-1)
    rotated_3 = torch.stack((y, -x), dim=-1)

    is_yaw_0 = (yaw_indices == 0).unsqueeze(-1)
    is_yaw_1 = (yaw_indices == 1).unsqueeze(-1)
    is_yaw_2 = (yaw_indices == 2).unsqueeze(-1)
    return torch.where(
        is_yaw_0,
        rotated_0,
        torch.where(
            is_yaw_1,
            rotated_1,
            torch.where(is_yaw_2, rotated_2, rotated_3),
        ),
    )
