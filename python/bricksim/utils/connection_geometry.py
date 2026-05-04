"""Vectorized connection-segment geometry helpers."""

import torch

from bricksim.units import BRICK_UNIT_LENGTH, PLATE_UNIT_HEIGHT
from bricksim.utils.c4_rotation import c4_rotate_2d


def connection_interface_transforms(
    offsets: torch.Tensor,
    yaws: torch.Tensor,
    dtype: torch.dtype | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Return stud-interface to hole-interface transforms for connection rows.

    Args:
        offsets: Integer tensor with shape ``(..., 2)``. Values are the hole
            interface origin relative to the stud interface origin, in stud
            grid units.
        yaws: Integer C4 yaw tensor broadcastable to ``offsets.shape[:-1]``.
            Values are normalized modulo four.
        dtype: Floating-point dtype for returned tensors. Defaults to
            ``torch.get_default_dtype()``.

    Returns:
        Tuple ``(pos, quat)`` for ``T_stud_interface_hole_interface``. Positions
        have shape ``broadcast(offsets.shape[:-1], yaws.shape) + (3,)`` in SI
        meters. Quaternions have the same leading shape plus ``(4,)`` in WXYZ
        order.
    """
    if offsets.shape[-1] != 2:
        raise ValueError("offsets must have shape (..., 2)")
    if dtype is None:
        dtype = torch.get_default_dtype()

    device = offsets.device
    offset_xy = offsets.to(device=device, dtype=dtype) * BRICK_UNIT_LENGTH
    yaw_indices = yaws.to(device=device, dtype=torch.long).remainder(4)
    x, yaw_indices = torch.broadcast_tensors(offset_xy[..., 0], yaw_indices)
    y = torch.broadcast_to(offset_xy[..., 1], x.shape)
    zero = torch.zeros_like(x)
    pos = torch.stack((x, y, zero), dim=-1)

    sqrt_half = torch.sqrt(torch.tensor(0.5, device=device, dtype=dtype))
    sqrt_half = torch.broadcast_to(sqrt_half, x.shape)
    quat_0 = torch.stack((torch.ones_like(x), zero, zero, zero), dim=-1)
    quat_1 = torch.stack((sqrt_half, zero, zero, sqrt_half), dim=-1)
    quat_2 = torch.stack((zero, zero, zero, torch.ones_like(x)), dim=-1)
    quat_3 = torch.stack((sqrt_half, zero, zero, -sqrt_half), dim=-1)

    is_yaw_0 = (yaw_indices == 0).unsqueeze(-1)
    is_yaw_1 = (yaw_indices == 1).unsqueeze(-1)
    is_yaw_2 = (yaw_indices == 2).unsqueeze(-1)
    quat = torch.where(
        is_yaw_0,
        quat_0,
        torch.where(
            is_yaw_1,
            quat_1,
            torch.where(is_yaw_2, quat_2, quat_3),
        ),
    )
    return pos, quat


def connection_brick_transforms(
    stud_dimensions: tuple[int, int, int],
    hole_dimensions: tuple[int, int, int],
    offsets: torch.Tensor,
    yaws: torch.Tensor,
    dtype: torch.dtype | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Return stud-brick to hole-brick transforms for connection rows.

    Args:
        stud_dimensions: Stud-side brick dimensions ``(L, W, H)`` in brick units.
        hole_dimensions: Hole-side brick dimensions ``(L, W, H)`` in brick units.
        offsets: Integer tensor with shape ``(..., 2)``. Values are the hole
            interface origin relative to the stud interface origin, in stud
            grid units.
        yaws: Integer C4 yaw tensor broadcastable to ``offsets.shape[:-1]``.
            Values are normalized modulo four.
        dtype: Floating-point dtype for returned tensors. Defaults to
            ``torch.get_default_dtype()``.

    Returns:
        Tuple ``(pos, quat)`` for ``T_stud_hole``. Positions have shape
        ``broadcast(offsets.shape[:-1], yaws.shape) + (3,)`` in SI meters.
        Quaternions have the same leading shape plus ``(4,)`` in WXYZ order.
    """
    offset_pos, rel_quat = connection_interface_transforms(
        offsets=offsets,
        yaws=yaws,
        dtype=dtype,
    )
    dtype = offset_pos.dtype
    device = offsets.device

    stud_pos = torch.tensor(
        [
            -float(stud_dimensions[0]) * BRICK_UNIT_LENGTH / 2.0,
            -float(stud_dimensions[1]) * BRICK_UNIT_LENGTH / 2.0,
            float(stud_dimensions[2]) * PLATE_UNIT_HEIGHT,
        ],
        device=device,
        dtype=dtype,
    )
    hole_xy = torch.tensor(
        [
            -float(hole_dimensions[0]) * BRICK_UNIT_LENGTH / 2.0,
            -float(hole_dimensions[1]) * BRICK_UNIT_LENGTH / 2.0,
        ],
        device=device,
        dtype=dtype,
    )
    rotated_hole_xy = c4_rotate_2d(
        hole_xy,
        yaws.to(device=device, dtype=torch.long),
    ).to(dtype=dtype)
    rotated_hole_xy = torch.broadcast_to(
        rotated_hole_xy,
        offset_pos.shape[:-1] + (2,),
    )

    rel_pos = offset_pos.clone()
    rel_pos[..., 0] += stud_pos[0] - rotated_hole_xy[..., 0]
    rel_pos[..., 1] += stud_pos[1] - rotated_hole_xy[..., 1]
    rel_pos[..., 2] += stud_pos[2]
    return rel_pos, rel_quat


def connection_overlap_areas(
    stud_dimensions: tuple[int, int],
    hole_dimensions: tuple[int, int],
    offsets: torch.Tensor,
    yaws: torch.Tensor,
) -> torch.Tensor:
    """Return stud/hole overlap areas for connection segments.

    Args:
        stud_dimensions: Stud interface ``(L, W)`` in grid units.
        hole_dimensions: Hole interface ``(L, W)`` in grid units.
        offsets: Integer tensor with shape ``(..., 2)``.
        yaws: Integer yaw tensor broadcastable to ``offsets.shape[:-1]``.

    Returns:
        Integer tensor with shape ``broadcast(offsets.shape[:-1], yaws.shape)``.
    """
    if offsets.shape[-1] != 2:
        raise ValueError("offsets must have shape (..., 2)")

    device = offsets.device
    hole_extent = c4_rotate_2d(
        torch.tensor(hole_dimensions, device=device, dtype=torch.long),
        yaws.to(device=device, dtype=torch.long),
    )
    hole_start, hole_extent = torch.broadcast_tensors(
        offsets.to(dtype=torch.long), hole_extent
    )
    hole_end = hole_start + hole_extent
    hole_min = torch.minimum(hole_start, hole_end)
    hole_max = torch.maximum(hole_start, hole_end)

    overlap_start = torch.maximum(
        torch.zeros(2, device=device, dtype=torch.long),
        hole_min,
    )
    overlap_end = torch.minimum(
        torch.tensor(stud_dimensions, device=device, dtype=torch.long),
        hole_max,
    )
    overlap_widths = torch.clamp(overlap_end - overlap_start, min=0)
    return torch.prod(overlap_widths, dim=-1)


def enumerate_all_possible_connections(
    stud_dimensions: tuple[int, int],
    hole_dimensions: tuple[int, int],
    device: str | torch.device,
) -> torch.Tensor:
    """Return all positive-overlap connection rows.

    Args:
        stud_dimensions: Stud interface dimensions ``(L, W)`` in grid units.
        hole_dimensions: Hole interface dimensions ``(L, W)`` in grid units.
        device: Device for the returned tensor.

    Returns:
        Integer tensor with rows ``(offset_x, offset_y, yaw)``.
    """
    max_hole_extent = max(hole_dimensions)
    candidate_yaws = torch.arange(4, device=device, dtype=torch.long)
    candidate_x = torch.arange(
        -max_hole_extent + 1,
        stud_dimensions[0] + max_hole_extent,
        device=device,
        dtype=torch.long,
    )
    candidate_y = torch.arange(
        -max_hole_extent + 1,
        stud_dimensions[1] + max_hole_extent,
        device=device,
        dtype=torch.long,
    )
    yaw_grid, x_grid, y_grid = torch.meshgrid(
        candidate_yaws,
        candidate_x,
        candidate_y,
        indexing="ij",
    )
    offsets = torch.stack((x_grid.reshape(-1), y_grid.reshape(-1)), dim=1)
    yaws = yaw_grid.reshape(-1)
    overlap_areas = connection_overlap_areas(
        stud_dimensions=stud_dimensions,
        hole_dimensions=hole_dimensions,
        offsets=offsets,
        yaws=yaws,
    )
    valid = overlap_areas > 0
    return torch.cat((offsets[valid], yaws[valid].unsqueeze(1)), dim=1)
