"""Tests for vectorized connection rectangle geometry."""

import pytest
import torch

from bricksim.units import BRICK_UNIT_LENGTH, PLATE_UNIT_HEIGHT
from bricksim.utils.connection_geometry import (
    connection_brick_transforms,
    connection_interface_transforms,
    connection_overlap_areas,
    enumerate_all_possible_connections,
)


def test_connection_overlap_areas_handles_signed_c4_extents():
    """Overlap areas should follow the stud-frame rectangle model.

    The stud interface occupies the half-open grid rectangle ``[0, L] x [0, W]``.
    The hole interface starts at each ``offset`` and extends by its C4-rotated
    ``(L, W)`` vector. Negative rotated extents are normalized with min/max
    before measuring overlap area, so touching only at an edge has zero area.
    """
    offsets = torch.tensor(
        [
            (1, 1),
            (0, 0),
            (0, 0),
            (4, 3),
            (3, 1),
        ],
        dtype=torch.long,
    )
    yaws = torch.tensor([0, 1, 2, 0, 1], dtype=torch.long)

    areas = connection_overlap_areas((4, 3), (2, 1), offsets, yaws)

    assert torch.equal(areas, torch.tensor([2, 0, 0, 0, 2], dtype=torch.long))


def test_connection_overlap_areas_broadcasts_offsets_and_normalizes_yaws():
    """One offset can be checked against many C4 yaw indices at once.

    The output shape follows PyTorch broadcasting over ``offsets.shape[:-1]``
    and ``yaws.shape``. Yaw indices are reduced modulo four before rotating the
    hole rectangle.
    """
    offset = torch.tensor((0, 1), dtype=torch.long)
    yaws = torch.tensor([0, 1, 2, 3, -1], dtype=torch.long)

    areas = connection_overlap_areas((4, 3), (2, 1), offset, yaws)

    assert torch.equal(areas, torch.tensor([2, 0, 0, 1, 1], dtype=torch.long))


def test_connection_overlap_areas_rejects_offsets_without_xy_columns():
    """Offsets must store exactly two grid coordinates in the last dimension."""
    offsets = torch.zeros((3, 3), dtype=torch.long)
    yaws = torch.zeros(3, dtype=torch.long)

    with pytest.raises(ValueError, match=r"offsets must have shape \(\.\.\., 2\)"):
        connection_overlap_areas((4, 3), (2, 1), offsets, yaws)


def test_connection_interface_transforms_broadcast_and_normalize_yaws():
    """Interface transforms should match C4 yaw quaternions and stud-unit offsets."""
    offsets = torch.tensor([[(1, 2)], [(-1, 0)]], dtype=torch.long)
    yaws = torch.tensor([0, 1, -1], dtype=torch.long)

    pos, quat = connection_interface_transforms(offsets, yaws, dtype=torch.float64)

    expected_pos = torch.tensor(
        [
            [
                (BRICK_UNIT_LENGTH, 2.0 * BRICK_UNIT_LENGTH, 0.0),
                (BRICK_UNIT_LENGTH, 2.0 * BRICK_UNIT_LENGTH, 0.0),
                (BRICK_UNIT_LENGTH, 2.0 * BRICK_UNIT_LENGTH, 0.0),
            ],
            [
                (-BRICK_UNIT_LENGTH, 0.0, 0.0),
                (-BRICK_UNIT_LENGTH, 0.0, 0.0),
                (-BRICK_UNIT_LENGTH, 0.0, 0.0),
            ],
        ],
        dtype=torch.float64,
    )
    sqrt_half = torch.sqrt(torch.tensor(0.5, dtype=torch.float64))
    expected_quat = torch.tensor(
        [
            (1.0, 0.0, 0.0, 0.0),
            (sqrt_half, 0.0, 0.0, sqrt_half),
            (sqrt_half, 0.0, 0.0, -sqrt_half),
        ],
        dtype=torch.float64,
    ).expand(2, 3, 4)

    torch.testing.assert_close(pos, expected_pos)
    torch.testing.assert_close(quat, expected_quat)


def test_connection_brick_transforms_match_native_connection_formula():
    """Brick transforms should follow ``stud.pose * T_si_hi * inverse(hole.pose)``."""
    offsets = torch.tensor([(1, 2), (1, 2)], dtype=torch.long)
    yaws = torch.tensor([0, 1], dtype=torch.long)
    stud_dimensions = (4, 3, 2)
    hole_dimensions = (2, 1, 3)

    pos, quat = connection_brick_transforms(
        stud_dimensions=stud_dimensions,
        hole_dimensions=hole_dimensions,
        offsets=offsets,
        yaws=yaws,
        dtype=torch.float64,
    )

    stud_pos = torch.tensor(
        (-2.0 * BRICK_UNIT_LENGTH, -1.5 * BRICK_UNIT_LENGTH, 2.0 * PLATE_UNIT_HEIGHT),
        dtype=torch.float64,
    )
    offset_pos = torch.tensor(
        (BRICK_UNIT_LENGTH, 2.0 * BRICK_UNIT_LENGTH, 0.0),
        dtype=torch.float64,
    )
    hole_pos = torch.tensor(
        (-BRICK_UNIT_LENGTH, -0.5 * BRICK_UNIT_LENGTH, 0.0),
        dtype=torch.float64,
    )
    expected_pos = torch.stack(
        (
            stud_pos + offset_pos - hole_pos,
            stud_pos
            + offset_pos
            - torch.tensor(
                (0.5 * BRICK_UNIT_LENGTH, -BRICK_UNIT_LENGTH, 0.0),
                dtype=torch.float64,
            ),
        )
    )
    sqrt_half = torch.sqrt(torch.tensor(0.5, dtype=torch.float64))
    expected_quat = torch.tensor(
        [
            (1.0, 0.0, 0.0, 0.0),
            (sqrt_half, 0.0, 0.0, sqrt_half),
        ],
        dtype=torch.float64,
    )

    torch.testing.assert_close(pos, expected_pos)
    torch.testing.assert_close(quat, expected_quat)


def test_enumerate_all_possible_connections_matches_bruteforce_overlap_filter():
    """Enumeration should return every positive-area row and no zero-area rows.

    Rows are ordered as ``(offset_x, offset_y, yaw)`` in grid units. This test
    builds an independent brute-force set for a small stud/hole pair using the
    documented candidate bounds, then compares that set with the vectorized
    enumeration result.
    """
    stud_dimensions = (3, 2)
    hole_dimensions = (2, 1)

    rows = enumerate_all_possible_connections(
        stud_dimensions=stud_dimensions,
        hole_dimensions=hole_dimensions,
        device=torch.device("cpu"),
    )

    max_hole_extent = max(hole_dimensions)
    expected_rows: set[tuple[int, int, int]] = set()
    for yaw in range(4):
        for offset_x in range(
            -max_hole_extent + 1, stud_dimensions[0] + max_hole_extent
        ):
            for offset_y in range(
                -max_hole_extent + 1, stud_dimensions[1] + max_hole_extent
            ):
                area = connection_overlap_areas(
                    stud_dimensions,
                    hole_dimensions,
                    torch.tensor((offset_x, offset_y), dtype=torch.long),
                    torch.tensor(yaw, dtype=torch.long),
                )
                if int(area.item()) > 0:
                    expected_rows.add((offset_x, offset_y, yaw))

    actual_rows = {tuple(int(value) for value in row.tolist()) for row in rows}

    assert rows.dtype == torch.long
    assert rows.shape[1] == 3
    assert torch.unique(rows, dim=0).shape[0] == rows.shape[0]
    assert actual_rows == expected_rows
