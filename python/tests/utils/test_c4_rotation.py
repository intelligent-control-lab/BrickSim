"""Tests for vectorized C4 grid rotation."""

import torch

from bricksim.utils.c4_rotation import c4_rotate_2d


def test_c4_rotate_2d_rotates_batched_vectors():
    """C4 yaw indices should rotate each vector row independently."""
    vectors = torch.tensor(
        [
            (2, 4),
            (2, 4),
            (2, 4),
            (2, 4),
        ],
        dtype=torch.long,
    )
    yaws = torch.tensor([0, 1, 2, 3], dtype=torch.long)

    rotated = c4_rotate_2d(vectors, yaws)

    assert torch.equal(
        rotated,
        torch.tensor(
            [
                (2, 4),
                (-4, 2),
                (-2, -4),
                (4, -2),
            ],
            dtype=torch.long,
        ),
    )


def test_c4_rotate_2d_normalizes_yaws_and_broadcasts_vector():
    """Yaw normalization and vector broadcasting should match native C4 rotation."""
    vector = torch.tensor((2, 4), dtype=torch.long)
    yaws = torch.tensor([-1, 4, 5], dtype=torch.long)

    rotated = c4_rotate_2d(vector, yaws)

    assert torch.equal(
        rotated,
        torch.tensor(
            [
                (4, -2),
                (2, 4),
                (-4, 2),
            ],
            dtype=torch.long,
        ),
    )
