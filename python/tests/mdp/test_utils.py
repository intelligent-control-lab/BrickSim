"""Tests for MDP utility helpers."""

import torch

from bricksim.mdp.utils import enumerate_env_ids


def test_enumerate_env_ids_none_selects_all_envs():
    """``None`` should select every environment id."""
    env_ids = enumerate_env_ids(4, None)

    assert torch.equal(env_ids, torch.tensor([0, 1, 2, 3], dtype=torch.long))


def test_enumerate_env_ids_expands_slice_against_num_envs():
    """Slices should follow Python slice semantics over ``range(num_envs)``."""
    env_ids = enumerate_env_ids(6, slice(1, 6, 2))

    assert torch.equal(env_ids, torch.tensor([1, 3, 5], dtype=torch.long))


def test_enumerate_env_ids_handles_reverse_slice():
    """Negative-step slices should produce reversed environment ids."""
    env_ids = enumerate_env_ids(4, slice(None, None, -1))

    assert torch.equal(env_ids, torch.tensor([3, 2, 1, 0], dtype=torch.long))


def test_enumerate_env_ids_keeps_explicit_ids():
    """Explicit id sequences should be converted to long tensors."""
    env_ids = enumerate_env_ids(5, [4, 0, 2])

    assert torch.equal(env_ids, torch.tensor([4, 0, 2], dtype=torch.long))


def test_enumerate_env_ids_converts_tensor_dtype():
    """Tensor selectors should be converted to long dtype."""
    env_ids = enumerate_env_ids(5, torch.tensor([3, 1], dtype=torch.int32))

    assert torch.equal(env_ids, torch.tensor([3, 1], dtype=torch.long))


def test_enumerate_env_ids_honors_device():
    """The optional device argument should place generated ids on that device."""
    env_ids = enumerate_env_ids(3, slice(None), device=torch.device("cpu"))

    assert env_ids.device.type == "cpu"
    assert torch.equal(env_ids, torch.tensor([0, 1, 2], dtype=torch.long))
