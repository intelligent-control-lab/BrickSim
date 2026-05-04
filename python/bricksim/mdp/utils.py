"""Utility helpers shared by BrickSim MDP terms."""

from collections.abc import Sequence
from dataclasses import MISSING as _DATACLASS_MISSING
from typing import Never

import torch

MISSING: Never = _DATACLASS_MISSING  # ty: ignore[invalid-assignment]


def enumerate_env_ids(
    num_envs: int,
    env_ids: Sequence[int] | torch.Tensor | slice | None,
    device: str | torch.device | None = None,
) -> torch.Tensor:
    """Return explicit environment ids as a long tensor.

    Args:
        num_envs: Total number of environments.
        env_ids: Environment id selector. ``None`` and ``slice(None)`` select
            all environments. Other slices are expanded against ``num_envs``.
            Sequences and tensors are treated as explicit ids.
        device: Device for the returned tensor. If omitted, PyTorch's default
            tensor device is used for non-tensor selectors, and tensor selectors
            keep their existing device when possible.

    Returns:
        Long tensor of explicit environment ids.
    """
    env_ids = slice(None) if env_ids is None else env_ids
    if isinstance(env_ids, slice):
        return torch.arange(*env_ids.indices(num_envs), device=device, dtype=torch.long)
    return torch.as_tensor(env_ids, device=device, dtype=torch.long)
