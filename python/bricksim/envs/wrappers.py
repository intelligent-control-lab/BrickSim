"""Gymnasium wrappers used by BrickSim environments."""

from typing import SupportsFloat, TypeVar

import torch
from accelerate.utils import send_to_device
from gymnasium import Env, Wrapper
from gymnasium.core import RenderFrame

_ObsT = TypeVar("_ObsT")
_ActT = TypeVar("_ActT")
_DataT = TypeVar("_DataT")
_RenderResult = RenderFrame | list[RenderFrame] | None


class ToDeviceWrapper(Wrapper[_ObsT, _ActT, _ObsT, _ActT]):
    """Move observations, rewards, and info structures to one torch device."""

    def __init__(self, env: Env[_ObsT, _ActT], device: str | torch.device):
        """Initialize the wrapper with the destination device."""
        super().__init__(env)
        self.device = (
            device if isinstance(device, torch.device) else torch.device(device)
        )

    def _to_device(self, data: _DataT) -> _DataT:
        return send_to_device(data, self.device)

    def _to_device_render(self, data: _RenderResult) -> _RenderResult:
        return send_to_device(data, self.device)

    def step(
        self, action: _ActT
    ) -> tuple[_ObsT, SupportsFloat, bool, bool, dict[str, object]]:
        """Step the wrapped environment and move the result to the target device.

        Returns:
            Device-mapped result from ``env.step``.
        """
        obs, reward, terminated, truncated, info = self.env.step(action)
        return (
            self._to_device(obs),
            self._to_device(reward),
            self._to_device(terminated),
            self._to_device(truncated),
            self._to_device(info),
        )

    def reset(
        self, *, seed: int | None = None, options: dict[str, object] | None = None
    ) -> tuple[_ObsT, dict[str, object]]:
        """Reset the wrapped environment and move the result to the target device.

        Returns:
            Device-mapped result from ``env.reset``.
        """
        obs, info = self.env.reset(seed=seed, options=options)
        return self._to_device(obs), self._to_device(info)

    def render(self) -> _RenderResult:
        """Render the wrapped environment and move the result to the target device.

        Returns:
            Device-mapped result from ``env.render``.
        """
        return self._to_device_render(self.env.render())
