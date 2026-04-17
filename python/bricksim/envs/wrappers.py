"""Gymnasium wrappers used by BrickSim environments."""

from typing import Any, Mapping, Union

import torch
from gymnasium import Wrapper


class ToDeviceWrapper(Wrapper):
    """Move observations, rewards, and info structures to one torch device."""

    def __init__(self, env: Any, device: Union[str, torch.device]):
        """Initialize the wrapper with the destination device."""
        super().__init__(env)
        self.device = (
            device if isinstance(device, torch.device) else torch.device(device)
        )

    def _to_device(self, data: Any) -> Any:
        if isinstance(data, torch.Tensor):
            return data.to(self.device)
        elif isinstance(data, Mapping):
            return {k: self._to_device(v) for k, v in data.items()}
        elif isinstance(data, (list, tuple)):
            return type(data)(self._to_device(d) for d in data)
        else:
            return data

    def step(self, action):
        """Step the wrapped environment and move the result to the target device.

        Returns:
            Device-mapped result from ``env.step``.
        """
        return self._to_device(self.env.step(action))

    def reset(self, *, seed: None = None, options: None = None):
        """Reset the wrapped environment and move the result to the target device.

        Returns:
            Device-mapped result from ``env.reset``.
        """
        return self._to_device(self.env.reset(seed=seed, options=options))

    def render(self):
        """Render the wrapped environment and move the result to the target device.

        Returns:
            Device-mapped result from ``env.render``.
        """
        return self._to_device(self.env.render())
