import torch
from typing import Any, Mapping, Union
from gymnasium import Wrapper

class ToDeviceWrapper(Wrapper):
    def __init__(self, env: Any, device: Union[str, torch.device]):
        super().__init__(env)
        self.device = device if isinstance(device, torch.device) else torch.device(device)

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
        return self._to_device(self.env.step(action))

    def reset(self, *, seed: None = None, options: None = None):
        return self._to_device(self.env.reset(seed=seed, options=options))

    def render(self):
        return self._to_device(self.env.render())
