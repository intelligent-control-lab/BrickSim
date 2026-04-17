"""Assemble-brick task environment package."""

# Register the environment with Gym

import gymnasium as _gym

if "Lego-AssembleBrick-v0" in _gym.envs.registry:
    del _gym.envs.registry["Lego-AssembleBrick-v0"]
_gym.register(
    id="Lego-AssembleBrick-v0",
    entry_point=f"{__name__}.env:AssembleBrickEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env:AssembleBrickEnvCfg",
        "skrl_cfg_entry_point": f"{__name__}.cfg:skrl_ppo_cfg.yaml",
    },
    disable_env_checker=True,
)
