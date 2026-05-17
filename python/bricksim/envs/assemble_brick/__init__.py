"""Assemble-brick task environment package."""

# Register the environment with Gym

import gymnasium as _gym

_gym.register(
    id="Lego-AssembleBrick-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env:AssembleBrickEnvCfg",
    },
    disable_env_checker=True,
)

_gym.register(
    id="Lego-AssembleBrick-RGB-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env:AssembleBrickRGBEnvCfg",
    },
    disable_env_checker=True,
)

_gym.register(
    id="Lego-AssembleBrick-RGBD-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env:AssembleBrickRGBDEnvCfg",
    },
    disable_env_checker=True,
)
