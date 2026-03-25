import gymnasium as gym
from . import cfg

if "Lego-AssembleBrick-v0" in gym.envs.registry:
    del gym.envs.registry["Lego-AssembleBrick-v0"]
gym.register(
    id="Lego-AssembleBrick-v0",
    entry_point=f"{__name__}.env:AssembleBrickEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env:AssembleBrickEnvCfg",
        "skrl_cfg_entry_point": f"{cfg.__name__}:skrl_ppo_cfg.yaml",
    },
    disable_env_checker=True,
)
