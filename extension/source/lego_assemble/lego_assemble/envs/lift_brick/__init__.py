import gymnasium as gym
from . import cfg

gym.register(
    id="Lego-LiftBrick-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env:LiftBrickEnvCfg",
        "skrl_cfg_entry_point": f"{cfg.__name__}:skrl_ppo_cfg.yaml",
    },
    disable_env_checker=True,
)
