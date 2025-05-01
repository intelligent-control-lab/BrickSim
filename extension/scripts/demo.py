"""Launch Isaac Sim Simulator first."""

import os
import sys
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Enable lego_assemble extension
plugin_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "source"))
sys.argv += ["--ext-folder", plugin_folder, "--enable", "lego_assemble"]

# Verbose output
sys.argv += ["-v"]

app_launcher = AppLauncher(args_cli)
from isaacsim.simulation_app import SimulationApp
simulation_app: SimulationApp = app_launcher.app

"""Rest everything follows."""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from lego_assemble.envs.lift_brick.env import LiftBrickEnvCfg

def main():
    env_cfg = LiftBrickEnvCfg()
    env_cfg.scene.num_envs = 64
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env.reset()

    while simulation_app.is_running():
        with torch.inference_mode():
            joint_efforts = torch.randn_like(env.action_manager.action)
            env.step(joint_efforts)
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
