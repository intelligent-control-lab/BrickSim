"""Launch Isaac Sim Simulator first."""

import pyinstrument
import os
import sys
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

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
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env.reset()

    normal_step_profiled = False
    reset_step_profiled = False

    with torch.inference_mode():
        while simulation_app.is_running():
            joint_efforts = torch.randn_like(env.action_manager.action)

            profiler = pyinstrument.Profiler()
            profiler.start()

            obs, reward, reset_terminated, reset_time_outs, extras = env.step(joint_efforts)
            is_reset_step = reset_terminated.any() or reset_time_outs.any()

            profiler.stop()
            if not is_reset_step and not normal_step_profiled:
                profiler.write_html("step_profile_normal.html", timeline=False, show_all=True)
                print("Normal step profile saved!")
                normal_step_profiled = True
            if is_reset_step and not reset_step_profiled:
                profiler.write_html("step_profile_reset.html", timeline=False, show_all=True)
                print("Reset step profile saved!")
                reset_step_profiled = True
            if normal_step_profiled and reset_step_profiled:
                break
        
        # waiting for reset
        while not is_reset_step and simulation_app.is_running():
            joint_efforts = torch.randn_like(env.action_manager.action)
            obs, reward, reset_terminated, reset_time_outs, extras = env.step(joint_efforts)
            is_reset_step = reset_terminated.any() or reset_time_outs.any()

        profiler = pyinstrument.Profiler()
        profiler.start()

        while simulation_app.is_running():
            joint_efforts = torch.randn_like(env.action_manager.action)
            obs, reward, reset_terminated, reset_time_outs, extras = env.step(joint_efforts)
            is_reset_step = reset_terminated.any() or reset_time_outs.any()
            if is_reset_step:
                break

        profiler.stop()
        profiler.write_html("step_profile_whole.html", timeline=False, show_all=True)
        print("Whole step profile saved!")

    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
