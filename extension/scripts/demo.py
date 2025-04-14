import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="LEGO Assembly Demo")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = "isaacsim.exp.full.kit"
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.sim import SimulationCfg, SimulationContext

def main():
    sim_cfg = SimulationCfg(dt=0.01)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    sim.reset()

    while simulation_app.is_running():
        sim.step()

if __name__ == "__main__":
    main()
    simulation_app.close()
