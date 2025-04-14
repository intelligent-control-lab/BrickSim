import os
import sys
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="LEGO Assembly Demo")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = "isaaclab.python.rendering.kit"
ext_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "source"))
sys.argv += ["--ext-folder", ext_folder, "--enable", "lego_assemble"]
app_launcher = AppLauncher(args_cli)


from isaacsim.simulation_app import SimulationApp
from isaaclab.sim import SimulationCfg, SimulationContext
import isaacsim.core.utils.prims as prim_utils
import isaaclab.sim as sim_utils

simulation_app: SimulationApp = app_launcher.app

def design_scene():
    """Designs the scene by spawning ground plane, light, objects and meshes from usd files."""
    # Ground-plane
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

    # spawn distant light
    cfg_light_distant = sim_utils.DistantLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(1, 0, 10))

    # create a new xform prim for all objects to be spawned under
    prim_utils.create_prim("/World/Objects", "Xform")

def main():
    sim_cfg = SimulationCfg(dt=0.01, device="cpu")
    sim: SimulationContext = SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])
    design_scene()

    sim.reset()

    while simulation_app.is_running():
        sim.step()

if __name__ == "__main__":
    main()
    simulation_app.close()
