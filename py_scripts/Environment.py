from utils import *
from Robot import *

class ENV():
    def __init__(self, robot_usd, robot_base_pos, robot_base_ori, robot_name):
        root = os.getcwd()
        config = {
            "renderer": "RaytracedLighting", 
            "headless": False,
            "extra_args": ["--ext-folder", os.path.join(root, "IsaacLab/source"),
                           "--ext-folder", os.path.join(root, "exts"),
                           "--enable", "lego_assemble",
                           "--enable", "omni.kit.debug.python"]
        }
        self.kit = SimulationApp(config)
        self.kit.update()
        
        import omni
        from omni.isaac.core.simulation_context import SimulationContext
        
        stage_path = os.path.join(root, "resources/demo.usda")
        self.sim_context = SimulationContext()
        
        omni.usd.get_context().open_stage(stage_path)
        self.sim_context.initialize_physics()
        self.kit.update()
        self.robot = self.load_robot(robot_usd, robot_base_pos, robot_base_ori, robot_name)
        print("Environment Load Complete!")
    
    def start(self):
        self.sim_context.play()

    def pause(self):
        self.sim_context.stop()

    def update(self):
        self.sim_context.step(render=True)

    def load_robot(self, robot_usd, robot_base_pos, robot_base_ori, robot_name):
        from isaacsim.core.prims import Articulation
        from isaaclab.assets.articulation import ArticulationCfg
        robot = ROBOT(robot_usd, pos=robot_base_pos, ori=robot_base_ori, robot_name=robot_name)
        self.kit.update()
        articulated_system = Articulation(robot.name)
        articulated_system.initialize()
        self.sim_context.reset()
        robot.set_controller(articulated_system)
        return robot