from utils import *

class ROBOT():
    def __init__(self, robot_usd, pos, ori, robot_name):
        self.name = "/World/" + robot_name
        robot_usd = os.path.join(robot_usd)

        from isaacsim.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.prims import XFormPrim

        self.robot = add_reference_to_stage(usd_path=robot_usd, 
                                            prim_path=self.name)
        self.robot_xf = XFormPrim(prim_path=self.name)
        self.robot_xf.set_world_pose(position=pos, orientation=ori)
        
    def set_controller(self, controller):
        self.controller = controller
        self.joint_names = self.controller._joint_names
        self.joint_limits = self.controller.get_dof_limits()

    def set_pos(self, command):
        self.controller.set_joint_positions(command)

    def get_pos(self):
        self.joint_pos = self.controller.get_joint_positions()
        return self.joint_pos