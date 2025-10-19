import carb
import omni.usd
import omni.physx.scripts.physicsUtils as physicsUtils
from typing import Optional
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from lego_assemble._native import create_brick_prim
from lego_assemble.physics.lego_schemes import parse_color
from .assembler import path_for_brick
from .utils import add_to_collision_group

class BrickPhysicsInterface:
    def __init__(self):
        self.uniqueifier = 0 # Global monotonically-increasing brick id

    def create_brick(self, dimensions: tuple[int, int, int], color_name: str, env_id: Optional[int] = None, pos: Optional[tuple[float,float,float]] = None, quat: Optional[tuple[float,float,float,float]] = None) -> tuple[UsdGeom.Xform, int]:
        stage: Usd.Stage = omni.usd.get_context().get_stage()

        while True:
            brick_id = self.uniqueifier
            self.uniqueifier += 1
            path = path_for_brick(brick_id=brick_id, env_id=env_id)
            if not stage.GetPrimAtPath(path).IsValid():
                break

        create_brick_prim(path, dimensions, parse_color(color_name))
        brick = UsdGeom.Xform(stage.GetPrimAtPath(path))
        if env_id is not None:
            add_to_collision_group(stage, env_id, Sdf.Path(path))
        if pos is not None:
            physicsUtils.set_or_add_translate_op(brick, Gf.Vec3f(*pos))
        if quat is not None:
            physicsUtils.set_or_add_orient_op(brick, Gf.Quatf(*quat))
        carb.log_info(f"Added brick {path} ({dimensions[0]}x{dimensions[1]}x{dimensions[2]}) {color_name}")
        return brick, brick_id

    def reset_env(self, env_id: Optional[int] = None):
        stage: Usd.Stage = omni.usd.get_context().get_stage()

        root_path = f"/World/envs/env_{env_id}" if env_id is not None else "/World"
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim.IsValid():
            carb.log_warn(f"Resetting bricks in env {env_id} but it does not exist")
            return
        root_path_sdf: Sdf.Path = root_prim.GetPath()

        collision_group_prim = stage.GetPrimAtPath(f"/World/collisions/group{env_id}")
        if collision_group_prim.IsValid():
            collision_group = UsdPhysics.CollisionGroup(collision_group_prim)
            collider_collection: Usd.CollectionAPI = collision_group.GetCollidersCollectionAPI()
            collision_rel = collider_collection.GetIncludesRel()
        else:
            collision_rel = None

        for child in root_prim.GetAllChildrenNames():
            if child.startswith(("Brick_", "Conn_")):
                path = root_path_sdf.AppendChild(child)
                stage.RemovePrim(path)
                if collision_rel is not None:
                    collision_rel.RemoveTarget(path)

        carb.log_info(f"Resetting bricks in env {env_id}")

_brick_physics_interface = None

def get_brick_physics_interface() -> BrickPhysicsInterface:
    """Get the BrickPhysicsInterface instance."""
    global _brick_physics_interface
    if _brick_physics_interface is None:
        raise RuntimeError("BrickPhysicsInterface is not initialized.")
    return _brick_physics_interface

def init_brick_physics_interface():
    global _brick_physics_interface
    if _brick_physics_interface is not None:
        raise RuntimeError("BrickPhysicsInterface is already initialized.")
    _brick_physics_interface = BrickPhysicsInterface()
    return _brick_physics_interface

def deinit_brick_physics_interface():
    global _brick_physics_interface
    if _brick_physics_interface is None:
        raise RuntimeError("BrickPhysicsInterface is not initialized.")
    _brick_physics_interface = None
