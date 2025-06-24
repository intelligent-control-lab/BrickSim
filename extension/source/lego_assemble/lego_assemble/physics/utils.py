import pxr.PhysxSchema as PhysxSchema
from typing import Optional
from pxr import Sdf, Usd, UsdPhysics

def get_physics_scene(stage: Usd.Stage) -> Optional[PhysxSchema.PhysxSceneAPI]:
    prim: Usd.Prim
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            return PhysxSchema.PhysxSceneAPI(prim)
    return None

def add_to_collision_group(stage: Usd.Stage, env_id: int, prim_path: Sdf.Path):
    prim = stage.GetPrimAtPath(f"/World/collisions/group{env_id}")
    if prim.IsA(UsdPhysics.CollisionGroup):
        collision_group = UsdPhysics.CollisionGroup(prim)
        collider_collection: Usd.CollectionAPI = collision_group.GetCollidersCollectionAPI()
        collider_collection.GetIncludesRel().AddTarget(prim_path)
