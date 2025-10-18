from pxr import Sdf, Usd, UsdPhysics

def add_to_collision_group(stage: Usd.Stage, env_id: int, prim_path: Sdf.Path):
    prim = stage.GetPrimAtPath(f"/World/collisions/group{env_id}")
    if prim.IsA(UsdPhysics.CollisionGroup):
        collision_group = UsdPhysics.CollisionGroup(prim)
        collider_collection: Usd.CollectionAPI = collision_group.GetCollidersCollectionAPI()
        collider_collection.GetIncludesRel().AddTarget(prim_path)
