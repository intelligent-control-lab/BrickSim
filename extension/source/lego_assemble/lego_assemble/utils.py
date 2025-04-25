import pxr.PhysxSchema as PhysxSchema
from typing import Optional
from pxr import Usd

def get_physics_scene(stage: Usd.Stage) -> Optional[PhysxSchema.PhysxSceneAPI]:
    prim: Usd.Prim
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            return PhysxSchema.PhysxSceneAPI(prim)
    return None
