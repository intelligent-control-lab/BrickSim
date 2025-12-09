from pxr import Gf, Usd
from isaacsim.core.utils.stage import get_current_stage

def get_env_path(env_id: int) -> str:
    return "/World" if env_id == -1 else f"/World/envs/env_{env_id}"

def get_brick_dimensions(brick_path: str) -> tuple[int, int, int]:
    """Get the dimensions of a brick part.

    Args:
        brick_path: The USD prim path of the brick part.

    Returns:
        A tuple of (length, width, height) in studs.
    """
    stage = get_current_stage()
    prim: Usd.Prim = stage.GetPrimAtPath(brick_path)
    if not prim:
        raise ValueError(f"Brick prim not found at path: {brick_path}")
    dimensions_attr: Usd.Attribute = prim.GetAttribute("lego:brick_dimensions")
    if not dimensions_attr or not dimensions_attr.HasValue():
        return None
    dimensions_gf: Gf.Vec3i = dimensions_attr.Get()
    if not dimensions_gf:
        return None
    length = int(dimensions_gf[0])
    width = int(dimensions_gf[1])
    height = int(dimensions_gf[2])
    return (length, width, height)
