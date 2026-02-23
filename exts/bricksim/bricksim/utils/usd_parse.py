from typing import Optional
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

def parse_connection_prim(prim: Usd.Prim) -> Optional[tuple[str, int, str, int, tuple[int, int], int]]:
    """Parse a LegoConnection prim.

    Args:
        prim: The USD connection prim to parse.

    Returns:
        (stud_path, stud_if, hole_path, hole_if, offset, yaw) on success, or
        None if the prim is not a valid connection prim.
    """
    if not prim or not prim.IsValid():
        return None
    if prim.GetTypeName() != "LegoConnection":
        return None

    stud_rel: Usd.Relationship = prim.GetRelationship("lego:conn_stud")
    hole_rel: Usd.Relationship = prim.GetRelationship("lego:conn_hole")
    if not stud_rel or not hole_rel:
        return None

    stud_targets = stud_rel.GetTargets()
    hole_targets = hole_rel.GetTargets()
    if not stud_targets or not hole_targets:
        return None

    stud_path = str(stud_targets[0])
    hole_path = str(hole_targets[0])

    stud_if_attr: Usd.Attribute = prim.GetAttribute("lego:conn_stud_interface")
    hole_if_attr: Usd.Attribute = prim.GetAttribute("lego:conn_hole_interface")
    offset_attr: Usd.Attribute = prim.GetAttribute("lego:conn_offset")
    yaw_attr: Usd.Attribute = prim.GetAttribute("lego:conn_yaw")
    if not stud_if_attr or not hole_if_attr or not offset_attr or not yaw_attr:
        return None

    stud_if = stud_if_attr.Get()
    hole_if = hole_if_attr.Get()
    offset = offset_attr.Get()
    yaw = yaw_attr.Get()
    if stud_if is None or hole_if is None or offset is None or yaw is None:
        return None

    return (
        stud_path,
        int(stud_if),
        hole_path,
        int(hole_if),
        (int(offset[0]), int(offset[1])),
        int(yaw),
    )
