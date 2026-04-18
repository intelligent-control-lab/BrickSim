"""USD helpers for BrickSim brick prim metadata."""

from typing import TypeAlias

from pxr import Gf, Usd

BrickDimensions: TypeAlias = tuple[int, int, int]


def parse_brick_prim_dimensions(prim: Usd.Prim) -> BrickDimensions | None:
    """Get the dimensions of a brick part.

    Args:
        prim: The USD brick prim to parse.

    Returns:
        A tuple of (length, width, height) in studs.
    """
    if not prim:
        return None
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
