"""USD helpers for BrickSim brick prim metadata."""

from pxr import Gf, Usd


def parse_brick_prim_dimensions(prim: Usd.Prim) -> tuple[int, int, int] | None:
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
