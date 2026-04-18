"""USD helpers for BrickSim connection prim metadata."""

from dataclasses import dataclass

from pxr import Usd


@dataclass(frozen=True, slots=True)
class ParsedConnectionPrim:
    """Parsed LegoConnection USD prim metadata."""

    stud_path: str
    stud_interface: int
    hole_path: str
    hole_interface: int
    offset: tuple[int, int]
    yaw: int


def parse_connection_prim(
    prim: Usd.Prim,
) -> ParsedConnectionPrim | None:
    """Parse a LegoConnection prim.

    Args:
        prim: The USD connection prim to parse.

    Returns:
        Parsed connection metadata, or None if the prim is not a valid
        connection prim.
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

    return ParsedConnectionPrim(
        stud_path=stud_path,
        stud_interface=int(stud_if),
        hole_path=hole_path,
        hole_interface=int(hole_if),
        offset=(int(offset[0]), int(offset[1])),
        yaw=int(yaw),
    )
