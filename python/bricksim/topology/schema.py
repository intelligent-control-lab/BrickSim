"""Typed views of importer topology JSON.

These mirror the JSON structs in native/modules/bricksim/io/topology.cppm.
The native side stores offset, position, and rotation as fixed-size arrays;
the Python importer emits ordinary JSON arrays as lists.
"""

from typing import Final, Literal, TypeAlias, TypedDict

TopologySchema: TypeAlias = Literal["bricksim/lego_topology@2"]
SCHEMA_STRING: Final[TopologySchema] = "bricksim/lego_topology@2"

JsonBrickColor: TypeAlias = list[int]  # [r, g, b]
JsonConnectionOffset: TypeAlias = list[int]  # [x, y], in studs
JsonPosition: TypeAlias = list[float]  # [x, y, z], in meters
JsonWxyzQuaternion: TypeAlias = list[float]  # [w, x, y, z]


class JsonBrickPayload(TypedDict):
    """Payload serialized by BrickSerializer."""

    L: int
    W: int
    H: int
    color: JsonBrickColor


class JsonPart(TypedDict):
    """Serialized topology part."""

    id: int
    type: Literal["brick"]
    payload: JsonBrickPayload


class JsonConnection(TypedDict):
    """Serialized stud-hole connection segment."""

    id: int
    stud_id: int
    stud_iface: int
    hole_id: int
    hole_iface: int
    offset: JsonConnectionOffset
    yaw: int


class JsonPoseHint(TypedDict):
    """Serialized pose hint for one connected component."""

    part: int
    pos: JsonPosition
    rot: JsonWxyzQuaternion


class JsonTopology(TypedDict):
    """Serialized bricksim topology document."""

    schema: TopologySchema
    parts: list[JsonPart]
    connections: list[JsonConnection]
    pose_hints: list[JsonPoseHint]
