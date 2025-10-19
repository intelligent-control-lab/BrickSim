import omni.usd
from typing import Optional, TypeAlias
from dataclasses import dataclass, field
from pxr import Usd, Gf, Sdf
from lego_assemble.physics.interface import get_brick_physics_interface
from lego_assemble.physics.assembler import parse_brick_path, path_for_conn

SchemaString = "lego_assemble/lego_topology@1"

@dataclass
class Brick:
    id: int
    dimensions: tuple[int, int, int]
    color: str

@dataclass
class Connection:
    parent: int
    child: int
    offset: tuple[int, int]
    orientation: int

@dataclass
class PoseHint:
    brick: int
    pos: tuple[float, float, float]
    rot: tuple[float, float, float, float] # wxyz

@dataclass
class LegoTopology:
    schema: str = SchemaString
    bricks: list[Brick] = field(default_factory=list)
    connections: list[Connection] = field(default_factory=list)
    pose_hints: list[PoseHint] = field(default_factory=list)

Q: TypeAlias = int # Rotate along z-axis by 0, 90, 180, 270 degrees for 0, 1, 2, 3
S: TypeAlias = tuple[int, int, int] # (tx, ty, h)
M: TypeAlias = tuple[Q, S]

def _mul_qq(q2: Q, q1: Q) -> Q:
    return (q2 + q1) % 4

def _inv_q(q: Q) -> Q:
    return (-q) % 4

def _add_s(t1: S, t2: S) -> S:
    tx1, ty1, h1 = t1
    tx2, ty2, h2 = t2
    return (tx1 + tx2, ty1 + ty2, h1 + h2)

def _inv_s(t: S) -> S:
    tx, ty, h = t
    return (-tx, -ty, -h)

def _mul_qt(q: Q, t: S) -> S:
    tx, ty, h = t
    if q == 0:
        return (tx, ty, h)
    elif q == 1:
        return (-ty, tx, h)
    elif q == 2:
        return (-tx, -ty, h)
    elif q == 3:
        return (ty, -tx, h)
    else:
        raise ValueError(f"Invalid orientation: {q}")

def _mul_mm(m2: M, m1: M) -> M:
    q2, t2 = m2
    q1, t1 = m1
    q = _mul_qq(q2, q1)
    t = _add_s(_mul_qt(q2, t1), t2)
    return (q, t)

def _inv_m(m: M) -> M:
    q, t = m
    q_inv = _inv_q(q)
    t_inv = _inv_s(_mul_qt(q_inv, t))
    return (q_inv, t_inv)

class _PoseDisjointSet:
    def __init__(self, n: int):
        self.parent = list(range(n))
        self.m: list[M] = [(0, (0, 0, 0)) for _ in range(n)]
    def find(self, x: int) -> tuple[int, M]:
        if self.parent[x] == x:
            return x, (0, (0, 0, 0))
        root, m_r_p = self.find(self.parent[x])
        m_p_x = self.m[x]
        self.parent[x] = root
        self.m[x] = _mul_mm(m_r_p, m_p_x)
        return root, self.m[x]
    def union(self, x: int, y: int, m_x_y: M) -> bool:
        rx, m_rx_x = self.find(x)
        ry, m_ry_y = self.find(y)
        if rx == ry:
            return _mul_mm(m_rx_x, m_x_y) == m_ry_y
        else:
            self.parent[ry] = rx
            self.m[ry] = _mul_mm(_mul_mm(m_rx_x, m_x_y), _inv_m(m_ry_y))
            return True
    def roots(self) -> list[int]:
        return [i for i in range(len(self.parent)) if self.parent[i] == i]

def import_lego(topology: LegoTopology, env_id: Optional[int] = None, base_pos: Optional[Gf.Vec3d] = None, base_rot: Optional[Gf.Quatd] = None) -> None:
    if topology.schema != SchemaString:
        raise ValueError(f"Invalid schema: {topology.schema}")
    iface = get_brick_physics_interface()
    stage: Usd.Stage = omni.usd.get_context().get_stage()
    layer: Sdf.Layer = stage.GetEditTarget().GetLayer()
    with Sdf.ChangeBlock():
        paths = [iface.create_brick_sdf(brick.dimensions, brick.color, env_id) for brick in topology.bricks]

        brick_cnt = len(topology.bricks)
        id2idx = {brick.id: i for i, brick in enumerate(topology.bricks)}
        ds = _PoseDisjointSet(brick_cnt)
        for conn in topology.connections:
            if conn.parent not in id2idx:
                raise ValueError(f"Connection parent {conn.parent} not found")
            if conn.child not in id2idx:
                raise ValueError(f"Connection child {conn.child} not found")
            parent_idx = id2idx[conn.parent]
            child_idx = id2idx[conn.child]
            tx, ty = conn.offset
            q = conn.orientation
            h = topology.bricks[parent_idx].dimensions[2]
            ds.union(parent_idx, child_idx, (q, (tx, ty, h)))

            brick0_gid, _ = parse_brick_path(paths[parent_idx].pathString)
            brick1_gid, _ = parse_brick_path(paths[child_idx].pathString)
            conn_path = Sdf.Path(path_for_conn(brick0_gid, brick1_gid, env_id))
            conn_prim: Sdf.PrimSpec = Sdf.CreatePrimInLayer(layer, conn_path)
            conn_prim.specifier = Sdf.SpecifierDef
            conn_prim.typeName = "Xform"
            Sdf.RelationshipSpec(conn_prim, "lego_conn:body0").targetPathList.Append(paths[parent_idx])
            Sdf.RelationshipSpec(conn_prim, "lego_conn:body1").targetPathList.Append(paths[child_idx])
            Sdf.AttributeSpec(conn_prim, "lego_conn:offset_studs", Sdf.ValueTypeNames.Int2).default = Gf.Vec2i(tx, ty)
            Sdf.AttributeSpec(conn_prim, "lego_conn:yaw_index", Sdf.ValueTypeNames.Int).default = q
            Sdf.AttributeSpec(conn_prim, "lego_conn:pos0", Sdf.ValueTypeNames.Float3).default = ...
            Sdf.AttributeSpec(conn_prim, "lego_conn:rot0", Sdf.ValueTypeNames.Quatf).default = ...
            Sdf.AttributeSpec(conn_prim, "lego_conn:pos1", Sdf.ValueTypeNames.Float3).default = ...
            Sdf.AttributeSpec(conn_prim, "lego_conn:rot1", Sdf.ValueTypeNames.Quatf).default = ...
            Sdf.AttributeSpec(conn_prim, "lego_conn:overlap_xy", Sdf.ValueTypeNames.Float2).default = ...
            Sdf.AttributeSpec(conn_prim, "lego_conn:enabled", Sdf.ValueTypeNames.Bool).default = True
