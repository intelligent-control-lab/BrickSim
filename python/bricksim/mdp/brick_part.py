"""Spawner configs for BrickSim LEGO brick assets."""

from typing import Callable

from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedEnv
from isaaclab.sim import (
    RigidBodyPropertiesCfg,
    SpawnerCfg,
    clone,
    get_current_stage,
    modify_rigid_body_properties,
)
from isaaclab.utils import configclass
from isaacsim.core.utils.xforms import reset_and_set_xform_ops
from pxr import Gf, Usd, UsdGeom

from bricksim.colors import parse_color
from bricksim.core import allocate_unmanaged_brick_part
from bricksim.units import BRICK_STUD_HEIGHT, BRICK_UNIT_LENGTH, PLATE_UNIT_HEIGHT

from .utils import MISSING


def set_brick_part_xform(
    prim: Usd.Prim,
    translation: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> None:
    """Set a brick prim's authored local pose.

    Args:
        prim: USD prim whose local transform should be updated.
        translation: Local position ``(x, y, z)`` in SI meters.
        orientation: Local quaternion ``(w, x, y, z)``.

    Existing translate and orient ops are edited in place. If either op is
    missing, the xform stack is rewritten while preserving current local scale.
    Quaternions use WXYZ storage.
    """
    xformable = UsdGeom.Xformable(prim)
    translate_op = None
    orient_op = None
    for op in xformable.GetOrderedXformOps():
        op_type = op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
        elif op_type == UsdGeom.XformOp.TypeOrient:
            orient_op = op
    if translate_op is None or orient_op is None:
        reset_and_set_xform_ops(
            prim,
            Gf.Vec3d(*translation),
            Gf.Quatd(*orientation),
            Gf.Vec3d(Gf.Transform(xformable.GetLocalTransformation()).GetScale()),
        )
        return
    translate_op.Set(Gf.Vec3d(*translation))
    orient_op.Set(Gf.Quatd(*orientation))


@clone
def spawn_brick_part(
    prim_path: str,
    cfg: "BrickPartCfg",
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    **kwargs,
) -> Usd.Prim:
    """Spawn or update a native BrickSim rigid brick part.

    Returns:
        USD prim for the spawned brick part.
    """
    stage = get_current_stage()
    prim: Usd.Prim = stage.GetPrimAtPath(prim_path)
    if isinstance(cfg.color, str):
        color = parse_color(cfg.color)
    else:
        color = cfg.color
    if not prim.IsValid():
        allocate_unmanaged_brick_part(cfg.dimensions, color, prim_path)
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"Failed to spawn BrickPart at '{prim_path}'.")
    set_brick_part_xform(prim, translation=translation, orientation=orientation)
    if cfg.rigid_props is not None:
        modify_rigid_body_properties(prim_path, cfg.rigid_props)
    return prim


@configclass
class BrickPartCfg(SpawnerCfg):
    """Spawner config for a physical BrickSim brick part."""

    func: Callable = spawn_brick_part
    dimensions: tuple[int, int, int] = MISSING
    color: str | tuple[int, int, int] = MISSING
    rigid_props: RigidBodyPropertiesCfg | None = None


def _build_marker_wireframe_points(dimensions: tuple[int, int, int]) -> list[Gf.Vec3f]:
    length = float(dimensions[0]) * BRICK_UNIT_LENGTH
    width = float(dimensions[1]) * BRICK_UNIT_LENGTH
    top_z = float(dimensions[2]) * PLATE_UNIT_HEIGHT + BRICK_STUD_HEIGHT
    x0 = -length / 2.0
    y0 = -width / 2.0
    z0 = 0.0
    x1 = length / 2.0
    y1 = width / 2.0
    z1 = top_z

    p000 = Gf.Vec3f(x0, y0, z0)
    p100 = Gf.Vec3f(x1, y0, z0)
    p110 = Gf.Vec3f(x1, y1, z0)
    p010 = Gf.Vec3f(x0, y1, z0)
    p001 = Gf.Vec3f(x0, y0, z1)
    p101 = Gf.Vec3f(x1, y0, z1)
    p111 = Gf.Vec3f(x1, y1, z1)
    p011 = Gf.Vec3f(x0, y1, z1)

    return [
        p000,
        p100,
        p100,
        p110,
        p110,
        p010,
        p010,
        p000,
        p001,
        p101,
        p101,
        p111,
        p111,
        p011,
        p011,
        p001,
        p000,
        p001,
        p100,
        p101,
        p110,
        p111,
        p010,
        p011,
    ]


def _configure_marker_curves(
    stage: Usd.Stage,
    prim: Usd.Prim,
    dimensions: tuple[int, int, int],
    color: tuple[int, int, int],
) -> None:
    curves_path = prim.GetPath().AppendChild("EdgeCurves")
    existing = stage.GetPrimAtPath(curves_path)
    if existing.IsValid() and existing.GetTypeName() != "BasisCurves":
        raise RuntimeError(
            f"Cannot create marker wireframe at '{curves_path}': existing "
            f"child has type '{existing.GetTypeName()}'."
        )
    curves = UsdGeom.BasisCurves.Define(stage, curves_path)

    curves.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
    curves.CreateWrapAttr().Set(UsdGeom.Tokens.nonperiodic)
    curves.CreateCurveVertexCountsAttr().Set([2] * 12)
    curves.CreatePointsAttr().Set(_build_marker_wireframe_points(dimensions))
    curves.CreateWidthsAttr().Set([0.001])
    curves.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    curves.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set(
        [Gf.Vec3f(*(float(c) / 255.0 for c in color))]
    )


@clone
def spawn_marker_brick_part(
    prim_path: str,
    cfg: "MarkerBrickPartCfg",
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    **kwargs,
) -> Usd.Prim:
    """Spawn or update a visual marker brick wireframe.

    Returns:
        USD prim for the marker brick.
    """
    stage = get_current_stage()
    prim: Usd.Prim = stage.GetPrimAtPath(prim_path)
    if isinstance(cfg.color, str):
        color = parse_color(cfg.color)
    else:
        color = cfg.color
    if not prim.IsValid():
        prim = UsdGeom.Xform.Define(stage, prim_path).GetPrim()
        if not prim.IsValid():
            raise RuntimeError(f"Failed to spawn marker wireframe at '{prim_path}'.")
    set_brick_part_xform(prim, translation=translation, orientation=orientation)
    _configure_marker_curves(stage, prim, cfg.dimensions, color)
    return prim


@configclass
class MarkerBrickPartCfg(SpawnerCfg):
    """Spawner config for a non-physical marker brick wireframe."""

    func: Callable = spawn_marker_brick_part
    dimensions: tuple[int, int, int] = MISSING
    color: str | tuple[int, int, int] = MISSING


def scene_entity_brick_part_dimensions(
    env: ManagerBasedEnv,
    entity_name: str,
) -> tuple[int, int, int]:
    """Return BrickSim dimensions for a brick-part scene entity.

    Returns:
        Brick dimensions as ``(length, width, height)`` in BrickSim units.
    """
    entity_cfg = getattr(env.scene.cfg, entity_name)
    spawn_cfg = getattr(entity_cfg, "spawn")
    if not isinstance(spawn_cfg, (BrickPartCfg, MarkerBrickPartCfg)):
        raise TypeError(
            f"Scene entity '{entity_name}' must be spawned with a BrickSim "
            f"brick spawn cfg, got {type(spawn_cfg)}"
        )
    return spawn_cfg.dimensions


def resolve_brick_rigid_object(env: ManagerBasedEnv, entity_name: str) -> RigidObject:
    """Resolve a BrickSim brick scene entity to its runtime Isaac Lab asset.

    Args:
        env: The manager-based Isaac Lab environment.
        entity_name: Name of the scene entity in ``env.scene``.

    Returns:
        The resolved ``RigidObject`` instance.

    Raises:
        TypeError: If the named scene entity is not a ``RigidObject`` or was
            not spawned with ``BrickPartCfg``.

    This helper is intentionally narrow. It only supports connection-capable
    BrickSim bricks, which in the current design are represented as
    ``RigidObject`` instances spawned with ``BrickPartCfg``. Marker bricks and
    other non-physical assets are excluded.
    """
    asset = env.scene[entity_name]
    if not isinstance(asset, RigidObject):
        raise TypeError(
            f"Scene entity '{entity_name}' must resolve to RigidObject, got "
            f"{type(asset)}"
        )
    if not isinstance(asset.cfg.spawn, BrickPartCfg):
        raise TypeError(
            f"Scene entity '{entity_name}' must be spawned with BrickPartCfg, "
            f"got {type(asset.cfg.spawn)}"
        )
    return asset
