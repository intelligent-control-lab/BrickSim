"""Spawner configs for BrickSim LEGO brick assets."""

from typing import Callable

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

from .utils import MISSING


def _reset_brick_xform_ops(
    prim: Usd.Prim,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
) -> None:
    xformable = UsdGeom.Xformable(prim)
    local_transform = Gf.Transform(xformable.GetLocalTransformation())
    resolved_translation = (
        Gf.Vec3d(local_transform.GetTranslation())
        if translation is None
        else Gf.Vec3d(*translation)
    )
    resolved_orientation = (
        # BrickSim spawn inputs use WXYZ quaternions here.
        Gf.Quatd(local_transform.GetRotation().GetQuat())
        if orientation is None
        else Gf.Quatd(*orientation)
    )
    resolved_scale = Gf.Vec3d(local_transform.GetScale())
    reset_and_set_xform_ops(
        prim, resolved_translation, resolved_orientation, resolved_scale
    )


@clone
def spawn_brick_part(
    prim_path: str,
    cfg: "BrickPartCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
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
    _reset_brick_xform_ops(prim, translation=translation, orientation=orientation)
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
    length = float(dimensions[0]) * 0.008
    width = float(dimensions[1]) * 0.008
    top_z = float(dimensions[2]) * 0.0032 + 0.0017
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
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
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
    _reset_brick_xform_ops(prim, translation=translation, orientation=orientation)
    _configure_marker_curves(stage, prim, cfg.dimensions, color)
    return prim


@configclass
class MarkerBrickPartCfg(SpawnerCfg):
    """Spawner config for a non-physical marker brick wireframe."""

    func: Callable = spawn_marker_brick_part
    dimensions: tuple[int, int, int] = MISSING
    color: str | tuple[int, int, int] = MISSING
