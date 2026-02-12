from dataclasses import MISSING
from typing import Callable

from isaaclab.sim import (PreviewSurfaceCfg, RigidBodyPropertiesCfg, SpawnerCfg, bind_visual_material, clone,
                          get_current_stage, modify_rigid_body_properties,
                          standardize_xform_ops)
from isaaclab.utils import configclass
from lego_assemble._native import allocate_unmanaged_brick_part
from lego_assemble.colors import parse_color
from pxr import Usd, UsdPhysics


@clone
def spawn_brick_part(
    prim_path: str,
    cfg: 'BrickPartCfg',
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
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
    standardize_xform_ops(prim, translation=translation, orientation=orientation)
    if cfg.rigid_props is not None:
        modify_rigid_body_properties(prim_path, cfg.rigid_props)
    return prim

@configclass
class BrickPartCfg(SpawnerCfg):
    func: Callable = spawn_brick_part
    dimensions: tuple[int, int, int] = MISSING
    color: str | tuple[int, int, int] = MISSING
    rigid_props: RigidBodyPropertiesCfg | None = None

def _bind_marker_material(
    stage: Usd.Stage,
    prim_path: str,
    color: tuple[int, int, int],
    opacity: float,
) -> None:
    material_path = f"{prim_path}/Looks/Marker"
    if not stage.GetPrimAtPath(material_path).IsValid():
        material_cfg = PreviewSurfaceCfg(
            diffuse_color=tuple(c / 255.0 for c in color),
            opacity=opacity,
            roughness=1.0,
            metallic=0.5,
        )
        material_cfg.func(material_path, material_cfg)
    bind_visual_material(
        prim_path=prim_path,
        material_path=material_path,
        stage=stage,
        stronger_than_descendants=True,
    )

@clone
def spawn_marker_brick_part(
    prim_path: str,
    cfg: 'MarkerBrickPartCfg',
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
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
    standardize_xform_ops(prim, translation=translation, orientation=orientation)
    prim.SetInstanceable(False)
    prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    stage.OverridePrim(prim.GetPath().AppendChild("BodyCollider")).SetActive(False)
    stage.OverridePrim(prim.GetPath().AppendChild("TopCollider")).SetActive(False)
    _bind_marker_material(stage, prim_path, color, cfg.opacity)
    return prim

@configclass
class MarkerBrickPartCfg(SpawnerCfg):
    func: Callable = spawn_marker_brick_part
    dimensions: tuple[int, int, int] = MISSING
    color: str | tuple[int, int, int] = MISSING
    opacity: float = 0.1
