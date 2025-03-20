import os
import logging
import omni.physx.scripts.utils as physx_utils
from typing import Tuple
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema
from . import lego_schemes

logger = logging.getLogger(__name__)

def create_brick(stage: Usd.Stage, path: str, dimensions=(4,2,3), color_name="Black", use_cache=True):
    if use_cache:
        filepath = os.path.join(get_cache_dir(), f"{dimensions[0]}x{dimensions[1]}x{dimensions[2]}_{color_name}.usd")
        if not os.path.exists(filepath):
            build_brick_cache(dimensions, color_name, filepath)
        return create_brick_from_reference(stage, path, filepath)
    else:
        return build_brick(stage, path, dimensions, color_name)

def build_brick(stage: Usd.Stage, path: str, dimensions: Tuple[int, int, int], color_name: str):
    color = lego_schemes.parse_color(color_name)
    real_dimensions = lego_schemes.to_real_dimensions(dimensions)
    collider_dimensions = [real_dimensions[0], real_dimensions[1], real_dimensions[2]+lego_schemes.StudHeight]

    brick: UsdGeom.Xform = UsdGeom.Xform.Define(stage, path)
    brick.GetPrim().CreateAttribute("lego_dimensions", Sdf.ValueTypeNames.Int3).Set(Gf.Vec3i(*dimensions))
    brick.GetPrim().CreateAttribute("lego_color", Sdf.ValueTypeNames.String).Set(color_name)
    Usd.ModelAPI(brick).SetKind("component")
    physx_utils.setPhysics(brick.GetPrim(), kinematic=False)
    contactReport = PhysxSchema.PhysxContactReportAPI.Apply(brick.GetPrim())
    contactReport.CreateThresholdAttr().Set(0.0)

    collider: UsdGeom.Cube = UsdGeom.Cube.Define(stage, f"{path}/BodyCollider")
    collider.CreateSizeAttr(1.0)
    collider.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
    UsdGeom.XformCommonAPI(collider).SetScale(collider_dimensions)
    UsdGeom.XformCommonAPI(collider).SetTranslate((0, 0, collider_dimensions[2]/2))
    physx_utils.setCollider(collider.GetPrim())
    UsdPhysics.MassAPI.Apply(collider.GetPrim()).CreateMassAttr(lego_schemes.compute_mass(dimensions))

    body: UsdGeom.Cube = UsdGeom.Cube.Define(stage, f"{path}/Body")
    body.CreateSizeAttr(1.0)
    body.CreateDisplayColorAttr([color])
    UsdGeom.XformCommonAPI(body).SetScale(real_dimensions)
    UsdGeom.XformCommonAPI(body).SetTranslate((0, 0, real_dimensions[2]/2))

    for i in range(dimensions[0]):
        for j in range(dimensions[1]):
            stud: UsdGeom.Cylinder = UsdGeom.Cylinder.Define(stage, f"{path}/Stud_{i}_{j}")
            stud.CreateHeightAttr(1.0)
            stud.CreateDisplayColorAttr([color])
            x_offset = (i - (dimensions[0]-1)/2) * lego_schemes.BrickLength
            y_offset = (j - (dimensions[1]-1)/2) * lego_schemes.BrickLength
            z_offset = real_dimensions[2] + lego_schemes.StudHeight/2
            UsdGeom.XformCommonAPI(stud).SetTranslate((x_offset, y_offset, z_offset))
            UsdGeom.XformCommonAPI(stud).SetScale((lego_schemes.StudDiameter/2, lego_schemes.StudDiameter/2, lego_schemes.StudHeight))

    return brick

def create_brick_from_reference(stage: Usd.Stage, path: str, filepath: str):
    brick: UsdGeom.Xform = UsdGeom.Xform.Define(stage, path)
    brick.GetPrim().GetReferences().AddReference(filepath, "/Brick")
    return brick

def build_brick_cache(dimensions: Tuple[int, int, int], color_name: str, filepath: str):
    logger.info(f"Writing brick USD: {filepath}")
    stage: Usd.Stage = Usd.Stage.CreateInMemory("Brick")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    brick = build_brick(stage, "/Brick", dimensions, color_name)
    brick.GetPrim().SetInstanceable(True)
    stage.Export(filepath)

def get_cache_dir():
    return os.path.join(os.getcwd(), "resources", "bricks")
