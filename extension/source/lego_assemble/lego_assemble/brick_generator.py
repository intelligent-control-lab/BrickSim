import omni.physx.scripts.utils as physx_utils
import pxr.PhysxSchema as PhysxSchema
from typing import Tuple
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from . import lego_schemes

def create_brick(stage: Usd.Stage, path: str, dimensions=(4,2,3), color_name="Black", use_cache=True):
    brick: UsdGeom.Xform = UsdGeom.Xform.Define(stage, path)

    if use_cache:
        class_brick_prim_path = f"/__CLASS__/Brick_{dimensions[0]}x{dimensions[1]}x{dimensions[2]}_{color_name}"
        class_brick_prim = stage.GetPrimAtPath(class_brick_prim_path)
        if not class_brick_prim.IsValid():
            class_brick_prim = stage.CreateClassPrim(class_brick_prim_path)
            build_brick(stage, class_brick_prim, dimensions, color_name)
        brick.GetPrim().GetInherits().AddInherit(class_brick_prim.GetPath())
    else:
        build_brick(stage, brick.GetPrim(), dimensions, color_name)

    return brick

def build_brick(stage: Usd.Stage, brick_prim: Usd.Prim, dimensions: Tuple[int, int, int], color_name: str):
    brick_path: Sdf.Path = brick_prim.GetPath()
    color = lego_schemes.parse_color(color_name)
    real_dimensions = lego_schemes.to_real_dimensions(dimensions)
    collider_dimensions = [real_dimensions[0], real_dimensions[1], real_dimensions[2]+lego_schemes.StudHeight]

    brick_prim.CreateAttribute("lego_dimensions", Sdf.ValueTypeNames.Int3).Set(Gf.Vec3i(*dimensions))
    brick_prim.CreateAttribute("lego_color", Sdf.ValueTypeNames.String).Set(color_name)
    Usd.ModelAPI(brick_prim).SetKind("component")
    physx_utils.setPhysics(brick_prim, kinematic=False)
    contactReport: PhysxSchema.PhysxContactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(brick_prim)
    contactReport.CreateThresholdAttr().Set(0.0)

    collider: UsdGeom.Cube = UsdGeom.Cube.Define(stage, brick_path.AppendChild("BodyCollider"))
    collider.CreateSizeAttr(1.0)
    collider.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
    UsdGeom.XformCommonAPI(collider).SetScale(collider_dimensions)
    UsdGeom.XformCommonAPI(collider).SetTranslate((0, 0, collider_dimensions[2]/2))
    physx_utils.setCollider(collider.GetPrim())
    UsdPhysics.MassAPI.Apply(collider.GetPrim()).CreateMassAttr(lego_schemes.compute_mass(dimensions))

    body: UsdGeom.Cube = UsdGeom.Cube.Define(stage, brick_path.AppendChild("Body"))
    body.CreateSizeAttr(1.0)
    body.CreateDisplayColorAttr([color])
    UsdGeom.XformCommonAPI(body).SetScale(real_dimensions)
    UsdGeom.XformCommonAPI(body).SetTranslate((0, 0, real_dimensions[2]/2))

    for i in range(dimensions[0]):
        for j in range(dimensions[1]):
            stud: UsdGeom.Cylinder = UsdGeom.Cylinder.Define(stage, brick_path.AppendChild(f"Stud_{i}_{j}"))
            stud.CreateHeightAttr(1.0)
            stud.CreateDisplayColorAttr([color])
            x_offset = (i - (dimensions[0]-1)/2) * lego_schemes.BrickLength
            y_offset = (j - (dimensions[1]-1)/2) * lego_schemes.BrickLength
            z_offset = real_dimensions[2] + lego_schemes.StudHeight/2
            UsdGeom.XformCommonAPI(stud).SetTranslate((x_offset, y_offset, z_offset))
            UsdGeom.XformCommonAPI(stud).SetScale((lego_schemes.StudDiameter/2, lego_schemes.StudDiameter/2, lego_schemes.StudHeight))
