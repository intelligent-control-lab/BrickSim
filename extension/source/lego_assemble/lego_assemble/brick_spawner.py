import pxr.PhysxSchema as PhysxSchema
from typing import Tuple
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from . import lego_schemes
from .lego_schemes import StudHeight, StudDiameter, BrickLength

def create_brick(stage: Usd.Stage, path: str, dimensions: Tuple[int, int, int], color_name: str, use_cache: bool = True):
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
    collider_dimensions = [real_dimensions[0], real_dimensions[1], real_dimensions[2]+StudHeight]

    brick_prim.CreateAttribute("lego_dimensions", Sdf.ValueTypeNames.Int3).Set(Gf.Vec3i(*dimensions))
    brick_prim.CreateAttribute("lego_color", Sdf.ValueTypeNames.String).Set(color_name)
    Usd.ModelAPI(brick_prim).SetKind("component")
    rigidBodyAPI: UsdPhysics.RigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(brick_prim)
    rigidBodyAPI.CreateRigidBodyEnabledAttr(True)
    PhysxSchema.PhysxRigidBodyAPI.Apply(brick_prim)
    contactReportAPI: PhysxSchema.PhysxContactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(brick_prim)
    contactReportAPI.CreateThresholdAttr(0.0)

    collider: UsdGeom.Cube = UsdGeom.Cube.Define(stage, brick_path.AppendChild("BodyCollider"))
    collider.CreateSizeAttr(1.0)
    collider.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdGeom.XformCommonAPI(collider).SetScale(collider_dimensions)
    UsdGeom.XformCommonAPI(collider).SetTranslate((0, 0, collider_dimensions[2]/2))
    colliderPrim = collider.GetPrim()
    collisionAPI: UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI.Apply(colliderPrim)
    collisionAPI.CreateCollisionEnabledAttr(True)
    PhysxSchema.PhysxCollisionAPI.Apply(colliderPrim)
    massAPI = UsdPhysics.MassAPI.Apply(colliderPrim)
    massAPI.CreateMassAttr(lego_schemes.compute_mass(dimensions))

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
            x_offset = (i - (dimensions[0]-1)/2) * BrickLength
            y_offset = (j - (dimensions[1]-1)/2) * BrickLength
            z_offset = real_dimensions[2] + StudHeight/2
            UsdGeom.XformCommonAPI(stud).SetTranslate((x_offset, y_offset, z_offset))
            UsdGeom.XformCommonAPI(stud).SetScale((StudDiameter/2, StudDiameter/2, StudHeight))
