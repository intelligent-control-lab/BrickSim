import pxr.PhysxSchema as PhysxSchema
from typing import Tuple
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from . import lego_schemes
from .lego_schemes import StudHeight, StudDiameter, BrickLength

def create_brick(stage: Usd.Stage, path: str, dimensions: Tuple[int, int, int], color_name: str, use_cache: bool = True):
    brick: UsdGeom.Xform = UsdGeom.Xform.Define(stage, path)

    if use_cache:
        santitized_color_name = color_name.replace(" ", "_").replace("-", "_")
        class_brick_prim_path = f"/__CLASS__/Brick_{dimensions[0]}x{dimensions[1]}x{dimensions[2]}_{santitized_color_name}"
        class_brick_prim = stage.GetPrimAtPath(class_brick_prim_path)
        if not class_brick_prim.IsValid():
            class_brick_prim = stage.CreateClassPrim(class_brick_prim_path)
            build_brick(stage, class_brick_prim, dimensions, color_name)
        brick.GetPrim().GetInherits().AddInherit(class_brick_prim.GetPath())
    else:
        build_brick(stage, brick.GetPrim(), dimensions, color_name)

    return brick

def build_brick(stage: Usd.Stage, brick_prim: Usd.Prim, dimensions: Tuple[int, int, int], color_name: str):
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    kpu = UsdPhysics.GetStageKilogramsPerUnit(stage)

    brick_path: Sdf.Path = brick_prim.GetPath()
    color = lego_schemes.parse_color(color_name)
    real_dimensions = lego_schemes.to_real_dimensions(dimensions)

    brick_prim.CreateAttribute("lego_dimensions", Sdf.ValueTypeNames.Int3).Set(Gf.Vec3i(*dimensions))
    brick_prim.CreateAttribute("lego_color", Sdf.ValueTypeNames.String).Set(color_name)
    Usd.ModelAPI(brick_prim).SetKind("component")
    rigidBodyAPI: UsdPhysics.RigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(brick_prim)
    rigidBodyAPI.CreateRigidBodyEnabledAttr(True)
    PhysxSchema.PhysxRigidBodyAPI.Apply(brick_prim)
    contactReportAPI: PhysxSchema.PhysxContactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(brick_prim)
    contactReportAPI.CreateThresholdAttr(0.0)

    bc: UsdGeom.Cube = UsdGeom.Cube.Define(stage, brick_path.AppendChild("BodyCollider"))
    bc.CreateSizeAttr(1.0)
    bc.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdGeom.XformCommonAPI(bc).SetScale((
        real_dimensions[0] / mpu,
        real_dimensions[1] / mpu,
        real_dimensions[2] / mpu
    ))
    UsdGeom.XformCommonAPI(bc).SetTranslate((
        0,
        0,
        real_dimensions[2] / 2 / mpu
    ))
    bcPrim = bc.GetPrim()
    bcCollisionAPI: UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI.Apply(bcPrim)
    bcCollisionAPI.CreateCollisionEnabledAttr(True)
    bcMassAPI = UsdPhysics.MassAPI.Apply(bcPrim)
    bcMassAPI.CreateMassAttr(lego_schemes.compute_mass(dimensions) / kpu)

    tc: UsdGeom.Cube = UsdGeom.Cube.Define(stage, brick_path.AppendChild("TopCollider"))
    tc.CreateSizeAttr(1.0)
    tc.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdGeom.XformCommonAPI(tc).SetScale((
        real_dimensions[0] / mpu,
        real_dimensions[1] / mpu,
        StudHeight / mpu
    ))
    UsdGeom.XformCommonAPI(tc).SetTranslate((
        0,
        0,
        (real_dimensions[2] + StudHeight / 2) / mpu
    ))
    tcPrim = tc.GetPrim()
    tcCollisionAPI: UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI.Apply(tcPrim)
    tcCollisionAPI.CreateCollisionEnabledAttr(True)
    tcMassAPI = UsdPhysics.MassAPI.Apply(tcPrim)
    tcMassAPI.CreateMassAttr(0.0)

    body: UsdGeom.Cube = UsdGeom.Cube.Define(stage, brick_path.AppendChild("Body"))
    body.CreateSizeAttr(1.0)
    body.CreateDisplayColorAttr([color])
    UsdGeom.XformCommonAPI(body).SetScale((
        real_dimensions[0] / mpu,
        real_dimensions[1] / mpu,
        real_dimensions[2] / mpu
    ))
    UsdGeom.XformCommonAPI(body).SetTranslate((
        0,
        0,
        real_dimensions[2] / 2 / mpu
    ))

    for i in range(dimensions[0]):
        for j in range(dimensions[1]):
            stud: UsdGeom.Cylinder = UsdGeom.Cylinder.Define(stage, brick_path.AppendChild(f"Stud_{i}_{j}"))
            stud.CreateHeightAttr(1.0)
            stud.CreateDisplayColorAttr([color])
            x_offset = (i - (dimensions[0]-1)/2) * BrickLength
            y_offset = (j - (dimensions[1]-1)/2) * BrickLength
            z_offset = real_dimensions[2] + StudHeight/2
            UsdGeom.XformCommonAPI(stud).SetTranslate((
                x_offset / mpu,
                y_offset / mpu,
                z_offset / mpu
            ))
            UsdGeom.XformCommonAPI(stud).SetScale((
                StudDiameter / 2 / mpu,
                StudDiameter / 2 / mpu,
                StudHeight / mpu
            ))
