from typing import Tuple, Union
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from lego_assemble.physics import lego_schemes
from lego_assemble.physics.lego_schemes import StudHeight, StudDiameter, BrickLength

def create_brick(stage: Usd.Stage, path: Union[Sdf.Path, str], dimensions: Tuple[int, int, int], color_name: str, use_cache: bool = True) -> None:
    if isinstance(path, str):
        path = Sdf.Path(path)
    layer = stage.GetEditTarget().GetLayer()
    if use_cache:
        class_path = ensure_brick_class(stage, dimensions, color_name)
        with Sdf.ChangeBlock():
            prim: Sdf.PrimSpec = Sdf.CreatePrimInLayer(layer, path)
            prim.specifier = Sdf.SpecifierDef
            prim.typeName = "Xform"
            prim.inheritPathList.Append(class_path)
    else:
        construct_brick(stage, path, dimensions, color_name, is_class=False)

def ensure_brick_class(stage: Usd.Stage, dimensions: Tuple[int, int, int], color_name: str) -> Sdf.Path:
    layer: Sdf.Layer = stage.GetEditTarget().GetLayer()

    class_root_path = Sdf.Path("/__CLASS__")
    if not layer.GetPrimAtPath(class_root_path):
        with Sdf.ChangeBlock():
            class_root: Sdf.PrimSpec = Sdf.CreatePrimInLayer(layer, class_root_path)
            class_root.specifier = Sdf.SpecifierDef

    sanitized_color = color_name.replace(" ", "_").replace("-", "_")
    class_path: Sdf.Path = class_root_path.AppendChild(f"Brick_{dimensions[0]}x{dimensions[1]}x{dimensions[2]}_{sanitized_color}")
    if not layer.GetPrimAtPath(class_path):
        construct_brick(stage, class_path, dimensions, color_name, is_class=True)
    return class_path

def construct_brick(stage: Usd.Stage, root_path: Sdf.Path, dimensions: Tuple[int, int, int], color_name: str, is_class: bool) -> None:
    mpu: float = UsdGeom.GetStageMetersPerUnit(stage)
    kpu: float = UsdPhysics.GetStageKilogramsPerUnit(stage)
    color = lego_schemes.parse_color(color_name)
    real_dimensions = lego_schemes.to_real_dimensions(dimensions)

    layer: Sdf.Layer = stage.GetEditTarget().GetLayer()
    with Sdf.ChangeBlock():
        root: Sdf.PrimSpec = Sdf.CreatePrimInLayer(layer, root_path)
        if is_class:
            root.specifier = Sdf.SpecifierClass
        else:
            root.specifier = Sdf.SpecifierDef
            root.typeName = "Xform"
        root.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI"]))
        root.SetInfo("kind", "component")
        Sdf.AttributeSpec(root, "lego_brick:dimensions", Sdf.ValueTypeNames.Int3).default = Gf.Vec3i(dimensions)
        Sdf.AttributeSpec(root, "lego_brick:color", Sdf.ValueTypeNames.String).default = color_name
        Sdf.AttributeSpec(root, "physics:rigidBodyEnabled", Sdf.ValueTypeNames.Bool).default = True
        Sdf.AttributeSpec(root, "physxContactReport:threshold", Sdf.ValueTypeNames.Float).default = 0.0

        bc: Sdf.PrimSpec = Sdf.CreatePrimInLayer(layer, root_path.AppendChild("BodyCollider"))
        bc.specifier = Sdf.SpecifierDef
        bc.typeName = "Cube"
        bc.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=["PhysicsCollisionAPI", "PhysicsMassAPI"]))
        Sdf.AttributeSpec(bc, "size", Sdf.ValueTypeNames.Double).default = 1.0
        Sdf.AttributeSpec(bc, "visibility", Sdf.ValueTypeNames.Token).default = "invisible"
        Sdf.AttributeSpec(bc, "xformOp:scale", Sdf.ValueTypeNames.Float3).default = Gf.Vec3f((
            real_dimensions[0] / mpu,
            real_dimensions[1] / mpu,
            real_dimensions[2] / mpu,
        ))
        Sdf.AttributeSpec(bc, "xformOp:translate", Sdf.ValueTypeNames.Double3).default = Gf.Vec3f((
            0,
            0,
            real_dimensions[2] / 2 / mpu,
        ))
        Sdf.AttributeSpec(bc, "xformOpOrder", Sdf.ValueTypeNames.TokenArray).default = ["xformOp:translate", "xformOp:scale"]
        Sdf.AttributeSpec(bc, "physics:collisionEnabled", Sdf.ValueTypeNames.Bool).default = True
        Sdf.AttributeSpec(bc, "physics:mass", Sdf.ValueTypeNames.Float).default = lego_schemes.compute_mass(dimensions) / kpu

        tc: Sdf.PrimSpec = Sdf.CreatePrimInLayer(layer, root_path.AppendChild("TopCollider"))
        tc.specifier = Sdf.SpecifierDef
        tc.typeName = "Cube"
        tc.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=["PhysicsCollisionAPI", "PhysicsMassAPI"]))
        Sdf.AttributeSpec(tc, "size", Sdf.ValueTypeNames.Double).default = 1.0
        Sdf.AttributeSpec(tc, "visibility", Sdf.ValueTypeNames.Token).default = "invisible"
        Sdf.AttributeSpec(tc, "xformOp:scale", Sdf.ValueTypeNames.Float3).default = Gf.Vec3f((
            real_dimensions[0] / mpu,
            real_dimensions[1] / mpu,
            StudHeight / mpu,
        ))
        Sdf.AttributeSpec(tc, "xformOp:translate", Sdf.ValueTypeNames.Double3).default = Gf.Vec3f((
            0,
            0,
            (real_dimensions[2] + StudHeight / 2) / mpu,
        ))
        Sdf.AttributeSpec(tc, "xformOpOrder", Sdf.ValueTypeNames.TokenArray).default = ["xformOp:translate", "xformOp:scale"]
        Sdf.AttributeSpec(tc, "physics:collisionEnabled", Sdf.ValueTypeNames.Bool).default = True
        Sdf.AttributeSpec(tc, "physics:mass", Sdf.ValueTypeNames.Float).default = 0.0

        body: Sdf.PrimSpec = Sdf.CreatePrimInLayer(layer, root_path.AppendChild("Body"))
        body.specifier = Sdf.SpecifierDef
        body.typeName = "Cube"
        Sdf.AttributeSpec(body, "size", Sdf.ValueTypeNames.Double).default = 1.0
        Sdf.AttributeSpec(body, "xformOp:scale", Sdf.ValueTypeNames.Float3).default = Gf.Vec3f((
            real_dimensions[0] / mpu,
            real_dimensions[1] / mpu,
            real_dimensions[2] / mpu,
        ))
        Sdf.AttributeSpec(body, "xformOp:translate", Sdf.ValueTypeNames.Double3).default = Gf.Vec3f((
            0,
            0,
            real_dimensions[2] / 2 / mpu,
        ))
        Sdf.AttributeSpec(body, "xformOpOrder", Sdf.ValueTypeNames.TokenArray).default = ["xformOp:translate", "xformOp:scale"]
        Sdf.AttributeSpec(body, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).default = [color]

        for i in range(dimensions[0]):
            for j in range(dimensions[1]):
                stud: Sdf.PrimSpec = Sdf.CreatePrimInLayer(layer, root_path.AppendChild(f"Stud_{i}_{j}"))
                stud.specifier = Sdf.SpecifierDef
                stud.typeName = "Cylinder"
                Sdf.AttributeSpec(stud, "height", Sdf.ValueTypeNames.Double).default = 1.0
                Sdf.AttributeSpec(stud, "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).default = [color]                
                x_offset = (i - (dimensions[0]-1)/2) * BrickLength
                y_offset = (j - (dimensions[1]-1)/2) * BrickLength
                z_offset = real_dimensions[2] + StudHeight/2
                Sdf.AttributeSpec(stud, "xformOp:translate", Sdf.ValueTypeNames.Double3).default = Gf.Vec3f((
                    x_offset / mpu,
                    y_offset / mpu,
                    z_offset / mpu,
                ))
                Sdf.AttributeSpec(stud, "xformOp:scale", Sdf.ValueTypeNames.Float3).default = Gf.Vec3f((
                    StudDiameter / 2 / mpu,
                    StudDiameter / 2 / mpu,
                    StudHeight / mpu,
                ))
                Sdf.AttributeSpec(stud, "xformOpOrder", Sdf.ValueTypeNames.TokenArray).default = ["xformOp:translate", "xformOp:scale"]
