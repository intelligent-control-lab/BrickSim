import omni.physx.scripts.utils as physx_utils
from pxr import Usd, UsdGeom, UsdPhysics

brick_length = 0.008    # 8 mm per stud
plate_height = 0.0032   # 3.2 mm per plate
stud_diameter = 0.0048  # 4.8 mm stud diameter
stud_height   = 0.0017  # 1.7 mm stud height
density = 500           # kg/m^3

def create_brick(stage: Usd.Stage, path: str, length_studs=4, width_studs=2, height_plates=3, color=(0.3, 0.3, 0.3)):
    length = length_studs * brick_length
    width  = width_studs * brick_length
    height = height_plates * plate_height

    brick: UsdGeom.Xform = UsdGeom.Xform.Define(stage, path)
    physx_utils.setPhysics(brick.GetPrim(), kinematic=False)

    body: UsdGeom.Cube = UsdGeom.Cube.Define(stage, f"{path}/Body")
    body.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(body).SetScale((length, width, height))
    UsdGeom.XformCommonAPI(body).SetTranslate((0, 0, height/2))
    physx_utils.setCollider(body.GetPrim())
    UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateDensityAttr(density)
    body.CreateDisplayColorAttr([color])

    for i in range(length_studs):
        for j in range(width_studs):
            stud: UsdGeom.Cylinder = UsdGeom.Cylinder.Define(stage, f"{path}/Stud_{i}_{j}")
            stud.CreateHeightAttr(1.0)
            x_offset = (i - (length_studs-1)/2) * brick_length
            y_offset = (j - (width_studs-1)/2) * brick_length
            z_offset = height + stud_height/2
            UsdGeom.XformCommonAPI(stud).SetTranslate((x_offset, y_offset, z_offset))
            UsdGeom.XformCommonAPI(stud).SetScale((stud_diameter/2, stud_diameter/2, stud_height))
            stud.CreateDisplayColorAttr([color])

    return brick
