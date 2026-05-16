# Franka RealSense D435 Camera Mount USD

`realsense_d435_camera_mount.usd` is generated from Franka's RealSense D435
camera mount model. It contains the mount visual mesh, two SolidWorks-exported
visual materials, convex-hull collision, and one rigid body on `/D435_mount`.

- Setup guide: https://franka.de/hubfs/Setup%20Guide%20for%20Camera%20Mount%20on%20Franka%20Robots_1.1_EN.pdf?hsLang=en
- Source archive: https://download.franka.de/camera_mount_guide.zip
- Source CAD from archive: `RealSenseD435_Camera_Mount.STEP`
- SolidWorks export used for conversion: `RealSenseD435_Camera_Mount.gltf` with `RealSenseD435_Camera_Mount.bin`
- Note: the source archive contains STEP/STL files and no separate license file.

The USD was generated from the repository root with:

```bash
uv run python -m bricksim --exp isaacsim.exp.base \
  --enable omni.kit.asset_converter \
  /path/to/convert_franka_d435_mount_to_usd.py
```

Conversion script source:

```python
from pathlib import Path
from tempfile import TemporaryDirectory

from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


async def main():
    source_path = Path("/path/to/RealSenseD435_Camera_Mount/RealSenseD435_Camera_Mount.gltf")
    output_path = Path.cwd() / "python" / "bricksim" / "assets" / "robots" / "fr3" / "camera_mount" / "realsense_d435_camera_mount.usd"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    enable_extension("omni.kit.asset_converter")

    import omni.kit.app
    import omni.kit.asset_converter

    with TemporaryDirectory() as temp_dir:
        imported_path = Path(temp_dir) / "realsense_d435_camera_mount_imported.usd"

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        context = omni.kit.asset_converter.AssetConverterContext()
        context.ignore_materials = False
        context.keep_all_materials = True
        context.ignore_camera = True
        context.ignore_light = True
        context.ignore_animations = True
        context.use_meter_as_world_unit = True
        context.create_world_as_default_root_prim = False
        context.merge_all_meshes = False

        task = omni.kit.asset_converter.get_instance().create_converter_task(
            str(source_path),
            str(imported_path),
            None,
            context,
        )
        await task.wait_until_finished()

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        imported_stage = Usd.Stage.Open(str(imported_path))
        stage = Usd.Stage.CreateNew(str(output_path))
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        root = UsdGeom.Xform.Define(stage, "/D435_mount").GetPrim()
        UsdGeom.Xform.Define(stage, "/D435_mount/visuals")
        stage.SetDefaultPrim(root)

        Sdf.CopySpec(
            imported_stage.GetRootLayer(),
            Sdf.Path("/World/Looks"),
            stage.GetRootLayer(),
            Sdf.Path("/D435_mount/Looks"),
        )
        Sdf.CopySpec(
            imported_stage.GetRootLayer(),
            Sdf.Path("/World/RealSenseD435_Camera_Mount/Mesh0"),
            stage.GetRootLayer(),
            Sdf.Path("/D435_mount/visuals/Mesh0"),
        )

        for prim in stage.TraverseAll():
            for rel in prim.GetRelationships():
                targets = rel.GetTargets()
                if targets:
                    rel.SetTargets([
                        Sdf.Path(str(target).replace("/World/Looks", "/D435_mount/Looks"))
                        for target in targets
                    ])

        mesh = stage.GetPrimAtPath("/D435_mount/visuals/Mesh0")
        UsdPhysics.CollisionAPI.Apply(mesh).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr("convexHull")

        UsdPhysics.RigidBodyAPI.Apply(root).CreateRigidBodyEnabledAttr(True)
        mass = UsdPhysics.MassAPI.Apply(root)
        mass.CreateMassAttr(0.036)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(-0.0329003, 0.0, 0.0211546))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1.8285e-05, 1.4397e-05, 1.7712e-05))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))

        flat_layer = stage.Flatten()
        flat_layer.defaultPrim = "D435_mount"
        flat_layer.documentation = ""
        flat_layer.comment = ""
        flat_layer.Export(str(output_path))

    print(output_path)
    omni.kit.app.get_app().post_quit()
```
