# Intel RealSense D435 USD

`d435.usd` is a single USD generated from the D435 URDF/Xacro in
`realsense-ros`. It contains the D435 visual body, the URDF collision box, and
the nominal RealSense frame prims. It does not define an active camera sensor or
camera intrinsics.

- Repository: https://github.com/realsenseai/realsense-ros
- Commit: `6d87b071dcfef15f2a0407e1e78945256add70d0`
- Git descriptor: `4.57.6-42-g6d87b071`
- Source Xacro: `realsense2_description/urdf/test_d435_camera.urdf.xacro`
- Source mesh: `realsense2_description/meshes/d435.dae`
- ROS package: `realsense2_description` version `4.57.7`
- License: Apache License 2.0
- Upstream license: https://github.com/realsenseai/realsense-ros/blob/6d87b071dcfef15f2a0407e1e78945256add70d0/LICENSE
- Upstream notice: https://github.com/realsenseai/realsense-ros/blob/6d87b071dcfef15f2a0407e1e78945256add70d0/NOTICE.md

The USD was generated from the repository root with:

```bash
uv run python -m bricksim --exp isaacsim.exp.base \
  --enable isaacsim.asset.importer.urdf \
  --enable omni.kit.asset_converter \
  /path/to/convert_realsense_d435_urdf_to_usd_in_kit.py
```

Conversion script source:

```python
from pathlib import Path
from tempfile import TemporaryDirectory
from xml.etree import ElementTree

import xacro
import xacro.substitution_args


async def main():
    realsense_ros_dir = Path("/path/to/realsense-ros")
    package_dir = realsense_ros_dir / "realsense2_description"
    xacro_path = package_dir / "urdf" / "test_d435_camera.urdf.xacro"
    mesh_path = package_dir / "meshes" / "d435.dae"
    output_path = Path.cwd() / "python" / "bricksim" / "assets" / "sensors" / "realsense" / "d435" / "d435.usd"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    original_eval_find = xacro.substitution_args._eval_find

    def eval_find(package_name):
        if package_name == "realsense2_description":
            return str(package_dir)
        return original_eval_find(package_name)

    xacro.substitution_args._eval_find = eval_find
    doc = xacro.process_file(
        str(xacro_path),
        mappings={
            "use_nominal_extrinsics": "true",
            "add_plug": "false",
            "use_mesh": "true",
        },
    )

    robot = ElementTree.fromstring(doc.toxml())
    robot.set("name", "base_link")
    for child in list(robot):
        if child.tag == "link" and child.attrib["name"] == "base_link":
            robot.remove(child)
        if child.tag == "joint" and child.attrib["name"] == "camera_joint":
            robot.remove(child)
    for mesh in robot.findall(".//mesh"):
        if mesh.attrib["filename"] == "package://realsense2_description/meshes/d435.dae":
            mesh.set("filename", str(mesh_path))

    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.asset.importer.urdf")
    enable_extension("omni.kit.asset_converter")

    import omni.kit.app
    import omni.kit.commands
    from pxr import Gf, Sdf, Usd, UsdPhysics

    with TemporaryDirectory() as temp_dir:
        temp_dir_path = Path(temp_dir)
        urdf_path = temp_dir_path / "d435.urdf"
        imported_usd_path = temp_dir_path / "d435_imported.usd"
        urdf_path.write_text(ElementTree.tostring(robot, encoding="unicode"))

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = False
        import_config.collision_from_visuals = False
        import_config.make_default_prim = True

        omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(urdf_path),
            import_config=import_config,
            dest_path=str(imported_usd_path),
        )

        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        stage = Usd.Stage.Open(str(imported_usd_path))
        base_link = stage.GetPrimAtPath("/base_link")
        stage.SetDefaultPrim(base_link)
        stage.RemovePrim("/base_link/joints")

        banned_api_schemas = {
            "PhysicsRigidBodyAPI",
            "PhysxRigidBodyAPI",
            "PhysicsMassAPI",
            "PhysicsArticulationRootAPI",
            "PhysxArticulationAPI",
            "IsaacRobotAPI",
            "IsaacLinkAPI",
            "IsaacJointAPI",
        }
        for prim in stage.TraverseAll():
            if prim.IsInstance():
                prim.SetInstanceable(False)
            schemas = prim.GetMetadata("apiSchemas")
            if schemas:
                keep_schemas = [
                    schema for schema in schemas.GetAddedOrExplicitItems()
                    if schema not in banned_api_schemas
                ]
                if keep_schemas:
                    prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(keep_schemas))
                else:
                    prim.ClearMetadata("apiSchemas")
            for prop in prim.GetProperties():
                prop_name = prop.GetName()
                if prop_name.startswith("isaac:") or prop_name.startswith("physx") or prop_name.startswith("physics:"):
                    if "PhysicsCollisionAPI" in prim.GetAppliedSchemas() and prop_name in {"physics:collisionEnabled", "physics:simulationOwner"}:
                        continue
                    prim.RemoveProperty(prop_name)

        for path in [
            prim.GetPath() for prim in stage.TraverseAll()
            if prim.GetName() in {"visuals", "collisions"} and not prim.GetChildren()
        ]:
            stage.RemovePrim(path)

        rigid_body = UsdPhysics.RigidBodyAPI.Apply(base_link)
        rigid_body.CreateRigidBodyEnabledAttr(True)
        mass = UsdPhysics.MassAPI.Apply(base_link)
        mass.CreateMassAttr(0.072)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0106, 0.0175, 0.0125))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(0.003881243, 0.000498940, 0.003879257))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))

        flat_layer = stage.Flatten()
        flat_layer.documentation = ""
        flat_layer.comment = ""
        flat_layer.Export(str(output_path))

    print(output_path)
    omni.kit.app.get_app().post_quit()
```
