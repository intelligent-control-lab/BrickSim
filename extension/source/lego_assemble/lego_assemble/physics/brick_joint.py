import carb
import omni.usd
from carb.events import IEvent
from pxr import Sdf, Usd, UsdPhysics
from omni.physx.bindings._physx import SimulationEvent
import lego_assemble._native as _native

INV_MASS_SCALE0=0.2
INV_INERTIA_SCALE0=0.2
INV_MASS_SCALE1=1.0
INV_INERTIA_SCALE1=1.0

def configure_brick_joint(joint: UsdPhysics.FixedJoint):
    prim: Usd.Prim = joint.GetPrim()
    prim.CreateAttribute("lego_conn", Sdf.ValueTypeNames.Bool).Set(True)

    joint_path = joint.GetPath().pathString
    if not _native.set_physx_joint_inv_mass_inertia(joint_path, INV_MASS_SCALE0, INV_INERTIA_SCALE0, INV_MASS_SCALE1, INV_INERTIA_SCALE1):
        carb.log_error(f"Failed to set joint mass/inertia scales for: {joint_path}")

def on_simulation_event(event: IEvent):
    if event.type == SimulationEvent.RESUMED.value:
        stage: Usd.Stage = omni.usd.get_context().get_stage()
        if stage is None:
            carb.log_error("Stage is none on simulation resumed")
            return
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.FixedJoint) and prim.GetAttribute("lego_conn").Get() is True:
                _native.set_physx_joint_inv_mass_inertia(prim.GetPath().pathString, INV_MASS_SCALE0, INV_INERTIA_SCALE0, INV_MASS_SCALE1, INV_INERTIA_SCALE1)
