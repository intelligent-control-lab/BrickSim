import omni.usd
from pxr import Gf, Sdf, Usd
from omni.physx.bindings._physx import ContactEventHeaderVector, ContactDataVector, ContactEventHeader, ContactEventType
from omni.physx.scripts.physicsUtils import PhysicsSchemaTools

def contact_report_event_handler(contact_headers: ContactEventHeaderVector, contact_data: ContactDataVector):
    stage: Usd.Stage = omni.usd.get_context().get_stage()
    if stage is None:
        return

    contact: ContactEventHeader
    for contact in contact_headers:
        actor0: Sdf.Path = PhysicsSchemaTools.intToSdfPath(contact.actor0)
        actor1: Sdf.Path = PhysicsSchemaTools.intToSdfPath(contact.actor1)
        prim0 = stage.GetPrimAtPath(actor0.GetPrimPath())
        prim1 = stage.GetPrimAtPath(actor1.GetPrimPath())
        if prim0 is None or prim1 is None:
            continue
        dim_attr0 = prim0.GetAttribute("lego_dimensions")
        dim_attr1 = prim1.GetAttribute("lego_dimensions")
        if not dim_attr0 or not dim_attr1:
            continue
        dim0: Gf.Vec3i = dim_attr0.Get()
        dim1: Gf.Vec3i = dim_attr1.Get()

        print(f"[lego_assemble] Contact event {contact.type.name} between {prim0.GetPath()} ({dim0[0]}x{dim0[1]}x{dim0[2]}) and {prim1.GetPath()} ({dim1[0]}x{dim1[1]}x{dim1[2]})")
