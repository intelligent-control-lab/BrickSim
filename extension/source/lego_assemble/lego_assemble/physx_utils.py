import carb.settings
import omni.physx

def refresh_physx_simulation():
    physx = omni.physx.get_physx_interface()
    if physx.is_running():
        if carb.settings.get_settings().get_as_bool("physics/fabricEnabled"):
            __import__("omni.physxfabric").physxfabric.get_physx_fabric_interface().save_to_usd()
        physx.update_transformations(True, True, True, True)
        physx.release_physics_objects()
        physx.force_load_physics_from_usd()
