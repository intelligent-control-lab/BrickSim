def enqueue_joint_inv_mass_inertia(
    sdf_path: str,
    inv_mass0: float = -1.0,
    inv_inertia0: float = -1.0,
    inv_mass1: float = -1.0,
    inv_inertia1: float = -1.0,
) -> bool: ...

def create_stage_update_listener() -> bool: ...
def destroy_stage_update_listener() -> bool: ...
