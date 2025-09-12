def set_physx_joint_inv_mass_inertia(
    sdf_path: str,
    inv_mass0: float = 1.0,
    inv_inertia0: float = 1.0,
    inv_mass1: float = 1.0,
    inv_inertia1: float = 1.0,
) -> bool: ...
def set_default_lego_joint_inv_mass_inertia(
    inv_mass0: float = 0.2,
    inv_inertia0: float = 0.2,
    inv_mass1: float = 1.0,
    inv_inertia1: float = 1.0,
) -> bool: ...

def init_natives() -> bool: ...
def deinit_natives() -> bool: ...
