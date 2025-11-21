def allocate_brick_part(
    dimensions: tuple[int, int, int],
    color: tuple[int, int, int],
    env_id: int,
    rot: tuple[float, float, float, float],
    pos: tuple[float, float, float],
) -> str: ...

def deallocate_part(part_path: str) -> bool: ...

def create_connection(
    stud_path: str,
    stud_if: int,
    hole_path: str,
    hole_if: int,
    offset: tuple[int, int],
    yaw: int,
) -> str: ...

def deallocate_connection(connection_path: str) -> bool: ...

def deallocate_all_managed(env_id: int) -> bool: ...

def export_lego(env_id: int) -> str: ...

def import_lego(
    json_str: str,
    env_id: int,
    ref_rot: tuple[float, float, float, float],
    ref_pos: tuple[float, float, float],
) -> None: ...

def compute_connected_component(part_path: str) -> tuple[list[str], list[str]]: ...

class AssemblyThresholds:
    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...
    distance_tolerance: float
    max_penetration: float
    z_angle_tolerance: float
    required_force: float
    yaw_tolerance: float
    position_tolerance: float

def set_assembly_thresholds(thresholds: AssemblyThresholds) -> None: ...

def get_assembly_thresholds() -> AssemblyThresholds: ...
