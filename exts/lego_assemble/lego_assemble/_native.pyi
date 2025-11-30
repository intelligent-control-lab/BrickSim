from typing import Optional

def allocate_brick_part(
    dimensions: tuple[int, int, int],
    color: tuple[int, int, int],
    env_id: int,
    rot: Optional[tuple[float, float, float, float]] = None,
    pos: Optional[tuple[float, float, float]] = None,
) -> str: ...

def deallocate_part(part_path: str) -> bool: ...

def compute_graph_transform(
    a_path: str,
    b_path: str,
) -> tuple[tuple[float, float, float, float], tuple[float, float, float]]: ...

def compute_connection_transform(
    stud_path: str,
    stud_if: int,
    hole_path: str,
    hole_if: int,
    offset: tuple[int, int],
    yaw: int,
) -> tuple[tuple[float, float, float, float], tuple[float, float, float]]: ...

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
    ref_rot: Optional[tuple[float, float, float, float]] = None,
    ref_pos: Optional[tuple[float, float, float]] = None,
) -> None: ...

def compute_connected_component(part_path: str) -> tuple[list[str], list[str]]: ...

def arrange_bricks_on_table(
    parts_to_arrange: list[str],
    parts_to_avoid: list[str],
    obstacles: Optional[list[tuple[float, float, float, float]]],
    table_xy: tuple[float, float, float, float],
    table_z: float,
    clearance_xy: Optional[float],
    grid_resolution: Optional[float],
    allow_rotation: Optional[bool],
) -> tuple[list[str], list[str]]: ...

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

class AssemblyDebugInfo:
    def __repr__(self) -> str: ...
    accepted: bool
    relative_distance: float
    tilt: float
    projected_force: float
    yaw_error: float
    position_error: float
    grid_pos: tuple[float, float]
    grid_pos_snapped: tuple[int, int]
    stud_path: str
    stud_interface: int
    hole_path: str
    hole_interface: int

def get_assembly_debug_infos() -> list[AssemblyDebugInfo]: ...
