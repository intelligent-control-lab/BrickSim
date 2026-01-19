from typing import Any, Optional

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

def compute_connection_local_transform(
    stud_path: str,
    stud_if: int,
    hole_path: str,
    hole_if: int,
    offset: tuple[int, int],
    yaw: int,
) -> tuple[
    tuple[tuple[float, float, float, float], tuple[float, float, float]],
    tuple[tuple[float, float, float, float], tuple[float, float, float]],
]: ...

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

def export_lego(env_id: int) -> Any: ...

def import_lego(
    json: Any,
    env_id: int,
    ref_rot: Optional[tuple[float, float, float, float]] = None,
    ref_pos: Optional[tuple[float, float, float]] = None,
) -> tuple[dict[int, str], dict[int, str]]: ...

def compute_structure_transforms(
    json: Any,
    root: int,
) -> dict[int, tuple[tuple[float, float, float, float], tuple[float, float, float]]]: ...

def compute_connected_component(part_path: str) -> tuple[list[str], list[str]]: ...

def are_parts_connected(
    part_a_path: str,
    part_b_path: str,
) -> bool: ...

def does_connection_exist(
    stud_path: str,
    stud_if: int,
    hole_path: str,
    hole_if: int,
) -> bool: ...

def get_connection_utilization(connection_path: str) -> float: ...

def arrange_parts_on_table(
    parts_to_arrange: list[str],
    parts_to_avoid: Optional[list[str]],
    obstacles: Optional[list[tuple[float, float, float, float]]],
    table_xy: tuple[float, float, float, float],
    table_z: float,
    clearance_xy: Optional[float],
    grid_resolution: Optional[float],
    allow_rotation: Optional[bool],
    avoid_all_other_parts: Optional[bool],
    structure_ids: Optional[list[int]],
) -> tuple[list[str], list[str]]: ...

def compute_obstacle_regions(
    obstacle_paths: list[str],
    table_xy: tuple[float, float, float, float],
    table_z: float,
    clearance_height: float,
) -> list[tuple[float, float, float, float]]: ...

def arrange_parts_in_workspace(
    workspace_path: str,
    parts_to_arrange: list[str],
    structure_ids: Optional[list[int]],
) -> tuple[list[str], list[str]]: ...

class AssemblyThresholds:
    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...
    enabled: bool
    distance_tolerance: float
    max_penetration: float
    z_angle_tolerance: float
    required_force: float
    yaw_tolerance: float
    position_tolerance: float

def set_assembly_thresholds(thresholds: AssemblyThresholds) -> None: ...

def get_assembly_thresholds() -> AssemblyThresholds: ...

class BreakageThresholds:
    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...
    enabled: bool
    contact_normal_compliance: float
    clutch_normal_compliance: float
    clutch_shear_compliance: float
    friction_coefficient: float
    preloaded_force: float
    slack_fraction_warn: float
    slack_fraction_b_floor: float

def set_breakage_thresholds(thresholds: BreakageThresholds) -> None: ...

def get_breakage_thresholds() -> BreakageThresholds: ...

def enable_breakage_debug_dump(enable: bool) -> None: ...

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
