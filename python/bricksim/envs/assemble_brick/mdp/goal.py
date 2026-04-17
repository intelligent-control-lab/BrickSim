"""Goal specification for the assemble-brick task."""

from dataclasses import dataclass


@dataclass(frozen=True)
class AssembleBrickGoal:
    """Target stud/hole connection and pose tolerances for assembly."""

    stud_if: int
    hole_if: int
    offset: tuple[int, int]
    yaw: int
    pos_tol: float
    rot_tol: float
