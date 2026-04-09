from dataclasses import dataclass


@dataclass(frozen=True)
class AssembleBrickGoal:
    stud_if: int
    hole_if: int
    offset: tuple[int, int]
    yaw: int
    pos_tol: float
    rot_tol: float

