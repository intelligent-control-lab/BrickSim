import math
from dataclasses import dataclass

DistanceTolerance = 0.001           # Maximum distance between bricks (m)
MaxPenetration = 0.005              # Maximum penetration between bricks (m), penetration can happen due to simulation inaccuracies
ZAngleTolerance = math.radians(5)   # Maximum angle between z-axis of bricks (rad)
RequiredForce = 1.0                 # Minimum clutch power (N)
YawTolerance = math.radians(5)      # Maximum yaw error (rad)
PositionTolerance = 0.002           # Maximum position error (m)

@dataclass
class AssemblyEvent:
    brick0: str
    brick1: str
    joint: str
    p0: list[int]
    p1: list[int]
    yaw: float
