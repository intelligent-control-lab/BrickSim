import math
from typing import Optional

class Thresholds:
    DistanceTolerance = 0.001           # Maximum distance between bricks (m)
    MaxPenetration = 0.005              # Maximum penetration between bricks (m), penetration can happen due to simulation inaccuracies
    ZAngleTolerance = math.radians(5)   # Maximum angle between z-axis of bricks (rad)
    RequiredForce = 1.0                 # Minimum clutch power (N)
    YawTolerance = math.radians(5)      # Maximum yaw error (rad)
    PositionTolerance = 0.002           # Maximum position error (m)

def path_for_brick(brick_id: int, env_id: Optional[int] = None) -> str:
    if env_id is None:
        return f"/World/Brick_{brick_id}"
    else:
        return f"/World/envs/env_{env_id}/Brick_{brick_id}"

def parse_brick_path(path: str) -> tuple[int, Optional[int]]:
    if path.startswith("/World/Brick_"):
        brick_id = int(path[len("/World/Brick_"):])
        env_id = None
    elif path.startswith("/World/envs/env_"):
        env_id = int(path[len("/World/envs/env_"):path.index("/", len("/World/envs/env_"))])
        brick_id = int(path[path.index("Brick_")+len("Brick_"):])
    else:
        raise ValueError(f"Invalid brick path: {path}")
    return brick_id, env_id

def path_for_conn(brick0: int, brick1: int, env_id: Optional[int] = None) -> str:
    if env_id is None:
        return f"/World/Conn_{brick0}_{brick1}"
    else:
        return f"/World/envs/env_{env_id}/Conn_{brick0}_{brick1}"

def parse_conn_path(path: str) -> tuple[int, int, Optional[int]]:
    if path.startswith("/World/Conn_"):
        env_id = None
        brick0, brick1 = map(int, path[len("/World/Conn_"):].split("_"))
    elif path.startswith("/World/envs/env_"):
        env_id = int(path[len("/World/envs/env_"):path.index("/", len("/World/envs/env_"))])
        brick0, brick1 = map(int, path[path.index("Conn_")+len("Conn_"):].split("_"))
    else:
        raise ValueError(f"Invalid connection path: {path}")
    return brick0, brick1, env_id
