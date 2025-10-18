from typing import Optional

def path_for_brick(brick_id: int, env_id: Optional[int] = None) -> str:
    if env_id is None:
        return f"/World/Brick_{brick_id}"
    else:
        return f"/World/envs/env_{env_id}/Brick_{brick_id}"
