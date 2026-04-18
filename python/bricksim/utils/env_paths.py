"""USD path conventions for BrickSim environments."""


def get_env_path(env_id: int) -> str:
    """Return the USD root path for a BrickSim environment id.

    Returns:
        ``/World`` for the default env, otherwise the Isaac Lab env path.
    """
    return "/World" if env_id == -1 else f"/World/envs/env_{env_id}"
