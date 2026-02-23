from bricksim._native import get_assembled_connections, get_disassembled_connections, ConnectionInfo

def get_lego_assembled_connections(env) -> list[ConnectionInfo]:
    current_step = env.common_step_counter
    if getattr(env, "_lego_assembled_connections_key", None) != current_step:
        env._lego_assembled_connections = get_assembled_connections(clear=True)
        env._lego_assembled_connections_key = current_step
    return env._lego_assembled_connections

def get_lego_disassembled_connections(env) -> list[ConnectionInfo]:
    current_step = env.common_step_counter
    if getattr(env, "_lego_disassembled_connections_key", None) != current_step:
        env._lego_disassembled_connections = get_disassembled_connections(clear=True)
        env._lego_disassembled_connections_key = current_step
    return env._lego_disassembled_connections
