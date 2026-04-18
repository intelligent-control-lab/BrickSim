"""Load native breakage debug dumps for Python inspection."""

import json
from pathlib import Path
from typing import NotRequired, TypeAlias, TypedDict

from .matrix_loader import LoadedMatrix, MatrixJson, load_matrix


class BreakageThresholdsDump(TypedDict):
    """Breakage thresholds written by ``BreakageThresholds::to_json``."""

    enabled: bool
    contact_regularization: float
    clutch_axial_compliance: float
    clutch_radial_compliance: float
    clutch_tangential_compliance: float
    friction_coefficient: float
    preloaded_force: float
    slack_fraction_warn: float
    slack_fraction_b_floor: float
    debug_dump: bool
    breakage_cooldown_time: float


class OsqpStepInfoDump(TypedDict):
    """One OSQP solve info block written by ``osqp_info_to_json``."""

    status: str
    status_val: int
    status_polish: int
    obj_val: float
    dual_obj_val: float
    prim_res: float
    dual_res: float
    duality_gap: float
    iter: int
    rho_updates: int
    rho_estimate: float
    setup_time: float
    solve_time: float
    update_time: float
    polish_time: float
    run_time: float
    primdual_int: float
    rel_kkt_error: float


class OsqpInfoDump(TypedDict):
    """Breakage solver info written by ``OsqpInfo::to_json``."""

    converged: bool
    prj_converged: bool
    prj_info: OsqpStepInfoDump
    rlx_converged: bool
    rlx_info: OsqpStepInfoDump
    opt_converged: bool
    opt_info: OsqpStepInfoDump


class OsqpStateJson(TypedDict):
    """Raw OSQP warm-start state written by ``OsqpState::to_json``."""

    has_state: bool
    prj_sol: MatrixJson
    prj_dual: MatrixJson
    prj_rho: float
    rlx_sol: MatrixJson
    rlx_dual: MatrixJson
    rlx_rho: float
    opt_sol: MatrixJson
    opt_dual: MatrixJson
    opt_rho: float
    slack: MatrixJson


class OsqpStateDump(TypedDict):
    """Decoded OSQP warm-start state written by ``OsqpState::to_json``."""

    has_state: bool
    prj_sol: LoadedMatrix
    prj_dual: LoadedMatrix
    prj_rho: float
    rlx_sol: LoadedMatrix
    rlx_dual: LoadedMatrix
    rlx_rho: float
    opt_sol: LoadedMatrix
    opt_dual: LoadedMatrix
    opt_rho: float
    slack: LoadedMatrix


class BreakageSystemJson(TypedDict):
    """Raw system block written by ``BreakageSystem::to_json``."""

    num_parts: int
    num_clutches: int
    num_contact_vertices: int
    num_vars: int
    num_eq: int
    num_ineq: int
    num_relaxed_ineq: int
    Q: MatrixJson
    A: MatrixJson
    G: MatrixJson
    H: MatrixJson
    V: MatrixJson
    part_ids: list[int]
    clutch_ids: list[int]
    total_mass: float
    mass: MatrixJson
    q_CC: MatrixJson
    c_CC: MatrixJson
    I_CC: MatrixJson
    L0: float
    capacity_clutch_indices: list[int]
    capacities: MatrixJson
    clutch_whiten: MatrixJson


class BreakageSystemDump(TypedDict):
    """Decoded system block written by ``BreakageSystem::to_json``."""

    num_parts: int
    num_clutches: int
    num_contact_vertices: int
    num_vars: int
    num_eq: int
    num_ineq: int
    num_relaxed_ineq: int
    Q: LoadedMatrix
    A: LoadedMatrix
    G: LoadedMatrix
    H: LoadedMatrix
    V: LoadedMatrix
    part_ids: list[int]
    clutch_ids: list[int]
    total_mass: float
    mass: LoadedMatrix
    q_CC: LoadedMatrix
    c_CC: LoadedMatrix
    I_CC: LoadedMatrix
    L0: float
    capacity_clutch_indices: list[int]
    capacities: LoadedMatrix
    clutch_whiten: LoadedMatrix


class BreakageInputJson(TypedDict):
    """Raw input block written by ``BreakageInput::to_json``."""

    w: MatrixJson
    v: MatrixJson
    q: MatrixJson
    c: MatrixJson
    dt: float
    J: MatrixJson
    H: MatrixJson


class BreakageInputDump(TypedDict):
    """Decoded input block written by ``BreakageInput::to_json``."""

    w: LoadedMatrix
    v: LoadedMatrix
    q: LoadedMatrix
    c: LoadedMatrix
    dt: float
    J: LoadedMatrix
    H: LoadedMatrix


class BreakageStateJson(TypedDict):
    """Raw state block written by ``BreakageState::to_json``."""

    q_W_CC_prev: MatrixJson
    v_W_prev: MatrixJson
    L_prev: MatrixJson
    solver_state: OsqpStateJson


class BreakageStateDump(TypedDict):
    """Decoded state block written by ``BreakageState::to_json``."""

    q_W_CC_prev: LoadedMatrix
    v_W_prev: LoadedMatrix
    L_prev: LoadedMatrix
    solver_state: OsqpStateDump


class BreakageSolutionJson(TypedDict):
    """Raw solution block written by ``BreakageSolution::to_json``."""

    x: MatrixJson
    utilization: MatrixJson
    info: OsqpInfoDump
    slack_fraction: float


class BreakageSolutionDump(TypedDict):
    """Decoded solution block written by ``BreakageSolution::to_json``."""

    x: LoadedMatrix
    utilization: LoadedMatrix
    info: OsqpInfoDump
    slack_fraction: float


class BreakageDebugJson(TypedDict):
    """Raw debug dump written by ``BreakageDebugDump::to_json``.

    Keep in sync with native/modules/bricksim/physx/breakage.cppm and
    native/modules/bricksim/physx/osqp.cppm.
    """

    thresholds: BreakageThresholdsDump
    system: BreakageSystemJson
    input: BreakageInputJson
    state: BreakageStateJson
    solution: BreakageSolutionJson
    b: MatrixJson
    prev_state: NotRequired[BreakageStateJson]


class BreakageDebugDump(TypedDict):
    """Decoded debug dump written by ``BreakageDebugDump::to_json``."""

    thresholds: BreakageThresholdsDump
    system: BreakageSystemDump
    input: BreakageInputDump
    state: BreakageStateDump
    solution: BreakageSolutionDump
    b: LoadedMatrix
    prev_state: NotRequired[BreakageStateDump]


BreakageDebugPath: TypeAlias = str | Path


def _decode_osqp_state(state: OsqpStateJson) -> OsqpStateDump:
    return {
        "has_state": state["has_state"],
        "prj_sol": load_matrix(state["prj_sol"]),
        "prj_dual": load_matrix(state["prj_dual"]),
        "prj_rho": state["prj_rho"],
        "rlx_sol": load_matrix(state["rlx_sol"]),
        "rlx_dual": load_matrix(state["rlx_dual"]),
        "rlx_rho": state["rlx_rho"],
        "opt_sol": load_matrix(state["opt_sol"]),
        "opt_dual": load_matrix(state["opt_dual"]),
        "opt_rho": state["opt_rho"],
        "slack": load_matrix(state["slack"]),
    }


def _decode_system(system: BreakageSystemJson) -> BreakageSystemDump:
    return {
        "num_parts": system["num_parts"],
        "num_clutches": system["num_clutches"],
        "num_contact_vertices": system["num_contact_vertices"],
        "num_vars": system["num_vars"],
        "num_eq": system["num_eq"],
        "num_ineq": system["num_ineq"],
        "num_relaxed_ineq": system["num_relaxed_ineq"],
        "Q": load_matrix(system["Q"]),
        "A": load_matrix(system["A"]),
        "G": load_matrix(system["G"]),
        "H": load_matrix(system["H"]),
        "V": load_matrix(system["V"]),
        "part_ids": system["part_ids"],
        "clutch_ids": system["clutch_ids"],
        "total_mass": system["total_mass"],
        "mass": load_matrix(system["mass"]),
        "q_CC": load_matrix(system["q_CC"]),
        "c_CC": load_matrix(system["c_CC"]),
        "I_CC": load_matrix(system["I_CC"]),
        "L0": system["L0"],
        "capacity_clutch_indices": system["capacity_clutch_indices"],
        "capacities": load_matrix(system["capacities"]),
        "clutch_whiten": load_matrix(system["clutch_whiten"]),
    }


def _decode_input(input: BreakageInputJson) -> BreakageInputDump:
    return {
        "w": load_matrix(input["w"]),
        "v": load_matrix(input["v"]),
        "q": load_matrix(input["q"]),
        "c": load_matrix(input["c"]),
        "dt": input["dt"],
        "J": load_matrix(input["J"]),
        "H": load_matrix(input["H"]),
    }


def _decode_state(state: BreakageStateJson) -> BreakageStateDump:
    return {
        "q_W_CC_prev": load_matrix(state["q_W_CC_prev"]),
        "v_W_prev": load_matrix(state["v_W_prev"]),
        "L_prev": load_matrix(state["L_prev"]),
        "solver_state": _decode_osqp_state(state["solver_state"]),
    }


def _decode_solution(solution: BreakageSolutionJson) -> BreakageSolutionDump:
    return {
        "x": load_matrix(solution["x"]),
        "utilization": load_matrix(solution["utilization"]),
        "info": solution["info"],
        "slack_fraction": solution["slack_fraction"],
    }


def _decode_dump(dump: BreakageDebugJson) -> BreakageDebugDump:
    decoded: BreakageDebugDump = {
        "thresholds": dump["thresholds"],
        "system": _decode_system(dump["system"]),
        "input": _decode_input(dump["input"]),
        "state": _decode_state(dump["state"]),
        "solution": _decode_solution(dump["solution"]),
        "b": load_matrix(dump["b"]),
    }
    if "prev_state" in dump:
        decoded["prev_state"] = _decode_state(dump["prev_state"])
    return decoded


def load_breakage_debug_dump(path: BreakageDebugPath) -> BreakageDebugDump:
    """Load a BreakageChecker debug JSON dump and decode embedded matrices.

    The native code writes dense and sparse matrices using
    ``bricksim::matrix_to_json(...)``. This loader converts those entries into:
      - ``numpy.ndarray`` for dense matrices
      - ``scipy.sparse.csr_matrix`` / ``scipy.sparse.csc_matrix`` for sparse matrices

    Args:
        path: Path to a file like ``_local/breakage_debug_YYYYMMDD_HHMMSS.json``.

    Returns:
        Typed dump mirroring the native debug structure, with matrices decoded.

    Example:
        ```python
        from bricksim.utils.breakage_debug import load_breakage_debug_dump

        dbg = load_breakage_debug_dump("_local/breakage_debug_20251230_123540.json")
        A = dbg["system"]["A"]          # scipy.sparse.csc_matrix
        q = dbg["input"]["q"]           # np.ndarray, shape (num_parts, 4)
        converged = dbg["solution"]["info"]["converged"]
        ```

    Raises:
        ImportError: if the dump contains sparse matrices but SciPy is unavailable.
        KeyError: if the dump is missing keys required by the native dump schema.
    """
    p = Path(path)
    dump: BreakageDebugJson = json.loads(p.read_text())
    return _decode_dump(dump)
