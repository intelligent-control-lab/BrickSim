import json
from pathlib import Path
from typing import Any, Union

from .matrix_loader import load_matrix


def _decode_matrices(obj: Any) -> Any:
    if isinstance(obj, dict):
        kind = obj.get("kind")
        if kind in ("dense", "sparse"):
            return load_matrix(obj)
        return {k: _decode_matrices(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_decode_matrices(v) for v in obj]
    return obj


def load_breakage_debug_dump(path: Union[str, Path]) -> dict[str, Any]:
    """Load a BreakageChecker debug JSON dump and decode embedded matrices.

    The native code writes dense and sparse matrices using
    `bricksim::matrix_to_json(...)`. This loader converts those entries into:
      - `numpy.ndarray` for dense matrices
      - `scipy.sparse.csr_matrix` / `scipy.sparse.csc_matrix` for sparse matrices

    Args:
        path: Path to a file like `_local/breakage_debug_YYYYMMDD_HHMMSS.json`.

    Returns:
        A dict mirroring the JSON structure, with matrices decoded.

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
        KeyError: if the dump is missing required top-level keys.
    """
    p = Path(path)
    j = json.loads(p.read_text())

    # Fail fast: this is a schema'd debug dump we control.
    for k in ("thresholds", "system", "input", "state", "solution"):
        if k not in j:
            raise KeyError(f"Missing top-level key {k!r} in {p}")

    return _decode_matrices(j)
