# Python loader for matrices serialized by lego_assemble.utils.matrix_serialization

import base64
import json
from typing import Any, Dict, Union

import numpy as np

try:
    import scipy.sparse as sp  # optional, only needed for sparse
except Exception:  # pragma: no cover
    sp = None


def load_matrix(obj: Union[str, Dict[str, Any]]):
    """
    Load a matrix serialized by lego_assemble::matrix_to_json(...) into:
      - np.ndarray for dense
      - scipy.sparse.csr_matrix / csc_matrix for sparse

    Accepts:
      - dict (already-parsed JSON)
      - str  (JSON text)

    Notes:
      - dtype strings like "<f8", "<i4", "<c16" are passed to numpy dtype directly.
      - Dense uses order "C" (row-major) or "F" (col-major).
      - Sparse requires SciPy installed.
    """
    j = json.loads(obj) if isinstance(obj, str) else obj
    kind = j.get("kind")
    if kind not in ("dense", "sparse"):
        raise ValueError(f"Unknown kind: {kind!r}")

    if kind == "dense":
        dtype = np.dtype(j["dtype"])
        order = j.get("order", "C")
        if order not in ("C", "F"):
            raise ValueError(f"Invalid dense order: {order!r}")
        shape = tuple(int(x) for x in j["shape"])
        raw = base64.b64decode(j["data_b64"])
        arr = np.frombuffer(raw, dtype=dtype)
        # reshape with explicit order; copy to detach from the temporary bytes object
        return arr.reshape(shape, order=order).copy()

    # sparse
    if sp is None:
        raise ImportError("scipy is required to load sparse matrices")

    fmt = j["format"]
    if fmt not in ("csr", "csc"):
        raise ValueError(f"Invalid sparse format: {fmt!r}")

    shape = tuple(int(x) for x in j["shape"])
    nnz = int(j["nnz"])
    data_dtype = np.dtype(j["data_dtype"])
    index_dtype = np.dtype(j["index_dtype"])

    data_raw = base64.b64decode(j["data_b64"])
    indices_raw = base64.b64decode(j["indices_b64"])
    indptr_raw = base64.b64decode(j["indptr_b64"])

    data = np.frombuffer(data_raw, dtype=data_dtype)
    indices = np.frombuffer(indices_raw, dtype=index_dtype)
    indptr = np.frombuffer(indptr_raw, dtype=index_dtype)

    # basic consistency checks
    if data.size != nnz or indices.size != nnz:
        raise ValueError(f"nnz mismatch: expected {nnz}, got data={data.size}, indices={indices.size}")
    expected_indptr = (shape[0] + 1) if fmt == "csr" else (shape[1] + 1)
    if indptr.size != expected_indptr:
        raise ValueError(f"indptr length mismatch: expected {expected_indptr}, got {indptr.size}")

    # ensure arrays own their data (SciPy may keep references)
    data = np.array(data, copy=True)
    indices = np.array(indices, copy=True)
    indptr = np.array(indptr, copy=True)

    mat_cls = sp.csr_matrix if fmt == "csr" else sp.csc_matrix
    return mat_cls((data, indices, indptr), shape=shape)
