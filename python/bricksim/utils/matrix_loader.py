"""Load matrices serialized by BrickSim matrix JSON helpers."""

import base64
import json
from typing import Literal, TypeAlias, TypedDict

import numpy as np
import scipy.sparse as sp


class DenseMatrixJson(TypedDict):
    """Dense matrix JSON emitted by ``bricksim::matrix_to_json``.

    Keep in sync with native/modules/bricksim/utils/matrix_serialization.cppm.
    """

    kind: Literal["dense"]
    dtype: str
    order: Literal["C", "F"]
    shape: list[int]
    data_b64: str


class SparseMatrixJson(TypedDict):
    """Sparse matrix JSON emitted by ``bricksim::matrix_to_json``.

    Keep in sync with native/modules/bricksim/utils/matrix_serialization.cppm.
    """

    kind: Literal["sparse"]
    format: Literal["csr", "csc"]
    shape: list[int]
    nnz: int
    data_dtype: str
    index_dtype: str
    data_b64: str
    indices_b64: str
    indptr_b64: str


MatrixJson: TypeAlias = DenseMatrixJson | SparseMatrixJson
LoadedMatrix: TypeAlias = np.ndarray | sp.csr_matrix | sp.csc_matrix


def _shape_2d(shape: list[int]) -> tuple[int, int]:
    if len(shape) != 2:
        raise ValueError(f"shape must be [rows, cols], got {shape!r}")
    return int(shape[0]), int(shape[1])


def load_matrix(obj: str | MatrixJson) -> LoadedMatrix:
    """Load a matrix serialized by bricksim::matrix_to_json(...).

    Loads into:
      - np.ndarray for dense
      - scipy.sparse.csr_matrix / csc_matrix for sparse.

    Accepts:
      - dict (already-parsed JSON)
      - str  (JSON text)

    Notes:
      - dtype strings like "<f8", "<i4", "<c16" are passed to numpy dtype directly.
      - Dense uses order "C" (row-major) or "F" (col-major).
      - Sparse requires SciPy installed.

    Returns:
        ``numpy.ndarray`` for dense matrices, or a SciPy sparse matrix for
        sparse matrices.
    """
    j: MatrixJson = json.loads(obj) if isinstance(obj, str) else obj

    if j["kind"] == "dense":
        dtype = np.dtype(j["dtype"])
        order = j["order"]
        shape = _shape_2d(j["shape"])
        raw = base64.b64decode(j["data_b64"])
        arr = np.frombuffer(raw, dtype=dtype)
        # reshape with explicit order; copy to detach from the temporary bytes object
        return arr.reshape(shape, order=order).copy()

    fmt = j["format"]
    shape = _shape_2d(j["shape"])
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
        message = (
            f"nnz mismatch: expected {nnz}, got data={data.size}, "
            f"indices={indices.size}"
        )
        raise ValueError(message)
    expected_indptr = (shape[0] + 1) if fmt == "csr" else (shape[1] + 1)
    if indptr.size != expected_indptr:
        raise ValueError(
            f"indptr length mismatch: expected {expected_indptr}, got {indptr.size}"
        )

    # ensure arrays own their data (SciPy may keep references)
    data = np.array(data, copy=True)
    indices = np.array(indices, copy=True)
    indptr = np.array(indptr, copy=True)

    mat_cls = sp.csr_matrix if fmt == "csr" else sp.csc_matrix
    return mat_cls((data, indices, indptr), shape=shape)
