// SPDX-License-Identifier: MIT
//
// Shared CUDA linear-algebra helpers used by both kernels.cu and
// qp_solver.cu. The implementations live in src/linalg.cu.

#pragma once

#include "breakage_cuda/device.hpp"

#include <cublas_v2.h>
#include <cusolverDn.h>
#include <cusolverSp.h>
#include <cusparse.h>

namespace breakage_cuda::cuda_la {

// ---------------------------------------------------------------------------
// cuBLAS / cuSPARSE / cuSOLVER handle pool. One per process.
// ---------------------------------------------------------------------------

class Handles {
public:
  static Handles &instance();

  cublasHandle_t blas() const { return blas_; }
  cusparseHandle_t sparse() const { return sparse_; }
  cusolverDnHandle_t solver_dn() const { return solver_dn_; }
  cusolverSpHandle_t solver_sp() const { return solver_sp_; }

private:
  Handles();
  ~Handles();
  Handles(const Handles &) = delete;
  Handles &operator=(const Handles &) = delete;

  cublasHandle_t blas_{nullptr};
  cusparseHandle_t sparse_{nullptr};
  cusolverDnHandle_t solver_dn_{nullptr};
  cusolverSpHandle_t solver_sp_{nullptr};
};

// ---------------------------------------------------------------------------
// Reductions (single-block, used for small per-step inner products).
// ---------------------------------------------------------------------------

// Returns sum(x .* y). Uses cuBLAS dot (handle from Handles).
double dot(int n, const double *x_d, const double *y_d, cudaStream_t stream = 0);

// Returns ||x||_2.
double nrm2(int n, const double *x_d, cudaStream_t stream = 0);

// Returns ||x||_inf.
double inf_norm(int n, const double *x_d, cudaStream_t stream = 0);

// y <- alpha * x + y
void axpy(int n, double alpha, const double *x_d, double *y_d,
          cudaStream_t stream = 0);

// y <- alpha * x
void scal_copy(int n, double alpha, const double *x_d, double *y_d,
               cudaStream_t stream = 0);

// x <- alpha * x
void scal(int n, double alpha, double *x_d, cudaStream_t stream = 0);

// x <- value (broadcast).
void fill(int n, double value, double *x_d, cudaStream_t stream = 0);

// dst <- src (n elements)
void copy(int n, const double *src_d, double *dst_d, cudaStream_t stream = 0);

// ---------------------------------------------------------------------------
// SpMV: y <- alpha * op(A) * x + beta * y, with cuSPARSE generic API.
//
// `transpose` selects op(A): false -> A, true -> A^T.
// Caller pre-allocates buffer_d (size buffer_bytes_for_spmv) once.
// ---------------------------------------------------------------------------

std::size_t spmv_buffer_size(const DeviceCsr &A, bool transpose);

void spmv(const DeviceCsr &A, bool transpose, double alpha, const double *x_d,
          double beta, double *y_d, void *buffer_d, std::size_t buffer_bytes,
          cudaStream_t stream = 0);

// ---------------------------------------------------------------------------
// Element-wise box projection: out[i] = clamp(in[i], l[i], u[i]).
// ---------------------------------------------------------------------------

void clamp(int n, const double *in_d, const double *l_d, const double *u_d,
           double *out_d, cudaStream_t stream = 0);

// in-place: x[i] = clamp(x[i], l[i], u[i])
void clamp_inplace(int n, const double *l_d, const double *u_d, double *x_d,
                   cudaStream_t stream = 0);

// ---------------------------------------------------------------------------
// Dense 3x3 helpers exposed for tests.
//
// Each one takes a 3x3 row-major matrix as 9 doubles. Implemented on host.
// ---------------------------------------------------------------------------

// Computes R, t from the standard SVD-based Procrustes formula. Inputs are
// the centered matrices and the K matrix (cf. breakage.cppm fit_se3).
void svd_procrustes_3x3(const double S[9], double R[9]);

// 3x3 SPD LDLT solve: A * x = b, where A is SPD 3x3.
void spd_solve_3x3(const double A[9], const double b[3], double x[3]);

// 2x2 SPD inverse-sqrt (for Wk = inv_sqrt_spd(Ck)). Both row-major (4 doubles).
void inv_sqrt_spd_2x2(const double C[4], double W[4]);

// ---------------------------------------------------------------------------
// Quaternion <-> rotation matrix helpers (xyzw layout, row-major matrix).
// ---------------------------------------------------------------------------

void quat_xyzw_to_R(const double q[4], double R[9]);
void R_to_quat_xyzw(const double R[9], double q[4]);
void quat_xyzw_normalize(double q[4]);
// Returns r such that q_a * q_b = r (Hamilton product, xyzw).
void quat_xyzw_mul(const double a[4], const double b[4], double r[4]);
// Returns the conjugate (-x, -y, -z, w).
void quat_xyzw_conjugate(const double q[4], double r[4]);

} // namespace breakage_cuda::cuda_la
