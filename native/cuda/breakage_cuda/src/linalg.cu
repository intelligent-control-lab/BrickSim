// SPDX-License-Identifier: MIT
//
// Implementation of breakage_cuda/linalg.hpp:
//
//   * cuBLAS / cuSPARSE / cuSOLVER handle pool (Meyers singleton).
//   * Thin wrappers around level-1 BLAS we need from the ADMM solver.
//   * Generic-API SpMV.
//   * Element-wise box-clamp / fill kernels.
//   * Host-side 3x3 / 2x2 dense helpers (SVD-Procrustes, SPD LDLT solve,
//     SPD inverse-square-root) backed by Eigen.
//   * xyzw quaternion <-> rotation matrix helpers.
//
// All double-precision throughout: this is a verification / numerical-fidelity
// project, not a perf-shootout.

#include "breakage_cuda/linalg.hpp"
#include "breakage_cuda/device.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <cublas_v2.h>
#include <cuda_runtime.h>
#include <cusolverDn.h>
#include <cusolverSp.h>
#include <cusparse.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace breakage_cuda::cuda_la {

// ---------------------------------------------------------------------------
// Local error-check macro for cuBLAS (cuSPARSE/CUDA macros come from
// device.hpp).
// ---------------------------------------------------------------------------

#define BREAKAGE_CUBLAS_CHECK(call)                                            \
  do {                                                                         \
    cublasStatus_t _s = (call);                                                \
    if (_s != CUBLAS_STATUS_SUCCESS) {                                         \
      throw std::runtime_error(std::string("cuBLAS error at " __FILE__ ":") +  \
                               std::to_string(__LINE__) + " (status " +        \
                               std::to_string(static_cast<int>(_s)) + ")");    \
    }                                                                          \
  } while (0)

// ---------------------------------------------------------------------------
// Handles singleton.
// ---------------------------------------------------------------------------

Handles::Handles() {
  if (cublasCreate(&blas_) != CUBLAS_STATUS_SUCCESS) {
    throw std::runtime_error("cublasCreate failed");
  }
  if (cusparseCreate(&sparse_) != CUSPARSE_STATUS_SUCCESS) {
    cublasDestroy(blas_);
    blas_ = nullptr;
    throw std::runtime_error("cusparseCreate failed");
  }
  if (cusolverDnCreate(&solver_dn_) != CUSOLVER_STATUS_SUCCESS) {
    cusparseDestroy(sparse_);
    cublasDestroy(blas_);
    sparse_ = nullptr;
    blas_ = nullptr;
    throw std::runtime_error("cusolverDnCreate failed");
  }
  if (cusolverSpCreate(&solver_sp_) != CUSOLVER_STATUS_SUCCESS) {
    cusolverDnDestroy(solver_dn_);
    cusparseDestroy(sparse_);
    cublasDestroy(blas_);
    solver_dn_ = nullptr;
    sparse_ = nullptr;
    blas_ = nullptr;
    throw std::runtime_error("cusolverSpCreate failed");
  }
  // We always pass alpha/beta and consume reduction results through host
  // pointers, which is the default but be explicit so that an external caller
  // tinkering with the handles cannot break us.
  cublasSetPointerMode(blas_, CUBLAS_POINTER_MODE_HOST);
  cusparseSetPointerMode(sparse_, CUSPARSE_POINTER_MODE_HOST);
}

Handles::~Handles() {
  if (solver_sp_ != nullptr) {
    cusolverSpDestroy(solver_sp_);
  }
  if (solver_dn_ != nullptr) {
    cusolverDnDestroy(solver_dn_);
  }
  if (sparse_ != nullptr) {
    cusparseDestroy(sparse_);
  }
  if (blas_ != nullptr) {
    cublasDestroy(blas_);
  }
}

Handles &Handles::instance() {
  static Handles inst;
  return inst;
}

// ---------------------------------------------------------------------------
// cuBLAS-backed level-1 wrappers.
//
// dot/nrm2/inf_norm consume a host-side scalar so they internally synchronize
// the stream before returning. axpy/scal/copy/scal_copy are pure async kernel
// launches.
// ---------------------------------------------------------------------------

double dot(int n, const double *x_d, const double *y_d, cudaStream_t stream) {
  if (n <= 0) {
    return 0.0;
  }
  auto h = Handles::instance().blas();
  BREAKAGE_CUBLAS_CHECK(cublasSetStream(h, stream));
  double r = 0.0;
  BREAKAGE_CUBLAS_CHECK(cublasDdot(h, n, x_d, 1, y_d, 1, &r));
  // cuBLAS in HOST_POINTER mode is synchronous wrt host for scalar results,
  // but make it explicit so async kernel launches that follow are properly
  // ordered.
  BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));
  return r;
}

double nrm2(int n, const double *x_d, cudaStream_t stream) {
  if (n <= 0) {
    return 0.0;
  }
  auto h = Handles::instance().blas();
  BREAKAGE_CUBLAS_CHECK(cublasSetStream(h, stream));
  double r = 0.0;
  BREAKAGE_CUBLAS_CHECK(cublasDnrm2(h, n, x_d, 1, &r));
  BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));
  return r;
}

double inf_norm(int n, const double *x_d, cudaStream_t stream) {
  if (n <= 0) {
    return 0.0;
  }
  auto h = Handles::instance().blas();
  BREAKAGE_CUBLAS_CHECK(cublasSetStream(h, stream));
  int idx_1 = 0; // 1-based
  BREAKAGE_CUBLAS_CHECK(cublasIdamax(h, n, x_d, 1, &idx_1));
  if (idx_1 < 1 || idx_1 > n) {
    return 0.0;
  }
  double v = 0.0;
  BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(&v, x_d + (idx_1 - 1), sizeof(double),
                                      cudaMemcpyDeviceToHost, stream));
  BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));
  return std::abs(v);
}

void axpy(int n, double alpha, const double *x_d, double *y_d,
          cudaStream_t stream) {
  if (n <= 0) {
    return;
  }
  auto h = Handles::instance().blas();
  BREAKAGE_CUBLAS_CHECK(cublasSetStream(h, stream));
  BREAKAGE_CUBLAS_CHECK(cublasDaxpy(h, n, &alpha, x_d, 1, y_d, 1));
}

void scal(int n, double alpha, double *x_d, cudaStream_t stream) {
  if (n <= 0) {
    return;
  }
  auto h = Handles::instance().blas();
  BREAKAGE_CUBLAS_CHECK(cublasSetStream(h, stream));
  BREAKAGE_CUBLAS_CHECK(cublasDscal(h, n, &alpha, x_d, 1));
}

void copy(int n, const double *src_d, double *dst_d, cudaStream_t stream) {
  if (n <= 0) {
    return;
  }
  BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(dst_d, src_d,
                                      static_cast<std::size_t>(n) *
                                          sizeof(double),
                                      cudaMemcpyDeviceToDevice, stream));
}

void scal_copy(int n, double alpha, const double *x_d, double *y_d,
               cudaStream_t stream) {
  if (n <= 0) {
    return;
  }
  copy(n, x_d, y_d, stream);
  scal(n, alpha, y_d, stream);
}

// ---------------------------------------------------------------------------
// Element-wise kernels: fill / clamp.
// ---------------------------------------------------------------------------

namespace {

constexpr int kElemwiseBlock = 256;

inline int elemwise_grid(int n) {
  return (n + kElemwiseBlock - 1) / kElemwiseBlock;
}

__global__ void fill_kernel(int n, double v, double *x) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    x[i] = v;
  }
}

__global__ void clamp_kernel(int n, const double *in, const double *l,
                             const double *u, double *out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  double x = in[i];
  double lo = l[i];
  double hi = u[i];
  if (x < lo) {
    x = lo;
  }
  if (x > hi) {
    x = hi;
  }
  out[i] = x;
}

__global__ void clamp_inplace_kernel(int n, const double *l, const double *u,
                                     double *x_d) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  double x = x_d[i];
  double lo = l[i];
  double hi = u[i];
  if (x < lo) {
    x = lo;
  }
  if (x > hi) {
    x = hi;
  }
  x_d[i] = x;
}

} // namespace

void fill(int n, double value, double *x_d, cudaStream_t stream) {
  if (n <= 0) {
    return;
  }
  fill_kernel<<<elemwise_grid(n), kElemwiseBlock, 0, stream>>>(n, value, x_d);
}

void clamp(int n, const double *in_d, const double *l_d, const double *u_d,
           double *out_d, cudaStream_t stream) {
  if (n <= 0) {
    return;
  }
  clamp_kernel<<<elemwise_grid(n), kElemwiseBlock, 0, stream>>>(n, in_d, l_d,
                                                                u_d, out_d);
}

void clamp_inplace(int n, const double *l_d, const double *u_d, double *x_d,
                   cudaStream_t stream) {
  if (n <= 0) {
    return;
  }
  clamp_inplace_kernel<<<elemwise_grid(n), kElemwiseBlock, 0, stream>>>(n, l_d,
                                                                       u_d,
                                                                       x_d);
}

// ---------------------------------------------------------------------------
// SpMV via the cuSPARSE generic API.
//
// We lazily attach a cusparseSpMatDescr_t to each DeviceCsr on first use.
// system.cpp may also build it; we only do it here if it has not.
// ---------------------------------------------------------------------------

namespace {

void ensure_csr_descr(const DeviceCsr &A) {
  if (A.descr != nullptr) {
    return;
  }
  // descr is a member of DeviceCsr; the lazy-init cast preserves the
  // logical "const" of the SpMV operation (we do not touch the values).
  auto &mut = const_cast<DeviceCsr &>(A);
  BREAKAGE_CUSPARSE_CHECK(cusparseCreateCsr(
      &mut.descr, A.rows, A.cols, A.nnz,
      const_cast<int *>(A.rowptr.data()), const_cast<int *>(A.colind.data()),
      const_cast<double *>(A.values.data()), CUSPARSE_INDEX_32I,
      CUSPARSE_INDEX_32I, CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F));
}

} // namespace

std::size_t spmv_buffer_size(const DeviceCsr &A, bool transpose) {
  if (A.rows <= 0 || A.cols <= 0) {
    return 0;
  }
  ensure_csr_descr(A);
  auto h = Handles::instance().sparse();
  cusparseOperation_t op =
      transpose ? CUSPARSE_OPERATION_TRANSPOSE
                : CUSPARSE_OPERATION_NON_TRANSPOSE;
  int64_t x_n = transpose ? A.rows : A.cols;
  int64_t y_n = transpose ? A.cols : A.rows;
  cusparseDnVecDescr_t x_descr = nullptr, y_descr = nullptr;
  // bufferSize never dereferences the vector data; nullptr is the documented
  // "use placeholder" value.
  BREAKAGE_CUSPARSE_CHECK(
      cusparseCreateDnVec(&x_descr, x_n, nullptr, CUDA_R_64F));
  BREAKAGE_CUSPARSE_CHECK(
      cusparseCreateDnVec(&y_descr, y_n, nullptr, CUDA_R_64F));
  double alpha = 1.0;
  double beta = 0.0;
  std::size_t bytes = 0;
  BREAKAGE_CUSPARSE_CHECK(cusparseSpMV_bufferSize(
      h, op, &alpha, A.descr, x_descr, &beta, y_descr, CUDA_R_64F,
      CUSPARSE_SPMV_ALG_DEFAULT, &bytes));
  cusparseDestroyDnVec(x_descr);
  cusparseDestroyDnVec(y_descr);
  return bytes;
}

void spmv(const DeviceCsr &A, bool transpose, double alpha, const double *x_d,
          double beta, double *y_d, void *buffer_d, std::size_t buffer_bytes,
          cudaStream_t stream) {
  (void)buffer_bytes; // size validated by cuSPARSE itself
  if (A.rows <= 0 || A.cols <= 0) {
    return;
  }
  ensure_csr_descr(A);
  auto h = Handles::instance().sparse();
  BREAKAGE_CUSPARSE_CHECK(cusparseSetStream(h, stream));
  cusparseOperation_t op =
      transpose ? CUSPARSE_OPERATION_TRANSPOSE
                : CUSPARSE_OPERATION_NON_TRANSPOSE;
  int64_t x_n = transpose ? A.rows : A.cols;
  int64_t y_n = transpose ? A.cols : A.rows;
  cusparseDnVecDescr_t x_descr = nullptr, y_descr = nullptr;
  BREAKAGE_CUSPARSE_CHECK(cusparseCreateDnVec(
      &x_descr, x_n, const_cast<double *>(x_d), CUDA_R_64F));
  BREAKAGE_CUSPARSE_CHECK(cusparseCreateDnVec(&y_descr, y_n, y_d, CUDA_R_64F));
  BREAKAGE_CUSPARSE_CHECK(cusparseSpMV(h, op, &alpha, A.descr, x_descr, &beta,
                                       y_descr, CUDA_R_64F,
                                       CUSPARSE_SPMV_ALG_DEFAULT, buffer_d));
  cusparseDestroyDnVec(x_descr);
  cusparseDestroyDnVec(y_descr);
}

// ---------------------------------------------------------------------------
// Host-side dense 3x3 / 2x2 helpers.
//
// Inputs / outputs are row-major flat arrays; we marshal to Eigen, do the
// math, and marshal back. Allocations are stack-only.
// ---------------------------------------------------------------------------

namespace {

inline Eigen::Matrix3d row_major_to_eigen3(const double M[9]) {
  Eigen::Matrix3d out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out(i, j) = M[3 * i + j];
    }
  }
  return out;
}

inline void eigen_to_row_major3(const Eigen::Matrix3d &M, double out[9]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[3 * i + j] = M(i, j);
    }
  }
}

inline Eigen::Matrix2d row_major_to_eigen2(const double M[4]) {
  Eigen::Matrix2d out;
  out(0, 0) = M[0];
  out(0, 1) = M[1];
  out(1, 0) = M[2];
  out(1, 1) = M[3];
  return out;
}

inline void eigen_to_row_major2(const Eigen::Matrix2d &M, double out[4]) {
  out[0] = M(0, 0);
  out[1] = M(0, 1);
  out[2] = M(1, 0);
  out[3] = M(1, 1);
}

} // namespace

void svd_procrustes_3x3(const double S[9], double R[9]) {
  Eigen::Matrix3d S_mat = row_major_to_eigen3(S);
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      S_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
  if ((V * U.transpose()).determinant() < 0.0) {
    D(2, 2) = -1.0;
  }
  Eigen::Matrix3d R_mat = V * D * U.transpose();
  eigen_to_row_major3(R_mat, R);
}

void spd_solve_3x3(const double A[9], const double b[3], double x[3]) {
  Eigen::Matrix3d A_mat = row_major_to_eigen3(A);
  Eigen::Vector3d b_vec(b[0], b[1], b[2]);
  Eigen::Vector3d x_vec = A_mat.ldlt().solve(b_vec);
  x[0] = x_vec[0];
  x[1] = x_vec[1];
  x[2] = x_vec[2];
}

void inv_sqrt_spd_2x2(const double C[4], double W[4]) {
  Eigen::Matrix2d C_mat = row_major_to_eigen2(C);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(C_mat);
  if (eig.info() != Eigen::Success) {
    throw std::invalid_argument(
        "inv_sqrt_spd_2x2: eigen decomposition failed");
  }
  Eigen::Vector2d lambda = eig.eigenvalues();
  Eigen::Matrix2d U = eig.eigenvectors();
  double lambda_max = lambda.maxCoeff();
  if (!(lambda_max > 0.0)) {
    throw std::invalid_argument("inv_sqrt_spd_2x2: non-positive matrix");
  }
  // Match breakage.cppm: clamp to avoid blow-up on near-degenerate patches.
  double eps = 1e-12 * lambda_max;
  lambda = lambda.cwiseMax(eps);
  Eigen::Matrix2d D = Eigen::Matrix2d::Zero();
  D(0, 0) = 1.0 / std::sqrt(lambda(0));
  D(1, 1) = 1.0 / std::sqrt(lambda(1));
  Eigen::Matrix2d W_mat = U * D * U.transpose();
  eigen_to_row_major2(W_mat, W);
}

// ---------------------------------------------------------------------------
// Quaternion (xyzw) helpers.
//
// xyzw matches Eigen::Quaterniond::coeffs() ordering, so we can round-trip
// through Quaterniond without reordering.
// ---------------------------------------------------------------------------

void quat_xyzw_to_R(const double q[4], double R[9]) {
  Eigen::Quaterniond q_eig;
  q_eig.coeffs() << q[0], q[1], q[2], q[3];
  Eigen::Matrix3d M = q_eig.toRotationMatrix();
  eigen_to_row_major3(M, R);
}

void R_to_quat_xyzw(const double R[9], double q[4]) {
  Eigen::Matrix3d M = row_major_to_eigen3(R);
  Eigen::Quaterniond q_eig(M);
  q_eig.normalize();
  q[0] = q_eig.x();
  q[1] = q_eig.y();
  q[2] = q_eig.z();
  q[3] = q_eig.w();
}

void quat_xyzw_normalize(double q[4]) {
  double n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  if (!(n2 > 0.0)) {
    return;
  }
  double inv = 1.0 / std::sqrt(n2);
  q[0] *= inv;
  q[1] *= inv;
  q[2] *= inv;
  q[3] *= inv;
}

void quat_xyzw_mul(const double a[4], const double b[4], double r[4]) {
  // Hamilton product, matching Eigen Quaternion::operator*.
  double ax = a[0], ay = a[1], az = a[2], aw = a[3];
  double bx = b[0], by = b[1], bz = b[2], bw = b[3];
  r[0] = aw * bx + ax * bw + ay * bz - az * by;
  r[1] = aw * by - ax * bz + ay * bw + az * bx;
  r[2] = aw * bz + ax * by - ay * bx + az * bw;
  r[3] = aw * bw - ax * bx - ay * by - az * bz;
}

void quat_xyzw_conjugate(const double q[4], double r[4]) {
  r[0] = -q[0];
  r[1] = -q[1];
  r[2] = -q[2];
  r[3] = q[3];
}

#undef BREAKAGE_CUBLAS_CHECK

} // namespace breakage_cuda::cuda_la
