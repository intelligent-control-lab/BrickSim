// SPDX-License-Identifier: MIT
//
// Hand-written CUDA ADMM implementation of the three-stage QP wrapper that
// lives on the CPU side in modules/bricksim/physx/osqp.cppm.
//
// Algorithmic summary (per stage; standard OSQP form
// `min 0.5 x^T P x + q^T x  s.t. l <= C x <= u`):
//
//   each ADMM iteration t:
//     1. rhs = sigma * x_k - q + C^T (rho * z_k - y_k)
//     2. solve  M xt = rhs           with M = P + sigma I + rho C^T C
//     3. zt = C xt                   (recovered from the reduced KKT)
//     4. xt_alpha = alpha * xt + (1 - alpha) * x_k
//        zt_alpha = alpha * zt + (1 - alpha) * z_k
//     5. z_{k+1}  = clamp(zt_alpha + y_k / rho, l, u)
//     6. y_{k+1} += rho * (zt_alpha - z_{k+1})
//     7. x_{k+1}  = xt_alpha
//
// The reduced KKT matrix M is symmetric positive definite (since
// P + sigma I > 0 and C^T C >= 0). We solve the inner system with a plain
// (un-preconditioned) conjugate-gradient method, applying M implicitly as
// `M x = P x + sigma x + rho C^T (C x)` via three SpMVs per CG step. This
// trades raw factorization speed for far simpler code: no symbolic Cholesky
// caching, no refactor on rho changes, no cuSolverSp roundtrips. The CG
// initial guess is warm-started from the previous outer iteration's xt so
// in steady state the inner loop converges in a handful of iterations.
//
// We deliberately do NOT replicate every OSQP feature - no polish, no
// infeasibility detection, no Ruiz scaling. The goal is to be close enough
// to OSQP that the breakage system behaves consistently.

// Eigen headers must come before the project headers because system.hpp uses
// Eigen::Quaterniond in HostState but only includes <Eigen/Dense> +
// <Eigen/SparseCore>. Pull the missing pieces in here so we don't have to
// touch system.hpp.
#include <Eigen/Geometry>
#include <Eigen/Sparse>

#include "breakage_cuda/qp_solver.hpp"

#include "breakage_cuda/device.hpp"
#include "breakage_cuda/kernels.hpp"
#include "breakage_cuda/linalg.hpp"
#include "breakage_cuda/system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace breakage_cuda::cuda_qp {

// ---------------------------------------------------------------------------
// File-local constants and device kernels
// ---------------------------------------------------------------------------

namespace {

// Stage 2 / Stage 3 inequality slack used by osqp.cppm. Hard-coded to match
// the production default; the OSQP wrapper would override this from
// OSQP_EPSILON env var, which we do not bother exposing here.
constexpr double kEpsilon = 1e-4;

// Diagonal regularisation on the x-block of P_rlx (matches osqp.cppm).
constexpr double kRelXReg = 1e-4;

// CG inner-solve tolerances. Inner tol is relative to ||rhs||; we keep it
// noticeably tighter than the outer ADMM tol so the inner solve never
// bottlenecks convergence.
constexpr double kInnerCgTol = 1e-9;
constexpr int kInnerCgMaxIter = 400;

// Bounds on the adaptive rho. OSQP uses similar limits.
constexpr double kRhoMin = 1e-6;
constexpr double kRhoMax = 1e6;

constexpr int kBlockSize = 256;

inline int grid_for(int n) {
  return (n + kBlockSize - 1) / kBlockSize;
}

// out[i] = a * x[i] + b * y[i]
__global__ void axby_kernel(int n, double a, const double *__restrict__ x,
                            double b, const double *__restrict__ y,
                            double *__restrict__ out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    out[i] = a * x[i] + b * y[i];
  }
}

// y[i] = a * x[i] + b * y[i]   (in-place version of axby_kernel)
__global__ void axpby_inplace_kernel(int n, double a,
                                     const double *__restrict__ x, double b,
                                     double *__restrict__ y) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    y[i] = a * x[i] + b * y[i];
  }
}

// rhs[i] = sigma * x_k[i] - q[i] + ct_term[i]
__global__ void rhs_assemble_kernel(int n, double sigma,
                                    const double *__restrict__ x_k,
                                    const double *__restrict__ q,
                                    const double *__restrict__ ct_term,
                                    double *__restrict__ rhs) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    rhs[i] = sigma * x_k[i] - q[i] + ct_term[i];
  }
}

// out[i] = rho * z_k[i] - y_k[i]
__global__ void rho_z_minus_y_kernel(int m, double rho,
                                     const double *__restrict__ z_k,
                                     const double *__restrict__ y_k,
                                     double *__restrict__ out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < m) {
    out[i] = rho * z_k[i] - y_k[i];
  }
}

// z_out[i] = clamp(zt_alpha[i] + y_k[i] / rho, l[i], u[i])
__global__ void z_update_kernel(int m, double inv_rho,
                                const double *__restrict__ zt_alpha,
                                const double *__restrict__ y_k,
                                const double *__restrict__ l,
                                const double *__restrict__ u,
                                double *__restrict__ z_out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < m) {
    double v = zt_alpha[i] + inv_rho * y_k[i];
    if (v < l[i])
      v = l[i];
    if (v > u[i])
      v = u[i];
    z_out[i] = v;
  }
}

// y[i] += rho * (zt_alpha[i] - z_new[i])
__global__ void y_update_kernel(int m, double rho,
                                const double *__restrict__ zt_alpha,
                                const double *__restrict__ z_new,
                                double *__restrict__ y) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < m) {
    y[i] += rho * (zt_alpha[i] - z_new[i]);
  }
}

// out[i] = max(0, x[i])
__global__ void relu_kernel(int n, const double *__restrict__ x,
                            double *__restrict__ out) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    out[i] = (x[i] > 0.0) ? x[i] : 0.0;
  }
}

// Tiny RAII wrapper around a raw byte buffer used for cuSPARSE SpMV temp
// storage. Sized lazily; never shrunk.
class ByteBuf {
public:
  ByteBuf() = default;
  ~ByteBuf() { free(); }
  ByteBuf(const ByteBuf &) = delete;
  ByteBuf &operator=(const ByteBuf &) = delete;
  ByteBuf(ByteBuf &&o) noexcept : ptr_(o.ptr_), bytes_(o.bytes_) {
    o.ptr_ = nullptr;
    o.bytes_ = 0;
  }
  ByteBuf &operator=(ByteBuf &&o) noexcept {
    if (this != &o) {
      free();
      ptr_ = o.ptr_;
      bytes_ = o.bytes_;
      o.ptr_ = nullptr;
      o.bytes_ = 0;
    }
    return *this;
  }

  void ensure(std::size_t bytes) {
    if (bytes <= bytes_)
      return;
    free();
    if (bytes > 0) {
      BREAKAGE_CUDA_CHECK(cudaMalloc(&ptr_, bytes));
      bytes_ = bytes;
    }
  }
  void *data() { return ptr_; }
  std::size_t size() const { return bytes_; }

private:
  void free() {
    if (ptr_) {
      cudaFree(ptr_);
      ptr_ = nullptr;
      bytes_ = 0;
    }
  }
  void *ptr_{nullptr};
  std::size_t bytes_{0};
};

} // namespace

// ---------------------------------------------------------------------------
// Stage: one ADMM run (P, C constant; q, l, u may change every solve).
//
// Stage owns:
//   * the constant matrices P, C, C^T (CSR, on device)
//   * a pre-allocated SpMV byte buffer per matrix
//   * warm-start state (x, z, y) plus the current rho
//   * scratch device buffers for the ADMM loop and the inner CG
// ---------------------------------------------------------------------------

class Stage {
public:
  // Builds device-side CSR copies of P (full symmetric) and C from Eigen
  // sparse host matrices, allocates all working buffers. Does NOT initialise
  // warm-start state - that happens lazily on the first solve.
  void setup(SparseMatrixCSC P_full, SparseMatrixCSC C, const Settings &s);

  // Runs ADMM until convergence or max_iter. q/l/u are host vectors
  // (re-uploaded each call). Output warm-start lives in x_, z_, y_ and is
  // also exposed via the const accessors below.
  StageInfo solve(const VectorXd &q_host, const VectorXd &l_host,
                  const VectorXd &u_host, cudaStream_t stream);

  // Drops warm-start state; next solve starts cold and resets rho to
  // settings_.rho_init.
  void reset_warm_start() { has_warm_ = false; }

  int n() const { return n_; }
  int m() const { return m_; }

  const double *x_device() const { return x_.data(); }
  const double *z_device() const { return z_.data(); }
  const double *y_device() const { return y_.data(); }

private:
  // Apply M = P + sigma I + rho C^T C to x (n elements) and write into Mx.
  // Uses scratch_m_ as the C*x scratchpad. Both x and Mx must be size n_.
  void apply_M(const double *x_d, double *Mx_d, double rho,
               cudaStream_t stream);

  // Solve M xt = rhs via CG, warm-started from xt_d (in-place). Returns the
  // number of CG iterations consumed (informational only).
  int solve_kkt_cg(const double *rhs_d, double *xt_d, double rho,
                   cudaStream_t stream);

  Settings settings_{};

  int n_{0};
  int m_{0};

  DeviceCsr P_;
  DeviceCsr C_;
  DeviceCsr Ct_;

  ByteBuf spmv_buf_P_;
  ByteBuf spmv_buf_C_;
  ByteBuf spmv_buf_Ct_;

  // Warm-start.
  DeviceBuffer<double> x_; // size n_
  DeviceBuffer<double> z_; // size m_
  DeviceBuffer<double> y_; // size m_
  bool has_warm_{false};
  double rho_{0.1};

  // Per-call uploads.
  DeviceBuffer<double> q_d_; // size n_
  DeviceBuffer<double> l_d_; // size m_
  DeviceBuffer<double> u_d_; // size m_

  // ADMM scratch buffers.
  DeviceBuffer<double> xt_;        // size n_
  DeviceBuffer<double> zt_;        // size m_
  DeviceBuffer<double> xt_alpha_;  // size n_
  DeviceBuffer<double> zt_alpha_;  // size m_
  DeviceBuffer<double> rhs_;       // size n_
  DeviceBuffer<double> scratch_n_; // size n_
  DeviceBuffer<double> scratch_m_; // size m_

  // Convergence-check scratch.
  DeviceBuffer<double> r_prim_; // size m_
  DeviceBuffer<double> r_dual_; // size n_

  // CG scratch.
  DeviceBuffer<double> cg_r_;  // size n_
  DeviceBuffer<double> cg_p_;  // size n_
  DeviceBuffer<double> cg_Ap_; // size n_
};

void Stage::setup(SparseMatrixCSC P_full, SparseMatrixCSC C,
                  const Settings &s) {
  settings_ = s;
  n_ = static_cast<int>(P_full.rows());
  m_ = static_cast<int>(C.rows());
  if (P_full.cols() != n_) {
    throw std::invalid_argument("Stage::setup: P must be square");
  }
  if (C.cols() != n_) {
    throw std::invalid_argument("Stage::setup: C cols mismatch");
  }

  if (!P_full.isCompressed())
    P_full.makeCompressed();
  if (!C.isCompressed())
    C.makeCompressed();

  upload_csc_as_csr(P_full, P_);
  upload_csc_as_csr(C, C_);

  // C^T as a separate CSR. cuSPARSE supports SpMV with op=TRANSPOSE on a CSR
  // input, but it is consistently slower than non-transpose, so we pay the
  // one-time host transpose to get faster inner-loop SpMVs.
  SparseMatrixCSC Ct = SparseMatrixCSC(C.transpose());
  Ct.makeCompressed();
  upload_csc_as_csr(Ct, Ct_);

  // Pre-size SpMV buffers. m_ or n_ may be zero in degenerate problems, in
  // which case spmv_buffer_size still works (cuSPARSE handles empty mats).
  spmv_buf_P_.ensure(cuda_la::spmv_buffer_size(P_, false));
  spmv_buf_C_.ensure(cuda_la::spmv_buffer_size(C_, false));
  spmv_buf_Ct_.ensure(cuda_la::spmv_buffer_size(Ct_, false));

  x_.resize(static_cast<std::size_t>(n_));
  z_.resize(static_cast<std::size_t>(m_));
  y_.resize(static_cast<std::size_t>(m_));
  q_d_.resize(static_cast<std::size_t>(n_));
  l_d_.resize(static_cast<std::size_t>(m_));
  u_d_.resize(static_cast<std::size_t>(m_));

  xt_.resize(static_cast<std::size_t>(n_));
  zt_.resize(static_cast<std::size_t>(m_));
  xt_alpha_.resize(static_cast<std::size_t>(n_));
  zt_alpha_.resize(static_cast<std::size_t>(m_));
  rhs_.resize(static_cast<std::size_t>(n_));
  scratch_n_.resize(static_cast<std::size_t>(n_));
  scratch_m_.resize(static_cast<std::size_t>(m_));

  r_prim_.resize(static_cast<std::size_t>(m_));
  r_dual_.resize(static_cast<std::size_t>(n_));

  cg_r_.resize(static_cast<std::size_t>(n_));
  cg_p_.resize(static_cast<std::size_t>(n_));
  cg_Ap_.resize(static_cast<std::size_t>(n_));

  has_warm_ = false;
  rho_ = settings_.rho_init;
}

void Stage::apply_M(const double *x_d, double *Mx_d, double rho,
                    cudaStream_t stream) {
  // Mx = P*x
  cuda_la::spmv(P_, false, 1.0, x_d, 0.0, Mx_d, spmv_buf_P_.data(),
                spmv_buf_P_.size(), stream);
  // Mx += sigma * x   (axpy: y += alpha * x)
  cuda_la::axpy(n_, settings_.sigma, x_d, Mx_d, stream);
  if (m_ > 0) {
    // scratch_m = C * x
    cuda_la::spmv(C_, false, 1.0, x_d, 0.0, scratch_m_.data(),
                  spmv_buf_C_.data(), spmv_buf_C_.size(), stream);
    // Mx += rho * C^T * scratch_m
    cuda_la::spmv(Ct_, false, rho, scratch_m_.data(), 1.0, Mx_d,
                  spmv_buf_Ct_.data(), spmv_buf_Ct_.size(), stream);
  }
}

int Stage::solve_kkt_cg(const double *rhs_d, double *xt_d, double rho,
                        cudaStream_t stream) {
  // Initial residual r = rhs - M*xt (xt is the warm-start input).
  // We reuse cg_Ap_ as the temporary for M*xt.
  apply_M(xt_d, cg_Ap_.data(), rho, stream);
  axby_kernel<<<grid_for(n_), kBlockSize, 0, stream>>>(
      n_, 1.0, rhs_d, -1.0, cg_Ap_.data(), cg_r_.data());
  // p = r
  cuda_la::copy(n_, cg_r_.data(), cg_p_.data(), stream);

  double rs_old = cuda_la::dot(n_, cg_r_.data(), cg_r_.data(), stream);
  // Tolerance is relative to the rhs norm so problems of different scale
  // all get the same relative accuracy.
  double rhs_norm = cuda_la::nrm2(n_, rhs_d, stream);
  double tol_sq = (kInnerCgTol * std::max(rhs_norm, 1e-30));
  tol_sq = tol_sq * tol_sq;
  if (rs_old <= tol_sq) {
    return 0;
  }

  for (int it = 0; it < kInnerCgMaxIter; ++it) {
    // cg_Ap = M * cg_p
    apply_M(cg_p_.data(), cg_Ap_.data(), rho, stream);
    double pAp = cuda_la::dot(n_, cg_p_.data(), cg_Ap_.data(), stream);
    if (!(pAp > 0.0)) {
      // Numerical breakdown - bail out with whatever we have.
      return it;
    }
    double alpha = rs_old / pAp;
    // xt += alpha * p
    cuda_la::axpy(n_, alpha, cg_p_.data(), xt_d, stream);
    // r  -= alpha * Ap
    cuda_la::axpy(n_, -alpha, cg_Ap_.data(), cg_r_.data(), stream);

    double rs_new = cuda_la::dot(n_, cg_r_.data(), cg_r_.data(), stream);
    if (rs_new <= tol_sq) {
      return it + 1;
    }
    double beta = rs_new / rs_old;
    // p = r + beta * p   (in-place: p = beta * p + 1.0 * r)
    axpby_inplace_kernel<<<grid_for(n_), kBlockSize, 0, stream>>>(
        n_, 1.0, cg_r_.data(), beta, cg_p_.data());
    rs_old = rs_new;
  }
  return kInnerCgMaxIter;
}

StageInfo Stage::solve(const VectorXd &q_host, const VectorXd &l_host,
                       const VectorXd &u_host, cudaStream_t stream) {
  if (q_host.size() != n_) {
    throw std::invalid_argument("Stage::solve: q size mismatch");
  }
  if (l_host.size() != m_ || u_host.size() != m_) {
    throw std::invalid_argument("Stage::solve: l/u size mismatch");
  }

  auto t_start = std::chrono::steady_clock::now();

  // Upload q, l, u (re-uploaded every call).
  if (n_ > 0) {
    q_d_.copy_from_host(q_host.data(), static_cast<std::size_t>(n_));
  }
  if (m_ > 0) {
    l_d_.copy_from_host(l_host.data(), static_cast<std::size_t>(m_));
    u_d_.copy_from_host(u_host.data(), static_cast<std::size_t>(m_));
  }

  // Cold-start state.
  if (!has_warm_) {
    if (n_ > 0)
      x_.zero();
    if (m_ > 0) {
      z_.zero();
      y_.zero();
    }
    rho_ = settings_.rho_init;
  }

  StageInfo info;
  info.rho = rho_;

  bool converged = false;
  int iter = 0;
  double prim_res = std::numeric_limits<double>::quiet_NaN();
  double dual_res = std::numeric_limits<double>::quiet_NaN();
  double q_norm_inf =
      (n_ > 0) ? cuda_la::inf_norm(n_, q_d_.data(), stream) : 0.0;

  for (iter = 0; iter < settings_.max_iter; ++iter) {
    // 1. Build rhs = sigma * x_k - q + C^T * (rho * z_k - y_k)
    if (m_ > 0) {
      // scratch_m = rho * z_k - y_k
      rho_z_minus_y_kernel<<<grid_for(m_), kBlockSize, 0, stream>>>(
          m_, rho_, z_.data(), y_.data(), scratch_m_.data());
      // scratch_n = C^T * scratch_m
      cuda_la::spmv(Ct_, false, 1.0, scratch_m_.data(), 0.0, scratch_n_.data(),
                    spmv_buf_Ct_.data(), spmv_buf_Ct_.size(), stream);
    } else if (n_ > 0) {
      scratch_n_.zero();
    }
    if (n_ > 0) {
      rhs_assemble_kernel<<<grid_for(n_), kBlockSize, 0, stream>>>(
          n_, settings_.sigma, x_.data(), q_d_.data(), scratch_n_.data(),
          rhs_.data());
    }

    // 2. Solve M xt = rhs via CG, warm-started from x_k.
    if (n_ > 0) {
      cuda_la::copy(n_, x_.data(), xt_.data(), stream);
      solve_kkt_cg(rhs_.data(), xt_.data(), rho_, stream);
    }

    // 3. zt = C * xt   (recovered from the reduced KKT)
    if (m_ > 0) {
      cuda_la::spmv(C_, false, 1.0, xt_.data(), 0.0, zt_.data(),
                    spmv_buf_C_.data(), spmv_buf_C_.size(), stream);
    }

    // 4. Over-relaxation:
    //    xt_alpha = alpha * xt + (1 - alpha) * x_k
    //    zt_alpha = alpha * zt + (1 - alpha) * z_k
    if (n_ > 0) {
      axby_kernel<<<grid_for(n_), kBlockSize, 0, stream>>>(
          n_, settings_.alpha, xt_.data(), 1.0 - settings_.alpha, x_.data(),
          xt_alpha_.data());
    }
    if (m_ > 0) {
      axby_kernel<<<grid_for(m_), kBlockSize, 0, stream>>>(
          m_, settings_.alpha, zt_.data(), 1.0 - settings_.alpha, z_.data(),
          zt_alpha_.data());

      // 5. z_{k+1} = clamp(zt_alpha + y_k / rho, l, u)  (into scratch_m)
      double inv_rho = 1.0 / rho_;
      z_update_kernel<<<grid_for(m_), kBlockSize, 0, stream>>>(
          m_, inv_rho, zt_alpha_.data(), y_.data(), l_d_.data(), u_d_.data(),
          scratch_m_.data());

      // 6. y += rho * (zt_alpha - z_{k+1})
      y_update_kernel<<<grid_for(m_), kBlockSize, 0, stream>>>(
          m_, rho_, zt_alpha_.data(), scratch_m_.data(), y_.data());

      // Promote z_{k+1} into z_  (z_ <- scratch_m).
      cuda_la::copy(m_, scratch_m_.data(), z_.data(), stream);
    }

    // 7. x_{k+1} = xt_alpha
    if (n_ > 0) {
      cuda_la::copy(n_, xt_alpha_.data(), x_.data(), stream);
    }

    // 8. Periodic convergence + rho-update check.
    bool last_iter = (iter + 1 == settings_.max_iter);
    if (((iter + 1) % std::max(1, settings_.check_every) == 0) || last_iter) {
      // Compute fresh A x_{k+1}, P x_{k+1}, A^T y_{k+1} for the residual
      // checks. These cost three SpMVs but only fire occasionally.
      double Ax_norm_inf = 0.0;
      double z_norm_inf = 0.0;
      double Px_norm_inf = 0.0;
      double Aty_norm_inf = 0.0;

      if (m_ > 0) {
        // scratch_m = A * x
        cuda_la::spmv(C_, false, 1.0, x_.data(), 0.0, scratch_m_.data(),
                      spmv_buf_C_.data(), spmv_buf_C_.size(), stream);
        Ax_norm_inf = cuda_la::inf_norm(m_, scratch_m_.data(), stream);
        // r_prim = scratch_m - z
        axby_kernel<<<grid_for(m_), kBlockSize, 0, stream>>>(
            m_, 1.0, scratch_m_.data(), -1.0, z_.data(), r_prim_.data());
        prim_res = cuda_la::inf_norm(m_, r_prim_.data(), stream);
        z_norm_inf = cuda_la::inf_norm(m_, z_.data(), stream);
      } else {
        prim_res = 0.0;
      }

      if (n_ > 0) {
        // scratch_n = P * x
        cuda_la::spmv(P_, false, 1.0, x_.data(), 0.0, scratch_n_.data(),
                      spmv_buf_P_.data(), spmv_buf_P_.size(), stream);
        Px_norm_inf = cuda_la::inf_norm(n_, scratch_n_.data(), stream);

        // r_dual = scratch_n + q
        axby_kernel<<<grid_for(n_), kBlockSize, 0, stream>>>(
            n_, 1.0, scratch_n_.data(), 1.0, q_d_.data(), r_dual_.data());

        if (m_ > 0) {
          // r_dual += C^T y    (uses scratch_n as a temporary)
          cuda_la::spmv(Ct_, false, 1.0, y_.data(), 0.0, scratch_n_.data(),
                        spmv_buf_Ct_.data(), spmv_buf_Ct_.size(), stream);
          Aty_norm_inf = cuda_la::inf_norm(n_, scratch_n_.data(), stream);
          cuda_la::axpy(n_, 1.0, scratch_n_.data(), r_dual_.data(), stream);
        }
        dual_res = cuda_la::inf_norm(n_, r_dual_.data(), stream);
      } else {
        dual_res = 0.0;
      }

      double prim_tol = settings_.eps_abs +
                        settings_.eps_rel *
                            std::max(Ax_norm_inf, z_norm_inf);
      double dual_tol = settings_.eps_abs +
                        settings_.eps_rel *
                            std::max({Px_norm_inf, q_norm_inf, Aty_norm_inf});

      info.prim_res = prim_res;
      info.dual_res = dual_res;
      info.iter = iter + 1;

      if (prim_res <= prim_tol && dual_res <= dual_tol) {
        converged = true;
        break;
      }

      // Adaptive rho. The implicit-M inner solver does not need a refactor
      // when rho changes - we only update the scalar.
      if (settings_.adaptive_rho) {
        double prim_scale = std::max({Ax_norm_inf, z_norm_inf, 1e-30});
        double dual_scale =
            std::max({Px_norm_inf, q_norm_inf, Aty_norm_inf, 1e-30});
        double prim_norm_rel = prim_res / prim_scale;
        double dual_norm_rel = dual_res / dual_scale;
        if (prim_norm_rel > 0.0 && dual_norm_rel > 0.0) {
          double rho_new = rho_ * std::sqrt(prim_norm_rel / dual_norm_rel);
          rho_new = std::clamp(rho_new, kRhoMin, kRhoMax);
          double ratio = rho_new / rho_;
          if (ratio > settings_.adaptive_rho_factor ||
              ratio < 1.0 / settings_.adaptive_rho_factor) {
            rho_ = rho_new;
            info.rho = rho_;
          }
        }
      }
    }
  }

  has_warm_ = true;
  info.converged = converged;
  info.rho = rho_;
  if (info.iter == 0) {
    info.iter = iter;
  }

  auto t_end = std::chrono::steady_clock::now();
  info.solve_time_s = std::chrono::duration<double>(t_end - t_start).count();
  return info;
}

// ---------------------------------------------------------------------------
// Helpers for building stage-specific Eigen sparse matrices on the host.
// These mirror the OsqpSolver::build_*_ helpers in osqp.cppm.
// ---------------------------------------------------------------------------

namespace {

using Triplet = Eigen::Triplet<double>;

// P_prj: diag(0 ... 0 (nx zeros), 1 ... 1 (me ones)) of size (nx+me, nx+me).
SparseMatrixCSC build_P_prj(int nx, int me) {
  std::vector<Triplet> trips;
  trips.reserve(static_cast<std::size_t>(me));
  for (int i = 0; i < me; ++i) {
    trips.emplace_back(nx + i, nx + i, 1.0);
  }
  SparseMatrixCSC P(nx + me, nx + me);
  P.setFromTriplets(trips.begin(), trips.end());
  P.makeCompressed();
  return P;
}

// C_prj: [A | -I_me] over the top me rows, [G | 0] underneath.
SparseMatrixCSC build_C_prj(const SparseMatrixCSC &A, const SparseMatrixCSC &G,
                            int nx, int me, int mi) {
  std::vector<Triplet> trips;
  trips.reserve(static_cast<std::size_t>(A.nonZeros() + G.nonZeros() + me));
  for (int k = 0; k < A.outerSize(); ++k) {
    for (SparseMatrixCSC::InnerIterator it(A, k); it; ++it) {
      trips.emplace_back(static_cast<int>(it.row()),
                         static_cast<int>(it.col()), it.value());
    }
  }
  for (int k = 0; k < G.outerSize(); ++k) {
    for (SparseMatrixCSC::InnerIterator it(G, k); it; ++it) {
      trips.emplace_back(me + static_cast<int>(it.row()),
                         static_cast<int>(it.col()), it.value());
    }
  }
  for (int i = 0; i < me; ++i) {
    trips.emplace_back(i, nx + i, -1.0);
  }
  SparseMatrixCSC C(me + mi, nx + me);
  C.setFromTriplets(trips.begin(), trips.end());
  C.makeCompressed();
  return C;
}

// P_rlx: diag(rel_x_reg * I_nx, I_nv).
SparseMatrixCSC build_P_rlx(int nx, int nv) {
  std::vector<Triplet> trips;
  trips.reserve(static_cast<std::size_t>(nx + nv));
  for (int i = 0; i < nx; ++i) {
    trips.emplace_back(i, i, kRelXReg);
  }
  for (int i = 0; i < nv; ++i) {
    trips.emplace_back(nx + i, nx + i, 1.0);
  }
  SparseMatrixCSC P(nx + nv, nx + nv);
  P.setFromTriplets(trips.begin(), trips.end());
  P.makeCompressed();
  return P;
}

// C_rlx: [ A 0 ; G 0 ; H -V ; 0 I_nv ].
SparseMatrixCSC build_C_rlx(const SparseMatrixCSC &A,
                            const SparseMatrixCSC &G,
                            const SparseMatrixCSC &H,
                            const SparseMatrixCSC &V, int nx, int me, int mi,
                            int mh, int nv) {
  std::vector<Triplet> trips;
  trips.reserve(static_cast<std::size_t>(A.nonZeros() + G.nonZeros() +
                                         H.nonZeros() + V.nonZeros() + nv));
  for (int k = 0; k < A.outerSize(); ++k) {
    for (SparseMatrixCSC::InnerIterator it(A, k); it; ++it) {
      trips.emplace_back(static_cast<int>(it.row()),
                         static_cast<int>(it.col()), it.value());
    }
  }
  for (int k = 0; k < G.outerSize(); ++k) {
    for (SparseMatrixCSC::InnerIterator it(G, k); it; ++it) {
      trips.emplace_back(me + static_cast<int>(it.row()),
                         static_cast<int>(it.col()), it.value());
    }
  }
  for (int k = 0; k < H.outerSize(); ++k) {
    for (SparseMatrixCSC::InnerIterator it(H, k); it; ++it) {
      trips.emplace_back(me + mi + static_cast<int>(it.row()),
                         static_cast<int>(it.col()), it.value());
    }
  }
  for (int k = 0; k < V.outerSize(); ++k) {
    for (SparseMatrixCSC::InnerIterator it(V, k); it; ++it) {
      trips.emplace_back(me + mi + static_cast<int>(it.row()),
                         nx + static_cast<int>(it.col()), -it.value());
    }
  }
  for (int i = 0; i < nv; ++i) {
    trips.emplace_back(me + mi + mh + i, nx + i, 1.0);
  }
  SparseMatrixCSC C(me + mi + mh + nv, nx + nv);
  C.setFromTriplets(trips.begin(), trips.end());
  C.makeCompressed();
  return C;
}

// P_opt: full symmetric Q. We don't restrict to the upper triangle here
// (osqp.cppm does for the OSQP front-end, but the inner ADMM works on the
// symmetric version directly).
SparseMatrixCSC build_P_opt(const SparseMatrixCSC &Q) {
  SparseMatrixCSC P = Q;
  P.makeCompressed();
  return P;
}

// C_opt: [ A ; G ; H ].
SparseMatrixCSC build_C_opt(const SparseMatrixCSC &A,
                            const SparseMatrixCSC &G,
                            const SparseMatrixCSC &H, int nx, int me, int mi,
                            int mh) {
  std::vector<Triplet> trips;
  trips.reserve(static_cast<std::size_t>(A.nonZeros() + G.nonZeros() +
                                         H.nonZeros()));
  for (int k = 0; k < A.outerSize(); ++k) {
    for (SparseMatrixCSC::InnerIterator it(A, k); it; ++it) {
      trips.emplace_back(static_cast<int>(it.row()),
                         static_cast<int>(it.col()), it.value());
    }
  }
  for (int k = 0; k < G.outerSize(); ++k) {
    for (SparseMatrixCSC::InnerIterator it(G, k); it; ++it) {
      trips.emplace_back(me + static_cast<int>(it.row()),
                         static_cast<int>(it.col()), it.value());
    }
  }
  for (int k = 0; k < H.outerSize(); ++k) {
    for (SparseMatrixCSC::InnerIterator it(H, k); it; ++it) {
      trips.emplace_back(me + mi + static_cast<int>(it.row()),
                         static_cast<int>(it.col()), it.value());
    }
  }
  SparseMatrixCSC C(me + mi + mh, nx);
  C.setFromTriplets(trips.begin(), trips.end());
  C.makeCompressed();
  return C;
}

// ---------------------------------------------------------------------------
// Side-allocated impl structs.
//
// Solver and Pipeline have fixed member layouts in their public headers and
// we are not allowed to touch those headers. The extra device buffers we
// need (system A/V, scratch b, etc.) live in heap-allocated impl structs
// kept in process-local maps keyed by the owner pointer. This is a bit
// unusual but keeps the public API frozen.
// ---------------------------------------------------------------------------

struct SolverImpl {
  DeviceCsr A_dev;
  ByteBuf spmv_buf_A;
  // V is small and stays on the host: V * v_relax is done with Eigen.
  SparseMatrixCSC V_host;
  // Scratch for b/b_prj on device.
  DeviceBuffer<double> b_d;
  DeviceBuffer<double> b_prj_d;
  // max(0, x_rlx.tail(nv)) staging buffer.
  DeviceBuffer<double> v_relax_full_d;
};

std::unordered_map<const Solver *, std::unique_ptr<SolverImpl>> &
solver_impl_map() {
  static std::unordered_map<const Solver *, std::unique_ptr<SolverImpl>> m;
  return m;
}
std::mutex &solver_impl_mutex() {
  static std::mutex m;
  return m;
}
SolverImpl &solver_impl(const Solver *self) {
  std::lock_guard<std::mutex> lk(solver_impl_mutex());
  auto &m = solver_impl_map();
  auto it = m.find(self);
  if (it == m.end()) {
    auto [ins, _] = m.emplace(self, std::make_unique<SolverImpl>());
    return *ins->second;
  }
  return *it->second;
}
void erase_solver_impl(const Solver *self) {
  std::lock_guard<std::mutex> lk(solver_impl_mutex());
  solver_impl_map().erase(self);
}
void move_solver_impl(const Solver *from, const Solver *to) {
  std::lock_guard<std::mutex> lk(solver_impl_mutex());
  auto &m = solver_impl_map();
  auto it = m.find(from);
  if (it != m.end()) {
    auto ptr = std::move(it->second);
    m.erase(it);
    m[to] = std::move(ptr);
  }
}

struct PipelineImpl {
  DeviceInput input;
  DeviceBuffer<double> v_W_prev_d; // (N, 3) row-major
  DeviceBuffer<double> L_prev_d;   // (N, 3) row-major
  std::vector<double> rm_buf;      // host scratch for row-major conversion
};

std::unordered_map<const Pipeline *, std::unique_ptr<PipelineImpl>> &
pipeline_impl_map() {
  static std::unordered_map<const Pipeline *, std::unique_ptr<PipelineImpl>> m;
  return m;
}
std::mutex &pipeline_impl_mutex() {
  static std::mutex m;
  return m;
}
PipelineImpl &pipeline_impl(const Pipeline *self) {
  std::lock_guard<std::mutex> lk(pipeline_impl_mutex());
  auto &m = pipeline_impl_map();
  auto it = m.find(self);
  if (it == m.end()) {
    auto [ins, _] = m.emplace(self, std::make_unique<PipelineImpl>());
    return *ins->second;
  }
  return *it->second;
}
void erase_pipeline_impl(const Pipeline *self) {
  std::lock_guard<std::mutex> lk(pipeline_impl_mutex());
  pipeline_impl_map().erase(self);
}

// Convert a host MatrixX3d (column-major Eigen) into a row-major flat array.
void to_row_major_N3(const MatrixX3d &src, std::vector<double> &dst) {
  const int N = static_cast<int>(src.rows());
  dst.resize(static_cast<std::size_t>(N) * 3);
  for (int i = 0; i < N; ++i) {
    dst[3 * i + 0] = src(i, 0);
    dst[3 * i + 1] = src(i, 1);
    dst[3 * i + 2] = src(i, 2);
  }
}

// Convert a row-major flat array back into a column-major MatrixX3d.
void from_row_major_N3(const std::vector<double> &src, MatrixX3d &dst, int N) {
  dst.resize(N, 3);
  for (int i = 0; i < N; ++i) {
    dst(i, 0) = src[3 * i + 0];
    dst(i, 1) = src[3 * i + 1];
    dst(i, 2) = src[3 * i + 2];
  }
}

} // namespace

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

Solver::Solver() = default;

Solver::~Solver() { erase_solver_impl(this); }

Solver::Solver(Solver &&o) noexcept
    : prj_(std::move(o.prj_)), rlx_(std::move(o.rlx_)),
      opt_(std::move(o.opt_)), settings_(o.settings_), nx_(o.nx_), me_(o.me_),
      mi_(o.mi_), mh_(o.mh_), nv_(o.nv_) {
  move_solver_impl(&o, this);
}

Solver &Solver::operator=(Solver &&o) noexcept {
  if (this != &o) {
    erase_solver_impl(this);
    prj_ = std::move(o.prj_);
    rlx_ = std::move(o.rlx_);
    opt_ = std::move(o.opt_);
    settings_ = o.settings_;
    nx_ = o.nx_;
    me_ = o.me_;
    mi_ = o.mi_;
    mh_ = o.mh_;
    nv_ = o.nv_;
    move_solver_impl(&o, this);
  }
  return *this;
}

int Solver::num_vars() const { return nx_; }
int Solver::num_eq() const { return me_; }
int Solver::num_clutches() const { return nv_; }

void Solver::reset_warm_start() {
  if (prj_)
    prj_->reset_warm_start();
  if (rlx_)
    rlx_->reset_warm_start();
  if (opt_)
    opt_->reset_warm_start();
}

void Solver::setup(const HostSystem &host, const DeviceSystem & /*dev*/,
                   const Settings &settings) {
  settings_ = settings;
  nx_ = host.num_vars;
  me_ = host.num_eq;
  mi_ = host.num_ineq;
  mh_ = host.num_relaxed_ineq;
  nv_ = host.num_clutches;

  if (host.A.rows() != me_ || host.A.cols() != nx_ || host.G.rows() != mi_ ||
      host.G.cols() != nx_ || host.H.rows() != mh_ || host.H.cols() != nx_ ||
      host.V.rows() != mh_ || host.V.cols() != nv_ || host.Q.rows() != nx_ ||
      host.Q.cols() != nx_) {
    throw std::invalid_argument(
        "Solver::setup: HostSystem matrix shape mismatch");
  }

  // Build host stage matrices (CSC, Eigen).
  SparseMatrixCSC P_prj = build_P_prj(nx_, me_);
  SparseMatrixCSC C_prj = build_C_prj(host.A, host.G, nx_, me_, mi_);
  SparseMatrixCSC P_rlx = build_P_rlx(nx_, nv_);
  SparseMatrixCSC C_rlx =
      build_C_rlx(host.A, host.G, host.H, host.V, nx_, me_, mi_, mh_, nv_);
  SparseMatrixCSC P_opt = build_P_opt(host.Q);
  SparseMatrixCSC C_opt =
      build_C_opt(host.A, host.G, host.H, nx_, me_, mi_, mh_);

  prj_ = std::make_unique<Stage>();
  rlx_ = std::make_unique<Stage>();
  opt_ = std::make_unique<Stage>();
  prj_->setup(std::move(P_prj), std::move(C_prj), settings_);
  rlx_->setup(std::move(P_rlx), std::move(C_rlx), settings_);
  opt_->setup(std::move(P_opt), std::move(C_opt), settings_);

  // Per-Solver impl: A on device for b_prj, V on host for V*v_relax.
  SolverImpl &impl = solver_impl(this);
  SparseMatrixCSC A_csc = host.A;
  if (!A_csc.isCompressed())
    A_csc.makeCompressed();
  upload_csc_as_csr(A_csc, impl.A_dev);
  impl.spmv_buf_A.ensure(cuda_la::spmv_buffer_size(impl.A_dev, false));
  impl.V_host = host.V;
  if (!impl.V_host.isCompressed())
    impl.V_host.makeCompressed();
  impl.b_d.resize(static_cast<std::size_t>(me_));
  impl.b_prj_d.resize(static_cast<std::size_t>(me_));
  impl.v_relax_full_d.resize(static_cast<std::size_t>(nv_));
}

SolveInfo Solver::solve(const VectorXd &b, double *x_d, double *slack_d,
                        double *v_relax_d, cudaStream_t stream) {
  if (b.size() != me_) {
    throw std::invalid_argument("Solver::solve: b size mismatch");
  }
  if (!prj_ || !rlx_ || !opt_) {
    throw std::runtime_error("Solver::solve: setup() not called");
  }
  SolveInfo info;
  SolverImpl &impl = solver_impl(this);
  if (me_ > 0) {
    impl.b_d.copy_from_host(b.data(), static_cast<std::size_t>(me_));
  }

  // -------- Stage 1: projection --------
  {
    VectorXd q_prj(nx_ + me_);
    q_prj.head(nx_).setZero();
    q_prj.tail(me_) = -b;
    VectorXd l_prj = VectorXd::Zero(me_ + mi_);
    VectorXd u_prj(me_ + mi_);
    u_prj.head(me_).setZero();
    u_prj.tail(mi_).setConstant(settings_.infinity);

    info.prj = prj_->solve(q_prj, l_prj, u_prj, stream);
  }

  // b_prj = A * x_prj.head(nx)   (SpMV on device, then download)
  VectorXd b_prj_host(me_);
  if (me_ > 0) {
    cuda_la::spmv(impl.A_dev, false, 1.0, prj_->x_device(), 0.0,
                  impl.b_prj_d.data(), impl.spmv_buf_A.data(),
                  impl.spmv_buf_A.size(), stream);
    BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(b_prj_host.data(), impl.b_prj_d.data(),
                                        me_ * sizeof(double),
                                        cudaMemcpyDeviceToHost, stream));
  }

  // slack_d = b - A * x_prj.head(nx) = b - b_prj
  if (slack_d != nullptr && me_ > 0) {
    BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(slack_d, impl.b_d.data(),
                                        me_ * sizeof(double),
                                        cudaMemcpyDeviceToDevice, stream));
    cuda_la::axpy(me_, -1.0, impl.b_prj_d.data(), slack_d, stream);
  }

  // Make sure b_prj_host is populated before we use it for stage 2 inputs.
  BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));

  // -------- Stage 2: relaxation --------
  {
    VectorXd q_rlx = VectorXd::Zero(nx_ + nv_);
    VectorXd l_rlx(me_ + mi_ + mh_ + nv_);
    VectorXd u_rlx(me_ + mi_ + mh_ + nv_);
    l_rlx.head(me_) = b_prj_host;
    u_rlx.head(me_) = b_prj_host;
    l_rlx.segment(me_, mi_).setConstant(-kEpsilon);
    u_rlx.segment(me_, mi_).setConstant(settings_.infinity);
    l_rlx.segment(me_ + mi_, mh_).setConstant(-settings_.infinity);
    u_rlx.segment(me_ + mi_, mh_).setConstant(1.0 - kEpsilon);
    l_rlx.tail(nv_).setZero();
    u_rlx.tail(nv_).setConstant(settings_.infinity);

    info.rlx = rlx_->solve(q_rlx, l_rlx, u_rlx, stream);
  }

  // v_relax = max(0, x_rlx.tail(nv))
  VectorXd v_relax_host(nv_);
  if (nv_ > 0) {
    const double *x_rlx_tail = rlx_->x_device() + nx_;
    relu_kernel<<<grid_for(nv_), kBlockSize, 0, stream>>>(
        nv_, x_rlx_tail, impl.v_relax_full_d.data());
    if (v_relax_d != nullptr) {
      BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(v_relax_d,
                                          impl.v_relax_full_d.data(),
                                          nv_ * sizeof(double),
                                          cudaMemcpyDeviceToDevice, stream));
    }
    BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(v_relax_host.data(),
                                        impl.v_relax_full_d.data(),
                                        nv_ * sizeof(double),
                                        cudaMemcpyDeviceToHost, stream));
    BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));
  }

  // -------- Stage 3: optimisation --------
  {
    VectorXd q_opt = VectorXd::Zero(nx_);
    VectorXd l_opt(me_ + mi_ + mh_);
    VectorXd u_opt(me_ + mi_ + mh_);
    l_opt.head(me_) = b_prj_host;
    u_opt.head(me_) = b_prj_host;
    l_opt.segment(me_, mi_).setConstant(-kEpsilon);
    u_opt.segment(me_, mi_).setConstant(settings_.infinity);
    l_opt.tail(mh_).setConstant(-settings_.infinity);
    // Match osqp.cppm / ref_cpu.cpp: opt-stage H-row upper bound is 1.0
    // (NOT 1.0 - epsilon -- that one belongs to the relaxation stage).
    u_opt.tail(mh_).setConstant(1.0);
    if (mh_ > 0 && nv_ > 0) {
      VectorXd add = impl.V_host * v_relax_host;
      u_opt.tail(mh_) += add;
    }

    info.opt = opt_->solve(q_opt, l_opt, u_opt, stream);
  }

  // Copy out x (size nx_).
  if (x_d != nullptr && nx_ > 0) {
    BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(x_d, opt_->x_device(),
                                        nx_ * sizeof(double),
                                        cudaMemcpyDeviceToDevice, stream));
  }

  info.converged =
      info.prj.converged && info.rlx.converged && info.opt.converged;
  return info;
}

// ---------------------------------------------------------------------------
// Pipeline - end-to-end driver mirroring BreakageChecker::solve.
// ---------------------------------------------------------------------------

Pipeline::Pipeline() = default;
Pipeline::~Pipeline() { erase_pipeline_impl(this); }

void Pipeline::setup(const HostSystem &host, const Thresholds &thr,
                     const Settings &settings) {
  host_ = host;
  thr_ = thr;
  upload_system(host_, dev_);
  qp_.setup(host_, dev_, settings);

  const int N = host_.num_parts;
  v_W_curr_.resize(static_cast<std::size_t>(N) * 3);
  L_curr_.resize(static_cast<std::size_t>(N) * 3);
  b_d_.resize(static_cast<std::size_t>(N) * 6);
  x_d_.resize(static_cast<std::size_t>(host_.num_vars));
  x_unwhiten_d_.resize(static_cast<std::size_t>(host_.num_vars));
  slack_d_.resize(static_cast<std::size_t>(host_.num_eq));
  v_relax_d_.resize(static_cast<std::size_t>(host_.num_clutches));
  utilization_d_.resize(static_cast<std::size_t>(host_.num_clutches));

  PipelineImpl &impl = pipeline_impl(this);
  impl.v_W_prev_d.resize(static_cast<std::size_t>(N) * 3);
  impl.L_prev_d.resize(static_cast<std::size_t>(N) * 3);
}

Solution Pipeline::solve(const HostInput &in, HostState &state,
                         cudaStream_t stream) {
  Solution sol;
  PipelineImpl &impl = pipeline_impl(this);

  const int N = host_.num_parts;
  const int K = host_.num_clutches;
  const int ncv = host_.num_contact_vertices;
  const int ncap = static_cast<int>(host_.capacity_clutch_indices.size());
  const int num_vars = host_.num_vars;
  const int me = host_.num_eq;

  // 0. Upload input + previous state. We always re-upload v_W_prev and
  //    L_prev from HostState in case the user mutated them between solves.
  upload_input(in, impl.input);

  if (state.v_W_prev.rows() != N || state.L_prev.rows() != N) {
    throw std::invalid_argument("Pipeline::solve: HostState dim mismatch");
  }
  to_row_major_N3(state.v_W_prev, impl.rm_buf);
  impl.v_W_prev_d.copy_from_host(impl.rm_buf.data(),
                                 static_cast<std::size_t>(N) * 3);
  to_row_major_N3(state.L_prev, impl.rm_buf);
  impl.L_prev_d.copy_from_host(impl.rm_buf.data(),
                               static_cast<std::size_t>(N) * 3);

  // 1. fit_se3 -> q_W_CC, t_W_CC (host scalars).
  double q_W_CC[4];
  double t_W_CC[3];
  cuda_kernels::fit_se3(N, dev_.q_CC.data(), dev_.c_CC.data(),
                        impl.input.q.data(), impl.input.c.data(),
                        dev_.mass.data(), dev_.total_mass,
                        dev_.L0 * dev_.L0 / 2.0, q_W_CC, t_W_CC, stream);

  // 2. fit_twist -> w0/v0 (host) + v_W_curr (device).
  // v0 is computed but unused by the rest of the pipeline (matches the CPU
  // baseline in breakage.cppm: only w0 feeds compute_L and v_W feeds b).
  double w0[3];
  double v0[3];
  cuda_kernels::fit_twist(N, impl.input.w.data(), impl.input.v.data(),
                          dev_.c_CC.data(), q_W_CC, t_W_CC, dev_.mass.data(),
                          dev_.total_mass, dev_.L0 * dev_.L0, w0, v0,
                          v_W_curr_.data(), stream);
  (void)v0;

  // 3. compute_Pi (host scalar work). Eigen::Quaterniond stores as (x,y,z,w).
  double q_prev[4];
  q_prev[0] = state.q_W_CC_prev.x();
  q_prev[1] = state.q_W_CC_prev.y();
  q_prev[2] = state.q_W_CC_prev.z();
  q_prev[3] = state.q_W_CC_prev.w();
  double Pi[9];
  cuda_kernels::compute_Pi_host(q_prev, q_W_CC, Pi);

  // 4. compute_L on the device.
  cuda_kernels::compute_L(N, dev_.I_CC.data(), q_W_CC, w0, L_curr_.data(),
                          stream);

  // 5. Assemble RHS b on the device.
  cuda_kernels::assemble_b(N, v_W_curr_.data(), impl.v_W_prev_d.data(),
                           L_curr_.data(), impl.L_prev_d.data(),
                           impl.input.J.data(), impl.input.H.data(),
                           dev_.mass.data(), Pi, in.dt, dev_.L0, b_d_.data(),
                           stream);

  // 6. Download b to host (small: 6*N doubles).
  VectorXd b_host(6 * N);
  if (N > 0) {
    BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(b_host.data(), b_d_.data(),
                                        6 * N * sizeof(double),
                                        cudaMemcpyDeviceToHost, stream));
    BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));
  }

  // 7. QP solve.
  sol.info = qp_.solve(b_host, x_d_.data(), slack_d_.data(), v_relax_d_.data(),
                       stream);

  // 8. slack_fraction = ||slack||_2 / max(||b||_2, floor)
  double slack_norm =
      (me > 0) ? cuda_la::nrm2(me, slack_d_.data(), stream) : 0.0;
  double b_norm = b_host.norm();
  sol.slack_fraction =
      slack_norm / std::max(b_norm, thr_.slack_fraction_b_floor);

  // 9. Postprocess: undo whitening + per-clutch utilisation.
  if (sol.info.converged) {
    cuda_kernels::postprocess(ncv, K, ncap, x_d_.data(),
                              dev_.capacity_clutch_indices.data(),
                              dev_.capacities.data(), dev_.clutch_whiten.data(),
                              x_unwhiten_d_.data(), utilization_d_.data(),
                              stream);

    sol.x.resize(num_vars);
    sol.utilization.resize(K);
    if (num_vars > 0) {
      BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(sol.x.data(), x_unwhiten_d_.data(),
                                          num_vars * sizeof(double),
                                          cudaMemcpyDeviceToHost, stream));
    }
    if (K > 0) {
      BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(sol.utilization.data(),
                                          utilization_d_.data(),
                                          K * sizeof(double),
                                          cudaMemcpyDeviceToHost, stream));
    }
    BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));
  }

  // 10. Persist HostState (host-side copies of v_W_curr, L_curr, q_W_CC).
  // Eigen::Quaterniond constructor takes (w, x, y, z).
  state.q_W_CC_prev =
      Eigen::Quaterniond(q_W_CC[3], q_W_CC[0], q_W_CC[1], q_W_CC[2]);

  if (state.v_W_prev.rows() != N) {
    state.v_W_prev.resize(N, 3);
  }
  if (state.L_prev.rows() != N) {
    state.L_prev.resize(N, 3);
  }
  if (N > 0) {
    impl.rm_buf.resize(static_cast<std::size_t>(N) * 3);
    BREAKAGE_CUDA_CHECK(cudaMemcpy(impl.rm_buf.data(), v_W_curr_.data(),
                                   3 * N * sizeof(double),
                                   cudaMemcpyDeviceToHost));
    from_row_major_N3(impl.rm_buf, state.v_W_prev, N);
    BREAKAGE_CUDA_CHECK(cudaMemcpy(impl.rm_buf.data(), L_curr_.data(),
                                   3 * N * sizeof(double),
                                   cudaMemcpyDeviceToHost));
    from_row_major_N3(impl.rm_buf, state.L_prev, N);
  }

  return sol;
}

} // namespace breakage_cuda::cuda_qp
