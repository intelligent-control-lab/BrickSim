// SPDX-License-Identifier: MIT
//
// Implementation of breakage_cuda/kernels.hpp:
//
//   * fit_se3      -- mass-weighted SE3 Procrustes (GPU reductions + host SVD)
//   * fit_twist    -- mass-weighted twist fit       (GPU reductions + host LDLT)
//   * compute_Pi   -- pure-host 3x3 (small, scalar work)
//   * compute_L    -- per-part inertia rotation     (one thread per part)
//   * assemble_b   -- per-part RHS row assembly     (one thread per part)
//   * postprocess  -- un-whiten optimization x and compute clutch utilization
//
// Strategy notes:
//   - Per-part data ((N,K) row-major) is small enough that we keep the kernels
//     simple: one thread per part with a block-stride loop, plus a single
//     shared-memory sum reduction inside each block.
//   - Reductions land in tiny global accumulators (<= 18 doubles) via per-block
//     atomicAdd; we then memcpy them back to host for the small dense post-
//     processing (SVD, LDLT) where Eigen on the host is plenty fast.
//   - All numerics in double; formulas mirror breakage.cppm verbatim.

#include "breakage_cuda/kernels.hpp"
#include "breakage_cuda/device.hpp"
#include "breakage_cuda/linalg.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace breakage_cuda::cuda_kernels {

namespace {

// ---------------------------------------------------------------------------
// Plain-old-data wrappers used as kernel by-value parameters.
// Mat3 is row-major.
// ---------------------------------------------------------------------------

struct Mat3 {
  double m[9];
};
struct Vec3 {
  double v[3];
};

// All reduction kernels assume blockDim.x == kBlock. The host launchers
// always launch with this exact block size.
constexpr int kBlock = 256;

// Cap the launch grid: reductions all use a block-stride loop so any value
// works, but a smaller grid lowers atomic-add contention on the tiny
// accumulators.
inline int grid_for(int N) {
  int g = (N + kBlock - 1) / kBlock;
  return std::min(g, 256);
}

// ---------------------------------------------------------------------------
// Device math helpers.
// ---------------------------------------------------------------------------

__device__ inline void d_quat_xyzw_to_R(double x, double y, double z, double w,
                                        double R[9]) {
  // Standard unit-quaternion to rotation-matrix conversion. Matches
  // Eigen::Quaternion::toRotationMatrix exactly when the input is unit-length.
  double xx = x * x, yy = y * y, zz = z * z;
  double xy = x * y, xz = x * z, yz = y * z;
  double wx = w * x, wy = w * y, wz = w * z;
  R[0] = 1.0 - 2.0 * (yy + zz);
  R[1] = 2.0 * (xy - wz);
  R[2] = 2.0 * (xz + wy);
  R[3] = 2.0 * (xy + wz);
  R[4] = 1.0 - 2.0 * (xx + zz);
  R[5] = 2.0 * (yz - wx);
  R[6] = 2.0 * (xz - wy);
  R[7] = 2.0 * (yz + wx);
  R[8] = 1.0 - 2.0 * (xx + yy);
}

// Block sum-reduce. Caller provides a `kBlock`-sized shared buffer. Returns
// the same value on every thread (sm[0] after the final sync).
__device__ inline double block_reduce_sum(double val, double *sm) {
  int tid = threadIdx.x;
  sm[tid] = val;
  __syncthreads();
  for (int s = kBlock / 2; s > 0; s >>= 1) {
    if (tid < s) {
      sm[tid] += sm[tid + s];
    }
    __syncthreads();
  }
  return sm[0];
}

// Reduce VARS thread-local doubles to per-block sums and atomicAdd them into
// `out` in one go.
template <int VARS>
__device__ inline void block_reduce_atomic_add(double *local, double *sm,
                                               double *out) {
  int tid = threadIdx.x;
  for (int k = 0; k < VARS; ++k) {
    double r = block_reduce_sum(local[k], sm);
    if (tid == 0) {
      atomicAdd(&out[k], r);
    }
    __syncthreads();
  }
}

// Atomic max for fp64 via CAS on the bit representation.
//
// CUDA does not provide an atomicMax(double*, double); we emulate it with a
// CAS retry loop on the bits. NaN inputs are silently ignored (matches the
// CPU baseline, which never produces them on this hot path).
__device__ inline void atomic_max_double(double *addr, double val) {
  auto *p = reinterpret_cast<unsigned long long *>(addr);
  unsigned long long old = *p;
  unsigned long long assumed;
  do {
    assumed = old;
    double cur = __longlong_as_double(static_cast<long long>(assumed));
    // `!(val > cur)` is true for NaN val too, so we do not write NaNs.
    if (!(val > cur)) {
      return;
    }
    unsigned long long candidate =
        static_cast<unsigned long long>(__double_as_longlong(val));
    old = atomicCAS(p, assumed, candidate);
  } while (assumed != old);
}

// ---------------------------------------------------------------------------
// fit_se3 reduction kernels.
// ---------------------------------------------------------------------------

// Pass 1: accumulate the centroid sums and the K matrix.
//
// Per-part contributions packed into 15 doubles:
//   local[ 0..2]  : m * t0
//   local[ 3..5]  : m * tx
//   local[ 6..14] : m * R(q0 * conj(qx))   (row-major flatten)
__global__ void se3_pass1_kernel(int N, const double *q0, const double *t0,
                                 const double *qx, const double *tx,
                                 const double *mass, double *out_15) {
  __shared__ double sm[kBlock];
  double local[15];
  for (int k = 0; k < 15; ++k) {
    local[k] = 0.0;
  }

  for (int i = blockIdx.x * kBlock + threadIdx.x; i < N;
       i += kBlock * gridDim.x) {
    double m = mass[i];
    double t0x = t0[3 * i + 0];
    double t0y = t0[3 * i + 1];
    double t0z = t0[3 * i + 2];
    double txx = tx[3 * i + 0];
    double txy = tx[3 * i + 1];
    double txz = tx[3 * i + 2];

    double q0x = q0[4 * i + 0], q0y = q0[4 * i + 1];
    double q0z = q0[4 * i + 2], q0w = q0[4 * i + 3];
    double qxx = qx[4 * i + 0], qxy = qx[4 * i + 1];
    double qxz = qx[4 * i + 2], qxw = qx[4 * i + 3];

    // q0 * conj(qx); conj(qx) = (-qxx, -qxy, -qxz, qxw)
    double bx = -qxx, by = -qxy, bz = -qxz, bw = qxw;
    double rx = q0w * bx + q0x * bw + q0y * bz - q0z * by;
    double ry = q0w * by - q0x * bz + q0y * bw + q0z * bx;
    double rz = q0w * bz + q0x * by - q0y * bx + q0z * bw;
    double rw = q0w * bw - q0x * bx - q0y * by - q0z * bz;

    double R[9];
    d_quat_xyzw_to_R(rx, ry, rz, rw, R);

    local[0] += m * t0x;
    local[1] += m * t0y;
    local[2] += m * t0z;
    local[3] += m * txx;
    local[4] += m * txy;
    local[5] += m * txz;
    for (int k = 0; k < 9; ++k) {
      local[6 + k] += m * R[k];
    }
  }

  block_reduce_atomic_add<15>(local, sm, out_15);
}

// Pass 2: accumulate the H = sum_i m_i (t0_i - t0_bar)(tx_i - tx_bar)^T outer
// product after the centroids are known (host-side).
__global__ void se3_pass2_kernel(int N, const double *t0, const double *tx,
                                 const double *mass, Vec3 t0_bar, Vec3 tx_bar,
                                 double *out_9) {
  __shared__ double sm[kBlock];
  double local[9];
  for (int k = 0; k < 9; ++k) {
    local[k] = 0.0;
  }

  for (int i = blockIdx.x * kBlock + threadIdx.x; i < N;
       i += kBlock * gridDim.x) {
    double m = mass[i];
    double dt0x = t0[3 * i + 0] - t0_bar.v[0];
    double dt0y = t0[3 * i + 1] - t0_bar.v[1];
    double dt0z = t0[3 * i + 2] - t0_bar.v[2];
    double dtxx = tx[3 * i + 0] - tx_bar.v[0];
    double dtxy = tx[3 * i + 1] - tx_bar.v[1];
    double dtxz = tx[3 * i + 2] - tx_bar.v[2];

    // Row-major outer product m * dt0 * dtx^T.
    local[0] += m * dt0x * dtxx;
    local[1] += m * dt0x * dtxy;
    local[2] += m * dt0x * dtxz;
    local[3] += m * dt0y * dtxx;
    local[4] += m * dt0y * dtxy;
    local[5] += m * dt0y * dtxz;
    local[6] += m * dt0z * dtxx;
    local[7] += m * dt0z * dtxy;
    local[8] += m * dt0z * dtxz;
  }

  block_reduce_atomic_add<9>(local, sm, out_9);
}

// ---------------------------------------------------------------------------
// fit_twist reduction kernels.
// ---------------------------------------------------------------------------

// Pass 1: c_W = R_w * c_CC + t_w; accumulate sum(m * c_W) for the centroid r.
__global__ void twist_pass1_kernel(int N, const double *c_CC,
                                   const double *mass, Mat3 R_w, Vec3 t_w,
                                   double *c_W_out, double *sum_mc_W_3) {
  __shared__ double sm[kBlock];
  double local[3] = {0.0, 0.0, 0.0};

  for (int i = blockIdx.x * kBlock + threadIdx.x; i < N;
       i += kBlock * gridDim.x) {
    double m = mass[i];
    double cx = c_CC[3 * i + 0];
    double cy = c_CC[3 * i + 1];
    double cz = c_CC[3 * i + 2];
    double cw0 = R_w.m[0] * cx + R_w.m[1] * cy + R_w.m[2] * cz + t_w.v[0];
    double cw1 = R_w.m[3] * cx + R_w.m[4] * cy + R_w.m[5] * cz + t_w.v[1];
    double cw2 = R_w.m[6] * cx + R_w.m[7] * cy + R_w.m[8] * cz + t_w.v[2];
    c_W_out[3 * i + 0] = cw0;
    c_W_out[3 * i + 1] = cw1;
    c_W_out[3 * i + 2] = cw2;
    local[0] += m * cw0;
    local[1] += m * cw1;
    local[2] += m * cw2;
  }

  block_reduce_atomic_add<3>(local, sm, sum_mc_W_3);
}

// Pass 2: d = c_W - r; emit d to scratch and accumulate the 18 doubles needed
// to assemble (LHS, RHS, v0) on host.
//
//   acc[ 0.. 8] : S = sum d[i] * (m * d[i])^T  (row-major 3x3)
//   acc[ 9..11] : L (cross-term L of breakage.cppm fit_twist)
//   acc[12..14] : sum(m * w)   -> regularization base
//   acc[15..17] : sum(m * v)   -> v0
__global__ void twist_pass2_kernel(int N, const double *c_W,
                                   const double *mass, const double *v,
                                   const double *w, Vec3 r, double *d_out,
                                   double *acc_18) {
  __shared__ double sm[kBlock];
  double local[18];
  for (int k = 0; k < 18; ++k) {
    local[k] = 0.0;
  }

  for (int i = blockIdx.x * kBlock + threadIdx.x; i < N;
       i += kBlock * gridDim.x) {
    double m = mass[i];
    double dx = c_W[3 * i + 0] - r.v[0];
    double dy = c_W[3 * i + 1] - r.v[1];
    double dz = c_W[3 * i + 2] - r.v[2];
    d_out[3 * i + 0] = dx;
    d_out[3 * i + 1] = dy;
    d_out[3 * i + 2] = dz;

    double dwx = m * dx, dwy = m * dy, dwz = m * dz;
    double vx = v[3 * i + 0], vy = v[3 * i + 1], vz = v[3 * i + 2];
    double wx = w[3 * i + 0], wy = w[3 * i + 1], wz = w[3 * i + 2];

    // S(r,c) = sum d[i,r] * d_w[i,c], row-major.
    local[0] += dx * dwx;
    local[1] += dx * dwy;
    local[2] += dx * dwz;
    local[3] += dy * dwx;
    local[4] += dy * dwy;
    local[5] += dy * dwz;
    local[6] += dz * dwx;
    local[7] += dz * dwy;
    local[8] += dz * dwz;

    // L (cross-term, mirrors breakage.cppm: dot(d_w.col(j), v.col(k)) ...)
    local[9] += dwy * vz - dwz * vy;
    local[10] += dwz * vx - dwx * vz;
    local[11] += dwx * vy - dwy * vx;

    // sum(m*w), sum(m*v)
    local[12] += m * wx;
    local[13] += m * wy;
    local[14] += m * wz;
    local[15] += m * vx;
    local[16] += m * vy;
    local[17] += m * vz;
  }

  block_reduce_atomic_add<18>(local, sm, acc_18);
}

// Pass 3: v_W[i] = w0 cross d[i] + v0.
//
// Original CPU code: `d.rowwise().cross(-w0) + v0.transpose()` --- since
// d × (-w0) = -(d × w0) = w0 × d, we reorder the cross to remove the negation.
__global__ void twist_pass3_kernel(int N, const double *d, Vec3 w0, Vec3 v0,
                                   double *v_W) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < N;
       i += gridDim.x * blockDim.x) {
    double dx = d[3 * i + 0];
    double dy = d[3 * i + 1];
    double dz = d[3 * i + 2];
    v_W[3 * i + 0] = w0.v[1] * dz - w0.v[2] * dy + v0.v[0];
    v_W[3 * i + 1] = w0.v[2] * dx - w0.v[0] * dz + v0.v[1];
    v_W[3 * i + 2] = w0.v[0] * dy - w0.v[1] * dx + v0.v[2];
  }
}

// ---------------------------------------------------------------------------
// compute_L kernel.
//
// L[i] = R_w * (I_CC[i] * w_CC), where I_CC[i] is symmetric. We pass R_w and
// w_CC by value (precomputed on host). One thread per part.
// ---------------------------------------------------------------------------

__global__ void compute_L_kernel(int N, const double *I_CC, Mat3 R_w,
                                 Vec3 w_CC, double *L_d) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < N;
       i += gridDim.x * blockDim.x) {
    // tmp = (sum_r w_CC[r] * I_CC_row(r)) -- row vector with components
    //   tmp[k] = sum_r w_CC[r] * I_CC[i, 3*r + k].
    // I_CC[i] is symmetric so this equals (I_CC[i] * w_CC).
    double tmp[3];
    for (int k = 0; k < 3; ++k) {
      tmp[k] = w_CC.v[0] * I_CC[9 * i + 0 + k] +
               w_CC.v[1] * I_CC[9 * i + 3 + k] +
               w_CC.v[2] * I_CC[9 * i + 6 + k];
    }
    // L[i, j] = (tmp_row * R_w^T)[j] = sum_k R_w[j, k] * tmp[k].
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += R_w.m[3 * j + k] * tmp[k];
      }
      L_d[3 * i + j] = s;
    }
  }
}

// ---------------------------------------------------------------------------
// assemble_b kernel.
//
// b_mat (N x 6) layout per part i:
//   linear  [j] = (m*(v_curr - v_prev) - J)[i] * Pi^T  / dt
//   angular [j] = (L_curr - L_prev - H)[i] * Pi^T      / (dt * L0)   (torque-scaled)
// ---------------------------------------------------------------------------

__global__ void assemble_b_kernel(int N, const double *v_W_curr,
                                  const double *v_W_prev,
                                  const double *L_curr, const double *L_prev,
                                  const double *J, const double *H,
                                  const double *mass, Mat3 Pi, double inv_dt,
                                  double inv_dt_L0, double *b) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < N;
       i += gridDim.x * blockDim.x) {
    double m = mass[i];
    double pre_lin[3];
    double pre_ang[3];
    for (int k = 0; k < 3; ++k) {
      pre_lin[k] =
          m * (v_W_curr[3 * i + k] - v_W_prev[3 * i + k]) - J[3 * i + k];
      pre_ang[k] = L_curr[3 * i + k] - L_prev[3 * i + k] - H[3 * i + k];
    }
    // out_row[j] = sum_k pre[k] * Pi^T[k, j] = sum_k Pi[j, k] * pre[k].
    for (int j = 0; j < 3; ++j) {
      double sl = 0.0;
      double sa = 0.0;
      for (int k = 0; k < 3; ++k) {
        double pijk = Pi.m[3 * j + k];
        sl += pijk * pre_lin[k];
        sa += pijk * pre_ang[k];
      }
      b[6 * i + j] = sl * inv_dt;
      b[6 * i + 3 + j] = sa * inv_dt_L0;
    }
  }
}

// ---------------------------------------------------------------------------
// postprocess kernels: un-whitening and per-clutch utilization.
// ---------------------------------------------------------------------------

// One thread per clutch. Three 2x1 multiplications by Wk per clutch:
//   x_unwhiten[j0+1:j0+3] = Wk * x[j0+1:j0+3]
//   x_unwhiten[j0+4:j0+6] = Wk * x[j0+4:j0+6]
//   x_unwhiten[j0+7:j0+9] = Wk * x[j0+7:j0+9]
// All other positions are direct copies done by the preceding cudaMemcpy.
__global__ void unwhiten_kernel(int ncv, int K, const double *clutch_whiten,
                                const double *x, double *x_unwhiten) {
  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= K) {
    return;
  }
  int j0 = ncv + 9 * k;
  double w00 = clutch_whiten[4 * k + 0];
  double w01 = clutch_whiten[4 * k + 1];
  double w10 = clutch_whiten[4 * k + 2];
  double w11 = clutch_whiten[4 * k + 3];

  const int offsets[3] = {1, 4, 7};
  for (int t = 0; t < 3; ++t) {
    int p = j0 + offsets[t];
    double v0 = x[p];
    double v1 = x[p + 1];
    x_unwhiten[p] = w00 * v0 + w01 * v1;
    x_unwhiten[p + 1] = w10 * v0 + w11 * v1;
  }
}

// One thread per capacity row; atomic-max into the per-clutch utilization.
__global__ void utilization_kernel(int ncv, int ncap,
                                   const int *clutch_indices,
                                   const double *capacities,
                                   const double *x_unwhiten,
                                   double *utilization) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= ncap) {
    return;
  }
  int clutch_index = clutch_indices[idx];
  double c[9];
  for (int k = 0; k < 9; ++k) {
    c[k] = capacities[9 * idx + k];
  }
  int xoff = ncv + 9 * clutch_index;
  double frc_used = c[0] * x_unwhiten[xoff + 0] +
                    c[1] * x_unwhiten[xoff + 1] +
                    c[2] * x_unwhiten[xoff + 2];
  double cap_dot = 0.0;
  for (int k = 0; k < 6; ++k) {
    cap_dot += c[3 + k] * x_unwhiten[xoff + 3 + k];
  }
  double frc_cap = 1.0 - cap_dot;
  double u_fp;
  if (frc_cap > 0.0) {
    u_fp = frc_used / frc_cap;
    if (u_fp < 0.0) {
      u_fp = 0.0;
    }
  } else {
    u_fp = 1e6;
  }
  atomic_max_double(&utilization[clutch_index], u_fp);
}

// ---------------------------------------------------------------------------
// Host-only SO(3) helpers used by compute_Pi_host (line-for-line copies of
// breakage.cppm:123-164 with double[] interfaces).
// ---------------------------------------------------------------------------

void so3_log_from_unit_quat_host(const double q_in[4], double phi[3]) {
  double q[4];
  for (int i = 0; i < 4; ++i) {
    q[i] = q_in[i];
  }
  cuda_la::quat_xyzw_normalize(q);
  // Pick the shortest representation (q.w >= 0) except for the pi case.
  if (q[3] < 0.0) {
    q[0] = -q[0];
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
  }
  double v[3] = {q[0], q[1], q[2]};
  double s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); // sin(theta/2)
  double w = q[3];                                               // cos(theta/2)

  constexpr double kSmallS = 5e-5;
  double theta_over_s;
  if (s < kSmallS) {
    // Series: theta/s = 2 + s^2/3 + 3 s^4/20 + O(s^6).
    double s2 = s * s;
    double s4 = s2 * s2;
    theta_over_s = 2.0 + (s2 / 3.0) + (3.0 * s4 / 20.0);
  } else {
    double theta = 2.0 * std::atan2(s, w);
    theta_over_s = theta / s;
  }
  phi[0] = theta_over_s * v[0];
  phi[1] = theta_over_s * v[1];
  phi[2] = theta_over_s * v[2];
}

void so3_jacobian_inv_host(const double phi[3], double J_inv[9]) {
  double theta2 = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
  constexpr double kSmallTheta = 1e-4;
  double c;
  if (theta2 < kSmallTheta * kSmallTheta) {
    // Series: c(theta) = 1/12 + theta^2/720 + theta^4/30240 + O(theta^6).
    double t2 = theta2;
    double t4 = t2 * t2;
    c = (1.0 / 12.0) + (t2 / 720.0) + (t4 / 30240.0);
  } else {
    double theta = std::sqrt(theta2);
    double half = 0.5 * theta;
    double cot_half = std::cos(half) / std::sin(half);
    c = (1.0 / theta2) - (cot_half / (2.0 * theta));
  }
  double px = phi[0], py = phi[1], pz = phi[2];
  // Phi = [ 0, -z,  y;
  //         z,  0, -x;
  //        -y,  x,  0 ]
  double Phi[9] = {0.0, -pz, py, pz, 0.0, -px, -py, px, 0.0};
  double P2[9];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) {
        sum += Phi[3 * i + k] * Phi[3 * k + j];
      }
      P2[3 * i + j] = sum;
    }
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double v = ((i == j) ? 1.0 : 0.0) - 0.5 * Phi[3 * i + j] +
                 c * P2[3 * i + j];
      J_inv[3 * i + j] = v;
    }
  }
}

} // namespace

// ---------------------------------------------------------------------------
// Host launchers.
// ---------------------------------------------------------------------------

void fit_se3(int N, const double *q0_d, const double *t0_d, const double *qx_d,
             const double *tx_d, const double *mass_d, double total_mass,
             double lambda_R, double q_out[4], double t_out[3],
             cudaStream_t stream) {
  if (N == 0) {
    q_out[0] = 0.0;
    q_out[1] = 0.0;
    q_out[2] = 0.0;
    q_out[3] = 1.0;
    t_out[0] = 0.0;
    t_out[1] = 0.0;
    t_out[2] = 0.0;
    return;
  }
  if (!(total_mass > 0.0)) {
    throw std::invalid_argument("fit_se3: total_mass must be positive");
  }

  int grid = grid_for(N);

  // Pass 1: 6 centroid sums + 9 K sums = 15 doubles.
  DeviceBuffer<double> p1(15);
  BREAKAGE_CUDA_CHECK(
      cudaMemsetAsync(p1.data(), 0, 15 * sizeof(double), stream));
  se3_pass1_kernel<<<grid, kBlock, 0, stream>>>(N, q0_d, t0_d, qx_d, tx_d,
                                                mass_d, p1.data());

  double p1_h[15];
  BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(p1_h, p1.data(), 15 * sizeof(double),
                                      cudaMemcpyDeviceToHost, stream));
  BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));

  Vec3 t0_bar;
  Vec3 tx_bar;
  for (int k = 0; k < 3; ++k) {
    t0_bar.v[k] = p1_h[k] / total_mass;
    tx_bar.v[k] = p1_h[3 + k] / total_mass;
  }
  double K[9];
  for (int k = 0; k < 9; ++k) {
    K[k] = p1_h[6 + k];
  }

  // Pass 2: H = sum_i m_i * (t0_i - t0_bar) * (tx_i - tx_bar)^T.
  DeviceBuffer<double> p2(9);
  BREAKAGE_CUDA_CHECK(
      cudaMemsetAsync(p2.data(), 0, 9 * sizeof(double), stream));
  se3_pass2_kernel<<<grid, kBlock, 0, stream>>>(N, t0_d, tx_d, mass_d, t0_bar,
                                                tx_bar, p2.data());

  double H[9];
  BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(H, p2.data(), 9 * sizeof(double),
                                      cudaMemcpyDeviceToHost, stream));
  BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));

  // Host: Procrustes on S = H + lambda_R * K, then derive (q, t).
  double S[9];
  for (int k = 0; k < 9; ++k) {
    S[k] = H[k] + lambda_R * K[k];
  }
  double R[9];
  cuda_la::svd_procrustes_3x3(S, R);

  // t = tx_bar - R * t0_bar
  for (int i = 0; i < 3; ++i) {
    double s = tx_bar.v[i];
    for (int j = 0; j < 3; ++j) {
      s -= R[3 * i + j] * t0_bar.v[j];
    }
    t_out[i] = s;
  }
  cuda_la::R_to_quat_xyzw(R, q_out);
  cuda_la::quat_xyzw_normalize(q_out);
}

void fit_twist(int N, const double *w_d, const double *v_d,
               const double *c_CC_d, const double q_W_CC[4],
               const double t_W_CC[3], const double *mass_d, double total_mass,
               double lambda_w, double w0_out[3], double v0_out[3],
               double *v_W_d, cudaStream_t stream) {
  if (N == 0) {
    for (int k = 0; k < 3; ++k) {
      w0_out[k] = 0.0;
      v0_out[k] = 0.0;
    }
    return;
  }
  if (!(total_mass > 0.0)) {
    throw std::invalid_argument("fit_twist: total_mass must be positive");
  }

  int grid = grid_for(N);

  Mat3 R_w;
  cuda_la::quat_xyzw_to_R(q_W_CC, R_w.m);
  Vec3 t_w;
  t_w.v[0] = t_W_CC[0];
  t_w.v[1] = t_W_CC[1];
  t_w.v[2] = t_W_CC[2];

  // Scratch: c_W and d both (N x 3) row-major.
  DeviceBuffer<double> c_W_buf(static_cast<std::size_t>(3) * N);
  DeviceBuffer<double> d_buf(static_cast<std::size_t>(3) * N);
  DeviceBuffer<double> p1(3);
  DeviceBuffer<double> p2(18);
  BREAKAGE_CUDA_CHECK(
      cudaMemsetAsync(p1.data(), 0, 3 * sizeof(double), stream));
  BREAKAGE_CUDA_CHECK(
      cudaMemsetAsync(p2.data(), 0, 18 * sizeof(double), stream));

  twist_pass1_kernel<<<grid, kBlock, 0, stream>>>(
      N, c_CC_d, mass_d, R_w, t_w, c_W_buf.data(), p1.data());

  double sum_mc[3];
  BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(sum_mc, p1.data(), 3 * sizeof(double),
                                      cudaMemcpyDeviceToHost, stream));
  BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));

  Vec3 r;
  for (int k = 0; k < 3; ++k) {
    r.v[k] = sum_mc[k] / total_mass;
  }

  twist_pass2_kernel<<<grid, kBlock, 0, stream>>>(
      N, c_W_buf.data(), mass_d, v_d, w_d, r, d_buf.data(), p2.data());

  double acc[18];
  BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(acc, p2.data(), 18 * sizeof(double),
                                      cudaMemcpyDeviceToHost, stream));
  BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));

  double S[9];
  for (int k = 0; k < 9; ++k) {
    S[k] = acc[k];
  }
  double L[3] = {acc[9], acc[10], acc[11]};
  double mw[3] = {acc[12], acc[13], acc[14]};
  double mv[3] = {acc[15], acc[16], acc[17]};

  // LHS = I * (trace(S) + lambda_w * total_mass) - S
  double trace = S[0] + S[4] + S[8];
  double k_const = trace + lambda_w * total_mass;
  double LHS[9] = {k_const - S[0], -S[1],          -S[2],          //
                   -S[3],          k_const - S[4], -S[5],          //
                   -S[6],          -S[7],          k_const - S[8]};
  double RHS[3] = {L[0] + lambda_w * mw[0], L[1] + lambda_w * mw[1],
                   L[2] + lambda_w * mw[2]};

  cuda_la::spd_solve_3x3(LHS, RHS, w0_out);
  v0_out[0] = mv[0] / total_mass;
  v0_out[1] = mv[1] / total_mass;
  v0_out[2] = mv[2] / total_mass;

  Vec3 w0_v = {{w0_out[0], w0_out[1], w0_out[2]}};
  Vec3 v0_v = {{v0_out[0], v0_out[1], v0_out[2]}};
  twist_pass3_kernel<<<grid, kBlock, 0, stream>>>(N, d_buf.data(), w0_v, v0_v,
                                                  v_W_d);
  // Sync so the scratch DeviceBuffers can be safely freed at scope exit.
  BREAKAGE_CUDA_CHECK(cudaStreamSynchronize(stream));
}

void compute_Pi_host(const double qm[4], const double qp[4],
                     double Pi_out[9]) {
  // Pi = J_inv(log(R_m^T R_p)) * R_m^T
  double qm_conj[4];
  cuda_la::quat_xyzw_conjugate(qm, qm_conj);
  double q_rel[4];
  cuda_la::quat_xyzw_mul(qm_conj, qp, q_rel);
  double phi[3];
  so3_log_from_unit_quat_host(q_rel, phi);
  double J_inv[9];
  so3_jacobian_inv_host(phi, J_inv);
  double R_m[9];
  cuda_la::quat_xyzw_to_R(qm, R_m);
  // Pi[i, j] = sum_k J_inv[i, k] * R_m^T[k, j] = sum_k J_inv[i, k] * R_m[j, k]
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += J_inv[3 * i + k] * R_m[3 * j + k];
      }
      Pi_out[3 * i + j] = s;
    }
  }
}

void compute_L(int N, const double *I_CC_d, const double q_W_CC[4],
               const double w0[3], double *L_d, cudaStream_t stream) {
  if (N == 0) {
    return;
  }
  // Precompute R_w (row-major) and w_CC = R_w^T * w0 on the host so the
  // kernel parameters stay tiny (12 doubles by-value).
  Mat3 R_w;
  cuda_la::quat_xyzw_to_R(q_W_CC, R_w.m);
  Vec3 w_CC;
  for (int i = 0; i < 3; ++i) {
    // R_w^T[i, k] = R_w[k, i]
    w_CC.v[i] = R_w.m[3 * 0 + i] * w0[0] + R_w.m[3 * 1 + i] * w0[1] +
                R_w.m[3 * 2 + i] * w0[2];
  }
  int grid = grid_for(N);
  compute_L_kernel<<<grid, kBlock, 0, stream>>>(N, I_CC_d, R_w, w_CC, L_d);
}

void assemble_b(int N, const double *v_W_curr_d, const double *v_W_prev_d,
                const double *L_curr_d, const double *L_prev_d,
                const double *J_d, const double *H_d, const double *mass_d,
                const double Pi[9], double dt, double L0, double *b_d,
                cudaStream_t stream) {
  if (N == 0) {
    return;
  }
  if (!(dt > 0.0)) {
    throw std::invalid_argument("assemble_b: dt must be positive");
  }
  if (!(L0 > 0.0)) {
    throw std::invalid_argument("assemble_b: L0 must be positive");
  }
  Mat3 Pi_v;
  for (int k = 0; k < 9; ++k) {
    Pi_v.m[k] = Pi[k];
  }
  double inv_dt = 1.0 / dt;
  // EnableTorqueScaling = true (matches breakage.cppm).
  double inv_dt_L0 = inv_dt / L0;
  int grid = grid_for(N);
  assemble_b_kernel<<<grid, kBlock, 0, stream>>>(
      N, v_W_curr_d, v_W_prev_d, L_curr_d, L_prev_d, J_d, H_d, mass_d, Pi_v,
      inv_dt, inv_dt_L0, b_d);
}

void postprocess(int ncv, int K, int ncap, const double *x_d,
                 const int *capacity_clutch_indices_d,
                 const double *capacities_d, const double *clutch_whiten_d,
                 double *x_unwhiten_d, double *utilization_d,
                 cudaStream_t stream) {
  // num_vars = ncv + 9 * K  (cf. BreakageSystem::num_vars_).
  int num_vars = ncv + 9 * K;
  if (num_vars > 0) {
    // Step 0: copy x verbatim. The unwhiten kernel below will overwrite the
    // 6 positions per clutch that need the Wk transform; everything else stays.
    BREAKAGE_CUDA_CHECK(cudaMemcpyAsync(
        x_unwhiten_d, x_d, static_cast<std::size_t>(num_vars) * sizeof(double),
        cudaMemcpyDeviceToDevice, stream));
  }
  if (K > 0) {
    // Initialize utilization to -1 before any atomicMax (matches CPU baseline).
    cuda_la::fill(K, -1.0, utilization_d, stream);
    int bs = 64;
    int grid = (K + bs - 1) / bs;
    unwhiten_kernel<<<grid, bs, 0, stream>>>(ncv, K, clutch_whiten_d, x_d,
                                             x_unwhiten_d);
  }
  if (ncap > 0) {
    int bs = 128;
    int grid = (ncap + bs - 1) / bs;
    utilization_kernel<<<grid, bs, 0, stream>>>(
        ncv, ncap, capacity_clutch_indices_d, capacities_d, x_unwhiten_d,
        utilization_d);
  }
}

} // namespace breakage_cuda::cuda_kernels
