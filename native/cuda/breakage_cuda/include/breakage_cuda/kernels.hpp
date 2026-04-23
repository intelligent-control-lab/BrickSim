// SPDX-License-Identifier: MIT
//
// Host-side launchers for the dense per-step CUDA kernels. Each one mirrors
// a function in breakage.cppm; the bodies live in src/kernels.cu and
// src/linalg.cu.

#pragma once

#include "breakage_cuda/device.hpp"

namespace breakage_cuda::cuda_kernels {

// ---------------------------------------------------------------------------
// fit_se3 (breakage.cppm:58-88).
//
// Inputs:
//   q0, t0  -- reference orientations / COMs in CC frame  (N x 4 / N x 3)
//   qx, tx  -- current orientations / COMs                (N x 4 / N x 3)
//   mass    -- per-part mass                              (N,)
//   total_mass, lambda_R
//
// Output (host scalars + 3x3 R + Vector3 t copied back):
//   q_out   -- world<-CC quaternion, xyzw                 (4,)
//   t_out   -- world<-CC translation                      (3,)
//
// Uses an internal scratch buffer for the partial 3x3 reductions.
// ---------------------------------------------------------------------------

void fit_se3(int N, const double *q0_d, const double *t0_d, const double *qx_d,
             const double *tx_d, const double *mass_d, double total_mass,
             double lambda_R, double q_out[4], double t_out[3],
             cudaStream_t stream = 0);

// ---------------------------------------------------------------------------
// fit_twist (breakage.cppm:96-121).
//
// Inputs:
//   w, v          -- per-part world-frame angular / linear velocities (N x 3)
//   c_CC          -- COM positions in CC frame                        (N x 3)
//   q_W_CC, t_W_CC -- world<-CC pose (xyzw quat + translation)
//   mass          -- (N,)
//   total_mass, lambda_w
//
// Outputs (host):
//   w0_out -- aggregate angular velocity (3,)
//   v0_out -- aggregate linear velocity  (3,)
//
// Outputs (device):
//   v_W_d -- per-part world-frame linear velocity reconstructed from twist
//            (N x 3 row-major). Caller pre-allocates.
// ---------------------------------------------------------------------------

void fit_twist(int N, const double *w_d, const double *v_d, const double *c_CC_d,
               const double q_W_CC[4], const double t_W_CC[3],
               const double *mass_d, double total_mass, double lambda_w,
               double w0_out[3], double v0_out[3], double *v_W_d,
               cudaStream_t stream = 0);

// ---------------------------------------------------------------------------
// compute_Pi (breakage.cppm:166-170). 3x3 output, host-side scalar work.
// ---------------------------------------------------------------------------

void compute_Pi_host(const double qm[4], const double qp[4], double Pi_out[9]);

// ---------------------------------------------------------------------------
// compute_L (breakage.cppm:173-180). One row per part on the GPU.
//
// Inputs:
//   I_CC_d -- (N, 9) row-major
//   q_W_CC -- (4,) xyzw
//   w0     -- (3,)
//
// Output:
//   L_d -- (N, 3) row-major. Caller pre-allocates.
// ---------------------------------------------------------------------------

void compute_L(int N, const double *I_CC_d, const double q_W_CC[4],
               const double w0[3], double *L_d, cudaStream_t stream = 0);

// ---------------------------------------------------------------------------
// assemble_b (breakage.cppm:1018-1035). One row (6 doubles) per part.
//
// Inputs (device):
//   v_W_curr_d, v_W_prev_d, L_curr_d, L_prev_d -- (N, 3)
//   J_d, H_d -- (N, 3)
//   mass_d -- (N,)
//
// Inputs (host):
//   Pi -- 3x3 row-major
//   dt, L0
//
// Output (device):
//   b_d -- (6 * N,) row-major-as-(N, 6). Caller pre-allocates.
// ---------------------------------------------------------------------------

void assemble_b(int N, const double *v_W_curr_d, const double *v_W_prev_d,
                const double *L_curr_d, const double *L_prev_d,
                const double *J_d, const double *H_d, const double *mass_d,
                const double Pi[9], double dt, double L0, double *b_d,
                cudaStream_t stream = 0);

// ---------------------------------------------------------------------------
// Postprocess: undo Wk whitening on the optimization solution and compute
// per-clutch utilization (breakage.cppm:1061-1097).
//
// Inputs (device):
//   x_d              -- (num_vars,) the OSQP optimization solution (whitened)
//   capacity_clutch_indices_d -- (ncap,) int
//   capacities_d     -- (ncap, 9) row-major
//   clutch_whiten_d  -- (K, 4) row-major flatten of 2x2
//
// Inputs (host):
//   ncv, K
//
// Outputs (device):
//   x_unwhiten_d  -- (num_vars,) the un-whitened x; caller pre-allocates.
//   utilization_d -- (K,) initialized to -1.0 here; caller pre-allocates.
// ---------------------------------------------------------------------------

void postprocess(int ncv, int K, int ncap, const double *x_d,
                 const int *capacity_clutch_indices_d,
                 const double *capacities_d, const double *clutch_whiten_d,
                 double *x_unwhiten_d, double *utilization_d,
                 cudaStream_t stream = 0);

} // namespace breakage_cuda::cuda_kernels
