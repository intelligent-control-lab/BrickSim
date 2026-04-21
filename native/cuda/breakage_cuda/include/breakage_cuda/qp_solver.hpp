// SPDX-License-Identifier: MIT
//
// Hand-written CUDA ADMM solver, structurally aligned with the three-stage
// OSQP wrapper in osqp.cppm (projection -> relaxation -> optimization).
//
// We do NOT replicate every OSQP feature (no polish, no infeasibility
// detection, no scaling/equilibration). The aim is to produce solutions that
// are close enough to OSQP for the same problem so that the breakage system
// behaves consistently.
//
// One "stage" corresponds to one ADMM run with its own KKT factorization.

#pragma once

#include "breakage_cuda/device.hpp"
#include "breakage_cuda/system.hpp"

#include <memory>

namespace breakage_cuda::cuda_qp {

// ---------------------------------------------------------------------------
// Per-stage parameters (mirror OSQP defaults).
// ---------------------------------------------------------------------------

struct Settings {
  double eps_abs{1e-3};
  double eps_rel{1e-3};
  double rho_init{0.1};
  double sigma{1e-6};
  double alpha{1.6};
  int max_iter{4000};
  int check_every{25};
  // Adaptive-rho settings (cf. OSQP's adaptive_rho heuristic).
  bool adaptive_rho{true};
  double adaptive_rho_factor{5.0};
  // Misc
  double infinity{1e30};
};

// ---------------------------------------------------------------------------
// Persistent device-resident state (warm starts, factorizations).
//
// One Stage owns the matrices, KKT factorization, and warm-start buffers for
// a single ADMM run. The Solver owns three of these (prj/rlx/opt).
//
// Stage matrices are constant across solves with the same HostSystem; only
// l/u/q vectors and the warm-start state change between calls.
// ---------------------------------------------------------------------------

class Stage; // pimpl

class Solver {
public:
  Solver();
  ~Solver();

  Solver(const Solver &) = delete;
  Solver &operator=(const Solver &) = delete;
  Solver(Solver &&) noexcept;
  Solver &operator=(Solver &&) noexcept;

  // Build the three stages from a (CPU) HostSystem. The DeviceSystem must
  // already be uploaded; we only need its CSR matrices for SpMV bookkeeping.
  void setup(const HostSystem &host, const DeviceSystem &dev,
             const Settings &settings);

  // Run all three stages on RHS b (host vector, length 6 * num_parts).
  //
  // Outputs:
  //   x_d        -- (num_vars,) device, optimization solution (still whitened)
  //   slack_d    -- (num_eq,)  device, b - A * x_prj
  //   v_relax_d  -- (num_clutches,) device, relaxation slack v
  SolveInfo solve(const VectorXd &b, double *x_d, double *slack_d,
                  double *v_relax_d, cudaStream_t stream = 0);

  // Drop the warm-start state; next solve starts cold.
  void reset_warm_start();

  // Total dims (forwarded from HostSystem).
  int num_vars() const;
  int num_eq() const;
  int num_clutches() const;

private:
  std::unique_ptr<Stage> prj_;
  std::unique_ptr<Stage> rlx_;
  std::unique_ptr<Stage> opt_;
  Settings settings_{};
  int nx_{0};
  int me_{0};
  int mi_{0};
  int mh_{0};
  int nv_{0};
};

// ---------------------------------------------------------------------------
// End-to-end driver mirroring breakage.cppm BreakageChecker::solve, but
// running everything (dense kernels + QP + postprocess) on the GPU.
//
// This is the entry point benchmarks and tests call.
// ---------------------------------------------------------------------------

class Pipeline {
public:
  Pipeline();
  ~Pipeline();

  void setup(const HostSystem &host, const Thresholds &thr,
             const Settings &settings = {});

  Solution solve(const HostInput &in, HostState &state,
                 cudaStream_t stream = 0);

private:
  HostSystem host_;
  Thresholds thr_;
  DeviceSystem dev_;
  Solver qp_;
  // Scratch buffers
  DeviceBuffer<double> v_W_curr_;
  DeviceBuffer<double> L_curr_;
  DeviceBuffer<double> b_d_;
  DeviceBuffer<double> x_d_;
  DeviceBuffer<double> x_unwhiten_d_;
  DeviceBuffer<double> slack_d_;
  DeviceBuffer<double> v_relax_d_;
  DeviceBuffer<double> utilization_d_;
};

} // namespace breakage_cuda::cuda_qp
