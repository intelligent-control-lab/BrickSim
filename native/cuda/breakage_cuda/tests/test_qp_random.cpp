// SPDX-License-Identifier: MIT
//
// Cross-validation test for the CUDA QP solver against the CPU reference
// (real OSQP). Builds a tiny synthetic HostSystem and runs the same RHS
// through both backends, then compares their stage-3 `x` outputs via a
// relative inf-norm error.
//
// Toy HostSystem layout (NOT a physical breakage problem; the QP solvers
// are oblivious to the per-part fields, so we only populate the matrices
// that `Solver::setup` / `InternalQpState::setup_from` actually read):
//
//   num_parts = 1            => num_eq = me = 6
//   num_clutches = 1         => nv = 1
//   num_contact_vertices = 1 => num_vars = nx = 1 + 9*1 = 10  (consistent
//                                                              with
//                                                              breakage.cppm
//                                                              invariant)
//   num_ineq = mi = 2
//   num_relaxed_ineq = mh = 2
//
//   A in R^{6x10}: [ I_6 | 0_{6x4} ]
//                  -> stage 1 finds b_prj = b for any b (range(A) = R^6),
//                     and stages 2/3 force x.head(6) == b exactly.
//   Q in R^{10x10}: I_10 + 0.5 * sum_{i=0..3} (e_i e_{6+i}^T + e_{6+i} e_i^T)
//                  -> SPD (each (i, 6+i) 2x2 block has eigenvalues 0.5, 1.5;
//                     other diagonals are 1.0; condition number 3).
//                  -> stage 3 unconstrained min over y = x.tail(4) is
//                     y* = -0.5 * b.head(4).
//   G in R^{2x10}: rows pick x[6], x[7] (so G x = [x[6]; x[7]] >= -eps).
//   H in R^{2x10}: rows pick x[8], x[9] (so H x = [x[8]; x[9]] <= 1+V*v).
//   V in R^{2x1}:  ones, so the single clutch slack v couples both H rows.
//
// We pick b = [4, 4, -4, -4, 0, 0]:
//   Stage 2 unconstrained optimum is x.tail(4) = 0, v = 0 -> v_relax = 0.
//   Stage 3 unconstrained optimum y* = (-2, -2, 2, 2) violates ALL FOUR
//   inequality constraints, so the constrained minimum hits the box at
//   (y[0], y[1], y[2], y[3]) = (-eps, -eps, 1, 1). All three stages run
//   with non-trivial active sets.
//
// The OSQP baseline and the hand-rolled CUDA ADMM are independent solvers
// with different inner KKT machinery (OSQP factors, CUDA uses CG on the
// reduced normal equations) and different exact convergence criteria, so
// we do NOT expect bit equality. Tolerance: relative inf-norm error
// `||x_cuda - x_cpu||_inf / max(1, ||x_cpu||_inf) <= 1e-2`. Per-stage
// iter / prim_res / dual_res are printed for both backends so we can see
// whether either one stalled.
//
// SOURCE-FILE FIX APPLIED while writing this test:
//   src/qp_solver.cu, Solver::solve, opt-stage upper bound:
//     `u_opt.tail(mh_).setConstant(1.0 - kEpsilon);` was wrong --
//     osqp.cppm (the source of truth) and ref_cpu.cpp both use plain 1.0
//     for the H-row upper bound in stage 3 (the `1 - epsilon` slack
//     belongs to stage 2 only). Changed to `setConstant(1.0)` so the two
//     backends solve the same QP. Without this fix the CUDA opt-stage
//     ceiling sits 1e-4 below OSQP's, which biases every active H-row
//     constraint by epsilon.

#include "breakage_cuda/device.hpp"
#include "breakage_cuda/qp_solver.hpp"
#include "breakage_cuda/ref_cpu.hpp"
#include "breakage_cuda/system.hpp"

#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include <cuda_runtime.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <vector>

using breakage_cuda::DeviceBuffer;
using breakage_cuda::DeviceSystem;
using breakage_cuda::HostState;
using breakage_cuda::HostSystem;
using breakage_cuda::MatrixX3d;
using breakage_cuda::MatrixX4d;
using breakage_cuda::SolveInfo;
using breakage_cuda::SparseMatrixCSC;
using breakage_cuda::StageInfo;
using breakage_cuda::VectorXd;
using breakage_cuda::relative_inf_error;

namespace {

SparseMatrixCSC dense_to_csc(const Eigen::MatrixXd &m) {
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<std::size_t>(m.rows()) *
                static_cast<std::size_t>(m.cols()));
  for (int j = 0; j < m.cols(); ++j) {
    for (int i = 0; i < m.rows(); ++i) {
      const double v = m(i, j);
      if (v != 0.0) {
        trips.emplace_back(i, j, v);
      }
    }
  }
  SparseMatrixCSC sp(m.rows(), m.cols());
  sp.setFromTriplets(trips.begin(), trips.end());
  sp.makeCompressed();
  return sp;
}

HostSystem build_toy_system() {
  HostSystem sys;
  const int N = 1;
  const int K = 1;
  const int ncv = 1;
  const int nx = ncv + 9 * K;
  const int me = 6 * N;
  const int mi = 2;
  const int mh = 2;

  sys.num_parts = N;
  sys.num_clutches = K;
  sys.num_contact_vertices = ncv;
  sys.num_vars = nx;
  sys.num_eq = me;
  sys.num_ineq = mi;
  sys.num_relaxed_ineq = mh;
  sys.total_mass = 1.0;
  sys.L0 = 1.0;

  // Per-part dense fields are unused by the QP solvers (they're only
  // touched by the dense kernels in the full pipeline). Populate them
  // with sane defaults so downstream consumers wouldn't trip on garbage.
  sys.mass = VectorXd::Constant(N, 1.0);
  sys.q_CC = MatrixX4d::Zero(N, 4);
  sys.q_CC(0, 3) = 1.0;
  sys.c_CC = MatrixX3d::Zero(N, 3);
  sys.I_CC.resize(N, 9);
  sys.I_CC.setZero();
  sys.I_CC(0, 0) = 1.0;
  sys.I_CC(0, 4) = 1.0;
  sys.I_CC(0, 8) = 1.0;

  Eigen::MatrixXd Qd = Eigen::MatrixXd::Identity(nx, nx);
  for (int i = 0; i < 4; ++i) {
    Qd(i, 6 + i) = 0.5;
    Qd(6 + i, i) = 0.5;
  }
  sys.Q = dense_to_csc(Qd);

  Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(me, nx);
  for (int i = 0; i < me; ++i) {
    Ad(i, i) = 1.0;
  }
  sys.A = dense_to_csc(Ad);

  Eigen::MatrixXd Gd = Eigen::MatrixXd::Zero(mi, nx);
  Gd(0, 6) = 1.0;
  Gd(1, 7) = 1.0;
  sys.G = dense_to_csc(Gd);

  Eigen::MatrixXd Hd = Eigen::MatrixXd::Zero(mh, nx);
  Hd(0, 8) = 1.0;
  Hd(1, 9) = 1.0;
  sys.H = dense_to_csc(Hd);

  Eigen::MatrixXd Vd = Eigen::MatrixXd::Ones(mh, K);
  sys.V = dense_to_csc(Vd);

  // Friction-cone metadata is only consulted by the postprocess step in
  // the full pipeline; leave empty for the QP-only test.
  sys.capacity_clutch_indices.clear();
  sys.capacities.resize(0, 9);
  sys.clutch_whiten.resize(K, 4);
  sys.clutch_whiten.setZero();
  sys.clutch_whiten(0, 0) = 1.0;
  sys.clutch_whiten(0, 3) = 1.0;

  return sys;
}

void print_stage(const char *name, const StageInfo &cpu, const StageInfo &cuda) {
  std::printf("  %s:\n", name);
  std::printf("    CPU  : converged=%d iter=%4d prim_res=%.3e dual_res=%.3e "
              "rho=%.3e\n",
              cpu.converged ? 1 : 0, cpu.iter, cpu.prim_res, cpu.dual_res,
              cpu.rho);
  std::printf("    CUDA : converged=%d iter=%4d prim_res=%.3e dual_res=%.3e "
              "rho=%.3e\n",
              cuda.converged ? 1 : 0, cuda.iter, cuda.prim_res, cuda.dual_res,
              cuda.rho);
}

}  // namespace

int main() {
  try {
    HostSystem sys = build_toy_system();

    VectorXd b(sys.num_eq);
    b << 4.0, 4.0, -4.0, -4.0, 0.0, 0.0;

    std::printf("=== test_qp_random ===\n");
    std::printf("Toy QP: N=%d K=%d ncv=%d  nx=%d  me=%d  mi=%d  mh=%d  nv=%d\n",
                sys.num_parts, sys.num_clutches, sys.num_contact_vertices,
                sys.num_vars, sys.num_eq, sys.num_ineq, sys.num_relaxed_ineq,
                sys.num_clutches);
    std::printf("RHS b = [%g, %g, %g, %g, %g, %g]\n", b[0], b[1], b[2], b[3],
                b[4], b[5]);

    // -------------------- CPU reference (OSQP) ----------------------------
    HostState cpu_state;
    cpu_state.v_W_prev = MatrixX3d::Zero(sys.num_parts, 3);
    cpu_state.L_prev = MatrixX3d::Zero(sys.num_parts, 3);
    VectorXd x_cpu, slack_cpu, v_relax_cpu;
    SolveInfo cpu_info = breakage_cuda::ref_cpu::solve_qp(
        sys, b, cpu_state, x_cpu, slack_cpu, v_relax_cpu);

    // -------------------- CUDA solver -------------------------------------
    // Solver::setup ignores its DeviceSystem argument (uses HostSystem
    // matrices directly), so a default-constructed one is fine here.
    DeviceSystem dev_dummy{};
    breakage_cuda::cuda_qp::Settings settings;
    breakage_cuda::cuda_qp::Solver cuda_solver;
    cuda_solver.setup(sys, dev_dummy, settings);

    DeviceBuffer<double> x_d(static_cast<std::size_t>(sys.num_vars));
    DeviceBuffer<double> slack_d(static_cast<std::size_t>(sys.num_eq));
    DeviceBuffer<double> v_relax_d(
        static_cast<std::size_t>(sys.num_clutches));

    SolveInfo cuda_info = cuda_solver.solve(b, x_d.data(), slack_d.data(),
                                            v_relax_d.data());

    VectorXd x_cuda(sys.num_vars);
    x_d.copy_to_host(x_cuda.data(), x_cuda.size());

    VectorXd v_relax_cuda(sys.num_clutches);
    if (sys.num_clutches > 0) {
      v_relax_d.copy_to_host(v_relax_cuda.data(), v_relax_cuda.size());
    }

    // -------------------- Per-stage info ----------------------------------
    std::printf("\nPer-stage convergence (CPU = OSQP, CUDA = ADMM+CG):\n");
    print_stage("prj", cpu_info.prj, cuda_info.prj);
    print_stage("rlx", cpu_info.rlx, cuda_info.rlx);
    print_stage("opt", cpu_info.opt, cuda_info.opt);
    std::printf("  v_relax: cpu = %.6e   cuda = %.6e\n", v_relax_cpu[0],
                v_relax_cuda[0]);

    // -------------------- Compare x ---------------------------------------
    const double tol = 1e-2;
    const double err = relative_inf_error(x_cuda, x_cpu, /*floor=*/1.0);
    std::printf("\nx (CPU vs CUDA), relative inf-norm error = %.3e (tol %.0e)\n",
                err, tol);
    std::printf("  i :    x_cpu        x_cuda      |diff|\n");
    for (int i = 0; i < sys.num_vars; ++i) {
      std::printf("  %2d: %12.6f %12.6f %.3e\n", i, x_cpu[i], x_cuda[i],
                  std::abs(x_cpu[i] - x_cuda[i]));
    }

    // -------------------- Pass / fail summary -----------------------------
    bool overall_ok = true;
    auto stage_pass = [&](const char *name, bool cpu_conv, bool cuda_conv) {
      const bool ok = cpu_conv && cuda_conv;
      std::printf("[%s] %s convergence (CPU=%d, CUDA=%d)\n",
                  ok ? "PASS" : "FAIL", name, cpu_conv ? 1 : 0,
                  cuda_conv ? 1 : 0);
      if (!ok) overall_ok = false;
    };
    stage_pass("prj", cpu_info.prj.converged, cuda_info.prj.converged);
    stage_pass("rlx", cpu_info.rlx.converged, cuda_info.rlx.converged);
    stage_pass("opt", cpu_info.opt.converged, cuda_info.opt.converged);

    if (err <= tol) {
      std::printf("[PASS] x agreement (err = %.3e <= %.0e)\n", err, tol);
    } else {
      std::printf("[FAIL] x agreement (err = %.3e > %.0e)\n", err, tol);
      overall_ok = false;
    }

    std::printf("\n%s overall\n", overall_ok ? "[PASS]" : "[FAIL]");
    return overall_ok ? 0 : 1;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "[FAIL] exception: %s\n", e.what());
    return 1;
  }
}
