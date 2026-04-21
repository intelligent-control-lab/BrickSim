// SPDX-License-Identifier: MIT
//
// End-to-end pipeline test: drives the OSQP-backed CPU baseline
// (`ref_cpu::solve_full`) and the hand-written CUDA pipeline
// (`cuda_qp::Pipeline`) on the same input and checks that they agree on the
// un-whitened solution `x` and the per-clutch utilization vector.
//
// Two modes:
//   1. JSON-driven: argv[1] is a path to a debug-dump JSON produced by
//      bricksim::BreakageChecker (i.e. one of the dumps that
//      `breakage_cuda::load_debug_dump` accepts).
//   2. Synthetic fallback (no args): builds the smallest non-trivial
//      HostSystem (N=2 parts, K=1 clutch, ncv=0) by hand. The matrices follow
//      the `BreakageSystem::check_shape` invariants from breakage.cppm:
//      `num_vars = ncv + 9*K`, `num_eq = 6*N`, V is (mh, K), etc.
//
// Tolerance: relative inf-norm error <= 1e-2. Both solvers are independent
// ADMM implementations so bit-exact agreement is not expected; we only check
// that they land in the same neighbourhood.

#include "breakage_cuda/qp_solver.hpp"
#include "breakage_cuda/ref_cpu.hpp"
#include "breakage_cuda/system.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SparseCore>

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace breakage_cuda;
using Trip = Eigen::Triplet<double>;

// ---------------------------------------------------------------------------
// Synthetic builder.
//
// Layout: 2 parts arranged on the x-axis at +/-0.5 m, joined by 1 clutch
// (so num_clutches = 1, num_contact_vertices = 0, num_vars = 9). Each per-
// part 3x3 inertia is the identity. Q is a small positive-definite diagonal
// (matches the contact regularization role from breakage.cppm). A is set up
// so that the first three "axial" variables of the clutch couple the two
// parts' linear residuals with opposite signs (Newton's third law for the
// linear part); the angular block of A is zero, so the angular components of
// b appear as projection-stage slack but the rest of the QP is well-posed.
// G enforces a non-negativity on the axial variable; H + V give two friction-
// cone-like rows so the relaxation stage has something to do.
// ---------------------------------------------------------------------------
HostSystem make_synthetic_system() {
  HostSystem sys;
  const int N = 2;
  const int K = 1;

  sys.num_parts = N;
  sys.num_clutches = K;
  sys.num_contact_vertices = 0;
  sys.num_vars = 9 * K;
  sys.num_eq = 6 * N;
  sys.num_ineq = 1;
  sys.num_relaxed_ineq = 2;
  sys.total_mass = 2.0;
  sys.L0 = 1.0;

  sys.mass = VectorXd::Constant(N, 1.0);

  sys.q_CC.resize(N, 4);
  sys.q_CC.row(0) << 0.0, 0.0, 0.0, 1.0;
  sys.q_CC.row(1) << 0.0, 0.0, 0.0, 1.0;

  sys.c_CC.resize(N, 3);
  sys.c_CC.row(0) << -0.5, 0.0, 0.0;
  sys.c_CC.row(1) << 0.5, 0.0, 0.0;

  sys.I_CC.resize(N, 9);
  for (int i = 0; i < N; ++i) {
    sys.I_CC.row(i) << 1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0;
  }

  // Q: 0.1 * I_9. Small positive definite; mirrors ContactRegularization on
  // the diagonal in breakage.cppm.
  std::vector<Trip> Qt;
  Qt.reserve(9);
  for (int i = 0; i < 9; ++i) Qt.emplace_back(i, i, 0.1);
  sys.Q.resize(9, 9);
  sys.Q.setFromTriplets(Qt.begin(), Qt.end());
  sys.Q.makeCompressed();

  // A: 12 x 9.
  // Linear coupling: -I_3 on cols 0..2 of A_i (rows 0..2),
  //                  +I_3 on cols 0..2 of A_j (rows 6..8).
  // Angular rows (3..5 and 9..11) are zero - the angular component of b
  // becomes projection-stage slack on both backends.
  std::vector<Trip> At;
  At.reserve(6);
  for (int axis = 0; axis < 3; ++axis) {
    At.emplace_back(axis, axis, -1.0);
    At.emplace_back(6 + axis, axis, 1.0);
  }
  sys.A.resize(12, 9);
  sys.A.setFromTriplets(At.begin(), At.end());
  sys.A.makeCompressed();

  // G: x[0] >= 0 (axial force is compressive).
  std::vector<Trip> Gt;
  Gt.emplace_back(0, 0, 1.0);
  sys.G.resize(1, 9);
  sys.G.setFromTriplets(Gt.begin(), Gt.end());
  sys.G.makeCompressed();

  // H: 2 friction-cone rows of small magnitude so the limit stays slack at
  // the optimum (we want to compare unconstrained-ish behaviour, not test
  // who clamps first).
  std::vector<Trip> Ht;
  Ht.emplace_back(0, 0, -0.1);
  Ht.emplace_back(0, 1, 0.1);
  Ht.emplace_back(1, 0, -0.1);
  Ht.emplace_back(1, 1, -0.1);
  sys.H.resize(2, 9);
  sys.H.setFromTriplets(Ht.begin(), Ht.end());
  sys.H.makeCompressed();

  // V: 2 x 1, both relaxed-ineq rows belong to clutch 0.
  std::vector<Trip> Vt;
  Vt.emplace_back(0, 0, 1.0);
  Vt.emplace_back(1, 0, 1.0);
  sys.V.resize(2, 1);
  sys.V.setFromTriplets(Vt.begin(), Vt.end());
  sys.V.makeCompressed();

  // Capacity rows: both for clutch 0; structured per breakage.cppm
  // (head<3>() = "force used", tail<6>() = "force capacity offset").
  sys.capacity_clutch_indices = {0, 0};
  sys.capacities.resize(2, 9);
  sys.capacities.row(0) << 1.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
  sys.capacities.row(1) << 1.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Wk = identity (so CPU's column-major Map and GPU's row-major read
  // produce the same matrix; in production both bricksim and breakage_cuda
  // assume Wk is symmetric so the two readouts agree by construction).
  sys.clutch_whiten.resize(1, 4);
  sys.clutch_whiten.row(0) << 1.0, 0.0, 0.0, 1.0;

  return sys;
}

HostInput make_synthetic_input() {
  HostInput in;
  const int N = 2;

  in.dt = 0.01;
  in.w = MatrixX3d::Zero(N, 3);

  // Equal-and-opposite y velocities. After fit_twist this picks up a tiny
  // z-axis rotation (w0_z = -0.01) and produces both linear AND angular
  // contributions to b.
  in.v.resize(N, 3);
  in.v.row(0) << 0.0, 0.05, 0.0;
  in.v.row(1) << 0.0, -0.05, 0.0;

  in.q.resize(N, 4);
  in.q.row(0) << 0.0, 0.0, 0.0, 1.0;
  in.q.row(1) << 0.0, 0.0, 0.0, 1.0;

  // Very small displacement off c_CC; fit_se3 still recovers ~identity.
  in.c.resize(N, 3);
  in.c.row(0) << -0.5, 0.0005, 0.0;
  in.c.row(1) << 0.5, -0.0005, 0.0;

  in.J = MatrixX3d::Zero(N, 3);
  in.H = MatrixX3d::Zero(N, 3);
  return in;
}

HostState make_synthetic_state(int N) {
  HostState st;
  st.q_W_CC_prev = Eigen::Quaterniond::Identity();
  st.v_W_prev = MatrixX3d::Zero(N, 3);
  st.L_prev = MatrixX3d::Zero(N, 3);
  return st;
}

// ---------------------------------------------------------------------------
// Pretty-printing.
// ---------------------------------------------------------------------------
void print_solve_info(const char *label, const SolveInfo &info) {
  std::printf("  %-3s prj iter=%4d res(p,d)=(%.2e,%.2e) %s | "
              "rlx iter=%4d res(p,d)=(%.2e,%.2e) %s | "
              "opt iter=%4d res(p,d)=(%.2e,%.2e) %s\n",
              label,
              info.prj.iter, info.prj.prim_res, info.prj.dual_res,
              info.prj.converged ? "OK  " : "FAIL",
              info.rlx.iter, info.rlx.prim_res, info.rlx.dual_res,
              info.rlx.converged ? "OK  " : "FAIL",
              info.opt.iter, info.opt.prim_res, info.opt.dual_res,
              info.opt.converged ? "OK  " : "FAIL");
}

bool run_compare(const char *label, const HostSystem &sys, const HostInput &in,
                 HostState state_cpu, HostState state_gpu,
                 const Thresholds &thr) {
  std::printf("[%s] num_parts=%d num_clutches=%d num_vars=%d num_eq=%d "
              "num_ineq=%d num_relaxed_ineq=%d\n",
              label, sys.num_parts, sys.num_clutches, sys.num_vars,
              sys.num_eq, sys.num_ineq, sys.num_relaxed_ineq);

  Solution sol_cpu = ref_cpu::solve_full(sys, in, state_cpu, thr);
  std::printf("  cpu converged=%d slack_fraction=%.3e\n",
              static_cast<int>(sol_cpu.info.converged),
              sol_cpu.slack_fraction);
  print_solve_info("cpu", sol_cpu.info);

  cuda_qp::Pipeline pipe;
  pipe.setup(sys, thr);
  Solution sol_gpu = pipe.solve(in, state_gpu);
  std::printf("  gpu converged=%d slack_fraction=%.3e\n",
              static_cast<int>(sol_gpu.info.converged),
              sol_gpu.slack_fraction);
  print_solve_info("gpu", sol_gpu.info);

  bool ok = true;
  if (!sol_cpu.info.converged) {
    std::printf("[FAIL] %s: cpu solver did not converge\n", label);
    ok = false;
  }
  if (!sol_gpu.info.converged) {
    std::printf("[FAIL] %s: gpu solver did not converge\n", label);
    ok = false;
  }
  if (!ok) return false;

  if (sol_cpu.x.size() != sol_gpu.x.size()) {
    std::printf("[FAIL] %s: x size mismatch (cpu=%lld gpu=%lld)\n", label,
                static_cast<long long>(sol_cpu.x.size()),
                static_cast<long long>(sol_gpu.x.size()));
    return false;
  }
  if (sol_cpu.utilization.size() != sol_gpu.utilization.size()) {
    std::printf("[FAIL] %s: utilization size mismatch (cpu=%lld gpu=%lld)\n",
                label,
                static_cast<long long>(sol_cpu.utilization.size()),
                static_cast<long long>(sol_gpu.utilization.size()));
    return false;
  }

  // Floor: 1e-3 absolute. With an x of order ~1, 1e-2 relative ~ 1e-2
  // absolute, which is well above the floor; tiny solutions still get a
  // sensible scale to compare against.
  constexpr double kFloor = 1e-3;
  constexpr double kTol = 1e-2;

  const double err_x = relative_inf_error(sol_gpu.x, sol_cpu.x, kFloor);
  std::printf("  x:           cpu_inf=%.4e gpu_inf=%.4e abs_diff_inf=%.4e "
              "rel_inf_err=%.4e\n",
              sol_cpu.x.cwiseAbs().maxCoeff(),
              sol_gpu.x.cwiseAbs().maxCoeff(),
              (sol_cpu.x - sol_gpu.x).cwiseAbs().maxCoeff(), err_x);
  if (err_x <= kTol) {
    std::printf("[PASS] full_pipeline.x           rel_inf=%.4e <= %.0e\n",
                err_x, kTol);
  } else {
    std::printf("[FAIL] full_pipeline.x           rel_inf=%.4e >  %.0e\n",
                err_x, kTol);
    ok = false;
  }

  const double err_u =
      relative_inf_error(sol_gpu.utilization, sol_cpu.utilization, kFloor);
  std::printf("  utilization: cpu_inf=%.4e gpu_inf=%.4e abs_diff_inf=%.4e "
              "rel_inf_err=%.4e\n",
              sol_cpu.utilization.cwiseAbs().maxCoeff(),
              sol_gpu.utilization.cwiseAbs().maxCoeff(),
              (sol_cpu.utilization - sol_gpu.utilization)
                  .cwiseAbs()
                  .maxCoeff(),
              err_u);
  if (err_u <= kTol) {
    std::printf("[PASS] full_pipeline.utilization rel_inf=%.4e <= %.0e\n",
                err_u, kTol);
  } else {
    std::printf("[FAIL] full_pipeline.utilization rel_inf=%.4e >  %.0e\n",
                err_u, kTol);
    ok = false;
  }

  return ok;
}

bool run_json_mode(const std::string &path) {
  std::printf("[full_pipeline] (JSON mode) loading %s\n", path.c_str());
  DebugDump dump = load_debug_dump(path);
  // Both solvers mutate state; clone so each sees a fresh copy.
  HostState state_cpu = dump.state;
  HostState state_gpu = dump.state;
  return run_compare("json", dump.system, dump.input, std::move(state_cpu),
                     std::move(state_gpu), dump.thresholds);
}

bool run_synthetic_mode() {
  std::printf("[full_pipeline] (synthetic mode)\n");
  HostSystem sys = make_synthetic_system();
  HostInput in = make_synthetic_input();
  HostState state_cpu = make_synthetic_state(sys.num_parts);
  HostState state_gpu = make_synthetic_state(sys.num_parts);
  Thresholds thr;
  thr.enabled = true;
  return run_compare("synthetic", sys, in, std::move(state_cpu),
                     std::move(state_gpu), thr);
}

} // namespace

int main(int argc, char **argv) {
  try {
    bool ok = (argc >= 2) ? run_json_mode(argv[1]) : run_synthetic_mode();
    std::printf("[full_pipeline] %s\n", ok ? "OK" : "FAILED");
    return ok ? 0 : 1;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "[ERROR] full_pipeline: %s\n", e.what());
    return 2;
  }
}
