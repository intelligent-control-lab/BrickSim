// SPDX-License-Identifier: MIT
//
// Wall-clock benchmark for the full breakage pipeline.
//
// Usage: ./breakage_cuda_bench [N_parts]   (default N_parts = 32)
//
// Builds a synthetic chain of N_parts parts joined by N_parts - 1 clutches
// (matching the dimensional layout enforced by `BreakageSystem::check_shape`
// in breakage.cppm) and times N iterations of:
//   * `cuda_qp::Pipeline::solve`  -- GPU end-to-end pipeline
//   * `ref_cpu::solve_full`       -- OSQP-backed CPU baseline (reference)
//
// We use std::chrono::steady_clock and call cudaDeviceSynchronize() before
// taking the GPU stop time so the measurement covers the full kernel
// completion, not just queueing.

#include "breakage_cuda/qp_solver.hpp"
#include "breakage_cuda/ref_cpu.hpp"
#include "breakage_cuda/system.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SparseCore>

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <vector>

namespace {

using namespace breakage_cuda;
using clock_type = std::chrono::steady_clock;
using Trip = Eigen::Triplet<double>;

// ---------------------------------------------------------------------------
// Synthetic builder for `N_parts` parts arranged on the x-axis at unit
// spacing. Each consecutive pair is joined by one clutch, giving K = N - 1.
//
// All matrices are tiled copies of the (N=2, K=1) building block in
// test_full_pipeline.cpp so the per-clutch numerics stay well-conditioned.
//
// Dimensions (per check_shape in breakage.cppm):
//   num_parts          = N
//   num_clutches       = K = N - 1
//   num_contact_vertices = 0
//   num_vars           = 9 * K
//   num_eq             = 6 * N
//   num_ineq           = K          (one axial-non-negativity per clutch)
//   num_relaxed_ineq   = 2 * K      (two friction-cone rows per clutch)
//   capacities         = 2 * K rows
//   clutch_whiten      = K rows
// ---------------------------------------------------------------------------
HostSystem make_synthetic_system_n(int N) {
  if (N < 2) N = 2;
  const int K = N - 1;

  HostSystem sys;
  sys.num_parts = N;
  sys.num_clutches = K;
  sys.num_contact_vertices = 0;
  sys.num_vars = 9 * K;
  sys.num_eq = 6 * N;
  sys.num_ineq = K;
  sys.num_relaxed_ineq = 2 * K;
  sys.total_mass = static_cast<double>(N);
  sys.L0 = 1.0;

  sys.mass = VectorXd::Constant(N, 1.0);

  sys.q_CC.resize(N, 4);
  sys.c_CC.resize(N, 3);
  sys.I_CC.resize(N, 9);
  for (int i = 0; i < N; ++i) {
    sys.q_CC.row(i) << 0.0, 0.0, 0.0, 1.0;
    sys.c_CC.row(i) << static_cast<double>(i) - (N - 1) / 2.0, 0.0, 0.0;
    sys.I_CC.row(i) << 1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0;
  }

  // Q: small positive-definite diagonal on every variable.
  std::vector<Trip> Qt;
  Qt.reserve(static_cast<std::size_t>(9 * K));
  for (int v = 0; v < 9 * K; ++v) Qt.emplace_back(v, v, 0.1);
  sys.Q.resize(9 * K, 9 * K);
  sys.Q.setFromTriplets(Qt.begin(), Qt.end());
  sys.Q.makeCompressed();

  // A: 6N x 9K. For each clutch k between parts k and k+1, the three axial
  // variables x[9k..9k+2] couple part k's linear residual with -I_3 and
  // part k+1's with +I_3. Angular rows of A_i / A_j stay zero - the angular
  // component of b becomes projection-stage slack on both backends.
  std::vector<Trip> At;
  At.reserve(static_cast<std::size_t>(6 * K));
  for (int k = 0; k < K; ++k) {
    const int v0 = 9 * k;
    const int r_i = 6 * k;
    const int r_j = 6 * (k + 1);
    for (int axis = 0; axis < 3; ++axis) {
      At.emplace_back(r_i + axis, v0 + axis, -1.0);
      At.emplace_back(r_j + axis, v0 + axis, 1.0);
    }
  }
  sys.A.resize(6 * N, 9 * K);
  sys.A.setFromTriplets(At.begin(), At.end());
  sys.A.makeCompressed();

  // G: x[9k] >= 0 (axial force compressive) for each clutch.
  std::vector<Trip> Gt;
  Gt.reserve(static_cast<std::size_t>(K));
  for (int k = 0; k < K; ++k) Gt.emplace_back(k, 9 * k, 1.0);
  sys.G.resize(K, 9 * K);
  sys.G.setFromTriplets(Gt.begin(), Gt.end());
  sys.G.makeCompressed();

  // H: 2 friction-cone rows per clutch, low magnitudes so the limits stay
  // slack and we do not measure the constraint-saturation regime.
  std::vector<Trip> Ht;
  Ht.reserve(static_cast<std::size_t>(4 * K));
  for (int k = 0; k < K; ++k) {
    const int v0 = 9 * k;
    const int r0 = 2 * k;
    Ht.emplace_back(r0, v0, -0.1);
    Ht.emplace_back(r0, v0 + 1, 0.1);
    Ht.emplace_back(r0 + 1, v0, -0.1);
    Ht.emplace_back(r0 + 1, v0 + 1, -0.1);
  }
  sys.H.resize(2 * K, 9 * K);
  sys.H.setFromTriplets(Ht.begin(), Ht.end());
  sys.H.makeCompressed();

  // V: 2K x K, identity on each (pair-of-rows, clutch) block.
  std::vector<Trip> Vt;
  Vt.reserve(static_cast<std::size_t>(2 * K));
  for (int k = 0; k < K; ++k) {
    Vt.emplace_back(2 * k, k, 1.0);
    Vt.emplace_back(2 * k + 1, k, 1.0);
  }
  sys.V.resize(2 * K, K);
  sys.V.setFromTriplets(Vt.begin(), Vt.end());
  sys.V.makeCompressed();

  sys.capacity_clutch_indices.resize(static_cast<std::size_t>(2 * K));
  for (int k = 0; k < K; ++k) {
    sys.capacity_clutch_indices[2 * k] = k;
    sys.capacity_clutch_indices[2 * k + 1] = k;
  }
  sys.capacities.resize(2 * K, 9);
  for (int k = 0; k < K; ++k) {
    sys.capacities.row(2 * k) << 1.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
    sys.capacities.row(2 * k + 1) << 1.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
        0.0;
  }

  // Wk = identity per clutch. (The CPU reads it column-major and the GPU
  // reads it row-major; both interpretations match for a symmetric Wk.)
  sys.clutch_whiten.resize(K, 4);
  for (int k = 0; k < K; ++k) {
    sys.clutch_whiten.row(k) << 1.0, 0.0, 0.0, 1.0;
  }

  return sys;
}

HostInput make_synthetic_input_n(int N) {
  HostInput in;
  in.dt = 0.01;
  in.w = MatrixX3d::Zero(N, 3);

  // Alternating-sign y velocities so consecutive parts pull on each other.
  in.v.resize(N, 3);
  for (int i = 0; i < N; ++i) {
    const double s = (i % 2 == 0) ? 0.05 : -0.05;
    in.v.row(i) << 0.0, s, 0.0;
  }

  in.q.resize(N, 4);
  in.c.resize(N, 3);
  for (int i = 0; i < N; ++i) {
    in.q.row(i) << 0.0, 0.0, 0.0, 1.0;
    const double dy = (i % 2 == 0) ? 0.0005 : -0.0005;
    in.c.row(i) << static_cast<double>(i) - (N - 1) / 2.0, dy, 0.0;
  }

  in.J = MatrixX3d::Zero(N, 3);
  in.H = MatrixX3d::Zero(N, 3);
  return in;
}

HostState make_synthetic_state_n(int N) {
  HostState st;
  st.q_W_CC_prev = Eigen::Quaterniond::Identity();
  st.v_W_prev = MatrixX3d::Zero(N, 3);
  st.L_prev = MatrixX3d::Zero(N, 3);
  return st;
}

// ---------------------------------------------------------------------------
// Timing helpers.
// ---------------------------------------------------------------------------
struct Stats {
  double mean_us{};
  double std_us{};
  double min_us{};
  double max_us{};
  int n{};
};

Stats summarize(const std::vector<double> &samples_us) {
  Stats s;
  s.n = static_cast<int>(samples_us.size());
  if (samples_us.empty()) return s;
  double sum = 0.0;
  s.min_us = samples_us[0];
  s.max_us = samples_us[0];
  for (double v : samples_us) {
    sum += v;
    if (v < s.min_us) s.min_us = v;
    if (v > s.max_us) s.max_us = v;
  }
  s.mean_us = sum / static_cast<double>(samples_us.size());
  double var = 0.0;
  for (double v : samples_us) {
    const double d = v - s.mean_us;
    var += d * d;
  }
  if (samples_us.size() > 1) {
    var /= static_cast<double>(samples_us.size() - 1);
  } else {
    var = 0.0;
  }
  s.std_us = std::sqrt(var);
  return s;
}

void print_stats(const char *label, const Stats &s) {
  std::printf("[bench] %s  n=%d  mean=%9.1f us  std=%8.1f us  "
              "min=%9.1f us  max=%9.1f us\n",
              label, s.n, s.mean_us, s.std_us, s.min_us, s.max_us);
}

// ---------------------------------------------------------------------------
// Per-backend benchmark drivers.
//
// Both warm-start across iterations (mirrors how BreakageChecker is used in
// production: state.qp persists between frames). The first few iterations
// are dropped to give the warm-start path a chance to stabilise.
// ---------------------------------------------------------------------------
Stats bench_gpu(const HostSystem &sys, const HostInput &in,
                const Thresholds &thr, int warmup, int measured) {
  const bool tolerate = []() {
    const char *e = std::getenv("BENCH_TOLERATE_NONCONVERGED");
    return e && std::atoi(e) > 0;
  }();

  cuda_qp::Pipeline pipe;
  pipe.setup(sys, thr);

  HostState state = make_synthetic_state_n(sys.num_parts);

  for (int i = 0; i < warmup; ++i) {
    Solution sol = pipe.solve(in, state);
    if (!sol.info.converged) {
      std::fprintf(stderr,
                   "[bench] gpu warmup %d/%d did not converge "
                   "(prj=%d rlx=%d opt=%d, prim=%.2e dual=%.2e)%s\n",
                   i + 1, warmup, static_cast<int>(sol.info.prj.converged),
                   static_cast<int>(sol.info.rlx.converged),
                   static_cast<int>(sol.info.opt.converged),
                   sol.info.opt.prim_res, sol.info.opt.dual_res,
                   tolerate ? " (tolerated)" : "");
      if (!tolerate) std::exit(2);
    }
  }
  BREAKAGE_CUDA_CHECK(cudaDeviceSynchronize());

  std::vector<double> samples_us;
  samples_us.reserve(static_cast<std::size_t>(measured));
  for (int i = 0; i < measured; ++i) {
    const auto t0 = clock_type::now();
    Solution sol = pipe.solve(in, state);
    BREAKAGE_CUDA_CHECK(cudaDeviceSynchronize());
    const auto t1 = clock_type::now();
    if (!sol.info.converged) {
      std::fprintf(stderr,
                   "[bench] gpu measured iter %d/%d did not converge%s\n",
                   i + 1, measured, tolerate ? " (tolerated)" : "");
      if (!tolerate) std::exit(2);
    }
    const double us =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
    samples_us.push_back(us);
  }
  return summarize(samples_us);
}

Stats bench_cpu(const HostSystem &sys, const HostInput &in,
                const Thresholds &thr, int warmup, int measured) {
  HostState state = make_synthetic_state_n(sys.num_parts);

  // BENCH_TOLERATE_NONCONVERGED=1 turns "did not converge" into a warning so
  // we can still get wall-clock numbers at very large N where OSQP/our ADMM
  // would otherwise hit max_iter and abort.
  const bool tolerate = []() {
    const char *e = std::getenv("BENCH_TOLERATE_NONCONVERGED");
    return e && std::atoi(e) > 0;
  }();

  for (int i = 0; i < warmup; ++i) {
    Solution sol = ref_cpu::solve_full(sys, in, state, thr);
    if (!sol.info.converged) {
      std::fprintf(stderr,
                   "[bench] cpu warmup %d/%d did not converge%s\n", i + 1,
                   warmup, tolerate ? " (tolerated)" : "");
      if (!tolerate) std::exit(2);
    }
  }

  std::vector<double> samples_us;
  samples_us.reserve(static_cast<std::size_t>(measured));
  for (int i = 0; i < measured; ++i) {
    const auto t0 = clock_type::now();
    Solution sol = ref_cpu::solve_full(sys, in, state, thr);
    const auto t1 = clock_type::now();
    if (!sol.info.converged) {
      std::fprintf(stderr,
                   "[bench] cpu measured iter %d/%d did not converge%s\n",
                   i + 1, measured, tolerate ? " (tolerated)" : "");
      if (!tolerate) std::exit(2);
    }
    const double us =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
    samples_us.push_back(us);
  }
  return summarize(samples_us);
}

} // namespace

int main(int argc, char **argv) {
  try {
    int N = 32;
    if (argc >= 2) {
      N = std::atoi(argv[1]);
      if (N < 2) {
        std::fprintf(stderr, "[bench] N_parts must be >= 2 (got %d)\n", N);
        return 1;
      }
    }

    std::printf("[bench] N_parts=%d\n", N);

    HostSystem sys = make_synthetic_system_n(N);
    HostInput in = make_synthetic_input_n(N);
    Thresholds thr;
    thr.enabled = true;

    std::printf("[bench]   num_parts=%d num_clutches=%d num_vars=%d "
                "num_eq=%d num_ineq=%d num_relaxed_ineq=%d\n",
                sys.num_parts, sys.num_clutches, sys.num_vars, sys.num_eq,
                sys.num_ineq, sys.num_relaxed_ineq);

    int kGpuWarmup = 5;
    int kGpuMeasured = 50;
    int kCpuWarmup = 5;
    int kCpuMeasured = 20;
    if (const char *q = std::getenv("BENCH_QUICK"); q && std::atoi(q) > 0) {
      kGpuWarmup = 1;
      kGpuMeasured = 1;
      kCpuWarmup = 1;
      kCpuMeasured = 1;
    }
    if (const char *g = std::getenv("BENCH_GPU_ITER"); g && std::atoi(g) > 0) {
      kGpuMeasured = std::atoi(g);
    }
    if (const char *c = std::getenv("BENCH_CPU_ITER"); c && std::atoi(c) > 0) {
      kCpuMeasured = std::atoi(c);
    }

    const Stats gpu = bench_gpu(sys, in, thr, kGpuWarmup, kGpuMeasured);
    const Stats cpu = bench_cpu(sys, in, thr, kCpuWarmup, kCpuMeasured);

    print_stats("gpu pipe.solve     ", gpu);
    print_stats("cpu solve_full     ", cpu);

    if (gpu.mean_us > 0.0) {
      const double speedup = cpu.mean_us / gpu.mean_us;
      std::printf("[bench] speedup     cpu_mean / gpu_mean = %.2fx\n",
                  speedup);
    }

    return 0;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "[bench] ERROR: %s\n", e.what());
    return 2;
  }
}
