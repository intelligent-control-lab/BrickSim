// SPDX-License-Identifier: MIT
//
// Correctness check: each dense CUDA kernel vs its CPU reference on a single
// synthetic problem (N = 8 parts).
//
// Coverage:
//   1. cuda_kernels::fit_se3       vs ref_cpu::fit_se3
//   2. cuda_kernels::fit_twist     vs ref_cpu::fit_twist
//   3. cuda_kernels::compute_Pi_host vs ref_cpu::compute_Pi
//   4. cuda_kernels::compute_L     vs ref_cpu::compute_L
//   5. cuda_kernels::assemble_b    vs ref_cpu::assemble_b
//
// Layout conventions (verified against src/system.cpp upload helpers):
//   * MatrixX3d / MatrixX4d aliases default to ColMajor; we transcode to
//     RowMajor before any DeviceBuffer upload, matching upload_input().
//   * I_CC is RowMajor on the host already (see system.hpp), so its raw
//     .data() can be uploaded directly.
//   * Quaternions are flat xyzw, length-4. Eigen::Quaterniond::coeffs()
//     returns xyzw, so we round-trip via that.
//
// No bug fixes were required in src/kernels.cu, src/linalg.cu, or
// src/ref_cpu.cpp to make this test pass.

#include "breakage_cuda/device.hpp"
#include "breakage_cuda/kernels.hpp"
#include "breakage_cuda/linalg.hpp"
#include "breakage_cuda/ref_cpu.hpp"
#include "breakage_cuda/system.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

namespace bc = breakage_cuda;

namespace {

constexpr int kN = 8;

// Per-test record collected so we can emit a final summary table.
struct TestResult {
  std::string name;
  bool passed = false;
  double max_abs_diff = 0.0;
  double tol = 0.0;
  std::string detail;
};

void log_pass(const std::string &name, double diff, const std::string &detail) {
  std::printf("[PASS] %-12s  max_abs_diff=%.3e  %s\n", name.c_str(), diff,
              detail.c_str());
}

void log_fail(const std::string &name, double diff, const std::string &detail) {
  std::printf("[FAIL] %-12s  max_abs_diff=%.3e  %s\n", name.c_str(), diff,
              detail.c_str());
}

// Sample a uniformly random unit quaternion via four Gaussians + normalize.
Eigen::Quaterniond random_unit_quat(std::mt19937 &rng) {
  std::normal_distribution<double> n(0.0, 1.0);
  Eigen::Quaterniond q;
  // (x, y, z, w) order in coeffs().
  q.coeffs() << n(rng), n(rng), n(rng), n(rng);
  q.normalize();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  return q;
}

// Compare two quaternions modulo the q ~ -q sign ambiguity.
double quat_diff_sign_invariant(const Eigen::Quaterniond &a,
                                const Eigen::Quaterniond &b) {
  double d_pos = (a.coeffs() - b.coeffs()).cwiseAbs().maxCoeff();
  double d_neg = (a.coeffs() + b.coeffs()).cwiseAbs().maxCoeff();
  return std::min(d_pos, d_neg);
}

// Max-abs diff between an N x 3 row-major flat buffer and an Eigen N x 3
// matrix (any storage order).
double max_abs_diff_n3(const double *buf_rm, const bc::MatrixX3d &ref) {
  Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>
      view(buf_rm, ref.rows(), 3);
  return (view - ref).cwiseAbs().maxCoeff();
}

// Max-abs diff between a 9-double row-major buffer and an Eigen Matrix3d
// (Eigen Matrix3d is column-major by default; we compare element-wise).
double max_abs_diff_3x3(const double *buf_rm, const Eigen::Matrix3d &ref) {
  double d = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      d = std::max(d, std::abs(buf_rm[3 * i + j] - ref(i, j)));
    }
  }
  return d;
}

// Random N x 3 dense matrix with entries from [lo, hi].
bc::MatrixX3d random_n3(std::mt19937 &rng, int N, double lo, double hi) {
  std::uniform_real_distribution<double> u(lo, hi);
  bc::MatrixX3d m(N, 3);
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < 3; ++j) m(i, j) = u(rng);
  return m;
}

// Pack an Eigen N x 3 (any layout) into a row-major flat std::vector for
// device upload.
std::vector<double> to_rm3_flat(const bc::MatrixX3d &m) {
  std::vector<double> v(static_cast<std::size_t>(m.rows() * 3));
  for (int i = 0; i < m.rows(); ++i) {
    v[3 * i + 0] = m(i, 0);
    v[3 * i + 1] = m(i, 1);
    v[3 * i + 2] = m(i, 2);
  }
  return v;
}

// Same for N x 4 (used for quaternions).
std::vector<double> to_rm4_flat(const bc::MatrixX4d &m) {
  std::vector<double> v(static_cast<std::size_t>(m.rows() * 4));
  for (int i = 0; i < m.rows(); ++i) {
    v[4 * i + 0] = m(i, 0);
    v[4 * i + 1] = m(i, 1);
    v[4 * i + 2] = m(i, 2);
    v[4 * i + 3] = m(i, 3);
  }
  return v;
}

}  // namespace

int main() {
  std::printf("test_dense_kernels: N=%d\n", kN);

  std::mt19937 rng(0xDEADBEEFu);
  std::uniform_real_distribution<double> u_mass(0.5, 2.0);
  std::uniform_real_distribution<double> u_pos(-1.0, 1.0);
  std::uniform_real_distribution<double> u_small(-0.05, 0.05);
  std::normal_distribution<double> n01(0.0, 1.0);

  // Per-part mass + total mass.
  bc::VectorXd mass(kN);
  for (int i = 0; i < kN; ++i) mass(i) = u_mass(rng);
  const double total_mass = mass.sum();

  // Reference frame: q0 random unit quat, t0 random in [-1, 1]^3.
  // Current frame: qx = q0 * small rotation, tx = t0 + small noise. This
  // keeps fit_se3's optimum well-defined and away from the q ~ -q boundary.
  bc::MatrixX4d q0(kN, 4);
  bc::MatrixX3d t0(kN, 3);
  bc::MatrixX4d qx(kN, 4);
  bc::MatrixX3d tx(kN, 3);
  for (int i = 0; i < kN; ++i) {
    Eigen::Quaterniond q = random_unit_quat(rng);
    q0(i, 0) = q.x();
    q0(i, 1) = q.y();
    q0(i, 2) = q.z();
    q0(i, 3) = q.w();
    t0(i, 0) = u_pos(rng);
    t0(i, 1) = u_pos(rng);
    t0(i, 2) = u_pos(rng);

    Eigen::Vector3d axis(n01(rng), n01(rng), n01(rng));
    if (axis.norm() < 1e-12) axis = Eigen::Vector3d::UnitX();
    double angle = 0.05 * u_pos(rng);
    Eigen::Quaterniond rot(Eigen::AngleAxisd(angle, axis.normalized()));
    Eigen::Quaterniond qxq = q * rot;
    qxq.normalize();
    if (qxq.w() < 0.0) qxq.coeffs() *= -1.0;
    qx(i, 0) = qxq.x();
    qx(i, 1) = qxq.y();
    qx(i, 2) = qxq.z();
    qx(i, 3) = qxq.w();

    tx(i, 0) = t0(i, 0) + u_small(rng);
    tx(i, 1) = t0(i, 1) + u_small(rng);
    tx(i, 2) = t0(i, 2) + u_small(rng);
  }

  // Per-part w / v for fit_twist (small range).
  bc::MatrixX3d w_in = random_n3(rng, kN, -0.5, 0.5);
  bc::MatrixX3d v_in = random_n3(rng, kN, -0.5, 0.5);
  // c_CC[i] = t0[i] (per the task spec).
  bc::MatrixX3d c_CC = t0;

  // Inertia tensors: diag(2, 3, 5) + small symmetric off-diagonal noise.
  // Stored row-major as (N, 9), matching HostSystem::I_CC.
  Eigen::Matrix<double, Eigen::Dynamic, 9, Eigen::RowMajor> I_flat(kN, 9);
  for (int i = 0; i < kN; ++i) {
    double a01 = 0.05 * u_pos(rng);
    double a02 = 0.05 * u_pos(rng);
    double a12 = 0.05 * u_pos(rng);
    double diag[3] = {2.0, 3.0, 5.0};
    double M[9] = {diag[0], a01,     a02,     //
                   a01,     diag[1], a12,     //
                   a02,     a12,     diag[2]};
    for (int k = 0; k < 9; ++k) I_flat(i, k) = M[k];
  }

  // J / H impulses for assemble_b.
  bc::MatrixX3d J = random_n3(rng, kN, -0.1, 0.1);
  bc::MatrixX3d H = random_n3(rng, kN, -0.1, 0.1);

  // v_W / L scratch matrices (random) for assemble_b. Note these are
  // independent of fit_twist's outputs; the kernel just consumes them.
  bc::MatrixX3d v_W_curr = random_n3(rng, kN, -0.5, 0.5);
  bc::MatrixX3d v_W_prev = random_n3(rng, kN, -0.5, 0.5);
  bc::MatrixX3d L_curr = random_n3(rng, kN, -0.5, 0.5);
  bc::MatrixX3d L_prev = random_n3(rng, kN, -0.5, 0.5);

  const double lambda_R = 1e-3;
  const double lambda_w = 1e-3;
  const double dt = 1.0 / 240.0;
  const double L0 = 1.0;

  // -- Reusable device buffers ----------------------------------------------
  std::vector<double> q0_h = to_rm4_flat(q0);
  std::vector<double> qx_h = to_rm4_flat(qx);
  std::vector<double> t0_h = to_rm3_flat(t0);
  std::vector<double> tx_h = to_rm3_flat(tx);
  std::vector<double> c_CC_h = to_rm3_flat(c_CC);
  std::vector<double> w_h = to_rm3_flat(w_in);
  std::vector<double> v_h = to_rm3_flat(v_in);

  bc::DeviceBuffer<double> q0_d, qx_d, t0_d, tx_d, c_CC_d, w_d, v_d, mass_d;
  q0_d.copy_from_host(q0_h.data(), q0_h.size());
  qx_d.copy_from_host(qx_h.data(), qx_h.size());
  t0_d.copy_from_host(t0_h.data(), t0_h.size());
  tx_d.copy_from_host(tx_h.data(), tx_h.size());
  c_CC_d.copy_from_host(c_CC_h.data(), c_CC_h.size());
  w_d.copy_from_host(w_h.data(), w_h.size());
  v_d.copy_from_host(v_h.data(), v_h.size());
  mass_d.copy_from_host(mass.data(), static_cast<std::size_t>(mass.size()));

  std::vector<TestResult> results;

  // -------------------------------------------------------------------------
  // 1. fit_se3
  // -------------------------------------------------------------------------
  {
    bc::ref_cpu::Transformd cpu = bc::ref_cpu::fit_se3(
        q0, t0, qx, tx, mass, total_mass, lambda_R);

    double q_out[4] = {0, 0, 0, 0};
    double t_out[3] = {0, 0, 0};
    bc::cuda_kernels::fit_se3(kN, q0_d.data(), t0_d.data(), qx_d.data(),
                              tx_d.data(), mass_d.data(), total_mass,
                              lambda_R, q_out, t_out);

    Eigen::Quaterniond q_gpu;
    q_gpu.coeffs() << q_out[0], q_out[1], q_out[2], q_out[3];

    double q_diff = quat_diff_sign_invariant(cpu.q, q_gpu);
    double t_diff =
        (cpu.t - Eigen::Vector3d(t_out[0], t_out[1], t_out[2])).cwiseAbs().maxCoeff();
    double max_d = std::max(q_diff, t_diff);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "(q_diff=%.3e, t_diff=%.3e)", q_diff,
                  t_diff);
    constexpr double tol = 1e-9;
    TestResult r{"fit_se3", max_d <= tol, max_d, tol, buf};
    if (r.passed)
      log_pass(r.name, max_d, buf);
    else
      log_fail(r.name, max_d, buf);
    results.push_back(std::move(r));
  }

  // -------------------------------------------------------------------------
  // 2. fit_twist
  //
  // Use T_W_CC = identity quaternion (xyzw = (0, 0, 0, 1)) + zero translation
  // so the c_W = c_CC simplification holds and any divergence comes from the
  // twist solve itself.
  // -------------------------------------------------------------------------
  {
    bc::ref_cpu::Transformd T_W_CC{Eigen::Quaterniond::Identity(),
                                   bc::Vector3d::Zero()};
    bc::ref_cpu::TwistFitResult cpu = bc::ref_cpu::fit_twist(
        w_in, v_in, c_CC, T_W_CC, mass, total_mass, lambda_w);

    const double q_W_CC[4] = {0.0, 0.0, 0.0, 1.0};
    const double t_W_CC[3] = {0.0, 0.0, 0.0};
    double w0_out[3] = {0, 0, 0};
    double v0_out[3] = {0, 0, 0};
    bc::DeviceBuffer<double> v_W_d(static_cast<std::size_t>(3) * kN);

    bc::cuda_kernels::fit_twist(kN, w_d.data(), v_d.data(), c_CC_d.data(),
                                q_W_CC, t_W_CC, mass_d.data(), total_mass,
                                lambda_w, w0_out, v0_out, v_W_d.data());

    Eigen::Vector3d w0_gpu(w0_out[0], w0_out[1], w0_out[2]);
    Eigen::Vector3d v0_gpu(v0_out[0], v0_out[1], v0_out[2]);
    double w0_diff = (cpu.w0 - w0_gpu).cwiseAbs().maxCoeff();
    double v0_diff = (cpu.v0 - v0_gpu).cwiseAbs().maxCoeff();

    std::vector<double> v_W_h(static_cast<std::size_t>(3) * kN);
    v_W_d.copy_to_host(v_W_h.data(), v_W_h.size());
    double vW_diff = max_abs_diff_n3(v_W_h.data(), cpu.v_W);

    double max_d = std::max({w0_diff, v0_diff, vW_diff});
    char buf[200];
    std::snprintf(buf, sizeof(buf),
                  "(w0_diff=%.3e, v0_diff=%.3e, v_W_diff=%.3e)", w0_diff,
                  v0_diff, vW_diff);
    constexpr double tol = 1e-9;
    TestResult r{"fit_twist", max_d <= tol, max_d, tol, buf};
    if (r.passed)
      log_pass(r.name, max_d, buf);
    else
      log_fail(r.name, max_d, buf);
    results.push_back(std::move(r));
  }

  // -------------------------------------------------------------------------
  // 3. compute_Pi
  // -------------------------------------------------------------------------
  {
    Eigen::Quaterniond qm = random_unit_quat(rng);
    Eigen::Quaterniond qp = random_unit_quat(rng);

    Eigen::Matrix3d Pi_cpu = bc::ref_cpu::compute_Pi(qm, qp);

    double qm_a[4] = {qm.x(), qm.y(), qm.z(), qm.w()};
    double qp_a[4] = {qp.x(), qp.y(), qp.z(), qp.w()};
    double Pi_gpu[9] = {0};
    bc::cuda_kernels::compute_Pi_host(qm_a, qp_a, Pi_gpu);

    double max_d = max_abs_diff_3x3(Pi_gpu, Pi_cpu);
    char buf[80];
    std::snprintf(buf, sizeof(buf), "(3x3)");
    constexpr double tol = 1e-12;
    TestResult r{"compute_Pi", max_d <= tol, max_d, tol, buf};
    if (r.passed)
      log_pass(r.name, max_d, buf);
    else
      log_fail(r.name, max_d, buf);
    results.push_back(std::move(r));
  }

  // -------------------------------------------------------------------------
  // 4. compute_L
  // -------------------------------------------------------------------------
  {
    Eigen::Quaterniond q_W_CC = random_unit_quat(rng);
    Eigen::Vector3d w0(0.7, -0.4, 0.2);

    bc::MatrixX3d L_ref = bc::ref_cpu::compute_L(I_flat, q_W_CC, w0);

    bc::DeviceBuffer<double> I_CC_d;
    // I_flat is already RowMajor; .data() is row-major (N, 9) flatten.
    I_CC_d.copy_from_host(I_flat.data(),
                          static_cast<std::size_t>(I_flat.size()));

    double q_a[4] = {q_W_CC.x(), q_W_CC.y(), q_W_CC.z(), q_W_CC.w()};
    double w_a[3] = {w0.x(), w0.y(), w0.z()};
    bc::DeviceBuffer<double> L_d(static_cast<std::size_t>(3) * kN);
    bc::cuda_kernels::compute_L(kN, I_CC_d.data(), q_a, w_a, L_d.data());

    std::vector<double> L_h(static_cast<std::size_t>(3) * kN);
    L_d.copy_to_host(L_h.data(), L_h.size());

    double max_d = max_abs_diff_n3(L_h.data(), L_ref);
    char buf[80];
    std::snprintf(buf, sizeof(buf), "(N=%d, 3 cols)", kN);
    constexpr double tol = 1e-9;
    TestResult r{"compute_L", max_d <= tol, max_d, tol, buf};
    if (r.passed)
      log_pass(r.name, max_d, buf);
    else
      log_fail(r.name, max_d, buf);
    results.push_back(std::move(r));
  }

  // -------------------------------------------------------------------------
  // 5. assemble_b
  //
  // ref_cpu::assemble_b only reads sys.{num_parts, mass, L0} and
  // in.{dt, J, H} (sparse blocks of HostSystem are untouched), so we leave
  // the rest at default-constructed.
  // -------------------------------------------------------------------------
  {
    bc::HostSystem sys;
    sys.num_parts = kN;
    sys.mass = mass;
    sys.L0 = L0;

    bc::HostInput in;
    in.dt = dt;
    in.J = J;
    in.H = H;

    Eigen::Matrix3d Pi;
    Pi << 1.1, 0.2, -0.1,
          -0.05, 0.95, 0.07,
          0.03, -0.02, 1.03;

    bc::VectorXd b_cpu = bc::ref_cpu::assemble_b(sys, in, v_W_curr, v_W_prev,
                                                 L_curr, L_prev, Pi);

    std::vector<double> v_W_curr_h = to_rm3_flat(v_W_curr);
    std::vector<double> v_W_prev_h = to_rm3_flat(v_W_prev);
    std::vector<double> L_curr_h = to_rm3_flat(L_curr);
    std::vector<double> L_prev_h = to_rm3_flat(L_prev);
    std::vector<double> J_h = to_rm3_flat(J);
    std::vector<double> H_h = to_rm3_flat(H);

    bc::DeviceBuffer<double> v_W_curr_d, v_W_prev_d, L_curr_d, L_prev_d, J_d,
        H_d;
    v_W_curr_d.copy_from_host(v_W_curr_h.data(), v_W_curr_h.size());
    v_W_prev_d.copy_from_host(v_W_prev_h.data(), v_W_prev_h.size());
    L_curr_d.copy_from_host(L_curr_h.data(), L_curr_h.size());
    L_prev_d.copy_from_host(L_prev_h.data(), L_prev_h.size());
    J_d.copy_from_host(J_h.data(), J_h.size());
    H_d.copy_from_host(H_h.data(), H_h.size());

    // Pi in row-major flat form for the CUDA kernel.
    double Pi_flat[9];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) Pi_flat[3 * i + j] = Pi(i, j);

    bc::DeviceBuffer<double> b_d(static_cast<std::size_t>(6) * kN);
    bc::cuda_kernels::assemble_b(kN, v_W_curr_d.data(), v_W_prev_d.data(),
                                 L_curr_d.data(), L_prev_d.data(), J_d.data(),
                                 H_d.data(), mass_d.data(), Pi_flat, dt, L0,
                                 b_d.data());

    std::vector<double> b_h(static_cast<std::size_t>(6) * kN);
    b_d.copy_to_host(b_h.data(), b_h.size());

    // b_cpu is laid out row-major as (N, 6); compare element-wise to the
    // (already row-major) GPU buffer.
    double max_d = 0.0;
    for (std::size_t i = 0; i < b_h.size(); ++i) {
      max_d = std::max(max_d, std::abs(b_h[i] - b_cpu(static_cast<int>(i))));
    }
    char buf[80];
    std::snprintf(buf, sizeof(buf), "(6N=%d)", 6 * kN);
    constexpr double tol = 1e-9;
    TestResult r{"assemble_b", max_d <= tol, max_d, tol, buf};
    if (r.passed)
      log_pass(r.name, max_d, buf);
    else
      log_fail(r.name, max_d, buf);
    results.push_back(std::move(r));
  }

  // -------------------------------------------------------------------------
  // Summary table.
  // -------------------------------------------------------------------------
  std::printf("\n=========================== Summary ===========================\n");
  std::printf("%-15s %-6s %-12s %-12s %s\n", "kernel", "status", "max_diff",
              "tol", "detail");
  std::printf("---------------------------------------------------------------\n");
  bool all_pass = true;
  for (const auto &r : results) {
    std::printf("%-15s %-6s %-12.3e %-12.3e %s\n", r.name.c_str(),
                r.passed ? "PASS" : "FAIL", r.max_abs_diff, r.tol,
                r.detail.c_str());
    if (!r.passed) all_pass = false;
  }
  std::printf("===============================================================\n");
  std::printf("%s\n", all_pass ? "OVERALL: PASS" : "OVERALL: FAIL");
  return all_pass ? 0 : 1;
}
