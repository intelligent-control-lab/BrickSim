// SPDX-License-Identifier: MIT
//
// CPU reference implementation of the per-step BreakageChecker hot path.
//
// This file is a near-verbatim port of the algorithmic kernels in
// native/modules/bricksim/physx/breakage.cppm (fit_se3, fit_twist,
// so3_log_from_unit_quat, so3_jacobian_inv, compute_Pi, compute_L,
// the b-vector assembly and the unwhitening / utilization post-process)
// together with a hand-written re-implementation of the three-stage OSQP
// wrapper from native/modules/bricksim/physx/osqp.cppm. We deliberately
// avoid any bricksim C++26 module imports so this subproject stays
// self-contained.
//
// Constants (kSmallS, kSmallTheta, lambda regularizers, OSQP epsilon,
// rel_x_reg, rho overrides) are copied verbatim so that this baseline
// stays binary-compatible with the production BreakageChecker output up
// to OSQP's tolerance.

#include "breakage_cuda/ref_cpu.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <osqp.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace breakage_cuda {

namespace {

// Eigen 3.4 does not ship Vector3d::asSkewSymmetric() (which breakage.cppm
// uses); inline an equivalent here so the rest of the port can stay
// line-by-line faithful to the original.
inline Matrix3d skew(const Vector3d &v) {
  Matrix3d m;
  m << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return m;
}

// Wrap an Eigen CSC sparse matrix as an OSQP CSC view. The OSQP matrix
// borrows the Eigen pointers; the Eigen sparse must outlive osqp_setup()
// (which copies the data into its workspace) but may be freed afterwards.
inline OSQPCscMatrix to_osqp_csc(const SparseMatrixCSC &mat) {
  OSQPCscMatrix out{};
  out.m = static_cast<OSQPInt>(mat.rows());
  out.n = static_cast<OSQPInt>(mat.cols());
  out.p = const_cast<OSQPInt *>(mat.outerIndexPtr());
  out.i = const_cast<OSQPInt *>(mat.innerIndexPtr());
  out.x = const_cast<OSQPFloat *>(mat.valuePtr());
  out.nzmax = static_cast<OSQPInt>(mat.nonZeros());
  out.nz = -1;
  out.owned = 0;
  return out;
}

inline void check_osqp(OSQPInt ret) {
  if (ret != 0) {
    const char *msg = osqp_error_message(ret);
    throw std::runtime_error(std::string("osqp: ") +
                             (msg ? msg : "unknown error"));
  }
}

// Mirrors osqp.cppm's build_settings_(): verbose off, tight tolerances tied
// to the same epsilon used by the constraint slack, max_iter overridable
// from the OSQP_MAX_ITER environment variable to match the production code.
OSQPSettings make_settings(double epsilon) {
  OSQPSettings s;
  osqp_set_default_settings(&s);
  s.verbose = 0;
  s.max_iter = 10000;
  s.eps_abs = epsilon;
  s.eps_rel = epsilon;
  if (auto *env = std::getenv("OSQP_MAX_ITER")) {
    s.max_iter = std::stoi(env);
  }
  return s;
}

void fill_stage(StageInfo &si, const OSQPInfo &info, double rho) {
  si.converged = info.status_val == OSQP_SOLVED;
  si.iter = static_cast<int>(info.iter);
  si.prim_res = info.prim_res;
  si.dual_res = info.dual_res;
  si.rho = rho;
  si.solve_time_s = info.solve_time;
}

}  // namespace

// ---------------------------------------------------------------------------
// InternalQpState (forward-declared in system.hpp). The CPU baseline keeps
// the cached stage matrices, the warm-start vectors, and the three OSQP
// solver handles here so the HostState payload can stay opaque.
// ---------------------------------------------------------------------------

struct InternalQpState {
  InternalQpState() = default;
  explicit InternalQpState(const HostSystem &sys) { setup_from(sys); }

  // Captured dimensions (mirror osqp.cppm).
  int nx{0};  // primary variables
  int nv{0};  // relaxation slack variables (one per clutch)
  int me{0};  // equality rows = 6 * num_parts
  int mi{0};  // inequality rows
  int mh{0};  // relaxed inequality rows

  // A/V are needed at solve time (b_prj = A * x_prj.head(nx); u_opt += V*v).
  SparseMatrixCSC A;
  SparseMatrixCSC V;

  // Per-stage matrices/vectors, built once per setup().
  SparseMatrixCSC P_prj, C_prj;
  VectorXd l_prj, u_prj;
  SparseMatrixCSC P_rlx, C_rlx;
  VectorXd q_rlx;
  SparseMatrixCSC P_opt, C_opt;
  VectorXd q_opt;

  // OSQP tunables. epsilon mirrors osqp.cppm's default and honors the
  // OSQP_EPSILON env var like the production solver does.
  double epsilon{1e-4};
  double rel_x_reg{1e-4};

  // Persisted across calls (warm starts + last solve outputs).
  bool has_state{false};
  VectorXd prj_sol, prj_dual;
  double prj_rho{};
  VectorXd rlx_sol, rlx_dual;
  double rlx_rho{};
  VectorXd opt_sol, opt_dual;
  double opt_rho{};
  VectorXd slack;     // b - A * x_prj.head(nx)
  VectorXd v_relax;   // max(0, rlx_sol.tail(nv))

  // OSQP solver handles. Default-initialized to nullptr so the destructor
  // is a no-op until something is actually built.
  std::unique_ptr<OSQPSolver, decltype(&osqp_cleanup)> prj_solver{
      nullptr, &osqp_cleanup};
  std::unique_ptr<OSQPSolver, decltype(&osqp_cleanup)> rlx_solver{
      nullptr, &osqp_cleanup};
  std::unique_ptr<OSQPSolver, decltype(&osqp_cleanup)> opt_solver{
      nullptr, &osqp_cleanup};

  void setup_from(const HostSystem &sys);

  // Linear cost / bound builders, parametrized by the current right-hand
  // side. These mirror build_q_prj_ / build_l_rlx_ / ... in osqp.cppm.
  VectorXd build_q_prj(const VectorXd &b) const {
    VectorXd q(nx + me);
    q.head(nx).setZero();
    q.tail(me) = -b;
    return q;
  }

  VectorXd build_l_rlx(const VectorXd &b_prj) const {
    VectorXd l(me + mi + mh + nv);
    l.head(me) = b_prj;
    l.segment(me, mi).setConstant(-epsilon);
    l.segment(me + mi, mh).setConstant(-OSQP_INFTY);
    l.tail(nv).setZero();
    return l;
  }

  VectorXd build_u_rlx(const VectorXd &b_prj) const {
    VectorXd u(me + mi + mh + nv);
    u.head(me) = b_prj;
    u.segment(me, mi).setConstant(OSQP_INFTY);
    u.segment(me + mi, mh).setConstant(1.0 - epsilon);
    u.tail(nv).setConstant(OSQP_INFTY);
    return u;
  }

  VectorXd build_l_opt(const VectorXd &b_prj) const {
    VectorXd l(me + mi + mh);
    l.head(me) = b_prj;
    l.segment(me, mi).setConstant(-epsilon);
    l.tail(mh).setConstant(-OSQP_INFTY);
    return l;
  }

  VectorXd build_u_opt(const VectorXd &b_prj, const VectorXd &v) const {
    VectorXd u(me + mi + mh);
    u.head(me) = b_prj;
    u.segment(me, mi).setConstant(OSQP_INFTY);
    u.tail(mh).setConstant(1.0);
    u.tail(mh) += V * v;
    return u;
  }
};

void InternalQpState::setup_from(const HostSystem &sys) {
  using Trip = Eigen::Triplet<double>;

  nx = static_cast<int>(sys.Q.rows());
  nv = static_cast<int>(sys.V.cols());
  me = static_cast<int>(sys.A.rows());
  mi = static_cast<int>(sys.G.rows());
  mh = static_cast<int>(sys.H.rows());

  if (auto *env = std::getenv("OSQP_EPSILON")) {
    epsilon = std::stod(env);
  }

  if (sys.Q.cols() != nx || sys.A.cols() != nx || sys.G.cols() != nx ||
      sys.H.cols() != nx || sys.V.rows() != mh) {
    throw std::invalid_argument("InternalQpState: matrix dimension mismatch");
  }

  A = sys.A;
  V = sys.V;
  if (!A.isCompressed()) A.makeCompressed();
  if (!V.isCompressed()) V.makeCompressed();

  // -- Stage 1 (projection) --------------------------------------------------
  // Variables [x; s] of size nx + me. Objective is 1/2 ||s||^2 only, so P_prj
  // has identity on the s-block and zero on the x-block.
  {
    std::vector<Trip> tr;
    tr.reserve(static_cast<std::size_t>(me));
    for (int i = 0; i < me; ++i) {
      tr.emplace_back(nx + i, nx + i, 1.0);
    }
    P_prj.resize(nx + me, nx + me);
    P_prj.setFromTriplets(tr.begin(), tr.end());
    P_prj.makeCompressed();
  }
  // C_prj = [A  -I; G  0]: the upper block expresses A x - s = -b (q tail);
  // the lower block keeps Gx >= 0.
  {
    std::vector<Trip> tr;
    tr.reserve(static_cast<std::size_t>(A.nonZeros() + sys.G.nonZeros() + me));
    for (int k = 0; k < A.outerSize(); ++k) {
      for (SparseMatrixCSC::InnerIterator it(A, k); it; ++it) {
        tr.emplace_back(it.row(), it.col(), it.value());
      }
    }
    for (int k = 0; k < sys.G.outerSize(); ++k) {
      for (SparseMatrixCSC::InnerIterator it(sys.G, k); it; ++it) {
        tr.emplace_back(me + it.row(), it.col(), it.value());
      }
    }
    for (int i = 0; i < me; ++i) {
      tr.emplace_back(i, nx + i, -1.0);
    }
    C_prj.resize(me + mi, nx + me);
    C_prj.setFromTriplets(tr.begin(), tr.end());
    C_prj.makeCompressed();
  }
  l_prj = VectorXd::Zero(me + mi);
  u_prj.resize(me + mi);
  u_prj.head(me).setZero();             // equality rows: 0 <= ... <= 0
  u_prj.tail(mi).setConstant(OSQP_INFTY);  // inequality rows: 0 <= ... <= +inf

  // -- Stage 2 (relaxation) --------------------------------------------------
  // Variables [x; v] of size nx + nv. P_rlx penalizes x with rel_x_reg and
  // v with 1.0 (forces v small unless capacity constraints push it up).
  {
    std::vector<Trip> tr;
    tr.reserve(static_cast<std::size_t>(nx + nv));
    for (int i = 0; i < nx; ++i) tr.emplace_back(i, i, rel_x_reg);
    for (int i = 0; i < nv; ++i) tr.emplace_back(nx + i, nx + i, 1.0);
    P_rlx.resize(nx + nv, nx + nv);
    P_rlx.setFromTriplets(tr.begin(), tr.end());
    P_rlx.makeCompressed();
  }
  // C_rlx = [A 0; G 0; H -V; 0 I]
  {
    std::vector<Trip> tr;
    tr.reserve(static_cast<std::size_t>(A.nonZeros() + sys.G.nonZeros() +
                                        sys.H.nonZeros() + V.nonZeros() + nv));
    for (int k = 0; k < A.outerSize(); ++k) {
      for (SparseMatrixCSC::InnerIterator it(A, k); it; ++it) {
        tr.emplace_back(it.row(), it.col(), it.value());
      }
    }
    for (int k = 0; k < sys.G.outerSize(); ++k) {
      for (SparseMatrixCSC::InnerIterator it(sys.G, k); it; ++it) {
        tr.emplace_back(me + it.row(), it.col(), it.value());
      }
    }
    for (int k = 0; k < sys.H.outerSize(); ++k) {
      for (SparseMatrixCSC::InnerIterator it(sys.H, k); it; ++it) {
        tr.emplace_back(me + mi + it.row(), it.col(), it.value());
      }
    }
    for (int k = 0; k < V.outerSize(); ++k) {
      for (SparseMatrixCSC::InnerIterator it(V, k); it; ++it) {
        tr.emplace_back(me + mi + it.row(), nx + it.col(), -it.value());
      }
    }
    for (int i = 0; i < nv; ++i) {
      tr.emplace_back(me + mi + mh + i, nx + i, 1.0);
    }
    C_rlx.resize(me + mi + mh + nv, nx + nv);
    C_rlx.setFromTriplets(tr.begin(), tr.end());
    C_rlx.makeCompressed();
  }
  q_rlx = VectorXd::Zero(nx + nv);

  // -- Stage 3 (optimization) ------------------------------------------------
  // OSQP wants the upper triangle of the Hessian only.
  {
    std::vector<Trip> tr;
    tr.reserve(static_cast<std::size_t>(sys.Q.nonZeros()));
    for (int k = 0; k < sys.Q.outerSize(); ++k) {
      for (SparseMatrixCSC::InnerIterator it(sys.Q, k); it; ++it) {
        if (it.row() <= it.col()) {
          tr.emplace_back(it.row(), it.col(), it.value());
        }
      }
    }
    P_opt.resize(nx, nx);
    P_opt.setFromTriplets(tr.begin(), tr.end());
    P_opt.makeCompressed();
  }
  // C_opt = [A; G; H]
  {
    std::vector<Trip> tr;
    tr.reserve(static_cast<std::size_t>(A.nonZeros() + sys.G.nonZeros() +
                                        sys.H.nonZeros()));
    for (int k = 0; k < A.outerSize(); ++k) {
      for (SparseMatrixCSC::InnerIterator it(A, k); it; ++it) {
        tr.emplace_back(it.row(), it.col(), it.value());
      }
    }
    for (int k = 0; k < sys.G.outerSize(); ++k) {
      for (SparseMatrixCSC::InnerIterator it(sys.G, k); it; ++it) {
        tr.emplace_back(me + it.row(), it.col(), it.value());
      }
    }
    for (int k = 0; k < sys.H.outerSize(); ++k) {
      for (SparseMatrixCSC::InnerIterator it(sys.H, k); it; ++it) {
        tr.emplace_back(me + mi + it.row(), it.col(), it.value());
      }
    }
    C_opt.resize(me + mi + mh, nx);
    C_opt.setFromTriplets(tr.begin(), tr.end());
    C_opt.makeCompressed();
  }
  q_opt = VectorXd::Zero(nx);
}

namespace {

using OsqpSolverPtr = std::unique_ptr<OSQPSolver, decltype(&osqp_cleanup)>;

OsqpSolverPtr build_prj_solver(InternalQpState &qp, const VectorXd &b) {
  OSQPSettings s = make_settings(qp.epsilon);
  OSQPCscMatrix P = to_osqp_csc(qp.P_prj);
  OSQPCscMatrix C = to_osqp_csc(qp.C_prj);
  VectorXd q = qp.build_q_prj(b);
  OSQPSolver *solver = nullptr;
  check_osqp(osqp_setup(&solver, &P, q.data(), &C, qp.l_prj.data(),
                        qp.u_prj.data(), C.m, C.n, &s));
  return {solver, &osqp_cleanup};
}

OsqpSolverPtr build_rlx_solver(InternalQpState &qp, const VectorXd &b_prj) {
  OSQPSettings s = make_settings(qp.epsilon);
  OSQPCscMatrix P = to_osqp_csc(qp.P_rlx);
  OSQPCscMatrix C = to_osqp_csc(qp.C_rlx);
  VectorXd l = qp.build_l_rlx(b_prj);
  VectorXd u = qp.build_u_rlx(b_prj);
  OSQPSolver *solver = nullptr;
  check_osqp(osqp_setup(&solver, &P, qp.q_rlx.data(), &C, l.data(), u.data(),
                        C.m, C.n, &s));
  return {solver, &osqp_cleanup};
}

OsqpSolverPtr build_opt_solver(InternalQpState &qp, const VectorXd &b_prj,
                               const VectorXd &v) {
  OSQPSettings s = make_settings(qp.epsilon);
  // Mirror osqp.cppm: opt stage starts from rho=1.0 (not the default 0.1).
  s.rho = 1.0;
  OSQPCscMatrix P = to_osqp_csc(qp.P_opt);
  OSQPCscMatrix C = to_osqp_csc(qp.C_opt);
  VectorXd l = qp.build_l_opt(b_prj);
  VectorXd u = qp.build_u_opt(b_prj, v);
  OSQPSolver *solver = nullptr;
  check_osqp(osqp_setup(&solver, &P, qp.q_opt.data(), &C, l.data(), u.data(),
                        C.m, C.n, &s));
  return {solver, &osqp_cleanup};
}

void update_prj_solver(InternalQpState &qp, OSQPSolver *solver,
                       const VectorXd &b) {
  VectorXd q = qp.build_q_prj(b);
  check_osqp(osqp_update_data_vec(solver, q.data(), nullptr, nullptr));
}

void update_rlx_solver(InternalQpState &qp, OSQPSolver *solver,
                       const VectorXd &b_prj) {
  VectorXd l = qp.build_l_rlx(b_prj);
  VectorXd u = qp.build_u_rlx(b_prj);
  check_osqp(osqp_update_data_vec(solver, nullptr, l.data(), u.data()));
}

void update_opt_solver(InternalQpState &qp, OSQPSolver *solver,
                       const VectorXd &b_prj, const VectorXd &v) {
  VectorXd l = qp.build_l_opt(b_prj);
  VectorXd u = qp.build_u_opt(b_prj, v);
  check_osqp(osqp_update_data_vec(solver, nullptr, l.data(), u.data()));
}

}  // namespace

}  // namespace breakage_cuda

namespace breakage_cuda::ref_cpu {

// ---------------------------------------------------------------------------
// Pure dense kernels. Verbatim ports of breakage.cppm:58-180.
// ---------------------------------------------------------------------------

Transformd fit_se3(const MatrixX4d &q0, const MatrixX3d &t0,
                   const MatrixX4d &qx, const MatrixX3d &tx,
                   const VectorXd &mass, double total_mass, double lambda_R) {
  using Eigen::Index;
  using Eigen::Quaterniond;
  Index N = mass.size();
  Vector3d t0_bar = t0.transpose() * mass / total_mass;
  Vector3d tx_bar = tx.transpose() * mass / total_mass;
  Matrix3d H =
      ((t0.rowwise() - t0_bar.transpose()).array().colwise() * mass.array())
          .matrix()
          .transpose() *
      (tx.rowwise() - tx_bar.transpose());
  Matrix3d K = Matrix3d::Zero();
  for (Index i = 0; i < N; ++i) {
    K += mass(i) * (Quaterniond{q0.row(i).transpose()} *
                    Quaterniond{qx.row(i).transpose()}.conjugate())
                       .toRotationMatrix();
  }
  Matrix3d S = H + lambda_R * K;
  Eigen::JacobiSVD<Matrix3d> svd{S, Eigen::ComputeFullU | Eigen::ComputeFullV};
  Matrix3d U = svd.matrixU();
  Matrix3d V = svd.matrixV();
  Matrix3d D = Matrix3d::Identity();
  if ((V * U.transpose()).determinant() < 0.0) {
    D(2, 2) = -1.0;
  }
  Matrix3d R = V * D * U.transpose();
  Vector3d t = tx_bar - R * t0_bar;
  Quaterniond q{R};
  q.normalize();
  return {q, t};
}

TwistFitResult fit_twist(const MatrixX3d &w, const MatrixX3d &v,
                         const MatrixX3d &c_CC, const Transformd &T_W_CC,
                         const VectorXd &mass, double total_mass,
                         double lambda_w) {
  const auto &q_W_CC = T_W_CC.q;
  const auto &t_W_CC = T_W_CC.t;
  MatrixX3d c_W = (c_CC * q_W_CC.toRotationMatrix().transpose()).rowwise() +
                  t_W_CC.transpose();
  Vector3d r = c_W.transpose() * mass / total_mass;
  MatrixX3d d = c_W.rowwise() - r.transpose();
  MatrixX3d d_weighted = d.array().colwise() * mass.array();
  Matrix3d S = d.transpose() * d_weighted.matrix();
  Matrix3d LHS =
      Matrix3d::Identity() * (S.trace() + lambda_w * total_mass) - S;
  Vector3d L;
  L.x() = d_weighted.col(1).dot(v.col(2)) - d_weighted.col(2).dot(v.col(1));
  L.y() = d_weighted.col(2).dot(v.col(0)) - d_weighted.col(0).dot(v.col(2));
  L.z() = d_weighted.col(0).dot(v.col(1)) - d_weighted.col(1).dot(v.col(0));
  Vector3d regularization = lambda_w * (w.transpose() * mass);
  Vector3d RHS = L + regularization;
  TwistFitResult result;
  result.w0 = LHS.ldlt().solve(RHS);
  result.v0 = v.transpose() * mass / total_mass;
  result.v_W = d.rowwise().cross(-result.w0).rowwise() + result.v0.transpose();
  return result;
}

Vector3d so3_log_from_unit_quat(Eigen::Quaterniond q) {
  // Branch threshold on s = ||q.vec|| = sin(theta/2).
  constexpr double kSmallS = 5e-5;
  q.normalize();
  // Pick the shortest representation (theta in [0, pi]); the pi case (w==0)
  // is intrinsically ambiguous.
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  Vector3d v = q.vec();
  double s = v.norm();
  double w = q.w();
  if (s < kSmallS) {
    // theta/s = 2 + s^2/3 + 3 s^4/20 + O(s^6)
    double s2 = s * s;
    double s4 = s2 * s2;
    double theta_over_s = 2.0 + (s2 / 3.0) + (3.0 * s4 / 20.0);
    return theta_over_s * v;
  }
  double theta = 2.0 * std::atan2(s, w);
  return (theta / s) * v;
}

Matrix3d so3_jacobian_inv(const Vector3d &phi) {
  // Branch threshold on theta = ||phi||.
  constexpr double kSmallTheta = 1e-4;
  Matrix3d Phi = skew(phi);
  double theta2 = phi.squaredNorm();
  if (theta2 < kSmallTheta * kSmallTheta) {
    // c(theta) = 1/12 + theta^2/720 + theta^4/30240 + O(theta^6)
    double t2 = theta2;
    double t4 = t2 * t2;
    double c = (1.0 / 12.0) + (t2 / 720.0) + (t4 / 30240.0);
    return Matrix3d::Identity() - 0.5 * Phi + c * (Phi * Phi);
  }
  double theta = std::sqrt(theta2);
  double half = 0.5 * theta;
  double cot_half = std::cos(half) / std::sin(half);
  // c(theta) = 1/theta^2 - cot(theta/2)/(2 theta)
  double c = (1.0 / theta2) - (cot_half / (2.0 * theta));
  return Matrix3d::Identity() - 0.5 * Phi + c * (Phi * Phi);
}

Matrix3d compute_Pi(const Eigen::Quaterniond &qm,
                    const Eigen::Quaterniond &qp) {
  // Relative rotation: R_rel = Rm^T Rp.
  return so3_jacobian_inv(so3_log_from_unit_quat(qm.conjugate() * qp)) *
         qm.toRotationMatrix().transpose();
}

MatrixX3d compute_L(
    const Eigen::Matrix<double, Dynamic, 9, RowMajor> &I_flat,
    const Eigen::Quaterniond &q_W_CC, const Vector3d &w0) {
  Vector3d w_CC = q_W_CC.conjugate() * w0;
  return (I_flat.leftCols<3>() * w_CC.x() +
          I_flat.middleCols<3>(3) * w_CC.y() +
          I_flat.rightCols<3>() * w_CC.z()) *
         q_W_CC.toRotationMatrix().transpose();
}

// ---------------------------------------------------------------------------
// b vector assembly. Layout: row-major (num_parts, 6). Mirrors lines
// 1018-1035 of breakage.cppm.
// ---------------------------------------------------------------------------

VectorXd assemble_b(const HostSystem &sys, const HostInput &in,
                    const MatrixX3d &v_W_curr, const MatrixX3d &v_W_prev,
                    const MatrixX3d &L_curr, const MatrixX3d &L_prev,
                    const Matrix3d &Pi) {
  using Eigen::Index;
  Index N = sys.num_parts;
  VectorXd b(6 * N);
  Eigen::Map<Eigen::Matrix<double, Dynamic, 6, RowMajor>> b_mat{b.data(), N, 6};
  b_mat.leftCols<3>() =
      (((v_W_curr - v_W_prev).array().colwise() * sys.mass.array()).matrix() -
       in.J) *
      Pi.transpose() / in.dt;
  b_mat.rightCols<3>() = ((L_curr - L_prev - in.H) * Pi.transpose()) / in.dt;
  if constexpr (kEnableTorqueScaling) {
    b_mat.rightCols<3>() /= sys.L0;
  }
  return b;
}

VectorXd compute_b(const HostSystem &sys, const HostInput &in, HostState &state,
                   Eigen::Quaterniond *q_W_CC_out, MatrixX3d *v_W_out,
                   MatrixX3d *L_out) {
  Transformd T_W_CC = fit_se3(sys.q_CC, sys.c_CC, in.q, in.c, sys.mass,
                              sys.total_mass, sys.L0 * sys.L0 / 2.0);
  TwistFitResult twist =
      fit_twist(in.w, in.v, sys.c_CC, T_W_CC, sys.mass, sys.total_mass,
                sys.L0 * sys.L0);
  MatrixX3d L_curr = compute_L(sys.I_CC, T_W_CC.q, twist.w0);
  Matrix3d Pi = compute_Pi(state.q_W_CC_prev, T_W_CC.q);
  VectorXd b = assemble_b(sys, in, twist.v_W, state.v_W_prev, L_curr,
                          state.L_prev, Pi);
  if (q_W_CC_out) *q_W_CC_out = T_W_CC.q;
  if (v_W_out) *v_W_out = std::move(twist.v_W);
  if (L_out) *L_out = std::move(L_curr);
  return b;
}

// ---------------------------------------------------------------------------
// Three-stage QP solver. Mirrors OsqpSolver::solve() in osqp.cppm step by
// step: each stage either updates an existing OSQP solver in place (warm
// path) or rebuilds it (cold or post-reset path) and warm-starts it from
// the persisted state vectors.
// ---------------------------------------------------------------------------

SolveInfo solve_qp(const HostSystem &sys, const VectorXd &b, HostState &state,
                   VectorXd &x_out, VectorXd &slack_out,
                   VectorXd &v_relax_out) {
  if (b.size() != 6 * sys.num_parts) {
    throw std::invalid_argument("solve_qp: b dimension mismatch");
  }
  if (!state.qp) {
    state.qp = std::make_shared<InternalQpState>(sys);
  }
  InternalQpState &qp = *state.qp;
  if (qp.has_state) {
    if (qp.prj_sol.size() != qp.C_prj.cols() ||
        qp.prj_dual.size() != qp.C_prj.rows() ||
        qp.rlx_sol.size() != qp.C_rlx.cols() ||
        qp.rlx_dual.size() != qp.C_rlx.rows() ||
        qp.opt_sol.size() != qp.C_opt.cols() ||
        qp.opt_dual.size() != qp.C_opt.rows()) {
      throw std::invalid_argument("solve_qp: warm-start dimension mismatch");
    }
  }

  SolveInfo info;

  // -- Stage 1: projection ---------------------------------------------------
  if (qp.has_state) {
    if (qp.prj_solver) {
      update_prj_solver(qp, qp.prj_solver.get(), b);
    } else {
      qp.prj_solver = build_prj_solver(qp, b);
      check_osqp(osqp_warm_start(qp.prj_solver.get(), qp.prj_sol.data(),
                                 qp.prj_dual.data()));
      check_osqp(osqp_update_rho(qp.prj_solver.get(), qp.prj_rho));
    }
  } else {
    qp.prj_solver = build_prj_solver(qp, b);
  }
  check_osqp(osqp_solve(qp.prj_solver.get()));
  {
    OSQPSolver *s = qp.prj_solver.get();
    qp.prj_sol = Eigen::Map<const VectorXd>(s->solution->x, qp.C_prj.cols());
    qp.prj_dual = Eigen::Map<const VectorXd>(s->solution->y, qp.C_prj.rows());
    qp.prj_rho = s->settings->rho;
    fill_stage(info.prj, *s->info, qp.prj_rho);
  }
  VectorXd b_prj = qp.A * qp.prj_sol.head(qp.nx);
  qp.slack = b - b_prj;

  // -- Stage 2: relaxation ---------------------------------------------------
  if (qp.has_state) {
    if (qp.rlx_solver) {
      update_rlx_solver(qp, qp.rlx_solver.get(), b_prj);
    } else {
      qp.rlx_solver = build_rlx_solver(qp, b_prj);
      check_osqp(osqp_warm_start(qp.rlx_solver.get(), qp.rlx_sol.data(),
                                 qp.rlx_dual.data()));
      check_osqp(osqp_update_rho(qp.rlx_solver.get(), qp.rlx_rho));
    }
  } else {
    qp.rlx_solver = build_rlx_solver(qp, b_prj);
  }
  check_osqp(osqp_solve(qp.rlx_solver.get()));
  {
    OSQPSolver *s = qp.rlx_solver.get();
    qp.rlx_sol = Eigen::Map<const VectorXd>(s->solution->x, qp.C_rlx.cols());
    qp.rlx_dual = Eigen::Map<const VectorXd>(s->solution->y, qp.C_rlx.rows());
    qp.rlx_rho = s->settings->rho;
    fill_stage(info.rlx, *s->info, qp.rlx_rho);
  }
  qp.v_relax = qp.rlx_sol.tail(qp.nv).cwiseMax(0.0);

  // -- Stage 3: optimization -------------------------------------------------
  if (qp.has_state) {
    if (qp.opt_solver) {
      update_opt_solver(qp, qp.opt_solver.get(), b_prj, qp.v_relax);
    } else {
      qp.opt_solver = build_opt_solver(qp, b_prj, qp.v_relax);
      check_osqp(osqp_warm_start(qp.opt_solver.get(), qp.opt_sol.data(),
                                 qp.opt_dual.data()));
      check_osqp(osqp_update_rho(qp.opt_solver.get(), qp.opt_rho));
    }
  } else {
    qp.opt_solver = build_opt_solver(qp, b_prj, qp.v_relax);
  }
  check_osqp(osqp_solve(qp.opt_solver.get()));
  {
    OSQPSolver *s = qp.opt_solver.get();
    qp.opt_sol = Eigen::Map<const VectorXd>(s->solution->x, qp.C_opt.cols());
    qp.opt_dual = Eigen::Map<const VectorXd>(s->solution->y, qp.C_opt.rows());
    qp.opt_rho = s->settings->rho;
    fill_stage(info.opt, *s->info, qp.opt_rho);
  }
  info.converged =
      info.prj.converged && info.rlx.converged && info.opt.converged;
  qp.has_state = true;

  // x_out is left whitened; solve_full() does the un-whitening, mirroring the
  // CUDA pipeline which also returns whitened `x_d` from its solver.
  x_out = qp.opt_sol.head(qp.nx);
  slack_out = qp.slack;
  v_relax_out = qp.v_relax;
  return info;
}

// ---------------------------------------------------------------------------
// End-to-end driver. Mirrors BreakageChecker::solve() at breakage.cppm:996,
// minus the dump bookkeeping and logging which are out of scope here.
// ---------------------------------------------------------------------------

Solution solve_full(const HostSystem &sys, const HostInput &in,
                    HostState &state, const Thresholds &thr) {
  Eigen::Quaterniond q_W_CC_curr;
  MatrixX3d v_W_curr;
  MatrixX3d L_curr;
  VectorXd b = compute_b(sys, in, state, &q_W_CC_curr, &v_W_curr, &L_curr);

  Solution sol;
  if (!thr.enabled) {
    // Match the production code path: state is NOT advanced when the checker
    // is disabled. Re-enabling later restarts from the previously cached
    // q_W_CC_prev / v_W_prev / L_prev.
    return sol;
  }

  VectorXd x_raw, slack, v_relax;
  sol.info = solve_qp(sys, b, state, x_raw, slack, v_relax);
  sol.slack_fraction =
      slack.norm() / std::max(b.norm(), thr.slack_fraction_b_floor);

  // Advance the per-frame state (matches lines 1053-1055 of breakage.cppm:
  // done unconditionally after the solve, before the converged-only
  // postprocess block).
  state.q_W_CC_prev = q_W_CC_curr;
  state.v_W_prev = std::move(v_W_curr);
  state.L_prev = std::move(L_curr);

  if (sol.info.converged) {
    sol.x = std::move(x_raw);
    if constexpr (kEnableClutchWhitening) {
      const int ncv = sys.num_contact_vertices;
      for (int k = 0; k < sys.num_clutches; ++k) {
        // The 4 doubles per row in HostSystem.clutch_whiten are the raw
        // memory of bricksim's `std::vector<Matrix2d>` (column-major), so
        // mapping them back as a ColMajor 2x2 reproduces Wk exactly.
        // (Wk is symmetric anyway, so the choice does not affect the math.)
        Eigen::Map<const Eigen::Matrix2d> Wk(sys.clutch_whiten.data() + 4 * k);
        const int j0 = ncv + 9 * k;
        Eigen::Vector2d a_scaled = sol.x.segment<2>(j0 + 1);
        sol.x.segment<2>(j0 + 1) = Wk * a_scaled;
        Eigen::Vector2d b_scaled = sol.x.segment<2>(j0 + 4);
        sol.x.segment<2>(j0 + 4) = Wk * b_scaled;
        Eigen::Vector2d c_scaled = sol.x.segment<2>(j0 + 7);
        sol.x.segment<2>(j0 + 7) = Wk * c_scaled;
      }
    }
    sol.utilization.setConstant(sys.num_clutches, -1.0);
    const int ncv = sys.num_contact_vertices;
    for (std::size_t idx = 0; idx < sys.capacity_clutch_indices.size(); ++idx) {
      Eigen::Matrix<double, 9, 1> c =
          sys.capacities.row(static_cast<Eigen::Index>(idx)).transpose();
      const int clutch_index = sys.capacity_clutch_indices[idx];
      Eigen::Matrix<double, 9, 1> x =
          sol.x.segment<9>(ncv + 9 * clutch_index);
      double frc_used = c.head<3>().dot(x.head<3>());
      double frc_cap = 1.0 - c.tail<6>().dot(x.tail<6>());
      double u_fp;
      if (frc_cap > 0.0) {
        u_fp = frc_used / frc_cap;
        u_fp = std::max(u_fp, 0.0);
      } else {
        u_fp = 1e6;
      }
      double &u_max = sol.utilization(clutch_index);
      if (u_fp > u_max) {
        u_max = u_fp;
      }
    }
  }

  return sol;
}

}  // namespace breakage_cuda::ref_cpu
