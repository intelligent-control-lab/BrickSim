// SPDX-License-Identifier: MIT
//
// CPU reference implementation. Mirrors the per-step code paths of
// bricksim::BreakageChecker::solve in breakage.cppm:
//
//   1. fit_se3 / fit_twist (per-part dense ops)
//   2. compute_Pi, compute_L
//   3. RHS b assembly
//   4. Three-stage OSQP solve (projection / relaxation / optimization)
//   5. Whitening reversal + utilization
//
// Used both as a baseline for the CUDA implementation and to score the random
// QP and full-pipeline tests.

#pragma once

#include "breakage_cuda/system.hpp"

#include <Eigen/Geometry>

#include <span>

namespace breakage_cuda::ref_cpu {

// ---------------------------------------------------------------------------
// Compile-time switches mirroring breakage.cppm. They are hard-coded to true
// because BreakageChecker uses them as such; we expose them here so that
// tests can opt out if needed.
// ---------------------------------------------------------------------------

constexpr bool kEnableTorqueScaling = true;
constexpr bool kEnableClutchWhitening = true;
constexpr bool kEnableClutchTangentialFriction = true;
constexpr bool kEnableRealisticClutchFriction = true;

// ---------------------------------------------------------------------------
// Pure dense kernels (no QP). Each one matches a function in breakage.cppm.
// ---------------------------------------------------------------------------

struct Transformd {
  Eigen::Quaterniond q;
  Vector3d t;
};

Transformd fit_se3(const MatrixX4d &q0, const MatrixX3d &t0,
                   const MatrixX4d &qx, const MatrixX3d &tx,
                   const VectorXd &mass, double total_mass, double lambda_R);

struct TwistFitResult {
  Vector3d w0;
  Vector3d v0;
  MatrixX3d v_W; // (num_parts, 3)
};

TwistFitResult fit_twist(const MatrixX3d &w, const MatrixX3d &v,
                         const MatrixX3d &c_CC, const Transformd &T_W_CC,
                         const VectorXd &mass, double total_mass,
                         double lambda_w);

Vector3d so3_log_from_unit_quat(Eigen::Quaterniond q);

Matrix3d so3_jacobian_inv(const Vector3d &phi);

Matrix3d compute_Pi(const Eigen::Quaterniond &qm,
                    const Eigen::Quaterniond &qp);

// I_flat: row-major (N, 9), each row is row-major flatten of 3x3.
MatrixX3d compute_L(
    const Eigen::Matrix<double, Dynamic, 9, RowMajor> &I_flat,
    const Eigen::Quaterniond &q_W_CC, const Vector3d &w0);

// Builds the RHS b vector (size 6 * num_parts) per breakage.cppm:1018-1035.
//
// `Pi` is from compute_Pi; `v_W_curr`, `v_W_prev`, `L_curr`, `L_prev` are
// (num_parts, 3); J/H are external impulses (num_parts, 3).
//
// Output: b is laid out as 6 floats per part (linear x 3, angular x 3) in
// row-major order so it matches breakage.cppm's `b_mat` Map.
VectorXd assemble_b(const HostSystem &sys, const HostInput &in,
                    const MatrixX3d &v_W_curr, const MatrixX3d &v_W_prev,
                    const MatrixX3d &L_curr, const MatrixX3d &L_prev,
                    const Matrix3d &Pi);

// ---------------------------------------------------------------------------
// Three-stage QP solver mirroring osqp.cppm.
//
// On first call (state.qp == nullptr or InternalQpState empty), the solver
// is built from sys; subsequent calls warm-start from the previous solution.
// ---------------------------------------------------------------------------

SolveInfo solve_qp(const HostSystem &sys, const VectorXd &b, HostState &state,
                   VectorXd &x_out, VectorXd &slack_out, VectorXd &v_relax_out);

// ---------------------------------------------------------------------------
// End-to-end driver. Runs steps (1)-(5) above and returns the same Solution
// you would get from BreakageChecker::solve in CPU-only mode.
// ---------------------------------------------------------------------------

Solution solve_full(const HostSystem &sys, const HostInput &in,
                    HostState &state, const Thresholds &thr);

// Convenience: computes b and returns it (used by tests).
VectorXd compute_b(const HostSystem &sys, const HostInput &in, HostState &state,
                   Eigen::Quaterniond *q_W_CC_out = nullptr,
                   MatrixX3d *v_W_out = nullptr, MatrixX3d *L_out = nullptr);

} // namespace breakage_cuda::ref_cpu
