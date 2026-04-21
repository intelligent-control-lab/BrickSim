// SPDX-License-Identifier: MIT
//
// Plain-C++ mirror of the bricksim::Breakage* types, sufficient to drive the
// per-step hot path on either CPU (Eigen + OSQP) or CUDA. The JSON layout is
// the one produced by bricksim::to_json(BreakageDebugDump) in
// modules/bricksim/physx/breakage.cppm.
//
// We intentionally do NOT depend on any bricksim C++26 module: the goal of
// this subproject is to be a self-contained CUDA prototype.

#pragma once

#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include <nlohmann/json_fwd.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace breakage_cuda {

// ---------------------------------------------------------------------------
// Aliases (match the layout used by breakage.cppm).
// ---------------------------------------------------------------------------

using Eigen::Dynamic;
using Eigen::RowMajor;

using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;

using Matrix2d = Eigen::Matrix2d;
using Matrix3d = Eigen::Matrix3d;

using MatrixX3d = Eigen::Matrix<double, Dynamic, 3>;
using MatrixX4d = Eigen::Matrix<double, Dynamic, 4>;

using SparseMatrixCSC = Eigen::SparseMatrix<double, Eigen::ColMajor, int>;
using SparseMatrixCSR = Eigen::SparseMatrix<double, Eigen::RowMajor, int>;

// ---------------------------------------------------------------------------
// Thresholds (mirror BreakageThresholds).
// ---------------------------------------------------------------------------

struct Thresholds {
  bool enabled{true};
  double contact_regularization{0.1};
  double clutch_axial_compliance{1.0};
  double clutch_radial_compliance{1.0};
  double clutch_tangential_compliance{1.0};
  double friction_coefficient{0.2};
  double preloaded_force{3.5};
  double slack_fraction_warn{0.1};
  double slack_fraction_b_floor{1e-9};
  bool debug_dump{false};
  double breakage_cooldown_time{0.05};
};

// ---------------------------------------------------------------------------
// Host-side, non-CUDA representation of a problem.
//
// All matrices keep the original (column-major / CSC) layout from breakage
// so that we can hand them to OSQP unchanged. The CUDA layer converts to its
// own representation in src/system.cpp.
// ---------------------------------------------------------------------------

struct HostSystem {
  // Shapes (mirror BreakageSystem::check_shape).
  int num_parts{};
  int num_clutches{};
  int num_contact_vertices{};
  int num_vars{};
  int num_eq{};
  int num_ineq{};
  int num_relaxed_ineq{};

  double total_mass{};
  double L0{};

  // Per-part dense data.
  VectorXd mass;       // (num_parts,)
  MatrixX4d q_CC;      // (num_parts, 4) xyzw
  MatrixX3d c_CC;      // (num_parts, 3) m
  // I_CC stored as (num_parts, 9) row-major flattening of 3x3 inertia tensors.
  Eigen::Matrix<double, Dynamic, 9, RowMajor> I_CC;

  // Sparse matrices (CSC, owned, compressed).
  SparseMatrixCSC Q;   // (num_vars,        num_vars)
  SparseMatrixCSC A;   // (6*num_parts,     num_vars)
  SparseMatrixCSC G;   // (num_ineq,        num_vars)
  SparseMatrixCSC H;   // (num_relaxed_ineq, num_vars)
  SparseMatrixCSC V;   // (num_relaxed_ineq, num_clutches)

  // Per-clutch friction-cone metadata. Indices into clutches_; same length as
  // capacities.
  std::vector<int> capacity_clutch_indices;
  // (capacity_clutch_indices.size(), 9) row-major flattening of Vector9d.
  Eigen::Matrix<double, Dynamic, 9, RowMajor> capacities;
  // (num_clutches, 4) row-major flattening of 2x2 whitening matrices.
  Eigen::Matrix<double, Dynamic, 4, RowMajor> clutch_whiten;
};

// ---------------------------------------------------------------------------
// Per-frame input (mirror BreakageInput / BreakageInitialInput).
// ---------------------------------------------------------------------------

struct HostInput {
  // Initial-input fields.
  MatrixX3d w;         // angular velocity        (num_parts, 3)
  MatrixX3d v;         // linear velocity         (num_parts, 3)
  MatrixX4d q;         // orientation xyzw        (num_parts, 4)
  MatrixX3d c;         // COM position            (num_parts, 3)

  // Step-only fields.
  double dt{};
  MatrixX3d J;         // external linear impulse (num_parts, 3)
  MatrixX3d H;         // external angular imp.   (num_parts, 3)
};

// ---------------------------------------------------------------------------
// Per-frame solver state across steps (mirror BreakageState + OsqpState).
//
// The QP warm-start payload is opaque to the system layer; both the CPU and
// CUDA solvers cache their own state behind the InternalQpState pointer.
// ---------------------------------------------------------------------------

struct InternalQpState; // forward declaration; defined by each solver backend.
using InternalQpStatePtr = std::shared_ptr<InternalQpState>;

struct HostState {
  Eigen::Quaterniond q_W_CC_prev = Eigen::Quaterniond::Identity();
  MatrixX3d v_W_prev;  // (num_parts, 3)
  MatrixX3d L_prev;    // (num_parts, 3)

  // CPU baseline keeps three OSQP solvers + their warm-start vectors here.
  // CUDA backend keeps device buffers here. Either may be empty.
  InternalQpStatePtr qp;
};

// ---------------------------------------------------------------------------
// Per-stage info from a single solve.
// ---------------------------------------------------------------------------

struct StageInfo {
  bool converged{false};
  int iter{0};
  double prim_res{std::numeric_limits<double>::quiet_NaN()};
  double dual_res{std::numeric_limits<double>::quiet_NaN()};
  double rho{0.0};
  double solve_time_s{0.0};
};

struct SolveInfo {
  bool converged{false};
  StageInfo prj;
  StageInfo rlx;
  StageInfo opt;
};

// ---------------------------------------------------------------------------
// Result of a single solve.
// ---------------------------------------------------------------------------

struct Solution {
  VectorXd x;            // (num_vars,) un-whitened (matches breakage.cppm output)
  VectorXd utilization;  // (num_clutches,) per-clutch friction utilization
  double slack_fraction{0.0};
  SolveInfo info;
};

// ---------------------------------------------------------------------------
// JSON loaders.
//
// Layout: identical to BreakageDebugDump JSON (see breakage.cppm to_json).
// ---------------------------------------------------------------------------

void from_json(const nlohmann::json &j, Thresholds &out);
void from_json(const nlohmann::json &j, HostSystem &out);
void from_json(const nlohmann::json &j, HostInput &out);
void from_json(const nlohmann::json &j, HostState &out);

struct DebugDump {
  Thresholds thresholds;
  HostSystem system;
  HostInput input;
  HostState state;
  Solution solution;
  VectorXd b;
  std::optional<HostState> prev_state;
};

void from_json(const nlohmann::json &j, DebugDump &out);

DebugDump load_debug_dump(const std::string &path);

// ---------------------------------------------------------------------------
// Cross-validation helpers (used in tests).
// ---------------------------------------------------------------------------

// Compares two solutions' x within a relative tolerance (||a-b||_inf / max(||b||_inf, floor)).
double relative_inf_error(const VectorXd &a, const VectorXd &b,
                          double floor = 1e-12);

} // namespace breakage_cuda
