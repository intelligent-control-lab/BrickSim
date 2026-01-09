export module lego_assemble.physx.breakage;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.core.graph;
import lego_assemble.physx.touching_face_detection;
import lego_assemble.physx.polygon_clipping;
import lego_assemble.physx.osqp;
import lego_assemble.utils.transforms;
import lego_assemble.utils.unordered_pair;
import lego_assemble.utils.matrix_serialization;
import lego_assemble.utils.logging;
import lego_assemble.vendor;

namespace lego_assemble {

using Eigen::ComputeFullU;
using Eigen::ComputeFullV;
using Eigen::Dynamic;
using Eigen::Index;
using Eigen::JacobiSVD;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixBase;
using Eigen::Quaterniond;
using Eigen::RowMajor;
using Eigen::SparseMatrix;
using Eigen::Triplet;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Matrix3Xd = Matrix<double, 3, Dynamic>;
using Matrix4x3d = Matrix<double, 4, 3>;
using Matrix6d = Matrix<double, 6, 6>;
using Matrix6x3d = Matrix<double, 6, 3>;
using Matrix6x9d = Matrix<double, 6, 9>;
using MatrixX3d = Matrix<double, Dynamic, 3>;
using MatrixX4d = Matrix<double, Dynamic, 4>;

using QpSolver = OsqpSolver;
using QpSolverOptions = OsqpOptions;
using QpSolverState = OsqpState;
using QpSolverInfo = OsqpInfo;

struct AreaMoments {
	Vector2d centroid{Vector2d::Zero()};
	double area{};
	double Iuu{};
	double Ivv{};
	double Iuv{};

	Matrix3d gram_matrix() const {
		Matrix3d M = Matrix3d::Zero();
		M(0, 0) = area;
		M(1, 1) = Iuu;
		M(1, 2) = Iuv;
		M(2, 1) = Iuv;
		M(2, 2) = Ivv;
		return M;
	}

	static AreaMoments from(std::span<const Vector2d> p) {
		AreaMoments out;
		std::size_t N = p.size();
		if (N < 3) {
			return out;
		}

		double A2 = 0.0; // 2 * signed area
		double Cx_num = 0.0;
		double Cy_num = 0.0;

		// Second moments about origin:
		// Ixx0 = ∫ v^2 dA, Iyy0 = ∫ u^2 dA, Ixy0 = ∫ u v dA
		double Ixx_num = 0.0; // will divide by 12
		double Iyy_num = 0.0; // will divide by 12
		double Ixy_num = 0.0; // will divide by 24

		for (std::size_t i = 0; i < N; ++i) {
			const auto &a = p[i];
			const auto &b = p[(i + 1) % N];

			double x0 = a.x(), y0 = a.y();
			double x1 = b.x(), y1 = b.y();

			double cross = x0 * y1 - x1 * y0;
			A2 += cross;

			Cx_num += (x0 + x1) * cross;
			Cy_num += (y0 + y1) * cross;

			Ixx_num += (y0 * y0 + y0 * y1 + y1 * y1) * cross;
			Iyy_num += (x0 * x0 + x0 * x1 + x1 * x1) * cross;

			// Common form for polygon product of inertia
			Ixy_num += (x0 * y1 + 2 * x0 * y0 + 2 * x1 * y1 + x1 * y0) * cross;
		}

		// Robust degeneracy test (scale-free-ish)
		double absA2 = std::abs(A2);
		if (absA2 < 1e-18) { // tune if needed
			// fallback: average vertices
			Vector2d avg = Vector2d::Zero();
			for (auto &v : p) {
				avg += v;
			}
			avg /= double(N);
			out.centroid = avg;
			out.area = 0.0;
			return out;
		}

		// Centroid uses signed A2 (orientation cancels correctly)
		double invA2 = 1.0 / A2;
		double Cx = (Cx_num * invA2) / 3.0;
		double Cy = (Cy_num * invA2) / 3.0;

		// Convert to physical (positive) moments: flip if polygon is CW
		double Ixx0 = Ixx_num / 12.0;
		double Iyy0 = Iyy_num / 12.0;
		double Ixy0 = Ixy_num / 24.0;

		if (A2 < 0) {
			Ixx0 = -Ixx0;
			Iyy0 = -Iyy0;
			Ixy0 = -Ixy0;
		}

		// Central moments (about centroid)
		// Iyy0 corresponds to ∫ u^2 dA; Ixx0 corresponds to ∫ v^2 dA
		double area = 0.5 * absA2;
		out.area = area;
		out.centroid = {Cx, Cy};
		out.Iuu = Iyy0 - area * (Cx * Cx);
		out.Ivv = Ixx0 - area * (Cy * Cy);
		out.Iuv = Ixy0 - area * (Cx * Cy);
		return out;
	}

	static AreaMoments from_rect(double L, double W) {
		AreaMoments out;
		double A = L * W;
		out.area = A;
		out.Iuu = (A * L * L) / 12.0;
		out.Ivv = (A * W * W) / 12.0;
		out.Iuv = 0.0;
		return out;
	}
};

Transformd fit_se3(const MatrixX4d &q0, const MatrixX3d &t0,
                   const MatrixX4d &qx, const MatrixX3d &tx,
                   const VectorXd &mass, double total_mass, double lambda_R) {
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
	JacobiSVD<Matrix3d> svd{S, ComputeFullU | ComputeFullV};
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

struct TwistFitResult {
	Vector3d w0;
	Vector3d v0;
	MatrixX3d v_W;
};

TwistFitResult fit_twist(const MatrixX3d &w, const MatrixX3d &v,
                         const MatrixX3d &c_CC, const Transformd &T_W_CC,
                         const VectorXd &mass, double total_mass,
                         double lambda_w) {
	const auto &[q_W_CC, t_W_CC] = T_W_CC;
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
	result.v_W =
	    d.rowwise().cross(-result.w0).rowwise() + result.v0.transpose();
	return result;
}

Vector3d so3_log_from_unit_quat(Quaterniond q) {
	// log-map branch threshold on s = ||q.vec|| = sin(theta/2)
	constexpr double kSmallS = 5e-5;
	q.normalize();
	// Pick shortest representation (theta in [0, pi]) except the pi case (w==0) which is ambiguous anyway.
	if (q.w() < 0.0)
		q.coeffs() *= -1.0;
	Vector3d v = q.vec();
	double s = v.norm(); // sin(theta/2)
	double w = q.w();    // cos(theta/2)
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

// Inverse of the Jacobian corresponding to your report's convention:
// J(φ) = I + (1-cosθ)/θ^2 [φ]x + (θ-sinθ)/θ^3 [φ]x^2   (this is the "right Jacobian" in many texts)
Matrix3d so3_jacobian_inv(const Vector3d &phi) {
	// J^{-1} branch threshold on theta = ||phi||
	constexpr double kSmallTheta = 1e-4;
	Matrix3d Phi = phi.asSkewSymmetric();
	double theta2 = phi.squaredNorm();
	if (theta2 < kSmallTheta * kSmallTheta) {
		// c(θ) = 1/12 + θ^2/720 + θ^4/30240 + O(θ^6)
		double t2 = theta2;
		double t4 = t2 * t2;
		double c = (1.0 / 12.0) + (t2 / 720.0) + (t4 / 30240.0);
		return Matrix3d::Identity() - 0.5 * Phi + c * (Phi * Phi);
	}
	double theta = std::sqrt(theta2);
	double half = 0.5 * theta;
	double cot_half = std::cos(half) / std::sin(half);
	// c(θ) = 1/θ^2 - cot(θ/2)/(2θ)
	double c = (1.0 / theta2) - (cot_half / (2.0 * theta));
	return Matrix3d::Identity() - 0.5 * Phi + c * (Phi * Phi);
}

Matrix3d compute_Pi(const Quaterniond &qm, const Quaterniond &qp) {
	// Relative rotation: R_rel = Rm^T Rp
	return so3_jacobian_inv(so3_log_from_unit_quat(qm.conjugate() * qp)) *
	       qm.toRotationMatrix().transpose();
}

template <class Derived>
MatrixX3d compute_L(const MatrixBase<Derived> &Iflat, const Quaterniond &q_W_CC,
                    const Vector3d &w0) {
	Vector3d w_CC = q_W_CC.conjugate() * w0;
	return (Iflat.template leftCols<3>() * w_CC.x() +
	        Iflat.template middleCols<3>(3) * w_CC.y() +
	        Iflat.template rightCols<3>() * w_CC.z()) *
	       q_W_CC.toRotationMatrix().transpose();
}

Matrix6d transport_matrix(const Vector3d &x) {
	Matrix6d T = Matrix6d::Zero();
	T.block<3, 3>(0, 0) = Matrix3d::Identity();
	T.block<3, 3>(3, 3) = Matrix3d::Identity();
	T.block<3, 3>(3, 0) = x.asSkewSymmetric();
	return T;
}

template <class Derived>
void add_block_triplets(std::vector<Triplet<double>> &out, int row0, int col0,
                        const MatrixBase<Derived> &M, double tol = 0.0) {
	for (int r = 0; r < M.rows(); ++r) {
		for (int c = 0; c < M.cols(); ++c) {
			double v = M(r, c);
			if (std::abs(v) > tol) {
				out.emplace_back(row0 + r, col0 + c, v);
			}
		}
	}
}

export struct BreakageThresholds {
	bool Enabled{true};
	double ContactNormalCompliance{1e-2};
	double ClutchNormalCompliance{1.0};
	double ClutchShearCompliance{2.0};
	double MaxClutchNormalForce{0.7}; // in N
	double MaxClutchShearForce{0.7};  // in N
	double SlackFractionWarn{0.1};
	double SlackFractionBFloor{1e-9};
};

export void to_json(nlohmann::ordered_json &j,
                    const BreakageThresholds &thresholds) {
	j = {
	    {"enabled", thresholds.Enabled},
	    {"contact_normal_compliance", thresholds.ContactNormalCompliance},
	    {"clutch_normal_compliance", thresholds.ClutchNormalCompliance},
	    {"clutch_shear_compliance", thresholds.ClutchShearCompliance},
	    {"max_clutch_normal_force", thresholds.MaxClutchNormalForce},
	    {"max_clutch_shear_force", thresholds.MaxClutchShearForce},
	    {"slack_fraction_warn", thresholds.SlackFractionWarn},
	    {"slack_fraction_b_floor", thresholds.SlackFractionBFloor},
	};
}

export void from_json(const nlohmann::ordered_json &j,
                      BreakageThresholds &thresholds) {
	j.at("enabled").get_to(thresholds.Enabled);
	j.at("contact_normal_compliance")
	    .get_to(thresholds.ContactNormalCompliance);
	j.at("clutch_normal_compliance").get_to(thresholds.ClutchNormalCompliance);
	j.at("clutch_shear_compliance").get_to(thresholds.ClutchShearCompliance);
	j.at("max_clutch_normal_force").get_to(thresholds.MaxClutchNormalForce);
	j.at("max_clutch_shear_force").get_to(thresholds.MaxClutchShearForce);
	j.at("slack_fraction_warn").get_to(thresholds.SlackFractionWarn);
	j.at("slack_fraction_b_floor").get_to(thresholds.SlackFractionBFloor);
}

// FaceRef compares pid then fid,
// UnorderedPair ensures fref1 <= fref2,
// thus pid1 <= pid2,
// which agrees with the ordering in TouchingFacePair.
using FaceRefPair = UnorderedPair<FaceRef>;

export class BreakageSystem {
  public:
	int num_parts() const {
		return num_parts_;
	}
	int num_contacts() const {
		return num_contacts_;
	}
	int num_clutches() const {
		return num_clutches_;
	}
	int num_vars() const {
		return num_vars_;
	}
	int num_eq() const {
		return num_eq_;
	}
	int num_ineq() const {
		return num_ineq_;
	}
	std::span<const PartId> part_ids() const {
		return pids_;
	}
	const std::unordered_map<PartId, int> &part_id_to_index() const {
		return pid_to_index_;
	}
	std::span<const FaceRefPair> contact_pairs() const {
		return contact_pairs_;
	}
	const std::unordered_map<FaceRefPair, int> &contact_pair_to_index() const {
		return contact_pair_to_index_;
	}
	std::span<const ConnSegId> clutch_ids() const {
		return clutches_;
	}
	const std::unordered_map<ConnSegId, int> &clutch_id_to_index() const {
		return clutch_to_index_;
	}
	const VectorXd &mass() const {
		return mass_;
	}

	bool check_shape() const {
		return (num_parts_ > 0) && (num_contacts_ >= 0) &&
		       (num_clutches_ >= 0) &&
		       (num_vars_ == 3 * num_contacts_ + 9 * num_clutches_) &&
		       (num_eq_ == 6 * num_parts_) &&
		       (pids_.size() == static_cast<std::size_t>(num_parts_)) &&
		       (pid_to_index_.size() == static_cast<std::size_t>(num_parts_)) &&
		       (contact_pairs_.size() ==
		        static_cast<std::size_t>(num_contacts_)) &&
		       (contact_pair_to_index_.size() ==
		        static_cast<std::size_t>(num_contacts_)) &&
		       (clutches_.size() == static_cast<std::size_t>(num_clutches_)) &&
		       (clutch_to_index_.size() ==
		        static_cast<std::size_t>(num_clutches_)) &&
		       (total_mass_ > 0.0) && (mass_.size() == num_parts_) &&
		       (q_CC_.rows() == num_parts_) && (q_CC_.cols() == 4) &&
		       (c_CC_.rows() == num_parts_) && (c_CC_.cols() == 3) &&
		       (I_CC_.size() == static_cast<std::size_t>(num_parts_)) &&
		       (characteristic_radius_sq_ > 0.0) &&
		       (clutch_half_extents_.size() ==
		        static_cast<std::size_t>(num_clutches_)) &&
		       solver_.has_value() && (solver_->A().rows() == num_eq_) &&
		       (solver_->A().cols() == num_vars_) &&
		       (solver_->Q().rows() == num_vars_) &&
		       (solver_->Q().cols() == num_vars_) &&
		       (solver_->G().rows() == num_ineq_) &&
		       (solver_->G().cols() == num_vars_);
	}

  private:
	friend class BreakageChecker;
	friend void to_json(nlohmann::ordered_json &j, const BreakageSystem &sys);
	friend void from_json(const nlohmann::ordered_json &j, BreakageSystem &sys);

	int num_parts_{};
	int num_contacts_{};
	int num_clutches_{};
	int num_vars_{}; // 3 * num_contacts_ + 9 * num_clutches_
	int num_eq_{};   // 6 * num_parts_
	int num_ineq_{}; // sum_e |Vtx(\Omega_e)|

	std::vector<PartId> pids_{};
	std::unordered_map<PartId, int> pid_to_index_{};
	std::vector<FaceRefPair> contact_pairs_{};
	std::unordered_map<FaceRefPair, int> contact_pair_to_index_{};
	std::vector<ConnSegId> clutches_{};
	std::unordered_map<ConnSegId, int> clutch_to_index_{};

	// Total mass, in kg
	double total_mass_{};
	// Masses, in kg, num_parts_ x 1
	VectorXd mass_{};
	// Orientations in the CC frame, as quaternions, each row is x,y,z,w, num_parts_ x 4
	MatrixX4d q_CC_{};
	// COM positions in the CC frame, in m, num_parts_ x 3
	MatrixX3d c_CC_{};
	// Inertia tensors in the CC frame, num_parts_ x 3 x 3
	std::vector<Matrix3d> I_CC_{};
	// Squared characteristic length for regularization, in m^2
	double characteristic_radius_sq_;
	// Half extents of the clutch rectangles, in m, num_clutches_ x 2
	std::vector<Vector2d> clutch_half_extents_{};

	std::optional<QpSolver> solver_{};

	auto I_CC_matrix() const {
		return Map<const Matrix<double, Dynamic, 9, RowMajor>>{
		    reinterpret_cast<const double *>(I_CC_.data()),
		    static_cast<Index>(I_CC_.size()), 9};
	}

	auto I_CC_matrix() {
		return Map<Matrix<double, Dynamic, 9, RowMajor>>{
		    reinterpret_cast<double *>(I_CC_.data()),
		    static_cast<Index>(I_CC_.size()), 9};
	}

	auto clutch_half_extents_matrix() const {
		return Map<const Matrix<double, Dynamic, 2, RowMajor>>{
		    reinterpret_cast<const double *>(clutch_half_extents_.data()),
		    static_cast<Index>(clutch_half_extents_.size()), 2};
	}

	auto clutch_half_extents_matrix() {
		return Map<Matrix<double, Dynamic, 2, RowMajor>>{
		    reinterpret_cast<double *>(clutch_half_extents_.data()),
		    static_cast<Index>(clutch_half_extents_.size()), 2};
	}
};

export void to_json(nlohmann::ordered_json &j, const BreakageSystem &sys) {
	j = nlohmann::ordered_json{
	    {"num_parts", sys.num_parts_},
	    {"num_contacts", sys.num_contacts_},
	    {"num_clutches", sys.num_clutches_},
	    {"num_vars", sys.num_vars_},
	    {"num_eq", sys.num_eq_},
	    {"num_ineq", sys.num_ineq_},
	    {"A", matrix_to_json(sys.solver_->A())},
	    {"Q", matrix_to_json(sys.solver_->Q())},
	    {"G", matrix_to_json(sys.solver_->G())},
	    {"part_ids", sys.pids_},
	    {"contact_pairs", sys.contact_pairs_},
	    {"clutch_ids", sys.clutches_},
	    {"total_mass", sys.total_mass_},
	    {"mass", matrix_to_json(sys.mass_)},
	    {"q_CC", matrix_to_json(sys.q_CC_)},
	    {"c_CC", matrix_to_json(sys.c_CC_)},
	    {"I_CC", matrix_to_json(sys.I_CC_matrix())},
	    {"characteristic_radius_sq", sys.characteristic_radius_sq_},
	    {"clutch_half_extents",
	     matrix_to_json(sys.clutch_half_extents_matrix())},
	};
}

export void from_json(const nlohmann::ordered_json &j, BreakageSystem &sys) {
	j.at("num_parts").get_to(sys.num_parts_);
	j.at("num_contacts").get_to(sys.num_contacts_);
	j.at("num_clutches").get_to(sys.num_clutches_);
	j.at("num_vars").get_to(sys.num_vars_);
	j.at("num_eq").get_to(sys.num_eq_);
	j.at("num_ineq").get_to(sys.num_ineq_);
	sys.solver_.emplace(json_to_matrix<SparseMatrix<double>>(j.at("A")),
	                    json_to_matrix<SparseMatrix<double>>(j.at("Q")),
	                    json_to_matrix<SparseMatrix<double>>(j.at("G")));
	j.at("part_ids").get_to(sys.pids_);
	j.at("contact_pairs").get_to(sys.contact_pairs_);
	j.at("clutch_ids").get_to(sys.clutches_);
	j.at("total_mass").get_to(sys.total_mass_);
	sys.mass_ = json_to_matrix<VectorXd>(j.at("mass"));
	sys.q_CC_ = json_to_matrix<MatrixX4d>(j.at("q_CC"));
	sys.c_CC_ = json_to_matrix<MatrixX3d>(j.at("c_CC"));
	sys.I_CC_.resize(static_cast<std::size_t>(sys.num_parts_));
	sys.I_CC_matrix() =
	    json_to_matrix<Matrix<double, Dynamic, 9, RowMajor>>(j.at("I_CC"));
	j.at("characteristic_radius_sq").get_to(sys.characteristic_radius_sq_);
	sys.clutch_half_extents_.resize(
	    static_cast<std::size_t>(sys.num_clutches_));
	sys.clutch_half_extents_matrix() =
	    json_to_matrix<Matrix<double, Dynamic, 2, RowMajor>>(
	        j.at("clutch_half_extents"));

	// Rebuild index maps
	sys.pid_to_index_.clear();
	for (int i = 0; i < sys.num_parts_; ++i) {
		sys.pid_to_index_.emplace(sys.pids_[i], i);
	}
	sys.contact_pair_to_index_.clear();
	for (int i = 0; i < sys.num_contacts_; ++i) {
		sys.contact_pair_to_index_.emplace(sys.contact_pairs_[i], i);
	}
	sys.clutch_to_index_.clear();
	for (int i = 0; i < sys.num_clutches_; ++i) {
		sys.clutch_to_index_.emplace(sys.clutches_[i], i);
	}
}

export struct BreakageInitialInput {
	// Angular velocities of COMs, in rad/s, num_parts_ x 3
	MatrixX3d w{};
	// Linear velocities of COMs, in m/s, num_parts_ x 3
	MatrixX3d v{};
	// Orientations, as quaternions, each row is x,y,z,w, num_parts_ x 4
	MatrixX4d q{};
	// COM positions, in m, num_parts_ x 3
	MatrixX3d c{};

	bool check_shape(const BreakageSystem &sys) const {
		return (w.rows() == sys.num_parts()) && (w.cols() == 3) &&
		       (v.rows() == sys.num_parts()) && (v.cols() == 3) &&
		       (q.rows() == sys.num_parts()) && (q.cols() == 4) &&
		       (c.rows() == sys.num_parts()) && (c.cols() == 3);
	}
};

export void to_json(nlohmann::ordered_json &j,
                    const BreakageInitialInput &input) {
	j = nlohmann::ordered_json{
	    {"w", matrix_to_json(input.w)},
	    {"v", matrix_to_json(input.v)},
	    {"q", matrix_to_json(input.q)},
	    {"c", matrix_to_json(input.c)},
	};
}

export void from_json(const nlohmann::ordered_json &j,
                      BreakageInitialInput &input) {
	input.w = json_to_matrix<MatrixX3d>(j.at("w"));
	input.v = json_to_matrix<MatrixX3d>(j.at("v"));
	input.q = json_to_matrix<MatrixX4d>(j.at("q"));
	input.c = json_to_matrix<MatrixX3d>(j.at("c"));
}

export struct BreakageInput : public BreakageInitialInput {
	// Duration of the simulation step, in seconds
	double dt{};
	// External linear impulses w.r.t. COMs, in Ns, num_parts_ x 3
	MatrixX3d J{};
	// External angular impulses w.r.t. COMs, in Ns*m, num_parts_ x 3
	MatrixX3d H{};

	bool check_shape(const BreakageSystem &sys) const {
		return (dt > 0.0) && (J.rows() == sys.num_parts()) && (J.cols() == 3) &&
		       (H.rows() == sys.num_parts()) && (H.cols() == 3);
	}
};

export void to_json(nlohmann::ordered_json &j, const BreakageInput &input) {
	to_json(j, static_cast<const BreakageInitialInput &>(input));
	j["dt"] = input.dt;
	j["J"] = matrix_to_json(input.J);
	j["H"] = matrix_to_json(input.H);
}

export void from_json(const nlohmann::ordered_json &j, BreakageInput &input) {
	from_json(j, static_cast<BreakageInitialInput &>(input));
	j.at("dt").get_to(input.dt);
	input.J = json_to_matrix<MatrixX3d>(j.at("J"));
	input.H = json_to_matrix<MatrixX3d>(j.at("H"));
}

export class BreakageState {
  public:
	bool check_shape(const BreakageSystem &sys) const {
		return (v_W_prev.rows() == sys.num_parts()) && (v_W_prev.cols() == 3) &&
		       (L_prev.rows() == sys.num_parts()) && (L_prev.cols() == 3);
	}

  private:
	friend class BreakageChecker;
	friend void to_json(nlohmann::ordered_json &j, const BreakageState &state);
	friend void from_json(const nlohmann::ordered_json &j,
	                      BreakageState &state);

	Quaterniond q_W_CC_prev{};
	MatrixX3d v_W_prev{};
	MatrixX3d L_prev{};
	QpSolverState solver_state{};
};

export void to_json(nlohmann::ordered_json &j, const BreakageState &state) {
	j = nlohmann::ordered_json{
	    {"q_W_CC_prev", matrix_to_json(state.q_W_CC_prev.coeffs())},
	    {"v_W_prev", matrix_to_json(state.v_W_prev)},
	    {"L_prev", matrix_to_json(state.L_prev)},
	    {"solver_state", state.solver_state},
	};
}

export void from_json(const nlohmann::ordered_json &j, BreakageState &state) {
	state.q_W_CC_prev.coeffs() = json_to_matrix<Vector4d>(j.at("q_W_CC_prev"));
	state.v_W_prev = json_to_matrix<MatrixX3d>(j.at("v_W_prev"));
	state.L_prev = json_to_matrix<MatrixX3d>(j.at("L_prev"));
	j.at("solver_state").get_to(state.solver_state);
}

export struct BreakageSolution {
	VectorXd x{};
	VectorXd u_n{};
	VectorXd u_s{};
	VectorXd utilization{};
	QpSolverInfo info{};

	// ||A*x - b|| / max(||b||, floor)
	double slack_fraction{};
};

export void to_json(nlohmann::ordered_json &j,
                    const BreakageSolution &solution) {
	j = nlohmann::ordered_json{
	    {"x", matrix_to_json(solution.x)},
	    {"u_n", matrix_to_json(solution.u_n)},
	    {"u_s", matrix_to_json(solution.u_s)},
	    {"utilization", matrix_to_json(solution.utilization)},
	    {"info", solution.info},
	    {"slack_fraction", solution.slack_fraction},
	};
}

export void from_json(const nlohmann::ordered_json &j,
                      BreakageSolution &solution) {
	solution.x = json_to_matrix<VectorXd>(j.at("x"));
	solution.u_n = json_to_matrix<VectorXd>(j.at("u_n"));
	solution.u_s = json_to_matrix<VectorXd>(j.at("u_s"));
	solution.utilization = json_to_matrix<VectorXd>(j.at("utilization"));
	j.at("info").get_to(solution.info);
	j.at("slack_fraction").get_to(solution.slack_fraction);
}

export struct BreakageDebugDump {
	BreakageThresholds thresholds{};
	BreakageSystem system{};
	BreakageInput input{};
	BreakageState state{};
	BreakageSolution solution{};
	VectorXd b{};
	std::optional<BreakageState> prev_state{};
};

export void to_json(nlohmann::ordered_json &j, const BreakageDebugDump &dump) {
	j = nlohmann::ordered_json{
	    {"thresholds", dump.thresholds}, {"system", dump.system},
	    {"input", dump.input},           {"state", dump.state},
	    {"solution", dump.solution},     {"b", matrix_to_json(dump.b)},
	};
	if (dump.prev_state.has_value()) {
		j["prev_state"] = *dump.prev_state;
	}
}

export void from_json(const nlohmann::ordered_json &j,
                      BreakageDebugDump &dump) {
	j.at("thresholds").get_to(dump.thresholds);
	j.at("system").get_to(dump.system);
	j.at("input").get_to(dump.input);
	j.at("state").get_to(dump.state);
	j.at("solution").get_to(dump.solution);
	dump.b = json_to_matrix<VectorXd>(j.at("b"));
	if (j.contains("prev_state")) {
		dump.prev_state.emplace();
		j.at("prev_state").get_to(*dump.prev_state);
	}
}

export class BreakageChecker {
  public:
	BreakageThresholds thresholds;

	BreakageChecker(const BreakageThresholds &thresholds = {})
	    : thresholds{thresholds} {}

	template <class Graph>
	BreakageSystem build_system(const Graph &g, PartId rep) const {
		BreakageSystem sys;
		auto cc_view = g.component_view(rep);
		sys.num_parts_ = static_cast<int>(cc_view.size());

		// COMs in body frame
		std::vector<Transformd> T_CC_parts;
		T_CC_parts.reserve(sys.num_parts_);

		sys.pids_.reserve(sys.num_parts_);
		sys.pid_to_index_.reserve(sys.num_parts_);
		sys.mass_.resize(sys.num_parts_);
		sys.q_CC_.resize(sys.num_parts_, 4);
		sys.c_CC_.resize(sys.num_parts_, 3);
		sys.I_CC_.reserve(sys.num_parts_);
		for (auto [u, T_CC_u] : cc_view.transforms()) {
			int index_u = static_cast<int>(sys.pids_.size());
			sys.pids_.emplace_back(u);
			sys.pid_to_index_.emplace(u, index_u);

			const auto &[q_CC_u, t_CC_u] = T_CC_u;
			T_CC_parts.emplace_back(T_CC_u);
			sys.q_CC_.row(index_u) = q_CC_u.coeffs();

			g.parts().visit(u, [&](const auto &pw) {
				const auto &part = pw.wrapped();

				sys.c_CC_.row(index_u) = t_CC_u + q_CC_u * part.com();

				double mass = part.mass();
				sys.mass_(index_u) = mass;
				sys.total_mass_ += mass;

				Matrix3d R = q_CC_u.toRotationMatrix();
				sys.I_CC_.emplace_back(R * part.inertia_tensor() *
				                       R.transpose());
			});
		}

		std::vector<Triplet<double>> A_triplets;
		std::vector<Triplet<double>> Q_triplets;
		std::vector<Triplet<double>> G_triplets;

		for (TouchingFacePair p : detect_touching_faces(g, rep)) {
			std::vector<Vector2d> intersection =
			    convex_polygon_intersection(p.polygon_u, p.polygon_v);
			if (intersection.size() < 3) {
				continue;
			}
			AreaMoments m = AreaMoments::from(intersection);
			if (m.area < 1e-12) {
				continue;
			}
			int index_c = sys.num_contacts_++;
			sys.contact_pairs_.emplace_back(p.face_u.ref, p.face_v.ref);
			sys.contact_pair_to_index_.emplace(sys.contact_pairs_.back(),
			                                   index_c);

			PartId pid_i = p.face_u.ref.pid;
			PartId pid_j = p.face_v.ref.pid;
			int index_i = sys.pid_to_index_.at(pid_i);
			int index_j = sys.pid_to_index_.at(pid_j);

			const Transformd &T_CC_fu = p.face_u.T;
			const auto &[q_CC_fu, t_CC_fu] = T_CC_fu;
			const Quaterniond &q_CC_centroid = q_CC_fu;
			const Vector3d t_CC_centroid =
			    t_CC_fu +
			    q_CC_fu * Vector3d{m.centroid.x(), m.centroid.y(), 0.0};

			Matrix3d M = m.gram_matrix();
			Matrix3d::ColXpr g_0 = M.col(0);
			Matrix3d::ColXpr g_u = M.col(1);
			Matrix3d::ColXpr g_v = M.col(2);
			Vector3d u_hat = q_CC_centroid * Vector3d::UnitX();
			Vector3d v_hat = q_CC_centroid * Vector3d::UnitY();
			Vector3d n_hat = q_CC_centroid * Vector3d::UnitZ();
			Vector3d t_CC_com_i = sys.c_CC_.row(index_i);
			Vector3d t_CC_com_j = sys.c_CC_.row(index_j);
			Matrix6d T_i = transport_matrix(t_CC_centroid - t_CC_com_i);
			Matrix6d T_j = transport_matrix(t_CC_centroid - t_CC_com_j);

			Map<const Matrix<double, Dynamic, 2, RowMajor>> V{
			    reinterpret_cast<double *>(intersection.data()),
			    static_cast<Index>(intersection.size()), 2};

			Matrix6x3d J;
			J.block<3, 3>(0, 0) = -n_hat * g_0.transpose();
			J.block<3, 3>(3, 0) =
			    v_hat * g_u.transpose() - u_hat * g_v.transpose();
			Matrix6x3d A_i = T_i * J;
			Matrix6x3d A_j = -T_j * J;
			Matrix3d Q = thresholds.ContactNormalCompliance * M;
			MatrixX3d G;
			G.resize(intersection.size(), 3);
			G.col(0) = VectorXd::Ones(intersection.size());
			G.block(0, 1, intersection.size(), 2) =
			    V.rowwise() - m.centroid.transpose();

			int var_idx = 3 * index_c;
			add_block_triplets(A_triplets, 6 * index_i, var_idx, A_i);
			add_block_triplets(A_triplets, 6 * index_j, var_idx, A_j);
			add_block_triplets(Q_triplets, var_idx, var_idx, Q);
			int ineq_idx = sys.num_ineq_;
			sys.num_ineq_ += static_cast<int>(intersection.size());
			add_block_triplets(G_triplets, ineq_idx, var_idx, G);
		}

		for (typename Graph::ConnSegConstEntry cs_entry :
		     cc_view.connection_segments()) {
			int index_k = sys.num_clutches_++;
			ConnSegId csid = cs_entry.template key<ConnSegId>();
			sys.clutches_.emplace_back(csid);
			sys.clutch_to_index_.emplace(csid, index_k);

			const auto &[stud_ifref, hole_ifref] =
			    cs_entry.template key<ConnSegRef>();
			const auto &[stud_pid, stud_ifid] = stud_ifref;
			const auto &[hole_pid, hole_ifid] = hole_ifref;
			InterfaceSpec stud_spec = g.interface_spec_at(stud_ifref);
			InterfaceSpec hole_spec = g.interface_spec_at(hole_ifref);
			ConnectionLocalTransform conn =
			    cs_entry.value().wrapped().compute_local_transform(stud_spec,
			                                                       hole_spec);
			Vector2d overlap = BrickUnitLength * conn.overlap.cast<double>();
			Vector2d p = overlap / 2.0;
			AreaMoments m = AreaMoments::from_rect(overlap.x(), overlap.y());
			int index_i = sys.pid_to_index_.at(stud_pid);
			int index_j = sys.pid_to_index_.at(hole_pid);
			const Transformd &T_CC_i = T_CC_parts[index_i];
			Transformd T_CC_centroid = T_CC_i * conn.T_stud_local;
			const auto &[q_CC_centroid, t_CC_centroid] = T_CC_centroid;

			Matrix3d M = m.gram_matrix();
			Matrix3d::ColXpr g_0 = M.col(0);
			Matrix3d::ColXpr g_u = M.col(1);
			Matrix3d::ColXpr g_v = M.col(2);
			Vector3d u_hat = q_CC_centroid * Vector3d::UnitX();
			Vector3d v_hat = q_CC_centroid * Vector3d::UnitY();
			Vector3d n_hat = q_CC_centroid * Vector3d::UnitZ();
			const Vector3d &t_CC_com_i = sys.c_CC_.row(index_i);
			const Vector3d &t_CC_com_j = sys.c_CC_.row(index_j);
			Matrix6d T_i = transport_matrix(t_CC_centroid - t_CC_com_i);
			Matrix6d T_j = transport_matrix(t_CC_centroid - t_CC_com_j);

			Matrix6x9d J;
			J.block<3, 3>(0, 0) = n_hat * g_0.transpose();
			J.block<3, 3>(0, 3) = u_hat * g_0.transpose();
			J.block<3, 3>(0, 6) = v_hat * g_0.transpose();
			J.block<3, 3>(3, 0) =
			    u_hat * g_v.transpose() - v_hat * g_u.transpose();
			J.block<3, 3>(3, 3) = -n_hat * g_v.transpose();
			J.block<3, 3>(3, 6) = n_hat * g_u.transpose();
			Matrix6x9d A_i = T_i * J;
			Matrix6x9d A_j = -T_j * J;

			int var_idx = 3 * sys.num_contacts_ + 9 * index_k;
			add_block_triplets(A_triplets, 6 * index_i, var_idx, A_i);
			add_block_triplets(A_triplets, 6 * index_j, var_idx, A_j);

			add_block_triplets(Q_triplets, var_idx + 0, var_idx + 0,
			                   thresholds.ClutchNormalCompliance * M);
			add_block_triplets(Q_triplets, var_idx + 3, var_idx + 3,
			                   thresholds.ClutchShearCompliance * M);
			add_block_triplets(Q_triplets, var_idx + 6, var_idx + 6,
			                   thresholds.ClutchShearCompliance * M);

			Matrix4x3d G_block{
			    {1.0, p.x(), p.y()},
			    {1.0, -p.x(), p.y()},
			    {1.0, -p.x(), -p.y()},
			    {1.0, p.x(), -p.y()},
			};
			int ineq_idx = sys.num_ineq_;
			sys.num_ineq_ += 4;
			add_block_triplets(G_triplets, ineq_idx, var_idx, G_block);

			sys.clutch_half_extents_.emplace_back(p);
		}

		sys.num_vars_ = 3 * sys.num_contacts_ + 9 * sys.num_clutches_;
		sys.num_eq_ = 6 * sys.num_parts_;
		SparseMatrix<double> A, Q, G;
		A.resize(sys.num_eq_, sys.num_vars_);
		Q.resize(sys.num_vars_, sys.num_vars_);
		G.resize(sys.num_ineq_, sys.num_vars_);
		A.setFromTriplets(A_triplets.begin(), A_triplets.end());
		Q.setFromTriplets(Q_triplets.begin(), Q_triplets.end());
		G.setFromTriplets(G_triplets.begin(), G_triplets.end());
		sys.solver_.emplace(std::move(A), std::move(Q), std::move(G));

		Vector3d weighted_sum_pos = sys.c_CC_.transpose() * sys.mass_;
		double weighted_sum_sq_norm =
		    sys.c_CC_.rowwise().squaredNorm().dot(sys.mass_);
		sys.characteristic_radius_sq_ =
		    (weighted_sum_sq_norm -
		     weighted_sum_pos.squaredNorm() / sys.total_mass_) /
		    sys.total_mass_;
		if (sys.characteristic_radius_sq_ < 1e-6) {
			sys.characteristic_radius_sq_ = 1e-6;
		}

		if (!sys.check_shape()) {
			throw std::runtime_error(
			    "build_breakage_system: invalid system constructed");
		}
		return sys;
	}

	BreakageState build_initial_state(const BreakageSystem &sys,
	                                  const BreakageInitialInput &in) const {
		if (!sys.check_shape()) {
			throw std::runtime_error("solve_breakage: invalid system");
		}
		if (!in.check_shape(sys)) {
			throw std::runtime_error("solve_breakage: invalid input");
		}
		Transformd T_W_CC_curr =
		    fit_se3(sys.q_CC_, sys.c_CC_, in.q, in.c, sys.mass_,
		            sys.total_mass_, sys.characteristic_radius_sq_ / 2.0);
		auto &[q_W_CC_curr, t_W_CC_curr] = T_W_CC_curr;
		TwistFitResult twist_curr =
		    fit_twist(in.w, in.v, sys.c_CC_, T_W_CC_curr, sys.mass_,
		              sys.total_mass_, sys.characteristic_radius_sq_);
		auto &[w0_curr, v0_curr, v_W_curr] = twist_curr;
		MatrixX3d L_curr =
		    compute_L(sys.I_CC_matrix(), q_W_CC_curr, twist_curr.w0);
		BreakageState state;
		state.q_W_CC_prev = q_W_CC_curr;
		state.v_W_prev = std::move(v_W_curr);
		state.L_prev = std::move(L_curr);
		if (!state.check_shape(sys)) {
			throw std::runtime_error(
			    "build_initial_state: invalid state after initialization");
		}
		return state;
	}

	BreakageSolution solve(const BreakageSystem &sys, const BreakageInput &in,
	                       BreakageState &state) const {
		if (!sys.check_shape()) {
			throw std::runtime_error("solve_breakage: invalid system");
		}
		if (!in.check_shape(sys)) {
			throw std::runtime_error("solve_breakage: invalid input");
		}
		if (!state.check_shape(sys)) {
			throw std::runtime_error("solve_breakage: invalid state");
		}
		Transformd T_W_CC_curr =
		    fit_se3(sys.q_CC_, sys.c_CC_, in.q, in.c, sys.mass_,
		            sys.total_mass_, sys.characteristic_radius_sq_ / 2.0);
		auto &[q_W_CC_curr, t_W_CC_curr] = T_W_CC_curr;
		TwistFitResult twist_curr =
		    fit_twist(in.w, in.v, sys.c_CC_, T_W_CC_curr, sys.mass_,
		              sys.total_mass_, sys.characteristic_radius_sq_);
		auto &[w0_curr, v0_curr, v_W_curr] = twist_curr;
		MatrixX3d L_curr =
		    compute_L(sys.I_CC_matrix(), q_W_CC_curr, twist_curr.w0);
		Index N = sys.num_parts_;
		VectorXd b;
		b.resize(6 * N);
		Map<Matrix<double, Dynamic, 6, RowMajor>> b_mat{b.data(), N, 6};
		const Quaterniond &q_W_CC_prev = state.q_W_CC_prev;
		const MatrixX3d &v_W_prev = state.v_W_prev;
		const MatrixX3d &L_prev = state.L_prev;
		Matrix3d Pi = compute_Pi(q_W_CC_prev, q_W_CC_curr);
		b_mat.leftCols<3>() =
		    (((v_W_curr - v_W_prev).array().colwise() * sys.mass_.array())
		         .matrix() -
		     in.J) *
		    Pi.transpose() / in.dt;
		b_mat.rightCols<3>() =
		    ((L_curr - L_prev - in.H) * Pi.transpose()) / in.dt;

		std::unique_ptr<BreakageState> prev_state;
		if (manual_dump_) {
			// Only snapshot previous state if manual dump is enabled
			prev_state = std::make_unique<BreakageState>(state);
		}

		BreakageSolution sol;

		if (!thresholds.Enabled) {
			// Note we still update the state above even if breakage is disabled
			// So the state is always up-to-date, and ready for when breakage is enabled again
			sol.x = VectorXd::Zero(sys.num_vars_);
			sol.utilization = VectorXd::Constant(sys.num_clutches_, -1.0);
			// sol.info is default-initialized
			return sol;
		}

		sol.info = sys.solver_->solve(b, state.solver_state);
		sol.x = state.solver_state.x;
		VectorXd slack = sys.solver_->A() * sol.x - b;
		sol.slack_fraction =
		    slack.norm() / std::max(b.norm(), thresholds.SlackFractionBFloor);

		state.q_W_CC_prev = q_W_CC_curr;
		state.v_W_prev = std::move(v_W_curr);
		state.L_prev = std::move(L_curr);
		if (!state.check_shape(sys)) {
			throw std::runtime_error(
			    "solve_breakage: invalid state after update");
		}

		if (sol.info.converged) {
			auto extents = sys.clutch_half_extents_matrix();
			Map<const Matrix<double, Dynamic, 9, RowMajor>> Xk{
			    sol.x.data() + 3 * sys.num_contacts_, sys.num_clutches_, 9};
			double normal_scale = BrickUnitLength * BrickUnitLength /
			                      thresholds.MaxClutchNormalForce;
			sol.u_n =
			    normal_scale *
			    (Xk.col(0) + Xk.col(1).cwiseAbs().cwiseProduct(extents.col(0)) +
			     Xk.col(2).cwiseAbs().cwiseProduct(extents.col(1)));
			sol.u_n = sol.u_n.cwiseMax(0.0);
			double shear_scale = BrickUnitLength * BrickUnitLength /
			                     thresholds.MaxClutchShearForce;
			VectorXd tau_u = Xk.col(3) +
			                 Xk.col(4).cwiseAbs().cwiseProduct(extents.col(0)) +
			                 Xk.col(5).cwiseAbs().cwiseProduct(extents.col(1));
			VectorXd tau_v = Xk.col(6) +
			                 Xk.col(7).cwiseAbs().cwiseProduct(extents.col(0)) +
			                 Xk.col(8).cwiseAbs().cwiseProduct(extents.col(1));
			sol.u_s =
			    shear_scale * (tau_u.array().square() + tau_v.array().square())
			                      .sqrt()
			                      .matrix();
			sol.u_s = sol.u_s.cwiseMax(0.0);
			sol.utilization = sol.u_n.cwiseMax(sol.u_s);

		} else {
			sol.u_n = VectorXd::Constant(sys.num_clutches_, -1.0);
			sol.u_s = VectorXd::Constant(sys.num_clutches_, -1.0);
			sol.utilization = VectorXd::Constant(sys.num_clutches_, -1.0);
		}

		bool error = false;
		if (!sol.info.converged) {
			log_error(
			    "BreakageChecker: solver failed to converge, "
			    "r_eq_norm={:.3e}, r_ineq_norm={:.3e}, r_kkt_norm={:.3e}, "
			    "eps_eq={:.3e}, eps_ineq={:.3e}, eps_kkt={:.3e}",
			    sol.info.r_eq_norm, sol.info.r_ineq_norm, sol.info.r_kkt_norm,
			    sol.info.eps_eq, sol.info.eps_ineq, sol.info.eps_kkt);
			error = true;
		} else if (sol.slack_fraction > thresholds.SlackFractionWarn) {
			log_warn("BreakageChecker: abnormally high slack fraction {:.3e}",
			         sol.slack_fraction);
			error = true;
		}
		bool dump = false;
		if (error && !dumped_on_error_) {
			dump = true;
			dumped_on_error_ = true;
		}
		if (manual_dump_) {
			dump = true;
		}
		if (dump) {
			dump_debug_data(sys, in, state, sol, b, std::move(prev_state));
		}

		return sol;
	}

	void set_debug_dump_dir(std::string dir) {
		debug_dump_dir_ = std::move(dir);
	}

	void enable_manual_debug_dump(bool enable) {
		manual_dump_ = enable;
	}

  private:
	std::string debug_dump_dir_;
	bool manual_dump_{false};
	mutable bool dumped_on_error_{false};

	void dump_debug_data(const BreakageSystem &sys, const BreakageInput &in,
	                     const BreakageState &state,
	                     const BreakageSolution &sol, const VectorXd &b,
	                     std::unique_ptr<BreakageState> prev_state) const {
		BreakageDebugDump dump{
		    .thresholds = thresholds,
		    .system = sys,
		    .input = in,
		    .state = state,
		    .solution = sol,
		    .b = b,
		};
		if (prev_state) {
			dump.prev_state.emplace(std::move(*prev_state));
		}
		nlohmann::ordered_json j = dump;

		std::time_t t = std::chrono::system_clock::to_time_t(
		    std::chrono::system_clock::now());
		std::tm tm = *std::localtime(&t);
		char ts[32];
		std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);

		std::filesystem::path out_dir{debug_dump_dir_};
		std::string filename = std::format("breakage_debug_{}.json", ts);
		std::filesystem::path filepath = out_dir / filename;
		for (int seq = 1; std::filesystem::exists(filepath); ++seq) {
			filename = std::format("breakage_debug_{}_{}.json", ts, seq);
			filepath = out_dir / filename;
		}

		std::ofstream ofs(filepath);
		if (!ofs) {
			log_error("BreakageChecker: failed to open debug dump file {}",
			          filepath.string());
			return;
		}
		ofs << j.dump(4);
		log_info("BreakageChecker: dumped debug data to {}", filepath.string());
	}
};

} // namespace lego_assemble
