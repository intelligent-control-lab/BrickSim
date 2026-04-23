import std;
import bricksim.physx.breakage;
import bricksim.utils.transforms;
import bricksim.vendor;

#include <cassert>

using namespace bricksim;

using Eigen::AngleAxisd;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using MatrixX3d = Matrix<double, Dynamic, 3>;
using MatrixX4d = Matrix<double, Dynamic, 4>;
using MatrixX9d = Matrix<double, Dynamic, 9>;

// --------------------- helpers ---------------------

static constexpr double kTol = 1e-9;

template <class A, class B>
static bool approx_mat(const A &a, const B &b, double tol = kTol) {
	if (a.rows() != b.rows() || a.cols() != b.cols())
		return false;
	double scale = 1.0 + std::max(a.cwiseAbs().maxCoeff(),
	                              b.cwiseAbs().maxCoeff());
	return (a - b).cwiseAbs().maxCoeff() <= tol * scale;
}

// Eigen stores `coeffs()` as (x, y, z, w). Same convention is used by the
// breakage code (`fit_se3` reads rows of the input quat matrix as (x,y,z,w)
// and feeds them into Quaterniond's vector ctor).
static Vector4d quat_xyzw(const Quaterniond &q) {
	return q.coeffs();
}

static Quaterniond axis_angle(const Vector3d &axis_unit, double angle) {
	return Quaterniond{AngleAxisd{angle, axis_unit}};
}

// Force evaluation of a 3-component Eigen expression into a Vector3d. Avoids
// the brace-init ambiguity between `Vector3d(MatrixBase&)` and
// `Vector3d(Scalar, Scalar, Scalar)` that some compilers stumble on.
template <class Expr> static Vector3d v3(const Expr &e) {
	Vector3d out = e;
	return out;
}

// Closed-form right Jacobian J(phi) = I + (1-cosθ)/θ² [φ]× +
//                                     (θ-sinθ)/θ³ [φ]×²
// (matches the convention so3_jacobian_inv inverts).
static Matrix3d so3_jacobian_closed_form(const Vector3d &phi) {
	Matrix3d Phi = phi.asSkewSymmetric();
	double theta2 = phi.squaredNorm();
	if (theta2 < 1e-20) {
		return Matrix3d::Identity() + 0.5 * Phi +
		       (1.0 / 6.0) * (Phi * Phi);
	}
	double theta = std::sqrt(theta2);
	double a = (1.0 - std::cos(theta)) / theta2;
	double b = (theta - std::sin(theta)) / (theta2 * theta);
	return Matrix3d::Identity() + a * Phi + b * (Phi * Phi);
}

// --------------------- so3_log_from_unit_quat ---------------------

void test_so3_log_identity() {
	Quaterniond q = Quaterniond::Identity();
	Vector3d v = so3_log_from_unit_quat(q);
	assert(approx_mat(v, Vector3d::Zero()));
}

void test_so3_log_axis_angle_round_trip() {
	const Vector3d axis = Vector3d{0.3, -0.7, 0.4}.normalized();
	for (double angle :
	     {0.0, 1e-6, 1e-3, 0.1, 0.5, 1.0, 2.0, 3.0, 3.1,
	      std::numbers::pi - 1e-3}) {
		Quaterniond q = axis_angle(axis, angle);
		Vector3d phi = so3_log_from_unit_quat(q);
		assert(approx_mat(phi, axis * angle, 1e-9));
	}
}

void test_so3_log_negative_w_canonicalizes() {
	const Vector3d axis = Vector3d{1.0, 0.0, 0.0};
	Quaterniond q = axis_angle(axis, 1.2);
	Quaterniond q_neg(-q.w(), -q.x(), -q.y(), -q.z());
	Vector3d phi_pos = so3_log_from_unit_quat(q);
	Vector3d phi_neg = so3_log_from_unit_quat(q_neg);
	assert(approx_mat(phi_pos, phi_neg));
}

void test_so3_log_branch_continuity() {
	// 5e-5 is the documented small-s threshold; values just above and below
	// must agree to many digits.
	const Vector3d axis = Vector3d{0.0, 0.0, 1.0};
	for (double s : {4.0e-5, 5.0e-5, 6.0e-5, 1.0e-4}) {
		double angle = 2.0 * std::asin(s);
		Quaterniond q = axis_angle(axis, angle);
		Vector3d phi = so3_log_from_unit_quat(q);
		assert(approx_mat(phi, axis * angle, 1e-12));
	}
}

// --------------------- so3_jacobian_inv ---------------------

void test_so3_jacobian_inv_at_zero() {
	Matrix3d J = so3_jacobian_inv(Vector3d::Zero());
	assert(approx_mat(J, Matrix3d::Identity()));
}

void test_so3_jacobian_inv_inverts_jacobian() {
	for (Vector3d phi : {Vector3d{1e-6, 0, 0},
	                     Vector3d{1e-3, 1e-3, 0},
	                     Vector3d{0.1, -0.2, 0.3},
	                     Vector3d{1.0, -0.5, 0.7},
	                     Vector3d{2.5, 0.3, -1.1}}) {
		Matrix3d Jinv = so3_jacobian_inv(phi);
		Matrix3d J = so3_jacobian_closed_form(phi);
		Matrix3d prod = Jinv * J;
		assert(approx_mat(prod, Matrix3d::Identity(), 1e-9));
	}
}

void test_so3_jacobian_inv_branch_continuity() {
	// 1e-4 is the documented small-theta threshold.
	for (double th :
	     {0.5e-4, 1.0e-4, 1.5e-4, 2.0e-4, 5.0e-4}) {
		Vector3d phi{th, 0.0, 0.0};
		Matrix3d Jinv = so3_jacobian_inv(phi);
		Matrix3d J = so3_jacobian_closed_form(phi);
		assert(approx_mat(Jinv * J, Matrix3d::Identity(), 1e-9));
	}
}

// --------------------- compute_Pi ---------------------

void test_compute_Pi_qm_equals_qp() {
	// log(I) = 0  =>  J^-1(0) = I  =>  Pi = R_m^T
	for (Quaterniond q : {Quaterniond::Identity(),
	                      axis_angle(Vector3d::UnitX(), 0.5),
	                      axis_angle(Vector3d{1, 1, 1}.normalized(), 1.2)}) {
		Matrix3d Pi = compute_Pi(q, q);
		assert(approx_mat(Pi, q.toRotationMatrix().transpose()));
	}
}

void test_compute_Pi_small_relative_rotation() {
	// For a small relative rotation R_rel, log(R_rel) is small, J^-1 ~ I,
	// so Pi ~ R_m^T.
	Quaterniond qm = axis_angle(Vector3d{0.2, 0.3, -0.1}.normalized(), 0.7);
	Quaterniond qp = qm * axis_angle(Vector3d::UnitZ(), 1e-7);
	Matrix3d Pi = compute_Pi(qm, qp);
	assert(approx_mat(Pi, qm.toRotationMatrix().transpose(), 1e-6));
}

// --------------------- inv_sqrt_spd ---------------------

void test_inv_sqrt_spd_identity() {
	Matrix2d M = inv_sqrt_spd(Matrix2d::Identity());
	assert(approx_mat(M, Matrix2d::Identity()));
}

void test_inv_sqrt_spd_diagonal() {
	Matrix2d C;
	C << 4.0, 0.0, 0.0, 9.0;
	Matrix2d M = inv_sqrt_spd(C);
	Matrix2d expected;
	expected << 0.5, 0.0, 0.0, 1.0 / 3.0;
	assert(approx_mat(M, expected));
	assert(approx_mat(M * C * M.transpose(), Matrix2d::Identity()));
}

void test_inv_sqrt_spd_general() {
	Matrix2d C;
	C << 5.0, 1.5, 1.5, 2.0;
	Matrix2d M = inv_sqrt_spd(C);
	Matrix2d should_be_I = M * C * M.transpose();
	assert(approx_mat(should_be_I, Matrix2d::Identity(), 1e-12));
	assert(approx_mat(M, M.transpose()));
}

void test_inv_sqrt_spd_rejects_non_positive() {
	Matrix2d C;
	C << -1.0, 0.0, 0.0, 1.0;
	bool threw = false;
	try {
		(void)inv_sqrt_spd(C);
	} catch (const std::invalid_argument &) {
		threw = true;
	}
	assert(threw && "inv_sqrt_spd must reject non-positive matrices");
}

// --------------------- fit_se3 ---------------------

void test_fit_se3_identity() {
	const int N = 4;
	MatrixX4d q0(N, 4);
	MatrixX3d t0(N, 3);
	q0 << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
	t0 << 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0, -1.0, -2.0, -3.0;
	VectorXd mass(N);
	mass << 1.0, 1.0, 1.0, 1.0;
	double total = mass.sum();
	Transformd T = fit_se3(q0, t0, q0, t0, mass, total, 0.01);
	const auto &[q, t] = T;
	assert(approx_mat(q.toRotationMatrix(), Matrix3d::Identity()));
	assert(approx_mat(t, Vector3d::Zero(), 1e-12));
}

void test_fit_se3_pure_translation() {
	const int N = 3;
	const Vector3d shift{0.5, -1.5, 2.0};
	MatrixX4d q0(N, 4);
	MatrixX3d t0(N, 3);
	q0 << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
	t0 << 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0;
	MatrixX3d t1 = t0.rowwise() + shift.transpose();
	VectorXd mass(N);
	mass << 1.0, 2.0, 3.0;
	double total = mass.sum();
	Transformd T = fit_se3(q0, t0, q0, t1, mass, total, 0.0);
	const auto &[q, t] = T;
	assert(approx_mat(q.toRotationMatrix(), Matrix3d::Identity()));
	assert(approx_mat(t, shift, 1e-12));
}

void test_fit_se3_pure_rotation() {
	const int N = 5;
	// Reference points spread in 3D so SVD is well-conditioned.
	MatrixX3d t0(N, 3);
	t0 << 1.0, 0.0, 0.0,
	     -1.0, 0.0, 0.0,
	      0.0, 1.0, 0.0,
	      0.0, -1.0, 0.0,
	      0.0, 0.0, 1.0;
	MatrixX4d q0(N, 4);
	for (int i = 0; i < N; ++i) {
		q0.row(i) = quat_xyzw(Quaterniond::Identity()).transpose();
	}
	VectorXd mass = VectorXd::Ones(N);
	double total = mass.sum();
	Quaterniond R = axis_angle(Vector3d{0.2, -0.5, 0.8}.normalized(), 0.9);
	const Vector3d t_offset{2.0, -3.0, 1.0};
	MatrixX3d t1(N, 3);
	for (int i = 0; i < N; ++i) {
		t1.row(i) =
		    (R * v3(t0.row(i).transpose()) + t_offset).transpose();
	}
	MatrixX4d q1(N, 4);
	for (int i = 0; i < N; ++i) {
		q1.row(i) = quat_xyzw(R).transpose(); // body rotates with the rigid frame
	}
	Transformd T = fit_se3(q0, t0, q1, t1, mass, total, 0.01);
	const auto &[q_fit, t_fit] = T;
	assert(approx_mat(q_fit.toRotationMatrix(), R.toRotationMatrix(), 1e-9));
	assert(approx_mat(t_fit, t_offset, 1e-9));
}

// --------------------- fit_twist ---------------------

void test_fit_twist_zero() {
	const int N = 3;
	MatrixX3d w = MatrixX3d::Zero(N, 3);
	MatrixX3d v = MatrixX3d::Zero(N, 3);
	MatrixX3d c_CC(N, 3);
	c_CC << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
	Transformd T_W_CC{Quaterniond::Identity(), Vector3d::Zero()};
	VectorXd mass = VectorXd::Ones(N);
	double total = mass.sum();
	TwistFitResult r = fit_twist(w, v, c_CC, T_W_CC, mass, total, 0.0);
	assert(approx_mat(r.w0, Vector3d::Zero()));
	assert(approx_mat(r.v0, Vector3d::Zero()));
	assert(approx_mat(r.v_W, MatrixX3d::Zero(N, 3)));
}

void test_fit_twist_pure_translation() {
	const int N = 4;
	const Vector3d u{0.7, -0.4, 0.2};
	MatrixX3d c_CC(N, 3);
	c_CC << 1.0, 0.0, 0.0,
	       -1.0, 0.0, 0.0,
	        0.0, 2.0, 0.0,
	        0.0, 0.0, -1.5;
	MatrixX3d v(N, 3);
	for (int i = 0; i < N; ++i)
		v.row(i) = u.transpose();
	MatrixX3d w = MatrixX3d::Zero(N, 3);
	Transformd T_W_CC{Quaterniond::Identity(), Vector3d{1.0, 2.0, 3.0}};
	VectorXd mass(N);
	mass << 1.0, 2.0, 3.0, 4.0;
	double total = mass.sum();
	TwistFitResult r = fit_twist(w, v, c_CC, T_W_CC, mass, total, 0.0);
	assert(approx_mat(r.w0, Vector3d::Zero(), 1e-9));
	assert(approx_mat(r.v0, u, 1e-12));
	for (int i = 0; i < N; ++i)
		assert(approx_mat(v3(r.v_W.row(i).transpose()), u, 1e-9));
}

void test_fit_twist_rigid_rotation_recovers_w() {
	const int N = 5;
	MatrixX3d c_CC(N, 3);
	c_CC << 1.0, 0.0, 0.0,
	       -1.0, 0.0, 0.0,
	        0.0, 1.0, 0.0,
	        0.0, -1.0, 0.0,
	        0.0, 0.0, 1.5;
	VectorXd mass(N);
	mass << 1.0, 1.0, 2.0, 2.0, 3.0;
	double total = mass.sum();

	// Place CC in world with a non-identity pose to make sure fit_twist
	// projects everything via T_W_CC consistently.
	Quaterniond q_W_CC = axis_angle(Vector3d{0.4, 0.5, 0.7}.normalized(), 0.6);
	Vector3d t_W_CC{-0.3, 0.8, 1.2};
	Transformd T_W_CC{q_W_CC, t_W_CC};

	// Synthesize a pure rigid rotation about CoM.
	MatrixX3d c_W(N, 3);
	for (int i = 0; i < N; ++i)
		c_W.row(i) =
		    (q_W_CC * v3(c_CC.row(i).transpose()) + t_W_CC).transpose();
	Vector3d r = c_W.transpose() * mass / total;
	const Vector3d w_true{0.3, -0.5, 0.7};
	MatrixX3d v(N, 3);
	for (int i = 0; i < N; ++i) {
		Vector3d d = v3(c_W.row(i).transpose()) - r;
		v.row(i) = w_true.cross(d).transpose();
	}
	MatrixX3d w = MatrixX3d::Zero(N, 3); // body angular velocities ignored when lambda_w = 0

	TwistFitResult res = fit_twist(w, v, c_CC, T_W_CC, mass, total, 0.0);
	assert(approx_mat(res.w0, w_true, 1e-9));
	assert(approx_mat(res.v0, Vector3d::Zero(), 1e-9));
	for (int i = 0; i < N; ++i) {
		Vector3d expected = v.row(i).transpose();
		assert(approx_mat(v3(res.v_W.row(i).transpose()), expected, 1e-9));
	}
}

// --------------------- compute_L ---------------------

// Identity inertia per part: L_i = R * I * R^T * w0 = w0 (independent of R).
void test_compute_L_identity_inertia() {
	const int N = 3;
	MatrixX9d Iflat(N, 9);
	for (int i = 0; i < N; ++i) {
		Iflat.row(i) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	}
	const Vector3d w0{0.4, -0.7, 1.1};
	for (Quaterniond q :
	     {Quaterniond::Identity(),
	      axis_angle(Vector3d::UnitZ(), 0.7),
	      axis_angle(Vector3d{1, -2, 3}.normalized(), 1.4)}) {
		MatrixX3d L = compute_L(Iflat, q, w0);
		for (int i = 0; i < N; ++i) {
			assert(approx_mat(v3(L.row(i).transpose()), w0, 1e-12));
		}
	}
}

// Diagonal CC-frame inertia, identity orientation: L_i = I_i * w0.
void test_compute_L_diagonal_inertia_identity_pose() {
	const int N = 2;
	MatrixX9d Iflat(N, 9);
	Iflat.row(0) << 2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 5.0;
	Iflat.row(1) << 7.0, 0.0, 0.0, 0.0, 11.0, 0.0, 0.0, 0.0, 13.0;
	Quaterniond q = Quaterniond::Identity();
	Vector3d w0{1.0, 1.0, 1.0};
	MatrixX3d L = compute_L(Iflat, q, w0);
	assert(approx_mat(v3(L.row(0).transpose()), Vector3d{2.0, 3.0, 5.0},
	                  1e-12));
	assert(approx_mat(v3(L.row(1).transpose()), Vector3d{7.0, 11.0, 13.0},
	                  1e-12));
}

// Rotated diagonal inertia: L_W = R * I_CC * R^T * w0.
void test_compute_L_rotated_inertia() {
	const int N = 1;
	MatrixX9d Iflat(N, 9);
	Iflat.row(0) << 2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 5.0;
	Quaterniond q = axis_angle(Vector3d{0.3, -0.8, 0.5}.normalized(), 1.1);
	Vector3d w0{0.7, -0.4, 1.3};
	MatrixX3d L = compute_L(Iflat, q, w0);
	Matrix3d I_CC;
	I_CC << 2, 0, 0, 0, 3, 0, 0, 0, 5;
	Matrix3d R = q.toRotationMatrix();
	Vector3d expected = R * I_CC * R.transpose() * w0;
	assert(approx_mat(v3(L.row(0).transpose()), expected, 1e-12));
}

// --------------------- main ---------------------

int main() {
	test_so3_log_identity();
	test_so3_log_axis_angle_round_trip();
	test_so3_log_negative_w_canonicalizes();
	test_so3_log_branch_continuity();

	test_so3_jacobian_inv_at_zero();
	test_so3_jacobian_inv_inverts_jacobian();
	test_so3_jacobian_inv_branch_continuity();

	test_compute_Pi_qm_equals_qp();
	test_compute_Pi_small_relative_rotation();

	test_inv_sqrt_spd_identity();
	test_inv_sqrt_spd_diagonal();
	test_inv_sqrt_spd_general();
	test_inv_sqrt_spd_rejects_non_positive();

	test_fit_se3_identity();
	test_fit_se3_pure_translation();
	test_fit_se3_pure_rotation();

	test_fit_twist_zero();
	test_fit_twist_pure_translation();
	test_fit_twist_rigid_rotation_recovers_w();

	test_compute_L_identity_inertia();
	test_compute_L_diagonal_inertia_identity_pose();
	test_compute_L_rotated_inertia();

	std::cout << "All Breakage tests passed.\n";
	return 0;
}
