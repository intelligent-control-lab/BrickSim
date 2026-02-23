import std;
import bricksim.utils.conversions;
import bricksim.vendor;

#include <cassert>

using namespace bricksim;

static constexpr double EPS = 1e-6;

// --------------------- small helpers ---------------------
template <class A, class B> bool approx_equal(A a, B b, double eps = EPS) {
	double da = static_cast<double>(a), db = static_cast<double>(b);
	double scale = 1.0 + std::max(std::abs(da), std::abs(db));
	return std::abs(da - db) <= eps * scale;
}

template <mat_like M> auto to_arr(const M &m) {
	if constexpr (mat_cols_v<M> == 1) {
		return as_array<double, mat_rows_v<M>>(m);
	} else {
		return as_array<double, mat_rows_v<M>, mat_cols_v<M>>(m);
	}
}

template <mat_like A, mat_like B>
bool approx_mat(const A &a, const B &b, double eps = EPS) {
	static_assert(mat_rows_v<A> == mat_rows_v<B> &&
	              mat_cols_v<A> == mat_cols_v<B>);
	constexpr std::size_t R = mat_rows_v<A>, C = mat_cols_v<A>;
	auto aa = to_arr(a);
	auto bb = to_arr(b);
	if constexpr (C == 1) {
		for (std::size_t r = 0; r < R; ++r)
			if (!approx_equal(aa[r], bb[r], eps))
				return false;
	} else {
		for (std::size_t r = 0; r < R; ++r)
			for (std::size_t c = 0; c < C; ++c)
				if (!approx_equal(aa[r][c], bb[r][c], eps))
					return false;
	}
	return true;
}

template <quat_like Q1, quat_like Q2>
bool approx_quat(Q1 q1, Q2 q2, double eps = EPS) {
	// Compare as rotation matrices (numerically robust, sign-invariant)
	auto R1 = as<std::array<std::array<double, 3>, 3>>(q1);
	auto R2 = as<std::array<std::array<double, 3>, 3>>(q2);
	return approx_mat(R1, R2, eps);
}

template <transform_like X1, transform_like X2>
bool approx_xf(const X1 &a, const X2 &b, double eps = EPS) {
	auto Ra = transform_traits<X1>::template rotation33<
	    std::array<std::array<double, 3>, 3>>(a);
	auto Rb = transform_traits<X2>::template rotation33<
	    std::array<std::array<double, 3>, 3>>(b);
	if (!approx_mat(Ra, Rb, eps))
		return false;
	auto ta =
	    transform_traits<X1>::template translation3<std::array<double, 3>>(a);
	auto tb =
	    transform_traits<X2>::template translation3<std::array<double, 3>>(b);
	for (int i = 0; i < 3; ++i)
		if (!approx_equal(ta[i], tb[i], eps))
			return false;
	return true;
}

// --------------------- compile-time concept sanity ---------------------
static_assert(mat_like<std::array<float, 3>>);
static_assert(mat_rows_v<std::array<float, 3>> == 3 &&
              mat_cols_v<std::array<float, 3>> == 1);
static_assert(mat_like<std::array<std::array<double, 4>, 4>>);
static_assert(mat_rows_v<std::array<std::array<double, 4>, 4>> == 4 &&
              mat_cols_v<std::array<std::array<double, 4>, 4>> == 4);

// Eigen fixed-size
static_assert(mat_like<Eigen::Matrix<float, 3, 1>>);
static_assert(mat_like<Eigen::Matrix<double, 3, 3>>);

// Eigen expressions (must be mat_like due to generic expr traits)
using E33f = Eigen::Matrix<float, 3, 3>;
static_assert(mat_like<decltype(E33f::Identity() * 2.0f)>);
static_assert(mat_like<decltype(E33f::Identity() + E33f::Identity())>);

// Gf vec & mat
static_assert(mat_like<pxr::GfVec3f>);
static_assert(mat_like<pxr::GfMatrix3d>);

// PhysX vec & mat
static_assert(mat_like<physx::PxVec3T<float>>);
static_assert(mat_like<physx::PxMat33T<double>>);

// Quaternions
static_assert(quat_like<physx::PxQuatT<float>>);
static_assert(quat_like<Eigen::Quaterniond>);
static_assert(quat_like<pxr::GfQuatd>);

// Transforms
static_assert(transform_like<physx::PxTransform>);
static_assert(transform_like<pxr::GfTransform>);
static_assert(transform_like<Eigen::Transform<double, 3, Eigen::Affine>>);
static_assert(transform_like<
              Eigen::Matrix<float, 4, 4>>); // 4x4 mat_like => transform_like

// --------------------- tests ---------------------

void test_initializer_list_to_vector() {
	// std::array target
	auto a3 = as<std::array<double, 3>>({1, 2, 3});
	assert(approx_equal(a3[0], 1.0));
	assert(approx_equal(a3[1], 2.0));
	assert(approx_equal(a3[2], 3.0));

	// Eigen vector target (implicit narrowing from double -> float)
	auto e3f = as<Eigen::Matrix<float, 3, 1>>({1.0, 2.0, 3.5});
	Eigen::Matrix<float, 3, 1> e_expected;
	e_expected << 1.0f, 2.0f, 3.5f;
	assert(approx_mat(e3f, e_expected));

	// USD Gf vector target
	auto g3f = as<pxr::GfVec3f>({-1.0f, 0.5f, 10.0f});
	std::array<double, 3> g_expected{-1.0, 0.5, 10.0};
	assert(approx_mat(g3f, g_expected));

	// PhysX vector target
	auto p4d = as<physx::PxVec4T<double>>({1, 2, 3, 4});
	std::array<double, 4> p_expected{1.0, 2.0, 3.0, 4.0};
	assert(approx_mat(p4d, p_expected));

	// Size mismatch should throw std::invalid_argument
	bool threw = false;
	try {
		(void)as<std::array<double, 3>>({1.0, 2.0});
	} catch (const std::invalid_argument &) {
		threw = true;
	}
	assert(threw && "initializer_list with wrong size must throw");
}

void test_vectors_all() {
	// Gf -> std::array -> PhysX -> Eigen -> Gf
	pxr::GfVec3f gf(1.f, 2.f, 3.f);
	auto arr = as_array<double, 3>(gf);
	assert(arr[0] == 1.0 && arr[1] == 2.0 && arr[2] == 3.0);

	auto pv = as_vec<physx_tag, float, 3>(arr);
	auto ev = as<Eigen::Matrix<double, 3, 1>>(pv);
	auto gf2 = as_vec<gf_tag, double, 3>(ev);
	assert(approx_mat(gf, gf2));

	// Eigen expression -> array
	E33f e = E33f::Identity();
	auto expr = e * 2.0f;
	auto a33 = as_array<double, 3, 3>(expr);
	assert(approx_equal(a33[0][0], 2.0));
	assert(approx_equal(a33[1][1], 2.0));
	assert(approx_equal(a33[2][2], 2.0));
}

void test_matrices_all() {
	// PhysX 3x3 -> Gf 3x3 -> Eigen 3x3 -> std::array
	physx::PxMat33 m33(physx::PxIdentity);
	auto g3d = as_mat<gf_tag, double, 3, 3>(m33);
	auto e3f = as<Eigen::Matrix<float, 3, 3>>(g3d);
	auto a33 = as_array<double, 3, 3>(e3f);
	assert(approx_equal(a33[0][0], 1.0) && approx_equal(a33[1][1], 1.0) &&
	       approx_equal(a33[2][2], 1.0));
}

void test_quaternions_all() {
	// Construct a known rotation Rz(theta)
	const double th = 0.5;
	const double c = std::cos(th), s = std::sin(th);
	std::array<std::array<double, 3>, 3> R{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
	// R -> various quats
	auto q_e = as<Eigen::Quaterniond>(R);
	auto q_p = as<physx::PxQuatT<double>>(R);
	auto q_g = as<pxr::GfQuatd>(R);

	// quat -> R and compare
	auto R_e = as<std::array<std::array<double, 3>, 3>>(q_e);
	auto R_p = as<std::array<std::array<double, 3>, 3>>(q_p);
	auto R_g = as<std::array<std::array<double, 3>, 3>>(q_g);
	assert(approx_mat(R, R_e));
	assert(approx_mat(R, R_p));
	assert(approx_mat(R, R_g));

	// cross-convert quats and compare as rotations
	assert(approx_quat(q_e, q_p));
	assert(approx_quat(q_e, q_g));
	assert(approx_quat(q_p, q_g));

	// quat -> PhysX 3x4 (translation zero)
	auto M34 = as<physx::PxMat34>(q_e);
	auto R_from_M34 = std::array<std::array<double, 3>, 3>{
	    {{M34(0, 0), M34(0, 1), M34(0, 2)},
	     {M34(1, 0), M34(1, 1), M34(1, 2)},
	     {M34(2, 0), M34(2, 1), M34(2, 2)}}};
	assert(approx_mat(R, R_from_M34));
	assert(approx_equal(M34(0, 3), 0.0) && approx_equal(M34(1, 3), 0.0) &&
	       approx_equal(M34(2, 3), 0.0));
}

void test_transforms_physx_eigen() {
	// Build R,t
	const double th = 0.3;
	const double c = std::cos(th), s = std::sin(th);
	Eigen::Matrix<double, 3, 3> R;
	R << c, -s, 0, s, c, 0, 0, 0, 1;
	Eigen::Matrix<double, 3, 1> t;
	t << 1, 2, 3;

	// as<physx::PxTransform>(R,t)
	physx::PxTransform Xp = as<physx::PxTransform>(R, t);
	// extract pose as Eigen
	auto [Re, te] = as_pose_rt<eigen_tag, double>(Xp);
	assert(approx_mat(R, Re));
	assert(approx_mat(t, te));

	// 4x4 matrix -> PxTransform, and back to 4x4
	Eigen::Matrix<double, 4, 4> M44 = Eigen::Matrix<double, 4, 4>::Identity();
	M44.block<3, 3>(0, 0) = R;
	M44.block<3, 1>(0, 3) = t;
	physx::PxTransform Xp2 = as<physx::PxTransform>(M44);
	assert(approx_xf(Xp, Xp2));

	auto M44_back = as<Eigen::Matrix<double, 4, 4>>(Xp2);
	assert(approx_mat(M44, M44_back));

	// Eigen::Transform
	Eigen::Transform<double, 3, Eigen::Affine> Xe =
	    as<Eigen::Transform<double, 3, Eigen::Affine>>(R, t);
	assert(approx_xf(Xp, Xe));
}

void test_transforms_gf() {
	// Build R,t
	const double th = -0.7;
	const double c = std::cos(th), s = std::sin(th);
	std::array<std::array<double, 3>, 3> R{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
	std::array<double, 3> t{{4.0, -5.0, 6.0}};

	// Create GfTransform via our API (R,t)
	pxr::GfTransform Xg = as<pxr::GfTransform>(R, t);
	// Extract pose in other libs
	auto [Rp, tp] = as_pose_rt<physx_tag, float>(Xg);
	assert(approx_mat(R, Rp));
	assert(approx_mat(t, tp));

	// To 4x4 and back
	auto Mg = as<pxr::GfMatrix4d>(Xg);
	auto Xg2 = as<pxr::GfTransform>(Mg);
	assert(approx_xf(Xg, Xg2));

	// Extract quaternion + translation (Eigen)
	auto [q_e, te] = as_pose_qt<eigen_tag, double>(Xg);
	auto R_q = as<std::array<std::array<double, 3>, 3>>(q_e);
	assert(approx_mat(R, R_q));
	assert(approx_mat(t, te));
}

void test_gf_matrix4d_transform_traits() {
	// Build a pure GfMatrix4d using Gf's own API: rotation + translation.
	const double th = 0.5;
	const double c = std::cos(th * 0.5);
	const double s = std::sin(th * 0.5);
	pxr::GfVec3d im(0.0, 0.0, s);
	pxr::GfQuatd q_exp(c, im); // rotation about +Z by angle 'th'
	pxr::GfVec3d t_exp(1.0, -2.0, 3.5);

	pxr::GfMatrix4d M(1.0);
	M.SetRotate(q_exp);
	M.SetTranslateOnly(t_exp);

	// Sanity: Gf's own extractors see what we authored.
	auto q_usd = M.ExtractRotationQuat();
	auto t_usd = M.ExtractTranslation();
	assert(approx_quat(q_exp, q_usd));
	assert(approx_mat(as<std::array<double, 3>>(t_exp),
	                  as<std::array<double, 3>>(t_usd)));

	// Now go through transform_traits<pxr::GfMatrix4d> and ensure we decode the
	// same rotation and translation.
	auto R_dec = transform_traits<pxr::GfMatrix4d>::template rotation33<
	    std::array<std::array<double, 3>, 3>>(M);
	auto t_dec = transform_traits<pxr::GfMatrix4d>::template translation3<
	    std::array<double, 3>>(M);

	auto R_expected =
	    as<std::array<std::array<double, 3>, 3>>(q_usd); // canonical 3x3
	auto t_expected = as<std::array<double, 3>>(t_usd);

	assert(approx_mat(R_expected, R_dec));
	assert(approx_mat(t_expected, t_dec));

	// And the high-level pose_qt API must agree with q_usd, t_usd up to the
	// usual quaternion sign ambiguity.
	auto [q_read, t_read] = as_pose_qt<eigen_tag, double>(M);
	auto R_read = as<std::array<std::array<double, 3>, 3>>(q_read);
	assert(approx_mat(R_expected, R_read));
	assert(approx_mat(t_expected, t_read));
}

void test_gf_matrix4f_transform_traits() {
	// Same as the double-precision test, but for GfMatrix4f.
	const float th = 0.3f;
	const float c = std::cos(th * 0.5f);
	const float s = std::sin(th * 0.5f);
	pxr::GfVec3f im(0.0f, 0.0f, s);
	pxr::GfQuatf q_exp(c, im); // rotation about +Z
	pxr::GfVec3f t_exp(2.0f, -1.0f, 4.0f);

	pxr::GfMatrix4f M(1.0f);
	M.SetRotate(q_exp);
	M.SetTranslateOnly(t_exp);

	// Baseline: Gf's own extractors.
	auto q_usd = M.ExtractRotationQuat();
	auto t_usd = M.ExtractTranslation();

	// Decode via transform_traits<GfMatrix4f>.
	auto R_dec = transform_traits<pxr::GfMatrix4f>::template rotation33<
	    std::array<std::array<double, 3>, 3>>(M);
	auto t_dec = transform_traits<pxr::GfMatrix4f>::template translation3<
	    std::array<double, 3>>(M);

	auto R_expected =
	    as<std::array<std::array<double, 3>, 3>>(q_usd); // canonical 3x3
	auto t_expected = as<std::array<double, 3>>(t_usd);

	assert(approx_mat(R_expected, R_dec));
	assert(approx_mat(t_expected, t_dec));

	// High-level pose_qt API should agree as well.
	auto [q_read, t_read] = as_pose_qt<eigen_tag, float>(M);
	auto R_read = as<std::array<std::array<double, 3>, 3>>(q_read);
	auto t_read_arr = as<std::array<double, 3>>(t_read);

	assert(approx_mat(R_expected, R_read));
	assert(approx_mat(t_expected, t_read_arr));
}

int main() {
	// initializer_list -> vector
	test_initializer_list_to_vector();

	// vectors & matrices
	test_vectors_all();
	test_matrices_all();

	// quaternions
	test_quaternions_all();

	// transforms (PhysX/Eigen)
	test_transforms_physx_eigen();

	// transforms (USD) — requires you to apply the two small header fixes above
	test_transforms_gf();
	test_gf_matrix4d_transform_traits();
	test_gf_matrix4f_transform_traits();

	std::cout << "All Conversions tests passed.\n";
	return 0;
}
