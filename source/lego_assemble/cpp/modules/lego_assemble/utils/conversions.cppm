export module lego_assemble.utils.conversions;

import std;
import lego_assemble.vendor.eigen;
import lego_assemble.vendor.physx;
import lego_assemble.vendor.pxr;

namespace lego_assemble {

export template <class T> using remove_cvref_t = std::remove_cvref_t<T>;

export struct std_tag {};
export struct eigen_tag {};
export struct gf_tag {};
export struct physx_tag {};

export template <class T, class = void> struct mat_traits {
	static constexpr bool valid = false;
};

export template <class T>
concept mat_like = mat_traits<T>::valid;

export template <mat_like T>
using mat_scalar_t = typename mat_traits<T>::scalar_type;

export template <mat_like T>
constexpr std::size_t mat_rows_v = mat_traits<T>::rows;

export template <mat_like T>
constexpr std::size_t mat_cols_v = mat_traits<T>::cols;

// std::array adapters
// Vector: std::array<S,N> ==> Nx1
export template <class S, std::size_t N> struct mat_traits<std::array<S, N>> {
	static constexpr bool valid = true;
	using scalar_type = S;
	static constexpr std::size_t rows = N;
	static constexpr std::size_t cols = 1;

	static constexpr S get(const std::array<S, N> &v, std::size_t r,
	                       std::size_t c) noexcept {
		(void)c;
		return v[r];
	}
	template <class X>
	static constexpr void set(std::array<S, N> &v, std::size_t r, std::size_t c,
	                          X x) noexcept {
		(void)c;
		v[r] = static_cast<S>(x);
	}
};

// Matrix: std::array<std::array<S,C>, R> ==> RxC
export template <class S, std::size_t R, std::size_t C>
struct mat_traits<std::array<std::array<S, C>, R>> {
	static constexpr bool valid = true;
	using scalar_type = S;
	static constexpr std::size_t rows = R;
	static constexpr std::size_t cols = C;

	static constexpr S get(const std::array<std::array<S, C>, R> &m,
	                       std::size_t r, std::size_t c) noexcept {
		return m[r][c];
	}
	template <class X>
	static constexpr void set(std::array<std::array<S, C>, R> &m, std::size_t r,
	                          std::size_t c, X x) noexcept {
		m[r][c] = static_cast<S>(x);
	}
};

// Eigen fixed-size Matrix
export template <class S, int R, int C, int Opt, int MR, int MC>
struct mat_traits<Eigen::Matrix<S, R, C, Opt, MR, MC>> {
	static_assert(R > 0 && C > 0,
	              "Dynamic Eigen sizes are not supported in mat_traits.");
	static constexpr bool valid = true;
	using scalar_type = S;
	static constexpr std::size_t rows = static_cast<std::size_t>(R);
	static constexpr std::size_t cols = static_cast<std::size_t>(C);

	static S get(const Eigen::Matrix<S, R, C, Opt, MR, MC> &m, std::size_t r,
	             std::size_t c) noexcept {
		return m(static_cast<int>(r), static_cast<int>(c));
	}
	template <class X>
	static void set(Eigen::Matrix<S, R, C, Opt, MR, MC> &m, std::size_t r,
	                std::size_t c, X x) noexcept {
		m(static_cast<int>(r), static_cast<int>(c)) = static_cast<S>(x);
	}
};

// Generic fixed-shape indexable expressions (e.g., Eigen exprs)
// Accepts any E with fixed RowsAtCompileTime/ColsAtCompileTime > 0 and operator()(int,int).
export template <class E>
    requires requires(const remove_cvref_t<E> &e) {
	    { remove_cvref_t<E>::RowsAtCompileTime } -> std::convertible_to<int>;
	    { remove_cvref_t<E>::ColsAtCompileTime } -> std::convertible_to<int>;
	    { e(0, 0) };
    } && (remove_cvref_t<E>::RowsAtCompileTime > 0) &&
             (remove_cvref_t<E>::ColsAtCompileTime > 0)
struct mat_traits<E> {
	using X = remove_cvref_t<E>;
	static constexpr bool valid = true;
	using scalar_type =
	    remove_cvref_t<decltype(std::declval<const X &>()(0, 0))>;
	static constexpr std::size_t rows =
	    static_cast<std::size_t>(X::RowsAtCompileTime);
	static constexpr std::size_t cols =
	    static_cast<std::size_t>(X::ColsAtCompileTime);

	static scalar_type get(const E &e, std::size_t r, std::size_t c) noexcept {
		return static_cast<scalar_type>(
		    e(static_cast<int>(r), static_cast<int>(c)));
	}
	template <class V>
	static void set(E &, std::size_t, std::size_t, V) = delete; // read-only
};

// USD Gf adapters
export template <class T>
    requires(pxr::GfIsGfVec<T>::value)
struct mat_traits<T> {
	static constexpr bool valid = true;
	using scalar_type = typename T::ScalarType;
	static constexpr std::size_t rows = static_cast<std::size_t>(T::dimension);
	static constexpr std::size_t cols = 1;

	static scalar_type get(const T &v, std::size_t r, std::size_t c) noexcept {
		(void)c;
		return v[static_cast<int>(r)];
	}
	template <class X>
	static void set(T &v, std::size_t r, std::size_t c, X x) noexcept {
		(void)c;
		v[static_cast<int>(r)] = static_cast<scalar_type>(x);
	}
};

export template <class T>
    requires(pxr::GfIsGfMatrix<T>::value)
struct mat_traits<T> {
	static constexpr bool valid = true;
	using scalar_type = typename T::ScalarType;
	static constexpr std::size_t rows = static_cast<std::size_t>(T::numRows);
	static constexpr std::size_t cols = static_cast<std::size_t>(T::numColumns);

	static scalar_type get(const T &m, std::size_t r, std::size_t c) noexcept {
		return m[static_cast<int>(r)][static_cast<int>(c)];
	}
	template <class X>
	static void set(T &m, std::size_t r, std::size_t c, X x) noexcept {
		m[static_cast<int>(r)][static_cast<int>(c)] =
		    static_cast<scalar_type>(x);
	}
};

// PhysX adapters
export template <template <class> class PxVec, class S>
    requires(std::is_same_v<PxVec<S>, physx::PxVec2T<S>> ||
             std::is_same_v<PxVec<S>, physx::PxVec3T<S>> ||
             std::is_same_v<PxVec<S>, physx::PxVec4T<S>>)
struct mat_traits<PxVec<S>> {
	static constexpr bool valid = true;
	using scalar_type = S;
	using T = PxVec<S>;
	static constexpr std::size_t rows = std::is_same_v<T, physx::PxVec2T<S>> ? 2
	                                    : std::is_same_v<T, physx::PxVec3T<S>>
	                                        ? 3
	                                        : 4;
	static constexpr std::size_t cols = 1;

	static S get(const T &v, std::size_t r, std::size_t c) noexcept {
		(void)c;
		return v[static_cast<unsigned>(r)];
	}
	template <class X>
	static void set(T &v, std::size_t r, std::size_t c, X x) noexcept {
		(void)c;
		v[static_cast<unsigned>(r)] = static_cast<S>(x);
	}
};

export template <class S> struct mat_traits<physx::PxMat33T<S>> {
	static constexpr bool valid = true;
	using scalar_type = S;
	static constexpr std::size_t rows = 3, cols = 3;
	static S get(const physx::PxMat33T<S> &m, std::size_t r,
	             std::size_t c) noexcept {
		return m(static_cast<unsigned>(r), static_cast<unsigned>(c));
	}
	template <class X>
	static void set(physx::PxMat33T<S> &m, std::size_t r, std::size_t c,
	                X x) noexcept {
		m(static_cast<unsigned>(r), static_cast<unsigned>(c)) =
		    static_cast<S>(x);
	}
};

export template <class S> struct mat_traits<physx::PxMat34T<S>> {
	static constexpr bool valid = true;
	using scalar_type = S;
	static constexpr std::size_t rows = 3, cols = 4;
	static S get(const physx::PxMat34T<S> &m, std::size_t r,
	             std::size_t c) noexcept {
		return m(static_cast<unsigned>(r), static_cast<unsigned>(c));
	}
	template <class X>
	static void set(physx::PxMat34T<S> &m, std::size_t r, std::size_t c,
	                X x) noexcept {
		m(static_cast<unsigned>(r), static_cast<unsigned>(c)) =
		    static_cast<S>(x);
	}
};

export template <class S> struct mat_traits<physx::PxMat44T<S>> {
	static constexpr bool valid = true;
	using scalar_type = S;
	static constexpr std::size_t rows = 4, cols = 4;
	static S get(const physx::PxMat44T<S> &m, std::size_t r,
	             std::size_t c) noexcept {
		return m(static_cast<unsigned>(r), static_cast<unsigned>(c));
	}
	template <class X>
	static void set(physx::PxMat44T<S> &m, std::size_t r, std::size_t c,
	                X x) noexcept {
		m(static_cast<unsigned>(r), static_cast<unsigned>(c)) =
		    static_cast<S>(x);
	}
};

// ============================================================================
// (Tag, Scalar, R, C) -> concrete matrix/vector types
// ============================================================================

export template <class Tag, class S, std::size_t R, std::size_t C, class = void>
struct mat_of; // primary

// std::array
export template <class S, std::size_t N> struct mat_of<std_tag, S, N, 1, void> {
	using type = std::array<S, N>;
};

export template <class S, std::size_t R, std::size_t C>
struct mat_of<std_tag, S, R, C, void> {
	using type = std::array<std::array<S, C>, R>;
};

// Eigen fixed-size
export template <class S, std::size_t R, std::size_t C>
struct mat_of<eigen_tag, S, R, C, void> {
	using type = Eigen::Matrix<S, static_cast<int>(R), static_cast<int>(C), 0,
	                           static_cast<int>(R), static_cast<int>(C)>;
};

// USD Gf: vectors & square matrices (2/3/4)
export template <> struct mat_of<gf_tag, int, 2, 1, void> {
	using type = pxr::GfVec2i;
};
export template <> struct mat_of<gf_tag, float, 2, 1, void> {
	using type = pxr::GfVec2f;
};
export template <> struct mat_of<gf_tag, double, 2, 1, void> {
	using type = pxr::GfVec2d;
};
export template <> struct mat_of<gf_tag, int, 3, 1, void> {
	using type = pxr::GfVec3i;
};
export template <> struct mat_of<gf_tag, float, 3, 1, void> {
	using type = pxr::GfVec3f;
};
export template <> struct mat_of<gf_tag, double, 3, 1, void> {
	using type = pxr::GfVec3d;
};
export template <> struct mat_of<gf_tag, int, 4, 1, void> {
	using type = pxr::GfVec4i;
};
export template <> struct mat_of<gf_tag, float, 4, 1, void> {
	using type = pxr::GfVec4f;
};
export template <> struct mat_of<gf_tag, double, 4, 1, void> {
	using type = pxr::GfVec4d;
};
export template <> struct mat_of<gf_tag, float, 2, 2, void> {
	using type = pxr::GfMatrix2f;
};
export template <> struct mat_of<gf_tag, double, 2, 2, void> {
	using type = pxr::GfMatrix2d;
};
export template <> struct mat_of<gf_tag, float, 3, 3, void> {
	using type = pxr::GfMatrix3f;
};
export template <> struct mat_of<gf_tag, double, 3, 3, void> {
	using type = pxr::GfMatrix3d;
};
export template <> struct mat_of<gf_tag, float, 4, 4, void> {
	using type = pxr::GfMatrix4f;
};
export template <> struct mat_of<gf_tag, double, 4, 4, void> {
	using type = pxr::GfMatrix4d;
};

// PhysX
export template <class S> struct mat_of<physx_tag, S, 2, 1, void> {
	using type = physx::PxVec2T<S>;
};
export template <class S> struct mat_of<physx_tag, S, 3, 1, void> {
	using type = physx::PxVec3T<S>;
};
export template <class S> struct mat_of<physx_tag, S, 4, 1, void> {
	using type = physx::PxVec4T<S>;
};
export template <class S> struct mat_of<physx_tag, S, 3, 3, void> {
	using type = physx::PxMat33T<S>;
};
export template <class S> struct mat_of<physx_tag, S, 3, 4, void> {
	using type = physx::PxMat34T<S>;
};
export template <class S> struct mat_of<physx_tag, S, 4, 4, void> {
	using type = physx::PxMat44T<S>;
};

export template <class Tag, class S, std::size_t R, std::size_t C>
using mat_of_t = typename mat_of<Tag, std::remove_cv_t<S>, R, C, void>::type;

// ============================================================================
// Matrix/Vector core API
// ============================================================================
export template <mat_like To, mat_like From>
    requires(mat_rows_v<To> == mat_rows_v<From> &&
             mat_cols_v<To> == mat_cols_v<From>)
[[nodiscard]] constexpr To as(const From &src) {
	constexpr std::size_t R = mat_rows_v<To>;
	constexpr std::size_t C = mat_cols_v<To>;
	using STo = mat_scalar_t<To>;

	To out{};
	for (std::size_t r = 0; r < R; ++r)
		for (std::size_t c = 0; c < C; ++c)
			mat_traits<To>::set(
			    out, r, c, static_cast<STo>(mat_traits<From>::get(src, r, c)));
	return out;
}

// Special case: convert initializer_list to vector
export template <mat_like To, class S>
    requires(mat_cols_v<To> == 1 && std::is_convertible_v<S, mat_scalar_t<To>>)
[[nodiscard]] constexpr To as(std::initializer_list<S> src) {
	constexpr std::size_t N = mat_rows_v<To>;
	if (src.size() != N) {
		throw std::invalid_argument(
		    "Initializer list size does not match vector size.");
	}
	To out{};
	std::size_t r = 0;
	for (const auto &val : src) {
		mat_traits<To>::set(out, r, 0, static_cast<mat_scalar_t<To>>(val));
		++r;
	}
	return out;
}

export template <class Tag, class S, std::size_t R, std::size_t C,
                 mat_like From>
    requires(mat_rows_v<From> == R && mat_cols_v<From> == C)
[[nodiscard]] constexpr mat_of_t<Tag, S, R, C> as_mat(const From &src) {
	using To = mat_of_t<Tag, S, R, C>;
	static_assert(mat_like<To>,
	              "mat_of<Tag,S,R,C> does not map to a known matrix type.");
	return as<To>(src);
}

export template <class Tag, class S, std::size_t N, mat_like From>
    requires(mat_rows_v<From> == N && mat_cols_v<From> == 1)
[[nodiscard]] constexpr mat_of_t<Tag, S, N, 1> as_vec(const From &src) {
	using To = mat_of_t<Tag, S, N, 1>;
	static_assert(mat_like<To>,
	              "mat_of<Tag,S,N,1> does not map to a known vector type.");
	return as<To>(src);
}

export template <class S, std::size_t N, mat_like From>
    requires(mat_rows_v<From> == N && mat_cols_v<From> == 1)
[[nodiscard]] constexpr std::array<S, N> as_array(const From &src) {
	return as<std::array<S, N>>(src);
}

export template <class S, std::size_t R, std::size_t C, mat_like From>
    requires(mat_rows_v<From> == R && mat_cols_v<From> == C)
[[nodiscard]] constexpr std::array<std::array<S, C>, R>
as_array(const From &src) {
	return as<std::array<std::array<S, C>, R>>(src);
}

// ============================================================================
// Quaternion traits & concepts
// ============================================================================
export template <class T, class = void> struct quat_traits {
	static constexpr bool valid = false;
};
export template <class T>
concept quat_like = quat_traits<T>::valid;

export template <quat_like T>
using quat_scalar_t = typename quat_traits<T>::scalar_type;

// PhysX PxQuatT<S> (x,y,z,w fields; canonical order here is w,x,y,z)
export template <class S> struct quat_traits<physx::PxQuatT<S>> {
	static constexpr bool valid = true;
	using scalar_type = S;

	static S get(const physx::PxQuatT<S> &q, std::size_t i) noexcept {
		switch (i) {
		case 0:
			return q.w;
		case 1:
			return q.x;
		case 2:
			return q.y;
		default:
			return q.z;
		}
	}
	template <class X>
	static void set(physx::PxQuatT<S> &q, std::size_t i, X v) noexcept {
		const S s = static_cast<S>(v);
		switch (i) {
		case 0:
			q.w = s;
			break;
		case 1:
			q.x = s;
			break;
		case 2:
			q.y = s;
			break;
		default:
			q.z = s;
			break;
		}
	}
};

// Eigen Quaternion<S>
export template <class S, int Opt>
struct quat_traits<Eigen::Quaternion<S, Opt>> {
	static constexpr bool valid = true;
	using scalar_type = S;
	static S get(const Eigen::Quaternion<S, Opt> &q, std::size_t i) noexcept {
		switch (i) {
		case 0:
			return q.w();
		case 1:
			return q.x();
		case 2:
			return q.y();
		default:
			return q.z();
		}
	}
	template <class X>
	static void set(Eigen::Quaternion<S, Opt> &q, std::size_t i, X v) noexcept {
		const S s = static_cast<S>(v);
		switch (i) {
		case 0:
			q.w() = s;
			break;
		case 1:
			q.x() = s;
			break;
		case 2:
			q.y() = s;
			break;
		default:
			q.z() = s;
			break;
		}
	}
};

// USD GfQuat*
export template <class T>
    requires(pxr::GfIsGfQuat<T>::value)
struct quat_traits<T> {
	static constexpr bool valid = true;
	using scalar_type = typename T::ScalarType;

	static scalar_type get(const T &q, std::size_t i) noexcept {
		if (i == 0)
			return q.GetReal();
		const auto im = q.GetImaginary();
		return im[static_cast<int>(i - 1)];
	}
	template <class X> static void set(T &q, std::size_t i, X v) noexcept {
		using S = scalar_type;
		if (i == 0)
			q.SetReal(static_cast<S>(v));
		else {
			auto im = q.GetImaginary();
			im[static_cast<int>(i - 1)] = static_cast<S>(v);
			q.SetImaginary(im);
		}
	}
};

// (Tag, Scalar) -> concrete quaternion type
export template <class Tag, class S, class = void> struct quat_of; // primary

export template <class S> struct quat_of<eigen_tag, S, void> {
	using type = Eigen::Quaternion<S, 0>;
};

export template <class S> struct quat_of<physx_tag, S, void> {
	using type = physx::PxQuatT<S>;
};

export template <> struct quat_of<gf_tag, float, void> {
	using type = pxr::GfQuatf;
};

export template <> struct quat_of<gf_tag, double, void> {
	using type = pxr::GfQuatd;
};

export template <class Tag, class S>
using quat_of_t = typename quat_of<Tag, std::remove_cv_t<S>, void>::type;

// Quaternion conversions
export template <quat_like To, quat_like From>
[[nodiscard]] constexpr To as(const From &q) {
	using ST = quat_scalar_t<To>;
	To out{};
	quat_traits<To>::set(out, 0, static_cast<ST>(quat_traits<From>::get(q, 0)));
	quat_traits<To>::set(out, 1, static_cast<ST>(quat_traits<From>::get(q, 1)));
	quat_traits<To>::set(out, 2, static_cast<ST>(quat_traits<From>::get(q, 2)));
	quat_traits<To>::set(out, 3, static_cast<ST>(quat_traits<From>::get(q, 3)));
	return out;
}

export template <class S, quat_like From>
[[nodiscard]] constexpr std::array<S, 4> as_array(const From &q) {
	std::array<S, 4> out{};
	out[0] = static_cast<S>(quat_traits<From>::get(q, 0));
	out[1] = static_cast<S>(quat_traits<From>::get(q, 1));
	out[2] = static_cast<S>(quat_traits<From>::get(q, 2));
	out[3] = static_cast<S>(quat_traits<From>::get(q, 3));
	return out;
}

export template <class Tag, class S, quat_like From>
[[nodiscard]] constexpr quat_of_t<Tag, S> as_quat(const From &q) {
	using To = quat_of_t<Tag, S>;
	static_assert(quat_like<To>,
	              "quat_of<Tag,S> does not map to a known quaternion type.");
	return as<To>(q);
}

// Matrix(3x3) -> Quaternion
export template <quat_like To, mat_like From>
    requires(mat_rows_v<From> == 3 && mat_cols_v<From> == 3)
[[nodiscard]] constexpr To as(const From &m) {
	using ST = quat_scalar_t<To>;
	const double m00 = static_cast<double>(mat_traits<From>::get(m, 0, 0));
	const double m01 = static_cast<double>(mat_traits<From>::get(m, 0, 1));
	const double m02 = static_cast<double>(mat_traits<From>::get(m, 0, 2));
	const double m10 = static_cast<double>(mat_traits<From>::get(m, 1, 0));
	const double m11 = static_cast<double>(mat_traits<From>::get(m, 1, 1));
	const double m12 = static_cast<double>(mat_traits<From>::get(m, 1, 2));
	const double m20 = static_cast<double>(mat_traits<From>::get(m, 2, 0));
	const double m21 = static_cast<double>(mat_traits<From>::get(m, 2, 1));
	const double m22 = static_cast<double>(mat_traits<From>::get(m, 2, 2));

	double qw, qx, qy, qz;
	const double t = m00 + m11 + m22;
	if (t > 0.0) {
		const double s = std::sqrt(t + 1.0) * 2.0;
		qw = 0.25 * s;
		qx = (m21 - m12) / s;
		qy = (m02 - m20) / s;
		qz = (m10 - m01) / s;
	} else if (m00 > m11 && m00 > m22) {
		const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
		qw = (m21 - m12) / s;
		qx = 0.25 * s;
		qy = (m01 + m10) / s;
		qz = (m02 + m20) / s;
	} else if (m11 > m22) {
		const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
		qw = (m02 - m20) / s;
		qx = (m01 + m10) / s;
		qy = 0.25 * s;
		qz = (m12 + m21) / s;
	} else {
		const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
		qw = (m10 - m01) / s;
		qx = (m02 + m20) / s;
		qy = (m12 + m21) / s;
		qz = 0.25 * s;
	}

	To out{};
	quat_traits<To>::set(out, 0, static_cast<ST>(qw));
	quat_traits<To>::set(out, 1, static_cast<ST>(qx));
	quat_traits<To>::set(out, 2, static_cast<ST>(qy));
	quat_traits<To>::set(out, 3, static_cast<ST>(qz));
	return out;
}

// Quaternion -> Matrix(3x3)
export template <mat_like To, quat_like From>
    requires(mat_rows_v<To> == 3 && mat_cols_v<To> == 3)
[[nodiscard]] constexpr To as(const From &q) {
	using ST = mat_scalar_t<To>;
	const double w = static_cast<double>(quat_traits<From>::get(q, 0));
	const double x = static_cast<double>(quat_traits<From>::get(q, 1));
	const double y = static_cast<double>(quat_traits<From>::get(q, 2));
	const double z = static_cast<double>(quat_traits<From>::get(q, 3));

	const double xx = x * x, yy = y * y, zz = z * z;
	const double xy = x * y, xz = x * z, yz = y * z;
	const double wx = w * x, wy = w * y, wz = w * z;

	To m{};
	mat_traits<To>::set(m, 0, 0, static_cast<ST>(1.0 - 2.0 * (yy + zz)));
	mat_traits<To>::set(m, 0, 1, static_cast<ST>(2.0 * (xy - wz)));
	mat_traits<To>::set(m, 0, 2, static_cast<ST>(2.0 * (xz + wy)));

	mat_traits<To>::set(m, 1, 0, static_cast<ST>(2.0 * (xy + wz)));
	mat_traits<To>::set(m, 1, 1, static_cast<ST>(1.0 - 2.0 * (xx + zz)));
	mat_traits<To>::set(m, 1, 2, static_cast<ST>(2.0 * (yz - wx)));

	mat_traits<To>::set(m, 2, 0, static_cast<ST>(2.0 * (xz - wy)));
	mat_traits<To>::set(m, 2, 1, static_cast<ST>(2.0 * (yz + wx)));
	mat_traits<To>::set(m, 2, 2, static_cast<ST>(1.0 - 2.0 * (xx + yy)));
	return m;
}

// Quaternion -> PxMat34 (3x3 + zero translation)
export template <mat_like To, quat_like From>
    requires(mat_rows_v<To> == 3 && mat_cols_v<To> == 4)
[[nodiscard]] constexpr To as(const From &q) {
	using ST = mat_scalar_t<To>;
	std::array<std::array<ST, 3>, 3> R =
	    as<std::array<std::array<ST, 3>, 3>>(q);
	To out{};
	for (std::size_t r = 0; r < 3; ++r) {
		for (std::size_t c = 0; c < 3; ++c)
			mat_traits<To>::set(out, r, c, R[r][c]);
		mat_traits<To>::set(out, r, 3, static_cast<ST>(0));
	}
	return out;
}

// ============================================================================
// Rigid transforms
// ============================================================================
export template <class T, class = void> struct transform_traits {
	static constexpr bool valid = false;
};

export template <class T>
concept transform_like = transform_traits<T>::valid;

export template <transform_like T>
using transform_scalar_t = typename transform_traits<T>::scalar_type;

// Any 4x4 mat_like is also a transform
export template <class T>
    requires(mat_traits<T>::valid && mat_traits<T>::rows == 4 &&
             mat_traits<T>::cols == 4)
struct transform_traits<T> {
	static constexpr bool valid = true;
	using scalar_type = mat_scalar_t<T>;
	using S = scalar_type;

	template <mat_like R, mat_like V>
	    requires(mat_traits<R>::rows == 3 && mat_traits<R>::cols == 3 &&
	             mat_traits<V>::rows == 3 && mat_traits<V>::cols == 1)
	static T make_rt(const R &R33, const V &t) {
		std::array<std::array<S, 4>, 4> M{};
		for (std::size_t r = 0; r < 3; ++r) {
			for (std::size_t c = 0; c < 3; ++c)
				M[r][c] = static_cast<S>(mat_traits<R>::get(R33, r, c));
			M[r][3] =
			    static_cast<S>(mat_traits<V>::get(t, r, 0)); // last column
		}
		M[3][0] = M[3][1] = M[3][2] = S(0);
		M[3][3] = S(1);
		return as<T>(M);
	}

	template <quat_like Q, mat_like V>
	    requires(mat_traits<V>::rows == 3 && mat_traits<V>::cols == 1)
	static T make_qt(const Q &q, const V &t) {
		auto R33 = as<std::array<std::array<S, 3>, 3>>(q);
		return make_rt(R33, t);
	}

	template <mat_like R>
	    requires(mat_traits<R>::rows == 3 && mat_traits<R>::cols == 3)
	static R rotation33(const T &M) {
		std::array<std::array<S, 3>, 3> Rarr{};
		for (std::size_t r = 0; r < 3; ++r)
			for (std::size_t c = 0; c < 3; ++c)
				Rarr[r][c] = mat_traits<T>::get(M, r, c);
		return as<R>(Rarr);
	}

	template <mat_like V>
	    requires(mat_traits<V>::rows == 3 && mat_traits<V>::cols == 1)
	static V translation3(const T &M) {
		std::array<S, 3> Tarr{mat_traits<T>::get(M, 0, 3),
		                      mat_traits<T>::get(M, 1, 3),
		                      mat_traits<T>::get(M, 2, 3)};
		return as<V>(Tarr);
	}
};

// PhysX PxTransformT<S>
export template <class S> struct transform_traits<physx::PxTransformT<S>> {
	static constexpr bool valid = true;
	using scalar_type = S;
	using T = physx::PxTransformT<S>;
	using Q = physx::PxQuatT<S>;
	using V = physx::PxVec3T<S>;

	template <mat_like R, mat_like W>
	    requires(mat_traits<R>::rows == 3 && mat_traits<R>::cols == 3 &&
	             mat_traits<W>::rows == 3 && mat_traits<W>::cols == 1)
	static T make_rt(const R &R33, const W &t) {
		return T(as<V>(t), as<Q>(R33));
	}

	template <quat_like QQ, mat_like W>
	    requires(mat_traits<W>::rows == 3 && mat_traits<W>::cols == 1)
	static T make_qt(const QQ &q, const W &t) {
		return T(as<V>(t), as<Q>(q));
	}

	template <mat_like R>
	    requires(mat_traits<R>::rows == 3 && mat_traits<R>::cols == 3)
	static R rotation33(const T &X) {
		return as<R>(X.q);
	}

	template <mat_like W>
	    requires(mat_traits<W>::rows == 3 && mat_traits<W>::cols == 1)
	static W translation3(const T &X) {
		return as<W>(X.p);
	}
};

// USD GfTransform (double precision) — row-translation convention
export template <> struct transform_traits<pxr::GfTransform> {
	static constexpr bool valid = true;
	using scalar_type = double;
	using S = double;

	template <mat_like R, mat_like V>
	    requires(mat_traits<R>::rows == 3 && mat_traits<R>::cols == 3 &&
	             mat_traits<V>::rows == 3 && mat_traits<V>::cols == 1)
	static pxr::GfTransform make_rt(const R &R33, const V &t) {
		std::array<std::array<S, 4>, 4> M{};
		for (std::size_t r = 0; r < 3; ++r) {
			for (std::size_t c = 0; c < 3; ++c)
				M[r][c] = static_cast<S>(mat_traits<R>::get(R33, r, c));
			M[r][3] = S(0); // zero last column for rotation rows
		}
		// USD: translation in row 3
		M[3][0] = static_cast<S>(mat_traits<V>::get(t, 0, 0));
		M[3][1] = static_cast<S>(mat_traits<V>::get(t, 1, 0));
		M[3][2] = static_cast<S>(mat_traits<V>::get(t, 2, 0));
		M[3][3] = S(1);

		pxr::GfTransform out;
		out.SetMatrix(as<pxr::GfMatrix4d>(M)); // use your matrix bridge
		return out;
	}

	template <quat_like Q, mat_like V>
	    requires(mat_traits<V>::rows == 3 && mat_traits<V>::cols == 1)
	static pxr::GfTransform make_qt(const Q &q, const V &t) {
		auto R33 = as<std::array<std::array<S, 3>, 3>>(q);
		return make_rt(R33, t);
	}

	template <mat_like R>
	    requires(mat_traits<R>::rows == 3 && mat_traits<R>::cols == 3)
	static R rotation33(const pxr::GfTransform &X) {
		auto M = X.GetMatrix();
		std::array<std::array<S, 3>, 3> Rarr{};
		for (std::size_t r = 0; r < 3; ++r)
			for (std::size_t c = 0; c < 3; ++c)
				Rarr[r][c] = M[static_cast<int>(r)][static_cast<int>(c)];
		return as<R>(Rarr);
	}

	template <mat_like W>
	    requires(mat_traits<W>::rows == 3 && mat_traits<W>::cols == 1)
	static W translation3(const pxr::GfTransform &X) {
		auto M = X.GetMatrix();
		std::array<S, 3> Tarr{M[3][0], M[3][1], M[3][2]}; // row-translation
		return as<W>(Tarr);
	}
};

// Eigen::Transform<S,3,Mode,Opt>
export template <class S, int Mode, int Opt>
struct transform_traits<Eigen::Transform<S, 3, Mode, Opt>> {
	static constexpr bool valid = true;
	using scalar_type = S;
	using T = Eigen::Transform<S, 3, Mode, Opt>;

	template <mat_like R, mat_like V>
	    requires(mat_traits<R>::rows == 3 && mat_traits<R>::cols == 3 &&
	             mat_traits<V>::rows == 3 && mat_traits<V>::cols == 1)
	static T make_rt(const R &R33, const V &t) {
		T out;
		out.linear() = as<Eigen::Matrix<S, 3, 3, 0, 3, 3>>(R33);
		out.translation() = as<Eigen::Matrix<S, 3, 1, 0, 3, 1>>(t);
		return out;
	}

	template <quat_like Q, mat_like V>
	    requires(mat_traits<V>::rows == 3 && mat_traits<V>::cols == 1)
	static T make_qt(const Q &q, const V &t) {
		auto R33 = as<Eigen::Matrix<S, 3, 3, 0, 3, 3>>(q);
		return make_rt(R33, t);
	}

	template <mat_like R>
	    requires(mat_traits<R>::rows == 3 && mat_traits<R>::cols == 3)
	static R rotation33(const T &X) {
		return as<R>(X.linear());
	}

	template <mat_like W>
	    requires(mat_traits<W>::rows == 3 && mat_traits<W>::cols == 1)
	static W translation3(const T &X) {
		return as<W>(X.translation());
	}
};

// transform_of<Tag,S> aliases
export template <class Tag, class S, class = void> struct transform_of;

export template <class S> struct transform_of<std_tag, S, void> {
	using type = std::array<std::array<S, 4>, 4>;
};
export template <class S> struct transform_of<eigen_tag, S, void> {
	using type = Eigen::Matrix<S, 4, 4, 0, 4, 4>;
};
export template <class S> struct transform_of<physx_tag, S, void> {
	using type = physx::PxTransformT<S>;
};
export template <class S> struct transform_of<gf_tag, S, void> {
	using type = pxr::GfTransform;
};

export template <class Tag, class S>
using transform_of_t =
    typename transform_of<Tag, std::remove_cv_t<S>, void>::type;

// ── Transform bridging API ───────────────────────────────────────────────────
export template <transform_like To, mat_like From>
    requires(!mat_like<To>) && (mat_rows_v<From> == 4 && mat_cols_v<From> == 4)
[[nodiscard]] constexpr To as(const From &M44) {
	using Sx = transform_scalar_t<To>;
	std::array<std::array<Sx, 3>, 3> R{};
	std::array<Sx, 3> t{};
	for (std::size_t r = 0; r < 3; ++r) {
		for (std::size_t c = 0; c < 3; ++c)
			R[r][c] = static_cast<Sx>(mat_traits<From>::get(M44, r, c));
		t[r] = static_cast<Sx>(mat_traits<From>::get(M44, r, 3)); // last column
	}
	return transform_traits<To>::template make_rt<>(R, t);
}

export template <transform_like To, mat_like R, mat_like V>
    requires(!mat_like<To>) &&
            (mat_traits<R>::rows == 3 && mat_traits<R>::cols == 3) &&
            (mat_traits<V>::rows == 3 && mat_traits<V>::cols == 1)
[[nodiscard]] constexpr To as(const R &R33, const V &t) {
	return transform_traits<To>::template make_rt<>(R33, t);
}

export template <transform_like To, quat_like Q, mat_like V>
    requires(!mat_like<To>) &&
            (mat_traits<V>::rows == 3 && mat_traits<V>::cols == 1)
[[nodiscard]] constexpr To as(const Q &q, const V &t) {
	return transform_traits<To>::template make_qt<>(q, t);
}

export template <mat_like To, transform_like From>
    requires(!mat_like<From>) && (mat_rows_v<To> == 4 && mat_cols_v<To> == 4)
[[nodiscard]] constexpr To as(const From &X) {
	using S = mat_scalar_t<To>;
	auto R = transform_traits<From>::template rotation33<
	    std::array<std::array<double, 3>, 3>>(X);
	auto t =
	    transform_traits<From>::template translation3<std::array<double, 3>>(X);
	std::array<std::array<S, 4>, 4> M{};
	for (std::size_t r = 0; r < 3; ++r) {
		for (std::size_t c = 0; c < 3; ++c)
			M[r][c] = static_cast<S>(R[r][c]);
		M[r][3] = static_cast<S>(t[r]); // last column
	}
	M[3][0] = M[3][1] = M[3][2] = S(0);
	M[3][3] = S(1);
	return as<To>(M);
}

export template <quat_like To, transform_like From>
[[nodiscard]] constexpr To as(const From &X) {
	auto R = transform_traits<From>::template rotation33<
	    std::array<std::array<double, 3>, 3>>(X);
	return as<To>(R);
}

export template <mat_like To, transform_like From>
    requires(mat_rows_v<To> == 3 && mat_cols_v<To> == 1)
[[nodiscard]] constexpr To as(const From &X) {
	return transform_traits<From>::template translation3<To>(X);
}

export template <mat_like To, transform_like From>
    requires(mat_rows_v<To> == 3 && mat_cols_v<To> == 3)
[[nodiscard]] constexpr To as(const From &X) {
	return transform_traits<From>::template rotation33<To>(X);
}

// Pose extraction aliases
export template <class Tag, class S>
using pose_qt_of_t = std::pair<quat_of_t<Tag, S>, mat_of_t<Tag, S, 3, 1>>;

export template <class Tag, class S>
using pose_rt_of_t = std::pair<mat_of_t<Tag, S, 3, 3>, mat_of_t<Tag, S, 3, 1>>;

export template <class Tag, class S, transform_like From>
[[nodiscard]] constexpr pose_qt_of_t<Tag, S> as_pose_qt(const From &X) {
	return {as<quat_of_t<Tag, S>>(X), as<mat_of_t<Tag, S, 3, 1>>(X)};
}

export template <class Tag, class S, transform_like From>
[[nodiscard]] constexpr pose_rt_of_t<Tag, S> as_pose_rt(const From &X) {
	return {as<mat_of_t<Tag, S, 3, 3>>(X), as<mat_of_t<Tag, S, 3, 1>>(X)};
}

export template <class To, class From>
concept as_convertible = requires(From &&f) {
	{ as<To>(std::forward<From>(f)) } -> std::same_as<To>;
};

} // namespace lego_assemble
