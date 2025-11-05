export module lego_assemble.utils.transforms;

import std;
import lego_assemble.vendor.eigen;
import lego_assemble.utils.group;

namespace lego_assemble {

template <typename Scalar> struct SE3Group {
	using QuaternionType = Eigen::Quaternion<Scalar>;
	using TranslationType = Eigen::Vector<Scalar, 3>;
	using TransformType = std::pair<QuaternionType, TranslationType>;
	using ElementType = std::pair<QuaternionType, TranslationType>;
	TransformType identity() {
		return {QuaternionType::Identity(), TranslationType::Zero()};
	}
	TransformType combine(const TransformType &a, const TransformType &b) {
		const auto &[R_a, t_a] = a;
		const auto &[R_b, t_b] = b;
		return {
		    R_a * R_b,
		    R_a * t_b + t_a,
		};
	}
	TransformType invert(const TransformType &T) {
		const auto &[R, t] = T;
		return {
		    R.conjugate(),
		    R.conjugate() * (-t),
		};
	}
	TransformType project(const TransformType &T) {
		const auto &[R, t] = T;
		return {
		    R.normalized(),
		    t,
		};
	}
};

export template <typename Scalar, double RotationWeight = 1.0,
                 double TranslationWeight = 1.0, double Epsilon = 1e-6>
struct SE3MetricGroup : SE3Group<Scalar> {
	using Base = SE3Group<Scalar>;
	using QuaternionType = typename Base::QuaternionType;
	using TranslationType = typename Base::TranslationType;
	using TransformType = typename Base::TransformType;
	using ElementType = typename Base::ElementType;
	double distance(const TransformType &a, const TransformType &b) {
		QuaternionType qa = a.first.normalized();
		QuaternionType qb = b.first.normalized();
		const auto &ta = a.second;
		const auto &tb = b.second;
		double q_diff = static_cast<double>(qa.angularDistance(qb));
		double t_diff =
		    static_cast<double>((qa.conjugate() * (tb - ta)).norm());
		return std::hypot(RotationWeight * q_diff, TranslationWeight * t_diff);
	}
	bool almost_equal(const TransformType &a, const TransformType &b) {
		return distance(a, b) < Epsilon;
	}
};

export using SE3d = SE3MetricGroup<double>;
export using Transformd = SE3d::TransformType;
export Transformd operator*(const Transformd &a, const Transformd &b) {
	return SE3d{}.combine(a, b);
}
export Transformd inverse(const Transformd &T) {
	return SE3d{}.invert(T);
}
static_assert(GroupLike<SE3d>);
static_assert(ProjectableGroupLike<SE3d>);
static_assert(MetricGroupLike<SE3d>);

} // namespace lego_assemble
