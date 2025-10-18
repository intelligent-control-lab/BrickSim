#pragma once

#include <Eigen/Dense>
#include <foundation/PxQuat.h>
#include <foundation/PxTransform.h>

namespace lego_assemble {

template <typename T>
Eigen::Quaternion<T> pxQuatToEigen(const physx::PxQuatT<T> &q) {
	return {q.w, q.x, q.y, q.z};
}

template <typename T>
Eigen::Matrix<T, 3, 1> pxVec3ToEigen(const physx::PxVec3T<T> &v) {
	return {v.x, v.y, v.z};
}

template <typename To, typename From, size_t N>
Eigen::Matrix<To, N, 1> arrayToEigen(const std::array<From, N> &arr) {
	Eigen::Matrix<To, N, 1> m;
	for (size_t i = 0; i < N; ++i) {
		m(i) = static_cast<To>(arr[i]);
	}
	return m;
}

template <typename To, typename From>
physx::PxTransformT<To> eigenToPxTransform(const Eigen::Quaternion<From> &q,
                                           const Eigen::Matrix<From, 3, 1> &t) {
	return {physx::PxVec3T<To>(static_cast<To>(t.x()), static_cast<To>(t.y()),
	                           static_cast<To>(t.z())),
	        physx::PxQuatT<To>(static_cast<To>(q.x()), static_cast<To>(q.y()),
	                           static_cast<To>(q.z()), static_cast<To>(q.w()))};
}

template <typename To, typename From>
physx::PxTransformT<To> eigenToPxTransform(const Eigen::Matrix<From, 3, 3> &R,
                                           const Eigen::Matrix<From, 3, 1> &t) {
	Eigen::Quaternion<From> q(R);
	return eigenToPxTransform<To, From>(q, t);
}

template <typename D>
    requires std::derived_from<D, Eigen::EigenBase<D>> &&
             std::floating_point<typename D::Scalar>
auto round(const Eigen::DenseBase<D> &m) {
	using S = typename D::Scalar;
	return m.derived().unaryExpr([](S v) { return std::round(v); }).eval();
}

} // namespace lego_assemble
