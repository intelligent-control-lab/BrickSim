export module bricksim.utils.c4_rotation;

import std;
import bricksim.vendor;

namespace bricksim {

export enum class YawC4 : std::int8_t {
	DEG_0 = 0,
	DEG_90 = 1,
	DEG_180 = 2,
	DEG_270 = 3,
};

export constexpr YawC4 to_c4(int idx) {
	switch ((idx % 4 + 4) % 4) {
	case 0:
		return YawC4::DEG_0;
	case 1:
		return YawC4::DEG_90;
	case 2:
		return YawC4::DEG_180;
	case 3:
		return YawC4::DEG_270;
	default:
		std::unreachable();
	}
}

export constexpr YawC4 nearest_c4(double a, double &out_remainder) {
	constexpr double h = std::numbers::pi / 2;
	long long k = std::llround(a / h);
	out_remainder = a - k * h;
	int idx = static_cast<int>((k % 4 + 4) % 4);
	return static_cast<YawC4>(idx);
}

export constexpr YawC4 nearest_c4(double a) {
	double _;
	return nearest_c4(a, _);
}

export constexpr YawC4 c4_inverse(YawC4 q) {
	switch (q) {
	case YawC4::DEG_0:
		return YawC4::DEG_0;
	case YawC4::DEG_90:
		return YawC4::DEG_270;
	case YawC4::DEG_180:
		return YawC4::DEG_180;
	case YawC4::DEG_270:
		return YawC4::DEG_90;
	default:
		std::unreachable();
	}
}

export template <class S>
Eigen::Vector<S, 2> c4_rotate(YawC4 q, Eigen::Vector<S, 2> v) {
	switch (q) {
	case YawC4::DEG_0:
		return v;
	case YawC4::DEG_90:
		return {-v[1], v[0]};
	case YawC4::DEG_180:
		return {-v[0], -v[1]};
	case YawC4::DEG_270:
		return {v[1], -v[0]};
	default:
		std::unreachable();
	}
}

export template <class S> Eigen::Matrix<S, 2, 2> c4_to_matrix(YawC4 q) {
	switch (q) {
	case YawC4::DEG_0:
		return Eigen::Matrix<S, 2, 2>::Identity();
	case YawC4::DEG_90:
		return (Eigen::Matrix<S, 2, 2>() << S(0), S(-1), S(1), S(0)).finished();
	case YawC4::DEG_180:
		return (Eigen::Matrix<S, 2, 2>() << S(-1), S(0), S(0), S(-1))
		    .finished();
	case YawC4::DEG_270:
		return (Eigen::Matrix<S, 2, 2>() << S(0), S(1), S(-1), S(0)).finished();
	default:
		std::unreachable();
	}
}

export Eigen::Quaterniond c4_to_quat(YawC4 q) {
	switch (q) {
	case YawC4::DEG_0:
		return {1.0, 0.0, 0.0, 0.0};
	case YawC4::DEG_90:
		return {std::sqrt(2) / 2, 0.0, 0.0, std::sqrt(2) / 2};
	case YawC4::DEG_180:
		return {0.0, 0.0, 0.0, 1.0};
	case YawC4::DEG_270:
		return {std::sqrt(2) / 2, 0.0, 0.0, -std::sqrt(2) / 2};
	default:
		std::unreachable();
	}
}

} // namespace bricksim
