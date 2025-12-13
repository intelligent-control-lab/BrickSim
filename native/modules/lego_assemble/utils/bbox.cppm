export module lego_assemble.utils.bbox;

import std;
import lego_assemble.utils.transforms;
import lego_assemble.vendor;

namespace lego_assemble {

export struct BBox2d {
	Eigen::Vector2d min;
	Eigen::Vector2d max;
	bool operator==(const BBox2d &other) const = default;
};

export struct BBox3d {
	Eigen::Vector3d min;
	Eigen::Vector3d max;
	bool operator==(const BBox3d &other) const = default;

	void expand_to_include(const BBox3d &other) {
		min = min.cwiseMin(other.min);
		max = max.cwiseMax(other.max);
	}
	BBox2d to_2d() const {
		return {
		    .min = min.head<2>(),
		    .max = max.head<2>(),
		};
	}
	BBox3d transform(const Transformd &T) const {
		const auto &[q, t] = T;
		constexpr double inf = std::numeric_limits<double>::infinity();
		Eigen::Vector3d new_min{inf, inf, inf};
		Eigen::Vector3d new_max{-inf, -inf, -inf};
		for (int ix = 0; ix < 2; ++ix) {
			double cx = (ix == 0) ? min.x() : max.x();
			for (int iy = 0; iy < 2; ++iy) {
				double cy = (iy == 0) ? min.y() : max.y();
				for (int iz = 0; iz < 2; ++iz) {
					double cz = (iz == 0) ? min.z() : max.z();
					Eigen::Vector3d p_local{cx, cy, cz};
					Eigen::Vector3d p = q * p_local + t;
					new_min = new_min.cwiseMin(p);
					new_max = new_max.cwiseMax(p);
				}
			}
		}
		return {.min = new_min, .max = new_max};
	}
};

} // namespace lego_assemble
