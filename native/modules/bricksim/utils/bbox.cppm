export module bricksim.utils.bbox;

import std;
import bricksim.utils.transforms;
import bricksim.vendor;

namespace bricksim {

export struct BBox2d {
	static BBox2d from_vertices(std::span<const Eigen::Vector2d> vertices) {
		constexpr double inf = std::numeric_limits<double>::infinity();
		Eigen::Vector2d min{inf, inf};
		Eigen::Vector2d max{-inf, -inf};
		for (const auto &v : vertices) {
			min = min.cwiseMin(v);
			max = max.cwiseMax(v);
		}
		return {.min = min, .max = max};
	}

	Eigen::Vector2d min;
	Eigen::Vector2d max;
	bool operator==(const BBox2d &other) const = default;

	void expand_to_include(const Eigen::Vector2d &point) {
		min = min.cwiseMin(point);
		max = max.cwiseMax(point);
	}
	void expand_to_include(const BBox2d &other) {
		min = min.cwiseMin(other.min);
		max = max.cwiseMax(other.max);
	}
	bool overlaps(const BBox2d &other) const {
		return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
		       (min.y() <= other.max.y() && max.y() >= other.min.y());
	}
};

export struct BBox3d {
	static BBox3d from_vertices(std::span<const Eigen::Vector3d> vertices) {
		constexpr double inf = std::numeric_limits<double>::infinity();
		Eigen::Vector3d min{inf, inf, inf};
		Eigen::Vector3d max{-inf, -inf, -inf};
		for (const auto &v : vertices) {
			min = min.cwiseMin(v);
			max = max.cwiseMax(v);
		}
		return {.min = min, .max = max};
	}

	Eigen::Vector3d min;
	Eigen::Vector3d max;
	bool operator==(const BBox3d &other) const = default;

	void expand_to_include(const Eigen::Vector3d &point) {
		min = min.cwiseMin(point);
		max = max.cwiseMax(point);
	}
	void expand_to_include(const BBox3d &other) {
		min = min.cwiseMin(other.min);
		max = max.cwiseMax(other.max);
	}
	bool overlaps(const BBox3d &other) const {
		return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
		       (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
		       (min.z() <= other.max.z() && max.z() >= other.min.z());
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

} // namespace bricksim
