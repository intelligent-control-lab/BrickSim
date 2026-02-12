export module lego_assemble.core.connections;

import std;

import lego_assemble.core.specs;
import lego_assemble.utils.c4_rotation;
import lego_assemble.utils.transforms;
import lego_assemble.utils.unordered_pair;
import lego_assemble.utils.memory;
import lego_assemble.utils.math;
import lego_assemble.vendor;

namespace lego_assemble {

export struct ConnectionOverlap {
	Eigen::Vector2i stud_start{};
	Eigen::Vector2i stud_end{};
	Eigen::Vector2i hole_start{};
	Eigen::Vector2i hole_end{};
	Eigen::Vector2i ov_start{};
	Eigen::Vector2i ov_end{};
	Eigen::Vector2d ov_mid{};
	Eigen::Vector2i overlap{};
};

export struct ConnectionLocalTransform {
	// Connection local frame:
	// - Origin: midpoint of the overlapping area
	// - Axes: stud interface axes
	Transformd T_stud_local{};
	Transformd T_hole_local{};
};

export struct ConnectionFrictionPoint {
	Eigen::Vector2d p_local{};
	Eigen::Vector2d n_local{};
};

export struct ConnectionSegment {
	// offset of hole's origin relative to stud in stud interface grid
	Eigen::Vector2i offset{};
	// yaw of hole relative to stud
	YawC4 yaw{};

	ConnectionOverlap compute_overlap(const InterfaceSpec &stud,
	                                  const InterfaceSpec &hole) const {
		using namespace Eigen;
		Vector2i stud_min = Vector2i::Zero();
		Vector2i stud_max{stud.L, stud.W};
		Vector2i hole_min = offset;
		Vector2i hole_max = hole_min + c4_rotate(yaw, Vector2i{hole.L, hole.W});
		Vector2i stud_start = stud_min;
		Vector2i stud_end = stud_max;
		Vector2i hole_start = hole_min.cwiseMin(hole_max);
		Vector2i hole_end = hole_min.cwiseMax(hole_max);
		Vector2i ov_start = stud_start.cwiseMax(hole_start);
		Vector2i ov_end = stud_end.cwiseMin(hole_end);
		Vector2d ov_mid = (ov_start + ov_end).cast<double>() / 2.0;
		Vector2i overlap = (ov_end - ov_start).cwiseMax(Vector2i::Zero());
		return {
		    .stud_start = stud_start,
		    .stud_end = stud_end,
		    .hole_start = hole_start,
		    .hole_end = hole_end,
		    .ov_start = ov_start,
		    .ov_end = ov_end,
		    .ov_mid = ov_mid,
		    .overlap = overlap,
		};
	}

	ConnectionLocalTransform
	compute_local_transform(const InterfaceSpec &stud,
	                        const InterfaceSpec &hole,
	                        const ConnectionOverlap &ov) const {
		using namespace Eigen;
		const auto &[R_stud, t_stud] = stud.pose;
		const auto &[R_hole, t_hole] = hole.pose;
		// Calculate T_stud_to_local
		Quaterniond R_stud_local = R_stud;
		Vector3d t_stud_local =
		    t_stud + R_stud * (Vector3d(ov.ov_mid(0), ov.ov_mid(1), 0.0) *
		                       BrickUnitLength);
		// Calculate T_hole_to_local
		YawC4 yaw_inv = c4_inverse(yaw);
		Quaterniond R_hi_local = c4_to_quat(yaw_inv);
		Vector2d t_hi_local_2d =
		    c4_rotate(yaw_inv, Vector2d((ov.ov_mid - offset.cast<double>()) *
		                                BrickUnitLength));
		Quaterniond R_hole_local = R_hole * R_hi_local;
		Vector3d t_hole_local =
		    t_hole + R_hole * Vector3d{t_hi_local_2d(0), t_hi_local_2d(1), 0.0};
		return {
		    .T_stud_local = {R_stud_local, t_stud_local},
		    .T_hole_local = {R_hole_local, t_hole_local},
		};
	}

	ConnectionLocalTransform
	compute_local_transform(const InterfaceSpec &stud,
	                        const InterfaceSpec &hole) const {
		ConnectionOverlap ov = compute_overlap(stud, hole);
		return compute_local_transform(stud, hole, ov);
	}

	// Compute T_stud_hole
	Transformd compute_transform(const InterfaceSpec &stud,
	                             const InterfaceSpec &hole) const {
		Transformd T_si_hi{
		    c4_to_quat(yaw),
		    {offset(0) * BrickUnitLength, offset(1) * BrickUnitLength, 0.0},
		};
		return stud.pose * T_si_hi * inverse(hole.pose);
	}

	// Returns the friction points in connection local frame and their normals
	aligned_generator<ConnectionFrictionPoint>
	friction_points([[maybe_unused]] const InterfaceSpec &stud,
	                const InterfaceSpec &hole,
	                const ConnectionOverlap &ov) const {
		using namespace Eigen;
		constexpr double R = StudDiameter / 2.0;
		constexpr double R_sqrt2 = R / std::numbers::sqrt2;
		for (int i = ov.ov_start.x(); i < ov.ov_end.x(); ++i) {
			for (int j = ov.ov_start.y(); j < ov.ov_end.y(); ++j) {
				Vector2d center =
				    (Vector2d{i + 0.5, j + 0.5} - ov.ov_mid) * BrickUnitLength;
				auto friction_point = [&](double x, double y) {
					return ConnectionFrictionPoint{
					    .p_local = center + Vector2d{x, y},
					    .n_local = Vector2d{x, y}.normalized(),
					};
				};
				if (hole.L == 1 || hole.W == 1) {
					// Single stud hole
					co_yield friction_point(R, 0);
					co_yield friction_point(0, R);
					co_yield friction_point(-R, 0);
					co_yield friction_point(0, -R);
				} else if (i == ov.hole_start.x() && j == ov.hole_start.y()) {
					co_yield friction_point(-R, 0);
					co_yield friction_point(0, -R);
					co_yield friction_point(R_sqrt2, R_sqrt2);
				} else if (i == ov.hole_start.x() && j == ov.hole_end.y() - 1) {
					co_yield friction_point(0, R);
					co_yield friction_point(-R, 0);
					co_yield friction_point(R_sqrt2, -R_sqrt2);
				} else if (i == ov.hole_end.x() - 1 && j == ov.hole_start.y()) {
					co_yield friction_point(0, -R);
					co_yield friction_point(R, 0);
					co_yield friction_point(-R_sqrt2, R_sqrt2);
				} else if (i == ov.hole_end.x() - 1 &&
				           j == ov.hole_end.y() - 1) {
					co_yield friction_point(R, 0);
					co_yield friction_point(0, R);
					co_yield friction_point(-R_sqrt2, -R_sqrt2);
				} else if (i == ov.hole_start.x()) {
					co_yield friction_point(-R, 0);
					co_yield friction_point(R_sqrt2, -R_sqrt2);
					co_yield friction_point(R_sqrt2, R_sqrt2);
				} else if (i == ov.hole_end.x() - 1) {
					co_yield friction_point(R, 0);
					co_yield friction_point(-R_sqrt2, R_sqrt2);
					co_yield friction_point(-R_sqrt2, -R_sqrt2);
				} else if (j == ov.hole_start.y()) {
					co_yield friction_point(0, -R);
					co_yield friction_point(R_sqrt2, R_sqrt2);
					co_yield friction_point(-R_sqrt2, R_sqrt2);
				} else if (j == ov.hole_end.y() - 1) {
					co_yield friction_point(0, R);
					co_yield friction_point(-R_sqrt2, -R_sqrt2);
					co_yield friction_point(R_sqrt2, -R_sqrt2);
				} else {
					co_yield friction_point(R_sqrt2, R_sqrt2);
					co_yield friction_point(-R_sqrt2, R_sqrt2);
					co_yield friction_point(-R_sqrt2, -R_sqrt2);
					co_yield friction_point(R_sqrt2, -R_sqrt2);
				}
			}
		}
	}

	std::vector<ConnectionFrictionPoint>
	friction_points_hull([[maybe_unused]] const InterfaceSpec &stud,
	                     const InterfaceSpec &hole,
	                     const ConnectionOverlap &ov) const {
		using namespace Eigen;
		constexpr double R = StudDiameter / 2.0;
		constexpr double R_sqrt2 = R / std::numbers::sqrt2;
		if (ov.overlap.x() <= 0 || ov.overlap.y() <= 0) {
			return {};
		}
		std::vector<ConnectionFrictionPoint> pts;
		pts.reserve(16);
		auto visit_point = [&](int i, int j) {
			Vector2d center =
			    (Vector2d{i + 0.5, j + 0.5} - ov.ov_mid) * BrickUnitLength;
			auto push_point = [&](double x, double y) {
				pts.push_back({
				    .p_local = center + Vector2d{x, y},
				    .n_local = Vector2d{x, y}.normalized(),
				});
			};
			if (hole.L == 1 || hole.W == 1) {
				// Single stud hole
				push_point(R, 0);
				push_point(0, R);
				push_point(-R, 0);
				push_point(0, -R);
			} else if (i == ov.hole_start.x() && j == ov.hole_start.y()) {
				push_point(-R, 0);
				push_point(0, -R);
				push_point(R_sqrt2, R_sqrt2);
			} else if (i == ov.hole_start.x() && j == ov.hole_end.y() - 1) {
				push_point(0, R);
				push_point(-R, 0);
				push_point(R_sqrt2, -R_sqrt2);
			} else if (i == ov.hole_end.x() - 1 && j == ov.hole_start.y()) {
				push_point(0, -R);
				push_point(R, 0);
				push_point(-R_sqrt2, R_sqrt2);
			} else if (i == ov.hole_end.x() - 1 && j == ov.hole_end.y() - 1) {
				push_point(R, 0);
				push_point(0, R);
				push_point(-R_sqrt2, -R_sqrt2);
			} else if (i == ov.hole_start.x()) {
				push_point(-R, 0);
				push_point(R_sqrt2, -R_sqrt2);
				push_point(R_sqrt2, R_sqrt2);
			} else if (i == ov.hole_end.x() - 1) {
				push_point(R, 0);
				push_point(-R_sqrt2, R_sqrt2);
				push_point(-R_sqrt2, -R_sqrt2);
			} else if (j == ov.hole_start.y()) {
				push_point(0, -R);
				push_point(R_sqrt2, R_sqrt2);
				push_point(-R_sqrt2, R_sqrt2);
			} else if (j == ov.hole_end.y() - 1) {
				push_point(0, R);
				push_point(-R_sqrt2, -R_sqrt2);
				push_point(R_sqrt2, -R_sqrt2);
			} else {
				push_point(R_sqrt2, R_sqrt2);
				push_point(-R_sqrt2, R_sqrt2);
				push_point(-R_sqrt2, -R_sqrt2);
				push_point(R_sqrt2, -R_sqrt2);
			}
		};
		visit_point(ov.ov_start.x(), ov.ov_start.y());
		if (ov.overlap.x() > 1) {
			visit_point(ov.ov_end.x() - 1, ov.ov_start.y());
		}
		if (ov.overlap.y() > 1) {
			visit_point(ov.ov_start.x(), ov.ov_end.y() - 1);
		}
		if (ov.overlap.x() > 1 && ov.overlap.y() > 1) {
			visit_point(ov.ov_end.x() - 1, ov.ov_end.y() - 1);
		}
		std::ranges::sort(pts, {}, [](const auto &p) {
			return std::tie(p.p_local.x(), p.p_local.y());
		});
		std::vector<ConnectionFrictionPoint> hull;
		hull.reserve(pts.size());
		for (const auto &pt : pts) {
			while (hull.size() >= 2 &&
			       cross2(hull[hull.size() - 2].p_local, hull.back().p_local,
			              pt.p_local) <= 0) {
				hull.pop_back();
			}
			hull.push_back(pt);
		}
		std::size_t lower_size = hull.size();
		for (auto it = pts.rbegin(); it != pts.rend(); ++it) {
			while (hull.size() > lower_size &&
			       cross2(hull[hull.size() - 2].p_local, hull.back().p_local,
			              it->p_local) <= 0) {
				hull.pop_back();
			}
			hull.push_back(*it);
		}
		if (hull.size() > 1)
			hull.pop_back();
		return hull;
	}

	bool operator==(const ConnectionSegment &other) const = default;
};

} // namespace lego_assemble
