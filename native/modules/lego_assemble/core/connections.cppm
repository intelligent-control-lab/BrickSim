export module lego_assemble.core.connections;

import std;

import lego_assemble.core.specs;
import lego_assemble.utils.c4_rotation;
import lego_assemble.utils.transforms;
import lego_assemble.utils.unordered_pair;
import lego_assemble.vendor.eigen;

namespace lego_assemble {

export struct ConnectionLocalTransform {
	// Connection local frame:
	// - Origin: midpoint of the overlapping area
	// - Axes: stud interface axes
	Transformd T_stud_local{};
	Transformd T_hole_local{};
	Eigen::Vector2i overlap{};
};

export struct ConnectionSegment {
	// offset of hole's origin relative to stud in stud interface grid
	Eigen::Vector2i offset{};
	// yaw of hole relative to stud
	YawC4 yaw{};

	ConnectionLocalTransform
	compute_local_transform(const InterfaceSpec &stud,
	                        const InterfaceSpec &hole) const {
		using namespace Eigen;
		const auto &[R_stud, t_stud] = stud.pose;
		const auto &[R_hole, t_hole] = hole.pose;
		// Calculate the min/max corners of the stud and hole areas in stud interface grid
		Vector2i stud_min = Vector2i::Zero();
		Vector2i stud_max{stud.L, stud.W};
		Vector2i hole_min = offset;
		Vector2i hole_max = hole_min + c4_rotate(yaw, Vector2i{hole.L, hole.W});
		// Calculate overlap
		Vector2i interval1_start = stud_min;
		Vector2i interval1_end = stud_max;
		Vector2i interval2_start = hole_min.cwiseMin(hole_max);
		Vector2i interval2_end = hole_min.cwiseMax(hole_max);
		Vector2i ov_start = interval1_start.cwiseMax(interval2_start);
		Vector2i ov_end = interval1_end.cwiseMin(interval2_end);
		Vector2d ov_mid = (ov_start + ov_end).cast<double>() / 2.0;
		Vector2i overlap = (ov_end - ov_start).cwiseMax(Vector2i::Zero());
		// Calculate T_stud_to_local
		Quaterniond R_stud_local = R_stud;
		Vector3d t_stud_local =
		    t_stud +
		    R_stud * (Vector3d(ov_mid(0), ov_mid(1), 0.0) * BrickUnitLength);
		// Calculate T_hole_to_local
		YawC4 yaw_inv = c4_inverse(yaw);
		Quaterniond R_hi_local = c4_to_quat(yaw_inv);
		Vector2d t_hi_local_2d =
		    c4_rotate(yaw_inv, Vector2d((ov_mid - offset.cast<double>()) *
		                                BrickUnitLength));
		Quaterniond R_hole_local = R_hole * R_hi_local;
		Vector3d t_hole_local =
		    t_hole + R_hole * Vector3d{t_hi_local_2d(0), t_hi_local_2d(1), 0.0};
		return {
		    .T_stud_local = {R_stud_local, t_stud_local},
		    .T_hole_local = {R_hole_local, t_hole_local},
		    .overlap = overlap,
		};
	}

	// Compute T_stud_hole
	Transformd compute_transform(const InterfaceSpec &stud,
	                             const InterfaceSpec &hole) const {
		using namespace Eigen;
		Transformd T_si_hi{
		    c4_to_quat(yaw),
		    {offset(0) * BrickUnitLength, offset(1) * BrickUnitLength, 0.0},
		};
		return stud.pose * T_si_hi * inverse(hole.pose);
	}

	bool operator==(const ConnectionSegment &other) const = default;
};

} // namespace lego_assemble
