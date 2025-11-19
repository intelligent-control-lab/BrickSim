export module lego_assemble.core.assembly;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.utils.transforms;
import lego_assemble.utils.c4_rotation;
import lego_assemble.vendor;

namespace lego_assemble {

export struct AssemblyThresholds {
	// Maximum distance between bricks (m)
	double DistanceTolerance = 0.001;

	// Maximum penetration between bricks (m), penetration can happen due to simulation inaccuracies
	double MaxPenetration = 0.005;

	// Maximum angle between z-axis of bricks (rad)
	double ZAngleTolerance = 5.0 * (std::numbers::pi / 180.0);

	// Minimum clutch power (N)
	double RequiredForce = 1.0;

	// Maximum yaw error (rad)
	double YawTolerance = 5.0 * (std::numbers::pi / 180.0);

	// Maximum position error (m)
	double PositionTolerance = 0.002;
};

export class AssemblyChecker {
  public:
	// Do not modify this while PhysX is stepping, really.
	AssemblyThresholds thresholds;

	AssemblyChecker(AssemblyThresholds thresholds = {})
	    : thresholds{thresholds} {}

	std::optional<ConnectionSegment>
	detect_assembly(const InterfaceSpec &stud_iface,
	                const InterfaceSpec &hole_iface, const Transformd &T_stud,
	                const Transformd &T_hole, const Eigen::Vector3d &force
	                // TODO: add moment
	) const {
		// Compute useful transforms
		Transformd T_studif = T_stud * stud_iface.pose;
		Transformd T_holeif = T_hole * hole_iface.pose;
		const auto &[q_stud, t_stud] = T_studif;
		Transformd T_rel = inverse(T_studif) * T_holeif; // aka T_studif_holeif
		const auto &[q_rel, t_rel] = T_rel;

		// Calculate relative distance
		double rel_dist = t_rel(2) - StudHeight;

		// Calculate z-axis alignment
		Eigen::Vector3d z_rel = q_rel * Eigen::Vector3d::UnitZ();
		double tilt = std::atan2(z_rel.head<2>().norm(), z_rel(2));

		// Calculate projected force along stud's z-axis
		double fz = force.dot(q_stud * Eigen::Vector3d::UnitZ());

		// Calculate snapped yaw & yaw error
		Eigen::Vector3d x_rel = q_rel * Eigen::Vector3d::UnitX();
		double rel_yaw = std::atan2(x_rel.y(), x_rel.x());
		double yaw_err;
		YawC4 yaw_snap = nearest_c4(rel_yaw, yaw_err);

		// Calculate snapped position in grid coordinates & position error
		Eigen::Vector2d p0 = t_rel.head<2>() / BrickUnitLength;
		Eigen::Vector2i p0_snap = p0.array().round().cast<int>();
		Eigen::Vector2i p1_snap =
		    p0_snap +
		    c4_rotate(yaw_snap, Eigen::Vector2i(hole_iface.L, hole_iface.W));
		double p_err = (p0 - p0_snap.cast<double>()).norm() * BrickUnitLength;

		// Calculate overlap
		Eigen::Vector2i ov_start1 = Eigen::Vector2i::Zero();
		Eigen::Vector2i ov_end1(stud_iface.L, stud_iface.W);
		Eigen::Vector2i ov_start2 = p0_snap.cwiseMin(p1_snap);
		Eigen::Vector2i ov_end2 = p0_snap.cwiseMax(p1_snap);
		Eigen::Vector2i ov_start = ov_start1.cwiseMax(ov_start2);
		Eigen::Vector2i ov_end = ov_end1.cwiseMin(ov_end2);
		Eigen::Vector2i overlap =
		    (ov_end - ov_start).cwiseMax(Eigen::Vector2i::Zero());

		// Filtering
		bool accept = (rel_dist <= thresholds.DistanceTolerance) &&
		              (rel_dist >= -thresholds.MaxPenetration) &&
		              (tilt <= thresholds.ZAngleTolerance) &&
		              (fz <= -thresholds.RequiredForce) &&
		              (std::abs(yaw_err) <= thresholds.YawTolerance) &&
		              (p_err <= thresholds.PositionTolerance) &&
		              ((overlap(0) > 0) && (overlap(1) > 0));
		if (!accept) {
			return std::nullopt;
		}
		return ConnectionSegment{
		    .offset = p0_snap,
		    .yaw = yaw_snap,
		};
	}
};

} // namespace lego_assemble
