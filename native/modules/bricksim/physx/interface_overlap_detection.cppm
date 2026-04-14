export module bricksim.physx.interface_overlap_detection;

import std;
import bricksim.core.specs;
import bricksim.core.graph;
import bricksim.core.connections;
import bricksim.physx.overlap_detection;
import bricksim.utils.c4_rotation;
import bricksim.utils.transforms;
import bricksim.utils.bbox;
import bricksim.vendor;

namespace bricksim {

export class InterfacePatch {
  public:
	InterfacePatch(PartId pid, const InterfaceSpec &iface,
	               const Transformd &T_part)
	    : pid_(pid), iface_(iface) {
		T_iface_ = T_part * iface.pose;
		T_query_ = compute_query_transform(T_iface_, iface);
	}

	PartId part_id() const {
		return pid_;
	}

	InterfaceId interface_id() const {
		return iface_.id;
	}

	InterfaceRef interface_ref() const {
		return {pid_, iface_.id};
	}

	const InterfaceSpec &interface_spec() const {
		return iface_;
	}

	InterfaceType interface_type() const {
		return iface_.type;
	}

	const Transformd &transform() const {
		return T_query_;
	}

	const Transformd &interface_transform() const {
		return T_iface_;
	}

	BBox2d bbox() const {
		double L = iface_.L * BrickUnitLength;
		double W = iface_.W * BrickUnitLength;
		return {.min = {0.0, 0.0}, .max = {L, W}};
	}

	std::array<Eigen::Vector2d, 4> polygon_vertices() const {
		double L = iface_.L * BrickUnitLength;
		double W = iface_.W * BrickUnitLength;
		return {
		    Eigen::Vector2d{0.0, 0.0},
		    Eigen::Vector2d{L, 0.0},
		    Eigen::Vector2d{L, W},
		    Eigen::Vector2d{0.0, W},
		};
	}

	InterfacePatch transformed(const Transformd &T_dst_src) const {
		InterfacePatch copy = *this;
		copy.T_iface_ = T_dst_src * copy.T_iface_;
		copy.T_query_ = compute_query_transform(copy.T_iface_, copy.iface_);
		return copy;
	}

  private:
	static Transformd compute_query_transform(const Transformd &T_iface,
	                                          const InterfaceSpec &iface) {
		if (iface.type == InterfaceType::Stud) {
			return T_iface;
		}
		Transformd T_adjust{
		    Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitX()),
		    Eigen::Vector3d{0.0, iface.W * BrickUnitLength, 0.0},
		};
		return T_iface * T_adjust;
	}

	PartId pid_;
	InterfaceSpec iface_;
	Transformd T_iface_;
	Transformd T_query_;
};

export struct CandidateConnection {
	InterfaceRef iface_stud;
	InterfaceRef iface_hole;
	ConnectionSegment conn_seg;
	ConnectionOverlap overlap;

	ConnSegRef conn_seg_ref() const {
		return {iface_stud, iface_hole};
	}
};

export class InterfaceBinMap {
  public:
	template <class G>
	void add_part(const G &g, PartId pid, const Transformd &T_part) {
		g.parts().visit(pid, [&](const auto &pw) {
			for (const auto &iface : pw.wrapped().interfaces()) {
				bins_.insert({pid, iface, T_part});
			}
		});
	}

	template <class G>
	void add_component(const G &g, PartId root,
	                   const Transformd &T_root = SE3d{}.identity()) {
		for (auto [pid, T_root_part] : g.component_view(root).transforms()) {
			add_part(g, pid, T_root * T_root_part);
		}
	}

	void clear() {
		bins_.clear();
	}

	std::size_t size() const {
		return bins_.size();
	}

	void
	for_each_induced(std::invocable<CandidateConnection> auto consumer) const {
		bins_.for_each_overlap([&](const auto &pair) {
			if (auto candidate = check_assembly(pair.patch_u, pair.patch_v)) {
				std::invoke(consumer, *candidate);
			}
		});
	}

	void for_each_induced_between(
	    const InterfaceBinMap &other, const Transformd &T_this_other,
	    std::invocable<CandidateConnection> auto consumer) const {
		bins_.for_each_overlap_between(
		    other.bins_, T_this_other, [&](const auto &pair) {
			    InterfacePatch patch_v_in_this =
			        pair.patch_v.transformed(T_this_other);
			    if (auto candidate =
			            check_assembly(pair.patch_u, patch_v_in_this)) {
				    std::invoke(consumer, *candidate);
			    }
		    });
	}

  private:
	using PatchBinMap = PlanarPatchBinMap<InterfacePatch>;
	using PatchIdx = typename PatchBinMap::PatchIdx;

	static std::optional<CandidateConnection>
	check_assembly(const InterfacePatch &patch_a,
	               const InterfacePatch &patch_b) {
		// Interfaces from the same part
		if (patch_a.part_id() == patch_b.part_id()) {
			return std::nullopt;
		}
		// Same type interfaces
		if (patch_a.interface_type() == patch_b.interface_type()) {
			return std::nullopt;
		}
		// Normalize stud vs hole order
		const InterfacePatch *stud_patch = &patch_a;
		const InterfacePatch *hole_patch = &patch_b;
		if (patch_a.interface_type() == InterfaceType::Hole) {
			stud_patch = &patch_b;
			hole_patch = &patch_a;
		}
		const InterfaceSpec &stud_iface = stud_patch->interface_spec();
		const InterfaceSpec &hole_iface = hole_patch->interface_spec();

		Transformd T_stud_hole = inverse(stud_patch->interface_transform()) *
		                         hole_patch->interface_transform();
		const auto &[q_rel, t_rel] = T_stud_hole;
		Eigen::Vector3d x_hat = q_rel * Eigen::Vector3d::UnitX();
		double rel_yaw = std::atan2(x_hat.y(), x_hat.x());
		double yaw_err;
		YawC4 yaw = nearest_c4(rel_yaw, yaw_err);

		Eigen::Vector2d p0 = t_rel.head<2>() / BrickUnitLength;
		Eigen::Vector2i p0_snap = p0.array().round().cast<int>();
		// To compute position error:
		// double p_err = (p0 - p0_snap.cast<double>()).norm() * BrickUnitLength;

		ConnectionSegment conn_seg{.offset = p0_snap, .yaw = yaw};
		ConnectionOverlap ov = conn_seg.compute_overlap(stud_iface, hole_iface);
		if (!ov.is_valid()) {
			return std::nullopt;
		}

		Transformd snapped_transform =
		    SE3d{}.project(conn_seg.compute_interface_transform());
		Transformd exact_transform = SE3d{}.project(T_stud_hole);
		if (!SE3d{}.almost_equal(exact_transform, snapped_transform)) {
			return std::nullopt;
		}

		return CandidateConnection{
		    .iface_stud = stud_patch->interface_ref(),
		    .iface_hole = hole_patch->interface_ref(),
		    .conn_seg = conn_seg,
		    .overlap = ov,
		};
	}

	PatchBinMap bins_;
};

} // namespace bricksim
