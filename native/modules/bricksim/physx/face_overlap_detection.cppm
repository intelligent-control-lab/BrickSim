export module bricksim.physx.face_overlap_detection;

import std;
import bricksim.core.specs;
import bricksim.core.graph;
import bricksim.physx.overlap_detection;
import bricksim.utils.transforms;
import bricksim.utils.bbox;
import bricksim.vendor;

namespace bricksim {

export template <class G> class FacePatch {
  public:
	FacePatch(PartId pid, const Transformd &T_part,
	          FaceSpecLike auto &&face_spec)
	    : pid_(pid) {
		fid_ = face_spec.id();
		T_ = T_part * face_spec.transform();
		bbox_ = face_spec.bbox();
		for (const Eigen::Vector2d &v : face_spec.polygon_vertices()) {
			polygon_.push_back(v);
		}
	}

	PartId part_id() const {
		return pid_;
	}

	FaceId face_id() const {
		return fid_;
	}

	const Transformd &transform() const {
		return T_;
	}

	const BBox2d &bbox() const {
		return bbox_;
	}

	std::span<const Eigen::Vector2d> polygon_vertices() const {
		return polygon_;
	}

  private:
	PartId pid_;
	FaceId fid_;
	Transformd T_;
	BBox2d bbox_;
	std::vector<Eigen::Vector2d> polygon_;
};

export template <class G>
void for_each_overlapping_face_pairs(const G &g, PartId root, auto &&consumer) {
	PlanarPatchBinMap<FacePatch<G>> bins;
	for (auto [pid, T_root_part] : g.component_view(root).transforms()) {
		g.parts().visit(pid, [&](const auto &pw) {
			for (const auto &face : pw.wrapped().faces()) {
				bins.insert({pid, T_root_part, face});
			}
		});
	}
	bins.for_each_overlap([&](const auto &pair) {
		if (pair.patch_u.part_id() == pair.patch_v.part_id()) {
			// Skip face pairs from the same part
			return;
		}
		std::invoke(consumer, pair);
	});
}

} // namespace bricksim
