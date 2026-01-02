export module lego_assemble.physx.filtering_reset;

import std;
import lego_assemble.core.graph;
import lego_assemble.vendor;

namespace lego_assemble {

export template <class G> class FilteringResetAggregator {
	static_assert(G::PartKeys::template contains<physx::PxRigidDynamic *>,
	              "FilteringResetAggregator: G must have physx::PxRigidDynamic "
	              "* as part key");

  private:
	const G &topology_;
	std::unordered_set<PartId> reqs_;

  public:
	explicit FilteringResetAggregator(const G &topology)
	    : topology_{topology} {}

	void mark_for_reset(PartId pid) {
		reqs_.insert(pid);
	}

	void mark_removed(PartId pid) {
		reqs_.erase(pid);
	}

	void commit() {
		auto &q = reqs_; // alias
		while (!q.empty()) {
			PartId u = *q.begin();
			q.erase(u);
			if (!topology_.parts().contains(u)) {
				throw std::runtime_error(
				    std::format("Part {} not found in topology during "
				                "filtering reset commit",
				                u));
			}
			for (PartId v : topology_.component_view(u).vertices()) {
				q.erase(v);
				auto actor =
				    topology_.parts().template key_of<physx::PxRigidDynamic *>(
				        v);
				actor->getScene()->resetFiltering(*actor);
			}
		}
	}
};

} // namespace lego_assemble
