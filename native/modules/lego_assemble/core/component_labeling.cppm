export module lego_assemble.core.component_labeling;

import std;
import lego_assemble.core.graph;
import lego_assemble.utils.type_list;
import lego_assemble.utils.multi_key_map;

namespace lego_assemble {

export using ComponentId = std::uint64_t;

export template <class PKs, class G> class ComponentLabeling;

export template <class... PKs, class G>
class ComponentLabeling<type_list<PKs...>, G> {
	static_assert(in_pack<PartId, PKs...>,
	              "ComponentLabeling: PartId must be one of the key types");

  public:
	using PartKeys = type_list<PKs...>;
	using ComponentMapping = MultiKeyMap<PartKeys, ComponentId>;
	using TopologyGraph = G;

	explicit ComponentLabeling(const TopologyGraph &g) : g_{g} {}

	void mark_dirty(PartId pid) {
		dirty_.insert(pid);
	}

	void mark_removed(PartId pid) {
		removed_.insert(pid);
		dirty_.erase(pid);
	}

	void commit() {
		// 1. Remove deleted vertices from mapping
		for (PartId pid : removed_) {
			// Erase from mapping if exists
			// It exists if the it was commited before
			// It does not exist if adding & removing are in the same commit
			mapping_.erase(pid);
		}
		removed_.clear();

		// 2. Recompute component ids for new / modified vertices
		auto &q = dirty_; // alias
		while (!q.empty()) {
			PartId u = *q.begin();
			q.erase(u);
			if (!g_.parts().contains(u)) {
				throw std::runtime_error(
				    std::format("Part id {} not found when recomputing CC", u));
			}
			ComponentId cc_id = next_component_id_++;
			for (PartId v : g_.component_view(u).vertices()) {
				q.erase(v);
				ComponentId *mapping_v = mapping_.find_value(v);
				if (mapping_v) {
					// Update existing
					*mapping_v = cc_id;
				} else {
					// Insert new
					auto entry_v = g_.parts().entry_of(v);
					bool inserted =
					    mapping_.emplace(entry_v.template key<PKs>()..., cc_id);
					if (!inserted) {
						throw std::runtime_error(
						    std::format("Key conflict for part id {}", v));
					}
				}
			}
		}
	}

	const ComponentMapping &mapping() const {
		return mapping_;
	}

  private:
	const TopologyGraph &g_;
	ComponentMapping mapping_;
	ComponentId next_component_id_ = 1;
	std::unordered_set<PartId> removed_;
	std::unordered_set<PartId> dirty_;
};

} // namespace lego_assemble
