export module lego_assemble.core.component_index;

import std;
import lego_assemble.core.graph;
import lego_assemble.utils.type_list;
import lego_assemble.utils.multi_key_map;

namespace lego_assemble {

export using ComponentId = std::uint64_t;

export template <class G, class PKs, class Data = void> class ComponentIndex;

export template <class G, class... PKs, class Data>
class ComponentIndex<G, type_list<PKs...>, Data> {
	static_assert(in_pack<PartId, PKs...>,
	              "ComponentIndex: PartId must be one of the key types");
	static_assert((G::PartKeys::template contains<PKs> && ...),
	              "ComponentIndex: All PKs must be part keys of G");

  public:
	static constexpr bool HasData = !std::is_void_v<Data>;
	using TopologyGraph = G;
	using PartKeys = type_list<PKs...>;
	using IdMapping = MultiKeyMap<PartKeys, ComponentId>;
	using SizeMapping = std::unordered_map<ComponentId, std::size_t>;
	using DataMapping =
	    std::conditional_t<HasData, std::unordered_map<ComponentId, Data>,
	                       std::monostate>;

	explicit ComponentIndex(const TopologyGraph &g) : g_{g} {}

	void mark_dirty_topological(PartId pid) {
		dirty_topological_.insert(pid);
	}

	void mark_dirty_data_by_cc(ComponentId cid)
	    requires(HasData)
	{
		dirty_data_.insert(cid);
	}

	void mark_dirty_data_by_part(PartId pid)
	    requires(HasData)
	{
		ComponentId *cid_ptr = ids_.find_value(pid);
		if (cid_ptr == nullptr) {
			// Part not committed yet
			// It will be updated (created) when committed
			return;
		}
		dirty_data_.insert(*cid_ptr);
	}

	void mark_removed(PartId pid) {
		removed_.insert(pid);
		dirty_topological_.erase(pid);
	}

	void commit(std::invocable<ComponentId, PartId> auto &&data_creator,
	            std::invocable<ComponentId, Data &> auto &&data_updater)
	    requires(HasData)
	{
		commit_removed_();
		commit_topologically_changed_(
		    std::forward<decltype(data_creator)>(data_creator));
		commit_data_changed_(
		    std::forward<decltype(data_updater)>(data_updater));
	}

	void commit()
	    requires(!HasData)
	{
		commit_removed_();
		commit_topologically_changed_([](ComponentId, PartId) {});
	}

	const IdMapping &ids() const {
		return ids_;
	}

	const SizeMapping &cc_sizes() const {
		return cc_sizes_;
	}

	const DataMapping &data() const
	    requires(HasData)
	{
		return data_;
	}

  private:
	using DirtyDataSet =
	    std::conditional_t<HasData, std::unordered_set<ComponentId>,
	                       std::monostate>;

	const TopologyGraph &g_;
	IdMapping ids_;
	SizeMapping cc_sizes_;
	ComponentId next_component_id_ = 1;
	std::unordered_set<PartId> removed_;
	std::unordered_set<PartId> dirty_topological_;
	[[no_unique_address]] DataMapping data_;
	[[no_unique_address]] DirtyDataSet dirty_data_;

	void decrease_cc_size_(ComponentId cid) {
		std::size_t &size = cc_sizes_.at(cid);
		if (--size == 0) {
			// Component is gone
			cc_sizes_.erase(cid);
			if constexpr (HasData) {
				data_.erase(cid);
				dirty_data_.erase(cid);
			}
		}
	}

	void commit_removed_() {
		// Remove deleted vertices from mapping
		for (PartId u : removed_) {
			// Erase from mapping if exists
			// It exists if the it was commited before
			// It does not exist if adding & removing are in the same commit
			ComponentId *mapping_u = ids_.find_value(u);
			if (mapping_u != nullptr) {
				ComponentId id = *mapping_u;
				decrease_cc_size_(id);
				ids_.erase(u);
			}
		}
		removed_.clear();
	}

	void commit_topologically_changed_(auto &&data_creator) {
		// Recompute component ids for new / modified vertices
		auto &q = dirty_topological_; // alias
		while (!q.empty()) {
			PartId u = *q.begin();
			q.erase(u);
			if (!g_.parts().contains(u)) {
				throw std::runtime_error(
				    std::format("Part id {} not found when recomputing CC", u));
			}
			PartId representative = u;
			ComponentId new_id = next_component_id_++;
			cc_sizes_.emplace(new_id, 0);
			std::size_t &new_size = cc_sizes_.at(new_id);
			for (PartId v : g_.component_view(u).vertices()) {
				q.erase(v);
				if (v < representative) {
					representative = v;
				}
				ComponentId *mapping_v = ids_.find_value(v);
				if (mapping_v) {
					// Update existing
					ComponentId old_id = *mapping_v;
					decrease_cc_size_(old_id);
					++new_size;
					*mapping_v = new_id;
				} else {
					// Insert new
					auto entry_v = g_.parts().entry_of(v);
					bool inserted =
					    ids_.emplace(entry_v.template key<PKs>()..., new_id);
					if (!inserted) {
						throw std::runtime_error(
						    std::format("Key conflict for part id {}", v));
					}
					++new_size;
				}
			}
			if constexpr (HasData) {
				data_.emplace(
				    new_id, std::invoke(data_creator, new_id, representative));
			}
		}
	}

	void commit_data_changed_(auto &&data_updater)
	    requires(HasData)
	{
		for (ComponentId cid : dirty_data_) {
			Data &d = data_.at(cid);
			std::invoke(data_updater, cid, d);
		}
		dirty_data_.clear();
	}
};

} // namespace lego_assemble
