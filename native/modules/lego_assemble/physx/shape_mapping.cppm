export module lego_assemble.physx.shape_mapping;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.utils.pair;
import lego_assemble.utils.concepts;
import lego_assemble.utils.type_list;
import lego_assemble.utils.multi_key_map;
import lego_assemble.vendor;

namespace lego_assemble {

export using InterfaceShapePair = std::pair<InterfaceId, physx::PxShape *>;

export using ActorShapePair =
    std::pair<physx::PxRigidDynamic *, physx::PxShape *>;

export using ActorShapePairHash =
    PairHash<physx::PxRigidDynamic *, std::hash<physx::PxRigidDynamic *>,
             physx::PxShape *, std::hash<physx::PxShape *>>;

export constexpr ActorShapePair NullActorShapePair{nullptr, nullptr};

export template <class T>
concept PartWrapperWithShape = PartWrapperLike<T> && requires(const T &pw) {
	{ pw.interface_shapes() } -> range_of<const InterfaceShapePair>;
};

export using ShapeMap =
    MultiKeyMap<type_list<InterfaceRef, ActorShapePair>, InterfaceSpec,
                type_list<InterfaceRefHash, ActorShapePairHash>>;

export template <class G> class ShapeMapping {
	static_assert(
	    G::PartKeys::template contains<physx::PxRigidDynamic *>,
	    "ShapeMapping: G must have physx::PxRigidDynamic * as part key");

  public:
	using TopologyGraph = G;

	const ShapeMap &committed_view() const {
		return map_;
	}

	// Find in both committed and uncommitted changes
	std::optional<std::tuple<const ActorShapePair &, const InterfaceSpec &>>
	find(const InterfaceRef &ifref) const {
		auto it_to_add = to_add_.find(ifref);
		if (it_to_add != to_add_.end()) {
			const auto &[actor_shape, iface] = it_to_add->second;
			return {{actor_shape, iface}};
		}
		if (to_remove_.contains(ifref)) {
			return std::nullopt;
		}
		auto entry_committed = map_.find(ifref);
		if (!entry_committed) {
			return std::nullopt;
		}
		const auto &actor_shape =
		    entry_committed->template key<ActorShapePair>();
		const auto &iface = entry_committed->value();
		return {{actor_shape, iface}};
	}

	void add_part(typename G::PartEntry entry) {
		auto pid = entry.template key<PartId>();
		auto actor = entry.template key<physx::PxRigidDynamic *>();
		entry.visit([&](PartWrapperWithShape auto &pw) {
			for (const auto &[ifid, shape] : pw.interface_shapes()) {
				if (shape == nullptr) {
					throw std::runtime_error(std::format(
					    "Interface {} of part {} has null PxShape", ifid, pid));
				}
				auto iface_opt = pw.wrapped().get_interface(ifid);
				if (!iface_opt) {
					throw std::runtime_error(std::format(
					    "Interface {} of part {} not found", ifid, pid));
				}
				auto [_, inserted] = to_add_.emplace(
				    InterfaceRef{pid, ifid},
				    std::make_tuple(ActorShapePair{actor, shape},
				                    std::move(*iface_opt)));
				if (!inserted) {
					throw std::runtime_error(std::format(
					    "Shape for part {} interface {} already in add queue",
					    pid, ifid));
				}
			}
		});
	}

	void remove_part(typename G::PartEntry entry) {
		auto pid = entry.template key<PartId>();
		entry.visit([&](auto &pw) {
			for (const auto &[if_id, shape] : pw.interface_shapes()) {
				InterfaceRef ifref{pid, if_id};
				if (to_add_.erase(ifref)) {
					// Was going to add, but now removed before commit
				} else {
					// Already in map, so mark for removal
					auto [_, inserted] = to_remove_.emplace(ifref);
					if (!inserted) {
						throw std::runtime_error(
						    std::format("Shape for part {} interface {} "
						                "already in remove queue",
						                pid, if_id));
					}
				}
			}
		});
	}

	void commit() {
		for (const InterfaceRef &ifref : to_remove_) {
			const auto &[pid, ifid] = ifref;
			if (!map_.erase(ifref)) {
				throw std::runtime_error(
				    std::format("Shape mapping for interface ({}, {}) not "
				                "found when removing",
				                pid, ifid));
			}
		}
		to_remove_.clear();

		for (auto &&[ifref, payload] : to_add_) {
			const auto &[pid, ifid] = ifref;
			auto &&[actor_shape, iface] = payload;
			if (!map_.emplace(ifref, std::move(actor_shape),
			                  std::move(iface))) {
				throw std::runtime_error(
				    std::format("Shape mapping for interface ({}, {}) already "
				                "exists when adding",
				                pid, ifid));
			}
		}
		to_add_.clear();
	}

  private:
	ShapeMap map_;

	std::unordered_map<InterfaceRef, std::tuple<ActorShapePair, InterfaceSpec>,
	                   InterfaceRefHash>
	    to_add_;
	std::unordered_set<InterfaceRef, InterfaceRefHash> to_remove_;
};

} // namespace lego_assemble
