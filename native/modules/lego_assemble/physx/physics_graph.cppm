export module lego_assemble.physx.physics_graph;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.core.assembly;
import lego_assemble.core.component_labeling;
import lego_assemble.physx.constraint_scheduler;
import lego_assemble.physx.weld_constraint;
import lego_assemble.physx.patcher;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;
import lego_assemble.utils.transforms;
import lego_assemble.utils.conversions;
import lego_assemble.utils.multi_key_map;
import lego_assemble.utils.pair;
import lego_assemble.utils.unordered_pair;
import lego_assemble.utils.metric_system;
import lego_assemble.vendor;

namespace lego_assemble {

struct PendingAssembly {
	ConnSegRef csref;
	ConnectionSegment conn_seg;
};
struct PendingDisassembly {
	ConnSegId csid;
};

export using InterfaceShapePair = std::pair<InterfaceId, physx::PxShape *>;
export using ActorShapePair =
    std::pair<physx::PxRigidActor *, physx::PxShape *>;
export using ActorShapePairHash =
    PairHash<physx::PxRigidActor *, std::hash<physx::PxRigidActor *>,
             physx::PxShape *, std::hash<physx::PxShape *>>;
export constexpr ActorShapePair NullActorShapePair{nullptr, nullptr};

export template <PartLike P> struct PhysicsPartWrapper : SimplePartWrapper<P> {
	template <class... Args>
	explicit PhysicsPartWrapper(
	    std::vector<InterfaceShapePair> interface_shapes, Args &&...args)
	    : SimplePartWrapper<P>(std::forward<Args>(args)...),
	      interface_shapes_{std::move(interface_shapes)} {}

	std::span<const InterfaceShapePair> interface_shapes() const {
		return interface_shapes_;
	}

  private:
	std::vector<InterfaceShapePair> interface_shapes_;
};

export struct PhysicsConnectionSegmentWrapper
    : SimpleWrapper<ConnectionSegment> {
	ActorShapePair px_stud_shape{nullptr, nullptr};
	ActorShapePair px_hole_shape{nullptr, nullptr};
	template <class... Args>
	explicit PhysicsConnectionSegmentWrapper(Args &&...args)
	    : SimpleWrapper<ConnectionSegment>(std::forward<Args>(args)...) {}
};

export using PhysicsConnectionBundleWrapper = SimpleWrapper<ConnectionBundle>;

export enum class ContactExclusionLevel {
	Shape,
	Actor,
	ConnectedComponent,
};

using ContactExclusionPair = UnorderedPair<ActorShapePair>;
using ContactExclusionPairHash =
    typename ContactExclusionPair::Hasher<ActorShapePairHash>;
using ContactExclusionSet =
    std::unordered_set<ContactExclusionPair, ContactExclusionPairHash>;
using ShapeMapping =
    MultiKeyMap<type_list<InterfaceRef, ActorShapePair>, InterfaceSpec,
                type_list<InterfaceRefHash, ActorShapePairHash>>;

physx::PxRigidActor *cast_rigid_actor(physx::PxActor *actor) {
	auto type = actor->getType();
	if (type == physx::PxActorType::eRIGID_DYNAMIC ||
	    type == physx::PxActorType::eRIGID_STATIC) {
		return static_cast<physx::PxRigidActor *>(actor);
	} else {
		return nullptr;
	}
}

export struct PhysicsAssemblyDebugInfo : AssemblyDebugInfo {
	ConnSegRef csref;
};

export template <class Ps, class Hooks> class PhysicsLegoGraph;

export template <PartLike... Ps, class Hooks>
class PhysicsLegoGraph<type_list<Ps...>, Hooks> {
  private:
	class TopologyHooks;

  public:
	using Self = PhysicsLegoGraph<type_list<Ps...>, Hooks>;

	using TopologyGraph = LegoGraph<
	    type_list<Ps...>, PhysicsPartWrapper, type_list<physx::PxRigidActor *>,
	    type_list<std::hash<physx::PxRigidActor *>>, type_list<std::equal_to<>>,
	    PhysicsConnectionSegmentWrapper, type_list<>, type_list<>, type_list<>,
	    PhysicsConnectionBundleWrapper, TopologyHooks>;
	using ConstraintSchedulingPolicy =
	    CombinedSchedulingPolicy<TopologyGraph, TreeOnlySchedulingPolicy,
	                             RamanujanLikeSchedulingPolicy>;
	// using ConstraintSchedulingPolicy = CombinedSchedulingPolicy<TopologyGraph, FullGraphSchedulingPolicy, RamanujanLikeSchedulingPolicy>;
	// using ConstraintSchedulingPolicy = RamanujanLikeSchedulingPolicy<TopologyGraph>;
	// using ConstraintSchedulingPolicy = CombinedSchedulingPolicy<TopologyGraph, FullGraphSchedulingPolicy, ExponentialSkipSchedulingPolicy>;
	// using ConstraintSchedulingPolicy = FullGraphSchedulingPolicy<TopologyGraph>;
	// using ConstraintSchedulingPolicy = ExponentialSkipSchedulingPolicy<TopologyGraph>;
	// using ConstraintSchedulingPolicy = TreeOnlySchedulingPolicy<TopologyGraph>;
	using PhysicsConstraintScheduler =
	    ConstraintScheduler<TopologyGraph, ConstraintSchedulingPolicy,
	                        physx::PxConstraint *,
	                        std::function<physx::PxConstraint *(
	                            PartId, PartId, const Transformd &)>,
	                        std::function<void(physx::PxConstraint *)>>;
	using PhysicsComponentLabeling =
	    ComponentLabeling<type_list<PartId, physx::PxRigidActor *>,
	                      TopologyGraph>;
	using ShapeMapping =
	    MultiKeyMap<type_list<InterfaceRef, ActorShapePair>, InterfaceSpec,
	                type_list<InterfaceRefHash, ActorShapePairHash>>;

	static constexpr bool HasOnAssembledHook =
	    requires(Hooks &hooks, ConnSegId csid, const ConnSegRef &csref,
	             const ConnectionSegment &conn_seg) {
		    {
			    // Called after a new connection segment is assembled
			    hooks.on_assembled(csid, csref, conn_seg)
		    } -> std::same_as<void>;
	    };

	static constexpr bool HasOnDisassembledHook =
	    requires(Hooks &hooks, ConnSegId csid) {
		    {
			    // Called after a connection segment is disassembled
			    hooks.on_disassembled(csid)
		    } -> std::same_as<void>;
	    };

  private:
	// Aggregates filtering reset requests
	class FilteringResetAggregator {
	  private:
		// Reference to owner's fields
		const ContactExclusionLevel &contact_exclusion_level_;
		const TopologyGraph &topology_;

		// Requested resets
		std::unordered_set<PartId> reqs_;

		void commit_as_individual_() {
			for (PartId pid : reqs_) {
				physx::PxRigidActor *const *actor_ptr =
				    topology_.parts().template find_key<physx::PxRigidActor *>(
				        pid);
				if (!actor_ptr) {
					throw std::runtime_error(
					    std::format("Part id {} not found in topology during "
					                "filtering reset commit",
					                pid));
				}
				physx::PxRigidActor *actor = *actor_ptr;
				actor->getScene()->resetFiltering(*actor);
			}
			reqs_.clear();
		}

		void commit_as_cc_() {
			while (!reqs_.empty()) {
				PartId pid_u = *reqs_.begin();
				reqs_.erase(pid_u);
				if (!topology_.parts().contains(pid_u)) {
					throw std::runtime_error(
					    std::format("Part id {} not found in topology during "
					                "filtering reset commit",
					                pid_u));
				}
				for (PartId pid_v :
				     topology_.component_view(pid_u).vertices()) {
					reqs_.erase(pid_v);
					physx::PxRigidActor *actor =
					    topology_.parts()
					        .template key_of<physx::PxRigidActor *>(pid_v);
					actor->getScene()->resetFiltering(*actor);
				}
			}
		}

	  public:
		explicit FilteringResetAggregator(Self *owner)
		    : contact_exclusion_level_{owner->contact_exclusion_level_},
		      topology_{owner->topology_} {}

		void request_reset(PartId pid) {
			reqs_.insert(pid);
		}

		void mark_deleted(PartId pid) {
			reqs_.erase(pid);
		}

		void commit() {
			if (contact_exclusion_level_ ==
			    ContactExclusionLevel::ConnectedComponent) {
				// We need to reset all parts in the affected connected components
				commit_as_cc_();
			} else {
				// Otherwise, just reset the affected parts
				commit_as_individual_();
			}
		}
	};

	// Manages data needed for simulation
	// USD / Kit thread can read/write while holding exclusive lock
	// Physics threads can read while holding shared lock (const version)
	class SimInputData {
	  private:
		using G = TopologyGraph;

		// Reference to owner's fields
		// These are needed during updates
		const ContactExclusionLevel &contact_exclusion_level_;
		const TopologyGraph &topology_;

	  public:
		// ==== Should only be accessed with mutex held ====
		mutable std::shared_mutex mutex;
		// Data structures protected by mutex:
		ShapeMapping shape_mapping;
		ContactExclusionSet contact_exclusions;

		explicit SimInputData(Self *owner)
		    : contact_exclusion_level_{owner->contact_exclusion_level_},
		      topology_{owner->topology_} {}

		// ==== Update methods (called from USD hooks) ====

		void add_part(G::PartEntry entry) {
			PartId pid = entry.template key<PartId>();
			auto px_actor = entry.template key<physx::PxRigidActor *>();
			std::unique_lock lock{mutex};

			// Update shape mapping
			entry.visit([&](auto &pw) {
				for (const auto &[if_id, shape] : pw.interface_shapes()) {
					if (shape == nullptr) {
						throw std::runtime_error(std::format(
						    "Interface {} of part id {} has null PxShape",
						    if_id, pid));
					}
					auto iface_opt = pw.wrapped().get_interface(if_id);
					if (!iface_opt) {
						throw std::runtime_error(
						    std::format("Interface {} of part id {} not found",
						                if_id, pid));
					}
					if (!shape_mapping.emplace(InterfaceRef{pid, if_id},
					                           ActorShapePair{px_actor, shape},
					                           std::move(*iface_opt))) {
						throw std::runtime_error(
						    std::format("Shape mapping for interface {} of "
						                "part id {} already exists",
						                if_id, pid));
					}
				}
			});
		}

		void remove_part(G::PartEntry entry) {
			auto pid = entry.template key<PartId>();
			std::unique_lock lock{mutex};

			// Update shape mapping
			entry.visit([&](auto &pw) {
				for (const auto &[if_id, shape] : pw.interface_shapes()) {
					if (!shape_mapping.erase(InterfaceRef{pid, if_id})) {
						throw std::runtime_error(
						    std::format("Shape mapping for interface {} of "
						                "part id {} not found",
						                if_id, pid));
					}
				}
			});
		}

		void add_conn_seg(G::ConnSegEntry cs_entry) {
			// Update shape-level contact exclusion
			if (contact_exclusion_level_ != ContactExclusionLevel::Shape) {
				return;
			}

			const auto &[stud_ifref, hole_ifref] =
			    cs_entry.template key<ConnSegRef>();
			PhysicsConnectionSegmentWrapper &csw = cs_entry.value();

			std::unique_lock lock{mutex};

			// Set up PhysX shapes for connection segment
			if (auto stud_shape =
			        shape_mapping.template find_key<ActorShapePair>(
			            stud_ifref)) {
				csw.px_stud_shape = *stud_shape;
			}
			if (auto hole_shape =
			        shape_mapping.template find_key<ActorShapePair>(
			            hole_ifref)) {
				csw.px_hole_shape = *hole_shape;
			}

			// Add contact exclusion
			if (csw.px_stud_shape != NullActorShapePair &&
			    csw.px_hole_shape != NullActorShapePair) {
				auto [_, inserted] = contact_exclusions.emplace(
				    csw.px_stud_shape, csw.px_hole_shape);
				if (!inserted) {
					throw std::runtime_error(
					    std::format("Contact exclusion for connection segment "
					                "{} already exists",
					                cs_entry.template key<ConnSegId>()));
				}
			}
		}

		void remove_conn_seg(G::ConnSegEntry cs_entry) {
			// Update shape-level contact exclusion
			if (contact_exclusion_level_ != ContactExclusionLevel::Shape) {
				return;
			}

			PhysicsConnectionSegmentWrapper &csw = cs_entry.value();

			// Remove contact exclusion
			if (csw.px_stud_shape != NullActorShapePair &&
			    csw.px_hole_shape != NullActorShapePair) {
				std::unique_lock lock{mutex};
				if (!contact_exclusions.erase(
				        {csw.px_stud_shape, csw.px_hole_shape})) {
					throw std::runtime_error(std::format(
					    "Contact exclusion for connection segment {} not found",
					    cs_entry.template key<ConnSegId>()));
				}
			}
			csw.px_stud_shape = NullActorShapePair;
			csw.px_hole_shape = NullActorShapePair;
		}

		void add_conn_bundle(G::ConnBundleEntry cb_entry) {
			auto [pid_a, pid_b] = cb_entry.first;
			auto actor_a =
			    topology_.parts().template key_of<physx::PxRigidActor *>(pid_a);
			auto actor_b =
			    topology_.parts().template key_of<physx::PxRigidActor *>(pid_b);

			// Add contact exclusion if level is Actor
			if (contact_exclusion_level_ == ContactExclusionLevel::Actor) {
				std::unique_lock lock{mutex};
				auto [_, inserted] =
				    contact_exclusions.emplace(ContactExclusionPair{
				        {actor_a, nullptr}, {actor_b, nullptr}});
				if (!inserted) {
					throw std::runtime_error(
					    std::format("Contact exclusion for connection bundle "
					                "({}, {}) already exists",
					                pid_a, pid_b));
				}
			}
		}

		void remove_conn_bundle(G::ConnBundleEntry cb_entry) {
			auto [pid_a, pid_b] = cb_entry.first;
			auto actor_a =
			    topology_.parts().template key_of<physx::PxRigidActor *>(pid_a);
			auto actor_b =
			    topology_.parts().template key_of<physx::PxRigidActor *>(pid_b);

			// Remove contact exclusion if level is Actor
			if (contact_exclusion_level_ == ContactExclusionLevel::Actor) {
				std::unique_lock lock{mutex};
				if (!contact_exclusions.erase(ContactExclusionPair{
				        {actor_a, nullptr}, {actor_b, nullptr}})) {
					throw std::runtime_error(
					    std::format("Contact exclusion for connection bundle "
					                "({}, {}) not found",
					                pid_a, pid_b));
				}
			}
		}
	};

	class TopologyHooks {
	  private:
		using G = TopologyGraph;

		// Reference to owner's fields
		const ContactExclusionLevel &contact_exclusion_level_;
		PhysicsComponentLabeling &cc_labeling_;
		PhysicsConstraintScheduler &constraint_scheduler_;
		SimInputData &sim_in_;
		FilteringResetAggregator &filtering_reset_;

	  public:
		TopologyHooks(Self *owner)
		    : contact_exclusion_level_{owner->contact_exclusion_level_},
		      cc_labeling_{owner->cc_labeling_},
		      constraint_scheduler_{owner->constraint_scheduler_},
		      sim_in_{owner->sim_in_},
		      filtering_reset_{owner->filtering_reset_} {}

		~TopologyHooks() = default;
		TopologyHooks(const TopologyHooks &) = delete;
		TopologyHooks &operator=(const TopologyHooks &) = delete;
		TopologyHooks(TopologyHooks &&) = delete;
		TopologyHooks &operator=(TopologyHooks &&) = delete;

		void on_part_added(G::PartEntry entry) {
			PartId pid = entry.template key<PartId>();
			sim_in_.add_part(entry);
			cc_labeling_.mark_dirty(pid);
			constraint_scheduler_.notify_part_added(pid);
		}

		void on_part_removing(G::PartEntry entry) {
			// We need interface_shapes so we must call this before it's gone
			sim_in_.remove_part(entry);
		}

		void on_part_removed(PartId pid) {
			// Do this after removed so we schedule constraints on the updated graph
			constraint_scheduler_.notify_part_removed(pid);
			cc_labeling_.mark_removed(pid);
			filtering_reset_.mark_deleted(pid);
		}

		void on_connected(G::ConnSegEntry cs_entry,
		                  [[maybe_unused]] G::ConnBundleEntry cb_entry,
		                  [[maybe_unused]] const InterfaceSpec &stud_spec,
		                  [[maybe_unused]] const InterfaceSpec &hole_spec) {
			const auto &[stud_ifref, hole_ifref] =
			    cs_entry.template key<ConnSegRef>();
			const auto &[stud_pid, stud_ifid] = stud_ifref;

			sim_in_.add_conn_seg(cs_entry);

			// Reset filtering if in Shape level
			if (contact_exclusion_level_ == ContactExclusionLevel::Shape) {
				filtering_reset_.request_reset(stud_pid);
			}
		}

		void on_disconnecting(G::ConnSegEntry cs_entry,
		                      [[maybe_unused]] G::ConnBundleEntry cb_entry) {
			const auto &[stud_ifref, hole_ifref] =
			    cs_entry.template key<ConnSegRef>();
			const auto &[stud_pid, stud_ifid] = stud_ifref;

			sim_in_.remove_conn_seg(cs_entry);

			// Reset filtering if in Shape level
			if (contact_exclusion_level_ == ContactExclusionLevel::Shape) {
				filtering_reset_.request_reset(stud_pid);
			}
		}

		void on_bundle_created(G::ConnBundleEntry cb_entry) {
			auto [pid_a, pid_b] = cb_entry.first;

			sim_in_.add_conn_bundle(cb_entry);

			// One vertex is enough because both parts are in the same CC
			cc_labeling_.mark_dirty(pid_a);

			constraint_scheduler_.notify_connected(pid_a, pid_b);

			if (contact_exclusion_level_ == ContactExclusionLevel::Actor ||
			    contact_exclusion_level_ ==
			        ContactExclusionLevel::ConnectedComponent) {
				// Actor Level: Reset filtering for this pair of parts
				// ConnectedComponent Level: Reset filtering for all parts in the merged connected component
				filtering_reset_.request_reset(pid_a);
			}
		}

		void on_bundle_removing(G::ConnBundleEntry cb_entry) {
			sim_in_.remove_conn_bundle(cb_entry);

			if (contact_exclusion_level_ == ContactExclusionLevel::Actor ||
			    contact_exclusion_level_ ==
			        ContactExclusionLevel::ConnectedComponent) {
				// Actor Level: Reset filtering for this pair of parts
				// ConnectedComponent Level: Reset filtering for all parts in one of the connected components
				auto [pid_a, pid_b] = cb_entry.first;
				filtering_reset_.request_reset(pid_a);
			}
		}

		void on_bundle_removed(const ConnectionEndpoint &ep) {
			auto [pid_a, pid_b] = ep;

			// Mark both parts as dirty
			cc_labeling_.mark_dirty(pid_a);
			cc_labeling_.mark_dirty(pid_b);

			// Do this after removed so the scheduler sees the updated graph
			constraint_scheduler_.notify_disconnected(pid_a, pid_b);
		}
	};

	// Manages data that simulation threads write to
	class SimOutputData {
	  private:
		mutable std::mutex mutex;
		std::vector<PendingAssembly> pending_assemblies_;
		std::vector<PendingDisassembly> pending_disassemblies_;
		std::vector<PhysicsAssemblyDebugInfo> assembly_debug_infos_cur_;
		std::vector<PhysicsAssemblyDebugInfo> assembly_debug_infos_prev_;

	  public:
		void enqueue_assembly(auto &&...args) {
			std::lock_guard lock{mutex};
			pending_assemblies_.emplace_back(
			    std::forward<decltype(args)>(args)...);
		}

		void enqueue_assembly_debug_info(auto &&...args) {
			std::lock_guard lock{mutex};
			assembly_debug_infos_cur_.emplace_back(
			    std::forward<decltype(args)>(args)...);
		}

		void enqueue_disassembly(auto &&...args) {
			std::lock_guard lock{mutex};
			pending_disassemblies_.emplace_back(
			    std::forward<decltype(args)>(args)...);
		}

		std::vector<PendingAssembly> consume_assemblies() {
			std::lock_guard lock{mutex};
			std::vector<PendingAssembly> res = std::move(pending_assemblies_);
			pending_assemblies_.clear();
			return res;
		}

		std::vector<PendingDisassembly> consume_disassemblies() {
			std::lock_guard lock{mutex};
			std::vector<PendingDisassembly> res =
			    std::move(pending_disassemblies_);
			pending_disassemblies_.clear();
			return res;
		}

		std::vector<PhysicsAssemblyDebugInfo> get_assembly_debug_infos() const {
			std::lock_guard lock{mutex};
			return assembly_debug_infos_prev_;
		}

		void swap_debug_info_buffers() {
			std::lock_guard lock{mutex};
			assembly_debug_infos_prev_ = std::move(assembly_debug_infos_cur_);
			assembly_debug_infos_cur_.clear();
		}
	};

	class PhysxBinding : private PxSimulationFilterPatch,
	                     private PxSimulationEventPatch {
	  public:
		explicit PhysxBinding(Self *owner, physx::PxScene *px_scene)
		    : PxSimulationFilterPatch{px_scene},
		      PxSimulationEventPatch{px_scene}, metrics_{owner->metrics_},
		      cc_labeling_{owner->cc_labeling_}, sim_in_{owner->sim_in_},
		      assembly_checker_{owner->assembly_checker_},
		      contact_exclusion_level_{owner->contact_exclusion_level_},
		      collect_assembly_debug_info_{owner->collect_assembly_debug_info_},
		      sim_out_{owner->sim_out_}, px_scene_{px_scene} {}
		~PhysxBinding() {}
		PhysxBinding(const PhysxBinding &) = delete;
		PhysxBinding &operator=(const PhysxBinding &) = delete;
		PhysxBinding(PhysxBinding &&) = delete;
		PhysxBinding &operator=(PhysxBinding &&) = delete;

		// Thread safety:
		// PxSimulationFilterCallbackProxy (pairFound, ...) is called on PhysX/worker threads (multiple)
		// PxSimulationEventCallbackProxy (onContact, ...) is called on PhysX/stepper thread (single)

		virtual physx::PxFilterFlags pairFound(
		    physx::PxU64 pairID, physx::PxFilterObjectAttributes attributes0,
		    physx::PxFilterData filterData0, const physx::PxActor *ca0,
		    const physx::PxShape *cs0,
		    physx::PxFilterObjectAttributes attributes1,
		    physx::PxFilterData filterData1, const physx::PxActor *ca1,
		    const physx::PxShape *cs1, physx::PxPairFlags &pairFlags) override {

			physx::PxFilterFlags result = PxSimulationFilterPatch::pairFound(
			    pairID, attributes0, filterData0, ca0, cs0, attributes1,
			    filterData1, ca1, cs1, pairFlags);

			// Only if both actors are rigid actors
			auto *rb0 = cast_rigid_actor(const_cast<physx::PxActor *>(ca0));
			if (rb0 == nullptr) {
				return result;
			}
			auto *rb1 = cast_rigid_actor(const_cast<physx::PxActor *>(ca1));
			if (rb1 == nullptr) {
				return result;
			}
			auto *s0 = const_cast<physx::PxShape *>(cs0);
			auto *s1 = const_cast<physx::PxShape *>(cs1);

			// Thread safety: reading sim data structures
			std::shared_lock lock{sim_in_.mutex};

			if (contact_exclusion_level_ == ContactExclusionLevel::Shape) {
				if (sim_in_.contact_exclusions.contains(
				        ContactExclusionPair{{rb0, s0}, {rb1, s1}})) {
					return physx::PxFilterFlag::eKILL;
				}
			} else if (contact_exclusion_level_ ==
			           ContactExclusionLevel::Actor) {
				if (sim_in_.contact_exclusions.contains(
				        ContactExclusionPair{{rb0, nullptr}, {rb1, nullptr}})) {
					return physx::PxFilterFlag::eKILL;
				}
			}
			// For ConnectedComponent level, we will check later

			const auto &cc_mapping = cc_labeling_.mapping();
			const ComponentId *cc0 = cc_mapping.find_value(rb0);
			const ComponentId *cc1 = cc_mapping.find_value(rb1);
			if (cc0 == nullptr || cc1 == nullptr) {
				// Not part-part contact
				return result;
			}

			if (contact_exclusion_level_ ==
			    ContactExclusionLevel::ConnectedComponent) {
				if (*cc0 == *cc1) {
					return physx::PxFilterFlag::eKILL;
				}
			}

			// Enable contact pose report
			pairFlags |= physx::PxPairFlag::eCONTACT_EVENT_POSE;

			return result;
		}

		virtual void onContact(const physx::PxContactPairHeader &header,
		                       const physx::PxContactPair *pairs,
		                       physx::PxU32 nbPairs) override {

			PxSimulationEventPatch::onContact(header, pairs, nbPairs);

			// Ignore removed actors
			if (header.flags.isSet(
			        physx::PxContactPairHeaderFlag::eREMOVED_ACTOR_0) ||
			    header.flags.isSet(
			        physx::PxContactPairHeaderFlag::eREMOVED_ACTOR_1)) {
				return;
			}

			// Only if both actors are rigid actors
			physx::PxRigidActor *rb0 = cast_rigid_actor(header.actors[0]);
			physx::PxRigidActor *rb1 = cast_rigid_actor(header.actors[1]);
			if (rb0 == nullptr || rb1 == nullptr) {
				return;
			}

			// Thread safety: reading sim data structures
			std::shared_lock lock{sim_in_.mutex};

			physx::PxReal dt = getPxSceneElapsedTime(px_scene_);

			// Iterate over contact pairs
			physx::PxContactPairExtraDataIterator extra_it(
			    header.extraDataStream, header.extraDataStreamSize);
			const physx::PxContactPairPose *event_pose = nullptr;
			bool extra_has_next = extra_it.nextItemSet();
			for (physx::PxU32 i = 0; i < nbPairs; i++) {
				while (extra_has_next && extra_it.contactPairIndex <= i) {
					event_pose = extra_it.eventPose;
					extra_has_next = extra_it.nextItemSet();
				}
				if (event_pose == nullptr) {
					continue;
				}
				const auto &pair = pairs[i];
				if (!pair.flags.isSet(
				        physx::PxContactPairFlag::eINTERNAL_HAS_IMPULSES)) {
					continue;
				}
				if (!pair.contactPatches) {
					continue;
				}
				const auto &[shape0, shape1] = pair.shapes;

				const InterfaceSpec *if0 = sim_in_.shape_mapping.find_value(
				    ActorShapePair{rb0, shape0});
				const InterfaceSpec *if1 = sim_in_.shape_mapping.find_value(
				    ActorShapePair{rb1, shape1});
				if (!if0 || !if1) {
					continue;
				}

				// rb0 should offer stud, rb1 should offer hole
				bool to_swap;
				if (if0->type == InterfaceType::Stud &&
				    if1->type == InterfaceType::Hole) {
					to_swap = false;
				} else if (if0->type == InterfaceType::Hole &&
				           if1->type == InterfaceType::Stud) {
					to_swap = true;
				} else {
					continue;
				}

				// Sum up contact impulses
				physx::PxVec3 total_impulse(0, 0, 0);
				const auto *patches =
				    reinterpret_cast<const physx::PxContactPatch *>(
				        pair.contactPatches);
				for (physx::PxU32 j = 0; j < pair.patchCount; j++) {
					const auto &patch = patches[j];
					physx::PxReal patch_impulse = 0;
					for (physx::PxU32 k = 0; k < patch.nbContacts; k++) {
						patch_impulse +=
						    pair.contactImpulses[patch.startContactIndex + k];
					}
					total_impulse += patch.normal * patch_impulse;
				}

				const auto &[pose0, pose1] = event_pose->globalPose;

				if (to_swap) {
					process_assembly_contact(rb1, shape1, *if1, pose1, rb0,
					                         shape0, *if0, pose0,
					                         -total_impulse, dt);
				} else {
					process_assembly_contact(rb0, shape0, *if0, pose0, rb1,
					                         shape1, *if1, pose1, total_impulse,
					                         dt);
				}
			}
		}

	  private:
		const MetricSystem &metrics_;
		const PhysicsComponentLabeling &cc_labeling_;
		const SimInputData &sim_in_;
		const AssemblyChecker &assembly_checker_;
		const ContactExclusionLevel &contact_exclusion_level_;
		const bool &collect_assembly_debug_info_;
		SimOutputData &sim_out_;
		physx::PxScene *px_scene_;

		void process_assembly_contact(
		    physx::PxRigidActor *rb0, physx::PxShape *s0,
		    const InterfaceSpec &if0, const physx::PxTransform &pose0_px,
		    physx::PxRigidActor *rb1, physx::PxShape *s1,
		    const InterfaceSpec &if1, const physx::PxTransform &pose1_px,
		    const physx::PxVec3 &impulse_px, physx::PxReal dt) {

			// Metric conversion
			Transformd pose0 = metrics_.to_m(as<Transformd>(pose0_px));
			Transformd pose1 = metrics_.to_m(as<Transformd>(pose1_px));
			Eigen::Vector3d impulse =
			    metrics_.to_Ns(as<Eigen::Vector3d>(impulse_px));
			Eigen::Vector3d force = impulse / static_cast<double>(dt);

			PhysicsAssemblyDebugInfo debug_info;
			PhysicsAssemblyDebugInfo *debug_info_ptr;
			if (collect_assembly_debug_info_) {
				debug_info_ptr = &debug_info;
			} else {
				debug_info_ptr = nullptr;
			}

			std::optional<ConnectionSegment> result =
			    assembly_checker_.detect_assembly(if0, if1, pose0, pose1, force,
			                                      debug_info_ptr);

			if (collect_assembly_debug_info_) {
				const InterfaceRef *ifref0 =
				    sim_in_.shape_mapping.template find_key<InterfaceRef>(
				        ActorShapePair{rb0, s0});
				const InterfaceRef *ifref1 =
				    sim_in_.shape_mapping.template find_key<InterfaceRef>(
				        ActorShapePair{rb1, s1});
				if (ifref0 && ifref1) {
					debug_info.csref = {*ifref0, *ifref1};
					sim_out_.enqueue_assembly_debug_info(debug_info);
				}
			}

			if (!result.has_value()) {
				return;
			}

			// Thread safety: read lock already acquired in caller

			const InterfaceRef &ifref0 =
			    sim_in_.shape_mapping.template key_of<InterfaceRef>(
			        ActorShapePair{rb0, s0});
			const InterfaceRef &ifref1 =
			    sim_in_.shape_mapping.template key_of<InterfaceRef>(
			        ActorShapePair{rb1, s1});

			sim_out_.enqueue_assembly(ConnSegRef{ifref0, ifref1}, *result);
		}
	};

  public:
	explicit PhysicsLegoGraph(const MetricSystem &metrics, physx::PxPhysics *px,
	                          Hooks *hooks = nullptr,
	                          AssemblyThresholds thresholds = {},
	                          ContactExclusionLevel contact_exclusion_level =
	                              ContactExclusionLevel::ConnectedComponent,
	                          bool collect_assembly_debug_info = true)
	    : metrics_{metrics}, px_{px}, hooks_{hooks}, topology_hooks_{this},
	      topology_{&topology_hooks_},
	      constraint_scheduler_{
	          &topology_, ConstraintSchedulingPolicy{},
	          std::bind_front(&PhysicsLegoGraph::create_constraint, this),
	          std::bind_front(&PhysicsLegoGraph::release_constraint, this)},
	      assembly_checker_{thresholds},
	      contact_exclusion_level_{contact_exclusion_level},
	      collect_assembly_debug_info_{collect_assembly_debug_info},
	      cc_labeling_{topology_}, filtering_reset_{this}, sim_in_{this} {}
	~PhysicsLegoGraph() {
		unbind_physx_scene();

		// Release constraints
		constraint_scheduler_.clear();
	}
	PhysicsLegoGraph(const PhysicsLegoGraph &) = delete;
	PhysicsLegoGraph &operator=(const PhysicsLegoGraph &) = delete;
	PhysicsLegoGraph(PhysicsLegoGraph &&) = delete;
	PhysicsLegoGraph &operator=(PhysicsLegoGraph &&) = delete;

	TopologyGraph &topology() {
		return topology_;
	}

	const TopologyGraph &topology() const {
		return topology_;
	}

	AssemblyChecker &assembly_checker() {
		return assembly_checker_;
	}

	const AssemblyChecker &assembly_checker() const {
		return assembly_checker_;
	}

	std::vector<PhysicsAssemblyDebugInfo> get_assembly_debug_infos() const {
		return sim_out_.get_assembly_debug_infos();
	}

	// Should be called BEFORE simulation step on USD/Kit thread
	// Use a StageUpdateNode with order < 10 to implement this
	void do_pre_step() {
		flush_physx_ops();
	}

	// Should be called AFTER simulation step on USD/Kit thread
	// Use a StageUpdateNode with order > 10 to implement this
	void do_post_step() {
		flush_physx_ops();

		auto pending_assemblies = sim_out_.consume_assemblies();
		auto pending_disassemblies = sim_out_.consume_disassemblies();
		sim_out_.swap_debug_info_buffers();

		for (const auto &[csid] : pending_disassemblies) {
			bool disconnected = topology_.disconnect(csid).has_value();
			if (disconnected) {
				if constexpr (HasOnDisassembledHook) {
					if (hooks_) {
						hooks_->on_disassembled(csid);
					}
				}
			}
		}
		for (const auto &[csref, conn_seg] : pending_assemblies) {
			const auto &[stud_if, hole_if] = csref;
			std::optional<ConnSegId> csid =
			    topology_.connect(stud_if, hole_if, conn_seg);
			if (csid.has_value()) {
				if constexpr (HasOnAssembledHook) {
					if (hooks_) {
						hooks_->on_assembled(*csid, csref, conn_seg);
					}
				}
			}
		}
	}

	bool bind_physx_scene(physx::PxScene *px_scene) {
		if (physx_binding_) {
			return false;
		}
		physx_binding_ = std::make_unique<PhysxBinding>(this, px_scene);
		return true;
	}

	bool unbind_physx_scene() {
		if (!physx_binding_) {
			return false;
		}
		physx_binding_.reset();
		return true;
	}

	Hooks *get_hooks() noexcept {
		return hooks_;
	}

	void set_hooks(Hooks *hooks) noexcept {
		hooks_ = hooks;
	}

  private:
	MetricSystem metrics_;
	physx::PxPhysics *px_;
	Hooks *hooks_;
	TopologyHooks topology_hooks_;
	TopologyGraph topology_;
	PhysicsConstraintScheduler constraint_scheduler_;
	std::unique_ptr<PhysxBinding> physx_binding_;
	AssemblyChecker assembly_checker_;
	ContactExclusionLevel contact_exclusion_level_;
	bool collect_assembly_debug_info_;

	PhysicsComponentLabeling cc_labeling_;
	FilteringResetAggregator filtering_reset_;
	SimInputData sim_in_;
	SimOutputData sim_out_;

	void flush_physx_ops() {
		cc_labeling_.commit();
		constraint_scheduler_.commit();
		filtering_reset_.commit();
	}

	physx::PxConstraint *create_constraint(PartId a_id, PartId b_id,
	                                       const Transformd &T_a_b) {
		// PxConstraint shader uses body frames (COM frames) bA2w/bB2w.
		// Our T_a_b (from Python) is defined between actor-local origins (bottom centers).
		// Convert to COM-local frames so the weld aligns the intended anchor points.
		//
		// parentLocal (A_com -> B_com) = (A_com -> A_orig) * (A_orig -> B_orig) * (B_orig -> B_com)
		// childLocal is identity so cB2w = bB2w (B_com).
		physx::PxRigidActor *actor_a =
		    topology_.parts().template key_of<physx::PxRigidActor *>(a_id);
		physx::PxRigidActor *actor_b =
		    topology_.parts().template key_of<physx::PxRigidActor *>(b_id);

		auto get_com = [this](physx::PxRigidActor *actor) {
			if (auto *rb = actor->is<physx::PxRigidBody>()) {
				return metrics_.to_m(as<Transformd>(rb->getCMassLocalPose()));
			} else {
				return SE3d{}.identity();
			}
		};
		Transformd T_a_acom = get_com(actor_a);
		Transformd T_b_bcom = get_com(actor_b);
		Transformd T_acom_a = inverse(T_a_acom);
		Transformd parent_local = T_acom_a * T_a_b * T_b_bcom;
		physx::PxTransform parent_local_px =
		    as<physx::PxTransform>(metrics_.from_m(parent_local));
		return createWeldConstraint(
		    *px_, actor_a, actor_b,
		    WeldConstraintData{
		        .parentLocal = parent_local_px,
		        .childLocal = physx::PxTransform{physx::PxIdentity},
		    });
	}

	void release_constraint(physx::PxConstraint *constraint) {
		constraint->release();
	}

	static_assert(TopologyGraph::HasOnPartAddedHook);
	static_assert(TopologyGraph::HasOnPartRemovingHook);
	static_assert(TopologyGraph::HasOnPartRemovedHook);
	static_assert(TopologyGraph::HasOnConnectedHook);
	static_assert(TopologyGraph::HasOnBundleCreatedHook);
	static_assert(TopologyGraph::HasOnBundleRemovingHook);
	static_assert(TopologyGraph::HasOnBundleRemovedHook);
};

} // namespace lego_assemble
