export module lego_assemble.physx.physics_graph;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.core.assembly;
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

using ConstraintHandle = std::size_t;

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

using ContactExclusionPair = UnorderedPair<ActorShapePair>;
using ContactExclusionPairHash =
    typename ContactExclusionPair::Hasher<ActorShapePairHash>;
using ContactExclusionSet =
    std::pmr::unordered_set<ContactExclusionPair, ContactExclusionPairHash>;

physx::PxRigidActor *cast_rigid_actor(physx::PxActor *actor) {
	auto type = actor->getType();
	if (type == physx::PxActorType::eRIGID_DYNAMIC ||
	    type == physx::PxActorType::eRIGID_STATIC) {
		return static_cast<physx::PxRigidActor *>(actor);
	} else {
		return nullptr;
	}
}

export template <class Ps, class Hooks> class PhysicsLegoGraph;

export template <PartLike... Ps, class Hooks>
class PhysicsLegoGraph<type_list<Ps...>, Hooks> {
  public:
	using Self = PhysicsLegoGraph<type_list<Ps...>, Hooks>;

  private:
	struct TopologyHooks {
		Self *owner_;

		TopologyHooks(Self *owner) : owner_{owner} {}
		~TopologyHooks() = default;
		TopologyHooks(const TopologyHooks &) = delete;
		TopologyHooks &operator=(const TopologyHooks &) = delete;
		TopologyHooks(TopologyHooks &&) = delete;
		TopologyHooks &operator=(TopologyHooks &&) = delete;

		template <class P>
		void on_part_added(PartId pid, PhysicsPartWrapper<P> &pw) {
			// Update shape <-> interface mapping
			physx::PxRigidActor *const *px_actor_ptr =
			    owner_->topology_.parts()
			        .template project_key<PartId, physx::PxRigidActor *>(pid);
			if (!px_actor_ptr) {
				throw std::runtime_error(std::format(
				    "Part id {} has no associated PxRigidActor", pid));
			}
			physx::PxRigidActor *px_actor = *px_actor_ptr;

			// Update constraint scheduler
			owner_->constraint_scheduler_.on_part_added(pid);

			// Thread safety: modifying shape_mapping_ and part_actors_
			std::unique_lock<std::shared_mutex> lock(owner_->sim_mutex_);

			auto [_, inserted] = owner_->part_actors_.emplace(px_actor);
			if (!inserted) {
				throw std::runtime_error(std::format(
				    "PxRigidActor {:p} for part id {} already registered",
				    static_cast<const void *>(px_actor), pid));
			}

			for (const auto &[if_id, shape] : pw.interface_shapes()) {
				if (shape == nullptr) {
					throw std::runtime_error(std::format(
					    "Interface {} of part id {} has null PxShape", if_id,
					    pid));
				}
				std::optional<InterfaceSpec> iface =
				    get_interface_at(pw.wrapped(), if_id);
				if (!iface) {
					throw std::runtime_error(std::format(
					    "Interface {} not found in part id {}", if_id, pid));
				}
				if (!owner_->shape_mapping_.emplace(
				        {{pid, if_id}, {px_actor, shape}}, *iface)) {
					throw std::runtime_error(std::format(
					    "Failed to map shape to interface {} of part id {}",
					    if_id, pid));
				}
			}
		}

		template <class P>
		void on_part_removing(PartId pid, PhysicsPartWrapper<P> &pw) {
			// Update shape <-> interface mapping

			physx::PxRigidActor *const *px_actor_ptr =
			    owner_->topology_.parts()
			        .template project_key<PartId, physx::PxRigidActor *>(pid);
			if (!px_actor_ptr) {
				throw std::runtime_error(std::format(
				    "Part id {} has no associated PxRigidActor", pid));
			}
			physx::PxRigidActor *px_actor = *px_actor_ptr;

			// Thread safety: modifying shape_mapping_ and part_actors_
			std::unique_lock<std::shared_mutex> lock(owner_->sim_mutex_);

			if (!owner_->part_actors_.erase(px_actor)) {
				throw std::runtime_error(std::format(
				    "PxRigidActor {:p} for part id {} not registered",
				    static_cast<const void *>(px_actor), pid));
			}

			for (const auto &[if_id, shape] : pw.interface_shapes()) {
				if (!owner_->shape_mapping_.template erase_by_key<InterfaceRef>(
				        {pid, if_id})) {
					throw std::runtime_error(std::format(
					    "Failed to unmap shape from interface {} of part id {}",
					    if_id, pid));
				}
			}
		}

		void on_part_removed(PartId pid) {
			// Update constraint scheduler
			owner_->constraint_scheduler_.on_part_removed(pid);
		}

		void
		on_connected(ConnSegId csid, const ConnSegRef &csref,
		             [[maybe_unused]] const InterfaceSpec &stud_spec,
		             [[maybe_unused]] const InterfaceSpec &hole_spec,
		             PhysicsConnectionSegmentWrapper &csw,
		             [[maybe_unused]] PhysicsConnectionBundleWrapper &cbw) {
			if (!owner_->shape_level_collision_filtering_) {
				return;
			}

			const auto &[stud_if_ref, hole_if_ref] = csref;

			{
				// Thread safety: modifying contact_exclusions_ and reading shape_mapping_
				std::unique_lock<std::shared_mutex> lock(owner_->sim_mutex_);

				// Set up PhysX shapes for connection segment
				if (auto stud_shape_mapping =
				        owner_->shape_mapping_
				            .template project<InterfaceRef, ActorShapePair>(
				                stud_if_ref)) {
					csw.px_stud_shape = *stud_shape_mapping;
				}
				if (auto hole_shape_mapping =
				        owner_->shape_mapping_
				            .template project<InterfaceRef, ActorShapePair>(
				                hole_if_ref)) {
					csw.px_hole_shape = *hole_shape_mapping;
				}

				// Add contact exclusion
				if (csw.px_stud_shape != NullActorShapePair &&
				    csw.px_hole_shape != NullActorShapePair) {
					auto [_, inserted] = owner_->contact_exclusions_.emplace(
					    csw.px_stud_shape, csw.px_hole_shape);
					if (!inserted) {
						throw std::runtime_error(
						    std::format("Failed to add contact exclusion for "
						                "connection segment {}",
						                csid));
					}
				}
			}

			const auto &[stud_pid, stud_ifid] = stud_if_ref;
			owner_->reset_filtering_safely(stud_pid);
		}

		void
		on_disconnecting(ConnSegId csid, const ConnSegRef &csref,
		                 PhysicsConnectionSegmentWrapper &csw,
		                 [[maybe_unused]] PhysicsConnectionBundleWrapper &cbw) {
			if (!owner_->shape_level_collision_filtering_) {
				return;
			}

			// Remove contact exclusion
			if (csw.px_stud_shape != NullActorShapePair &&
			    csw.px_hole_shape != NullActorShapePair) {
				// Thread safety: modifying contact_exclusions_
				std::unique_lock<std::shared_mutex> lock(owner_->sim_mutex_);
				if (owner_->contact_exclusions_.erase(
				        {csw.px_stud_shape, csw.px_hole_shape}) == 0) {
					throw std::runtime_error(
					    std::format("Failed to remove contact exclusion for "
					                "connection segment {}",
					                csid));
				}
			}
			csw.px_stud_shape = NullActorShapePair;
			csw.px_hole_shape = NullActorShapePair;

			const auto &[stud_if_ref, hole_if_ref] = csref;
			const auto &[stud_pid, stud_ifid] = stud_if_ref;
			owner_->reset_filtering_safely(stud_pid);
		}

		void on_bundle_created(
		    const ConnectionEndpoint &ep,
		    [[maybe_unused]] PhysicsConnectionBundleWrapper &cbw) {
			auto [a_id, b_id] = ep;

			// Update constraint scheduler
			owner_->constraint_scheduler_.on_connected(a_id, b_id);

			if (owner_->shape_level_collision_filtering_) {
				return;
			}
			{
				// Thread safety: modifying contact_exclusions_
				std::unique_lock<std::shared_mutex> lock(owner_->sim_mutex_);
				physx::PxRigidActor *const *actor_a_ptr =
				    owner_->topology_.parts()
				        .template project_key<PartId, physx::PxRigidActor *>(
				            a_id);
				physx::PxRigidActor *const *actor_b_ptr =
				    owner_->topology_.parts()
				        .template project_key<PartId, physx::PxRigidActor *>(
				            b_id);
				if (!actor_a_ptr || !actor_b_ptr) {
					throw std::runtime_error(std::format(
					    "Part ids {} and {} have no associated PxRigidActors",
					    a_id, b_id));
				}
				physx::PxRigidActor *actor_a = *actor_a_ptr;
				physx::PxRigidActor *actor_b = *actor_b_ptr;
				auto [_, inserted] =
				    owner_->contact_exclusions_.emplace(ContactExclusionPair{
				        {actor_a, nullptr}, {actor_b, nullptr}});
				if (!inserted) {
					throw std::runtime_error(std::format(
					    "Failed to add contact exclusion for bundle between "
					    "parts {} and {}",
					    a_id, b_id));
				}
			}
			owner_->reset_filtering_safely(a_id);
		}

		void on_bundle_removing(
		    const ConnectionEndpoint &ep,
		    [[maybe_unused]] PhysicsConnectionBundleWrapper &cbw) {
			auto [a_id, b_id] = ep;
			if (owner_->shape_level_collision_filtering_) {
				return;
			}
			{
				// Thread safety: modifying contact_exclusions_
				std::unique_lock<std::shared_mutex> lock(owner_->sim_mutex_);
				physx::PxRigidActor *const *actor_a_ptr =
				    owner_->topology_.parts()
				        .template project_key<PartId, physx::PxRigidActor *>(
				            a_id);
				physx::PxRigidActor *const *actor_b_ptr =
				    owner_->topology_.parts()
				        .template project_key<PartId, physx::PxRigidActor *>(
				            b_id);
				if (!actor_a_ptr || !actor_b_ptr) {
					throw std::runtime_error(
					    std::format("Part ids {} and {} have no associated "
					                "PxRigidActors",
					                a_id, b_id));
				}
				physx::PxRigidActor *actor_a = *actor_a_ptr;
				physx::PxRigidActor *actor_b = *actor_b_ptr;
				if (owner_->contact_exclusions_.erase(ContactExclusionPair{
				        {actor_a, nullptr}, {actor_b, nullptr}}) == 0) {
					throw std::runtime_error(
					    std::format("Failed to remove contact exclusion "
					                "for bundle between "
					                "parts {} and {}",
					                a_id, b_id));
				}
			}
			owner_->reset_filtering_safely(a_id);
		}

		void on_bundle_removed(const ConnectionEndpoint &ep) {
			auto [a_id, b_id] = ep;
			// Update constraint scheduler
			owner_->constraint_scheduler_.on_disconnected(a_id, b_id);
		}

		auto change_block() {
			return owner_->constraint_scheduler_.change_block();
		}
	};

	class PhysxBinding : private PxSimulationFilterPatch,
	                     private PxSimulationEventPatch {
	  public:
		explicit PhysxBinding(Self *owner, physx::PxScene *px_scene)
		    : PxSimulationFilterPatch{px_scene},
		      PxSimulationEventPatch{px_scene}, owner_{owner},
		      px_scene_{px_scene} {}
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

			// Thread safety: reading contact_exclusions_ and part_actors_
			std::shared_lock<std::shared_mutex> lock(owner_->sim_mutex_);

			// Check contact exclusion
			std::initializer_list<ContactExclusionPair> exclusion_pairs{
			    {{rb0, s0}, {rb1, s1}},
			    {{rb0, nullptr}, {rb1, s1}},
			    {{rb0, s0}, {rb1, nullptr}},
			    {{rb0, nullptr}, {rb1, nullptr}},
			};
			for (const auto &exclusion_pair : exclusion_pairs) {
				if (owner_->contact_exclusions_.contains(exclusion_pair)) {
					return physx::PxFilterFlag::eKILL;
				}
			}

			// Enable contact pose report collision between parts
			bool is_part0 = owner_->part_actors_.contains(rb0);
			bool is_part1 = owner_->part_actors_.contains(rb1);
			if (is_part0 && is_part1) {
				pairFlags |= physx::PxPairFlag::eCONTACT_EVENT_POSE;
			}
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

			// Thread safety: reading shape_mapping_
			std::shared_lock<std::shared_mutex> lock(owner_->sim_mutex_);

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

				const InterfaceSpec *if0 =
				    owner_->shape_mapping_.template find<ActorShapePair>(
				        {rb0, shape0});
				const InterfaceSpec *if1 =
				    owner_->shape_mapping_.template find<ActorShapePair>(
				        {rb1, shape1});
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
		Self *owner_;
		physx::PxScene *px_scene_;

		void process_assembly_contact(
		    physx::PxRigidActor *rb0, physx::PxShape *s0,
		    const InterfaceSpec &if0, const physx::PxTransform &pose0_px,
		    physx::PxRigidActor *rb1, physx::PxShape *s1,
		    const InterfaceSpec &if1, const physx::PxTransform &pose1_px,
		    const physx::PxVec3 &impulse_px, physx::PxReal dt) {

			// Metric conversion
			const auto &metrics = owner_->metrics_;
			Transformd pose0 = metrics.to_m(as<Transformd>(pose0_px));
			Transformd pose1 = metrics.to_m(as<Transformd>(pose1_px));
			Eigen::Vector3d impulse =
			    metrics.to_Ns(as<Eigen::Vector3d>(impulse_px));
			Eigen::Vector3d force = impulse / static_cast<double>(dt);

			std::optional<ConnectionSegment> result =
			    owner_->assembly_checker_.detect_assembly(if0, if1, pose0,
			                                              pose1, force);
			if (!result.has_value()) {
				return;
			}

			// Thread safety: read lock already acquired in caller

			const InterfaceRef *ifref0 =
			    owner_->shape_mapping_
			        .template project<ActorShapePair, InterfaceRef>({rb0, s0});
			const InterfaceRef *ifref1 =
			    owner_->shape_mapping_
			        .template project<ActorShapePair, InterfaceRef>({rb1, s1});
			if (!ifref0 || !ifref1) {
				std::unreachable();
			}

			owner_->enqueue_pending_assembly(ConnSegRef{*ifref0, *ifref1},
			                                 *result);
		}
	};

  public:
	using TopologyGraph =
	    LegoGraph<type_list<Ps...>, PhysicsPartWrapper, pmr_vector_storage,
	              type_list<physx::PxRigidActor *>,
	              type_list<std::hash<physx::PxRigidActor *>>,
	              type_list<std::equal_to<>>, PhysicsConnectionSegmentWrapper,
	              type_list<>, type_list<>, type_list<>,
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
	using PhysicsConstraintScheduler = ConstraintScheduler<
	    TopologyGraph, ConstraintSchedulingPolicy,
	    std::function<ConstraintHandle(PartId, PartId, const Transformd &)>,
	    std::function<void(ConstraintHandle)>, ConstraintHandle>;
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

	explicit PhysicsLegoGraph(
	    const MetricSystem &metrics, physx::PxPhysics *px,
	    Hooks *hooks = nullptr, AssemblyThresholds thresholds = {},
	    bool shape_level_collision_filtering = true,
	    std::pmr::memory_resource *mr = std::pmr::get_default_resource())
	    : res_{mr}, metrics_{metrics}, px_{px}, hooks_{hooks},
	      topology_hooks_{this}, topology_{&topology_hooks_, mr},
	      constraint_scheduler_{
	          &topology_, ConstraintSchedulingPolicy{},
	          std::bind_front(&PhysicsLegoGraph::create_constraint_safely,
	                          this),
	          std::bind_front(&PhysicsLegoGraph::destroy_constraint_safely,
	                          this),
	          mr},
	      assembly_checker_{thresholds},
	      shape_level_collision_filtering_{shape_level_collision_filtering},
	      pending_assemblies_{mr}, pending_disassemblies_{mr},
	      shape_mapping_{mr}, contact_exclusions_{mr} {}
	~PhysicsLegoGraph() {
		unbind_physx_scene();
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

	// Should be called BEFORE simulation step on USD/Kit thread
	// Use a StageUpdateNode with order < 10 to implement this
	void do_pre_step() {
		flush_physx_ops();
		sim_running_ = true;
	}

	// Should be called AFTER simulation step on USD/Kit thread
	// Use a StageUpdateNode with order > 10 to implement this
	void do_post_step() {
		sim_running_ = false;
		flush_physx_ops();

		std::pmr::vector<PendingAssembly> pending_assemblies{res_};
		std::pmr::vector<PendingDisassembly> pending_disassemblies{res_};
		{
			std::lock_guard lock{pending_mutex_};
			pending_assemblies.swap(pending_assemblies_);
			pending_disassemblies.swap(pending_disassemblies_);
		}
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
			    topology_.connect(stud_if, hole_if, std::tuple{}, conn_seg);
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
	std::pmr::memory_resource *res_;
	MetricSystem metrics_;
	physx::PxPhysics *px_;
	Hooks *hooks_;
	TopologyHooks topology_hooks_;
	TopologyGraph topology_;
	PhysicsConstraintScheduler constraint_scheduler_;
	std::unique_ptr<PhysxBinding> physx_binding_;
	AssemblyChecker assembly_checker_;
	bool shape_level_collision_filtering_;

	std::mutex pending_mutex_;
	std::pmr::vector<PendingAssembly> pending_assemblies_;
	std::pmr::vector<PendingDisassembly> pending_disassemblies_;

	std::shared_mutex sim_mutex_;
	ShapeMapping shape_mapping_;
	ContactExclusionSet contact_exclusions_;
	std::unordered_set<physx::PxRigidActor *> part_actors_;

	bool sim_running_ = false;
	std::unordered_map<ConstraintHandle, physx::PxConstraint *>
	    realized_constraints_;
	std::unordered_set<ConstraintHandle> constraints_to_remove_;
	std::unordered_map<ConstraintHandle, std::tuple<PartId, PartId, Transformd>>
	    pending_constraints_;
	ConstraintHandle next_constraint_handle_ = 1;
	std::unordered_set<PartId> pending_reset_filtering_parts_;
	void flush_physx_ops() {
		if (sim_running_) {
			throw std::runtime_error("Cannot process pending PhysX operations "
			                         "while simulation is running");
		}
		for (ConstraintHandle handle : constraints_to_remove_) {
			auto it = realized_constraints_.find(handle);
			if (it != realized_constraints_.end()) {
				do_release_constraint(it->second);
				realized_constraints_.erase(it);
			}
		}
		constraints_to_remove_.clear();
		for (auto &[handle, tuple] : pending_constraints_) {
			auto &[pid_a, pid_b, tf] = tuple;
			physx::PxConstraint *constraint =
			    do_create_constraint(pid_a, pid_b, tf);
			auto [it, inserted] =
			    realized_constraints_.emplace(handle, constraint);
			if (!inserted) {
				throw std::runtime_error(std::format(
				    "Constraint handle {} already realized", handle));
			}
		}
		pending_constraints_.clear();
		for (PartId pid : pending_reset_filtering_parts_) {
			physx::PxRigidActor *const *actor_ptr =
			    topology_.parts()
			        .template project_key<PartId, physx::PxRigidActor *>(pid);
			if (!actor_ptr) {
				log_warn(
				    "Cannot find actor for part id {}, skipping filter reset",
				    pid);
				continue;
			}
			physx::PxRigidActor *actor = *actor_ptr;
			actor->getScene()->resetFiltering(*actor);
		}
		pending_reset_filtering_parts_.clear();
	}
	ConstraintHandle create_constraint_safely(PartId pid_a, PartId pid_b,
	                                          const Transformd &T_a_b) {
		// Always queue, because the rigid actor might haven't been set up yet
		ConstraintHandle handle = next_constraint_handle_++;
		auto [it, inserted] = pending_constraints_.emplace(
		    handle, std::make_tuple(pid_a, pid_b, T_a_b));
		if (!inserted) {
			throw std::runtime_error(std::format(
			    "Constraint handle {} already exist in pending constraints",
			    handle));
		}
		return handle;
	}
	void destroy_constraint_safely(ConstraintHandle handle) {
		auto it = realized_constraints_.find(handle);
		if (it != realized_constraints_.end()) {
			if (sim_running_) {
				constraints_to_remove_.insert(handle);
			} else {
				do_release_constraint(it->second);
				realized_constraints_.erase(it);
			}
		} else {
			auto pit = pending_constraints_.find(handle);
			if (pit != pending_constraints_.end()) {
				pending_constraints_.erase(pit);
			} else {
				log_error("Constraint handle {} not found for destruction",
				          handle);
			}
		}
	}
	physx::PxConstraint *do_create_constraint(PartId a_id, PartId b_id,
	                                          const Transformd &T_a_b) {
		// PxConstraint shader uses body frames (COM frames) bA2w/bB2w.
		// Our T_a_b (from Python) is defined between actor-local origins (bottom centers).
		// Convert to COM-local frames so the weld aligns the intended anchor points.
		//
		// parentLocal (A_com -> B_com) = (A_com -> A_orig) * (A_orig -> B_orig) * (B_orig -> B_com)
		// childLocal is identity so cB2w = bB2w (B_com).
		const auto *actor_a_ptr =
		    topology_.parts()
		        .template project_key<PartId, physx::PxRigidActor *>(a_id);
		const auto *actor_b_ptr =
		    topology_.parts()
		        .template project_key<PartId, physx::PxRigidActor *>(b_id);
		if (!actor_a_ptr || !actor_b_ptr) {
			throw std::runtime_error(std::format(
			    "Cannot find actors for part ids {} and {}", a_id, b_id));
		}
		physx::PxRigidActor *actor_a = *actor_a_ptr;
		physx::PxRigidActor *actor_b = *actor_b_ptr;

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
	void do_release_constraint(physx::PxConstraint *constraint) {
		constraint->release();
	}
	void reset_filtering_safely(PartId pid) {
		if (sim_running_) {
			pending_reset_filtering_parts_.insert(pid);
		} else {
			physx::PxRigidActor *const *actor_ptr =
			    topology_.parts()
			        .template project_key<PartId, physx::PxRigidActor *>(pid);
			if (!actor_ptr) {
				log_warn(
				    "Cannot find actor for part id {}, skipping filter reset",
				    pid);
				return;
			}
			physx::PxRigidActor *actor = *actor_ptr;
			actor->getScene()->resetFiltering(*actor);
		}
	}

	void enqueue_pending_assembly(auto &&...args) {
		std::lock_guard lock{pending_mutex_};
		pending_assemblies_.emplace_back(std::forward<decltype(args)>(args)...);
	}

	void enqueue_pending_disassembly(auto &&...args) {
		std::lock_guard lock{pending_mutex_};
		pending_disassemblies_.emplace_back(
		    std::forward<decltype(args)>(args)...);
	}

	static_assert(TopologyGraph::HasAllOnPartAddedHooks);
	static_assert(TopologyGraph::HasAllOnPartRemovingHooks);
	static_assert(TopologyGraph::HasOnPartRemovedHook);
	static_assert(TopologyGraph::HasOnConnectedHook);
	static_assert(TopologyGraph::HasOnBundleCreatedHook);
	static_assert(TopologyGraph::HasOnBundleRemovingHook);
	static_assert(TopologyGraph::HasOnBundleRemovedHook);
	static_assert(TopologyGraph::HasChangeBlockHook);
};

} // namespace lego_assemble
