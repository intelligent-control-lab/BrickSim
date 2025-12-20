export module lego_assemble.physx.physics_graph;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.core.assembly;
import lego_assemble.core.component_index;
import lego_assemble.physx.constraint_scheduler;
import lego_assemble.physx.filtering_reset;
import lego_assemble.physx.shape_mapping;
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

export using PhysicsConnectionSegmentWrapper = SimpleWrapper<ConnectionSegment>;
export using PhysicsConnectionBundleWrapper = SimpleWrapper<ConnectionBundle>;

using ContactExclusionPair = UnorderedPair<ActorShapePair>;
using ContactExclusionPairHash =
    typename ContactExclusionPair::Hasher<ActorShapePairHash>;
using ContactExclusionSet =
    std::unordered_set<ContactExclusionPair, ContactExclusionPairHash>;

physx::PxRigidActor *cast_rigid_actor(physx::PxActor *actor) {
	auto type = actor->getType();
	if (type == physx::PxActorType::eRIGID_DYNAMIC ||
	    type == physx::PxActorType::eRIGID_STATIC) {
		return static_cast<physx::PxRigidActor *>(actor);
	} else {
		return nullptr;
	}
}

struct ComponentData {
	PartId representative;
};

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
	using PhysicsComponentIndex =
	    ComponentIndex<TopologyGraph, type_list<PartId, physx::PxRigidActor *>,
	                   ComponentData>;
	using PhysicsFilteringResetAggregator =
	    FilteringResetAggregator<TopologyGraph>;
	using PhysicsShapeMapping = ShapeMapping<TopologyGraph>;

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
	class TopologyHooks {
	  private:
		using G = TopologyGraph;

		// Reference to owner's fields
		PhysicsShapeMapping &shape_mapping_;
		PhysicsComponentIndex &cc_index_;
		PhysicsConstraintScheduler &constraint_scheduler_;
		PhysicsFilteringResetAggregator &filtering_reset_;

	  public:
		TopologyHooks(Self *owner)
		    : shape_mapping_{owner->shape_mapping_},
		      cc_index_{owner->cc_index_},
		      constraint_scheduler_{owner->constraint_scheduler_},
		      filtering_reset_{owner->filtering_reset_} {}

		~TopologyHooks() = default;
		TopologyHooks(const TopologyHooks &) = delete;
		TopologyHooks &operator=(const TopologyHooks &) = delete;
		TopologyHooks(TopologyHooks &&) = delete;
		TopologyHooks &operator=(TopologyHooks &&) = delete;

		void on_part_added(G::PartEntry entry) {
			PartId pid = entry.template key<PartId>();
			shape_mapping_.add_part(entry);
			cc_index_.mark_dirty_topological(pid);
			constraint_scheduler_.notify_part_added(pid);
		}

		void on_part_removing(G::PartEntry entry) {
			shape_mapping_.remove_part(entry);
		}

		void on_part_removed(PartId pid) {
			// Do this after removed so we schedule constraints on the updated graph
			cc_index_.mark_removed(pid);
			constraint_scheduler_.notify_part_removed(pid);
			filtering_reset_.mark_removed(pid);
		}

		void on_bundle_created(G::ConnBundleEntry cb_entry) {
			auto [pid_a, pid_b] = cb_entry.first;

			// One vertex is enough because both parts are in the same CC
			cc_index_.mark_dirty_topological(pid_a);

			constraint_scheduler_.notify_connected(pid_a, pid_b);

			// Reset filtering for all parts in the merged connected component
			filtering_reset_.mark_for_reset(pid_a);
		}

		void on_bundle_removed(const ConnectionEndpoint &ep) {
			auto [pid_a, pid_b] = ep;

			// Mark both parts as dirty
			cc_index_.mark_dirty_topological(pid_a);
			cc_index_.mark_dirty_topological(pid_b);

			// Do this after removed so the scheduler sees the updated graph
			constraint_scheduler_.notify_disconnected(pid_a, pid_b);

			// Reset filtering for BOTH (needed for correctness in vertex-deletion case)
			filtering_reset_.mark_for_reset(pid_a);
			filtering_reset_.mark_for_reset(pid_b);
		}

		void on_connected(G::ConnSegEntry cs_entry,
		                  [[maybe_unused]] G::ConnBundleEntry cb_entry,
		                  [[maybe_unused]] const InterfaceSpec &stud_spec,
		                  [[maybe_unused]] const InterfaceSpec &hole_spec) {
			const auto &[stud_if_ref, hole_if_ref] =
			    cs_entry.template key<ConnSegRef>();
			const auto &[stud_pid, stud_ifid] = stud_if_ref;
			const auto &[hole_pid, hole_ifid] = hole_if_ref;
			cc_index_.mark_dirty_data_by_part(stud_pid);
			cc_index_.mark_dirty_data_by_part(hole_pid);
		}

		void on_disconnected([[maybe_unused]] ConnSegId csid,
		                     const ConnSegRef &csref) {
			const auto &[stud_if_ref, hole_if_ref] = csref;
			const auto &[stud_pid, stud_ifid] = stud_if_ref;
			const auto &[hole_pid, hole_ifid] = hole_if_ref;
			cc_index_.mark_dirty_data_by_part(stud_pid);
			cc_index_.mark_dirty_data_by_part(hole_pid);
		}
	};

	// Manages data that simulation threads write to
	class SimOutputData {
	  private:
		std::vector<PendingAssembly> pending_assemblies_;
		std::vector<PendingDisassembly> pending_disassemblies_;
		std::vector<PhysicsAssemblyDebugInfo> assembly_debug_infos_cur_;
		std::vector<PhysicsAssemblyDebugInfo> assembly_debug_infos_prev_;
		std::unordered_map<PartId, std::tuple<physx::PxVec3, physx::PxVec3>>
		    external_wrenches_;

	  public:
		void enqueue_assembly(auto &&...args) {
			pending_assemblies_.emplace_back(
			    std::forward<decltype(args)>(args)...);
		}

		void enqueue_assembly_debug_info(auto &&...args) {
			assembly_debug_infos_cur_.emplace_back(
			    std::forward<decltype(args)>(args)...);
		}

		void enqueue_disassembly(auto &&...args) {
			pending_disassemblies_.emplace_back(
			    std::forward<decltype(args)>(args)...);
		}

		void enqueue_external_wrench(
		    PartId pid,
		    const std::tuple<physx::PxVec3, physx::PxVec3> &wrench) {
			const auto &[force, torque] = wrench;
			auto it = external_wrenches_.find(pid);
			if (it != external_wrenches_.end()) {
				auto &[acc_force, acc_torque] = it->second;
				acc_force += force;
				acc_torque += torque;
			} else {
				external_wrenches_.emplace(pid, wrench);
			}
		}

		std::vector<PendingAssembly> consume_assemblies() {
			std::vector<PendingAssembly> res = std::move(pending_assemblies_);
			pending_assemblies_.clear();
			return res;
		}

		std::vector<PendingDisassembly> consume_disassemblies() {
			std::vector<PendingDisassembly> res =
			    std::move(pending_disassemblies_);
			pending_disassemblies_.clear();
			return res;
		}

		std::vector<PhysicsAssemblyDebugInfo> get_assembly_debug_infos() const {
			return assembly_debug_infos_prev_;
		}

		void swap_debug_info_buffers() {
			assembly_debug_infos_prev_ = std::move(assembly_debug_infos_cur_);
			assembly_debug_infos_cur_.clear();
		}

		std::unordered_map<PartId, std::tuple<physx::PxVec3, physx::PxVec3>>
		consume_external_wrenches() {
			std::unordered_map<PartId, std::tuple<physx::PxVec3, physx::PxVec3>>
			    res = std::move(external_wrenches_);
			external_wrenches_.clear();
			return res;
		}
	};

	class PhysxBinding : private PxSimulationFilterPatch,
	                     private PxSimulationEventPatch {
	  public:
		explicit PhysxBinding(Self *owner, physx::PxScene *px_scene)
		    : PxSimulationFilterPatch{px_scene},
		      PxSimulationEventPatch{px_scene}, metrics_{owner->metrics_},
		      shape_map_{owner->shape_mapping_.committed_view()},
		      cc_index_{owner->cc_index_},
		      assembly_checker_{owner->assembly_checker_},
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
			using namespace physx;

			PxFilterFlags result = PxSimulationFilterPatch::pairFound(
			    pairID, attributes0, filterData0, ca0, cs0, attributes1,
			    filterData1, ca1, cs1, pairFlags);

			const ComponentId *cc0 = lookup_cc(ca0);
			const ComponentId *cc1 = lookup_cc(ca1);

			if (cc0 == nullptr && cc1 == nullptr) {
				// Non-lego vs. non-lego contact
				return result;
			} else if (cc0 != nullptr && cc1 != nullptr) {
				// Lego vs. lego contact
				if (*cc0 == *cc1) {
					// Same connected component
					return PxFilterFlag::eKILL;
				} else {
					// Different connected components
					// Fallthrough
				}
			} else {
				// Lego vs. non-lego contact
				// Fallthrough
			}

			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
			pairFlags |= PxPairFlag::eCONTACT_EVENT_POSE;

			return result;
		}

		virtual void onContact(const physx::PxContactPairHeader &header,
		                       const physx::PxContactPair *pairs,
		                       physx::PxU32 nbPairs) override {
			using namespace physx;

			PxSimulationEventPatch::onContact(header, pairs, nbPairs);

			// Ignore removed actors
			if (header.flags.isSet(PxContactPairHeaderFlag::eREMOVED_ACTOR_0)) {
				return;
			}
			if (header.flags.isSet(PxContactPairHeaderFlag::eREMOVED_ACTOR_1)) {
				return;
			}

			PxActor *a0 = header.actors[0];
			PxActor *a1 = header.actors[1];
			const ComponentId *cc0 = lookup_cc(a0);
			const ComponentId *cc1 = lookup_cc(a1);
			if (cc0 == nullptr && cc1 == nullptr) {
				// Non-lego vs. non-lego contact
				return;
			} else if (cc0 != nullptr && cc1 != nullptr) {
				// Lego vs. lego contact
				if (*cc0 == *cc1) {
					// Same connected component
					return;
				} else {
					// Different connected components
					// Fallthrough
				}
			} else {
				// Lego vs. non-lego contact
				// Fallthrough
			}

			// 1. For lego-lego contact, check assembly
			if (cc0 != nullptr && cc1 != nullptr) {
				process_assembly_contacts(header, pairs, nbPairs);
			}

			// 2. Collect all external wrenches
			auto [w0, w1] = accumulate_wrenches(header, pairs, nbPairs);
			if (cc0 != nullptr) {
				PartId pid0 = *lookup_part_id(a0);
				sim_out_.enqueue_external_wrench(pid0, w0);
			}
			if (cc1 != nullptr) {
				PartId pid1 = *lookup_part_id(a1);
				sim_out_.enqueue_external_wrench(pid1, w1);
			}
		}

	  private:
		const MetricSystem &metrics_;
		const ShapeMap &shape_map_;
		const PhysicsComponentIndex &cc_index_;
		const AssemblyChecker &assembly_checker_;
		const bool &collect_assembly_debug_info_;
		SimOutputData &sim_out_;
		physx::PxScene *px_scene_;

		static void
		iterate_contact_itemsets(const physx::PxContactPairHeader &header,
		                         const physx::PxContactPair *pairs,
		                         physx::PxU32 nbPairs, auto &&fn) {
			using namespace physx;
			if (!header.extraDataStream || header.extraDataStreamSize == 0) {
				return;
			}
			PxContactPairExtraDataIterator it{header.extraDataStream,
			                                  header.extraDataStreamSize};
			// Read first item set
			if (!it.nextItemSet())
				return;
			PxU32 begin = it.contactPairIndex;
			const PxContactPairPose *eventPose = it.eventPose;
			while (it.nextItemSet()) {
				const PxU32 nextStart = it.contactPairIndex;
				if (eventPose) {
					PxU32 end = std::min(nextStart, nbPairs);
					std::span<const PxContactPair> span_pairs{&pairs[begin],
					                                          end - begin};
					std::invoke(fn, span_pairs, eventPose->globalPose[0],
					            eventPose->globalPose[1]);
				}
				// Advance "current" to the set we just read
				begin = nextStart;
				eventPose = it.eventPose;
			}
			// Process the last item set (ends at nbPairs)
			if (eventPose) {
				std::span<const PxContactPair> span_pairs{&pairs[begin],
				                                          nbPairs - begin};
				std::invoke(fn, span_pairs, eventPose->globalPose[0],
				            eventPose->globalPose[1]);
			}
		}

		static void
		iterate_contact_pairs(std::span<const physx::PxContactPair> pairs,
		                      auto &&fn) {
			using namespace physx;
			for (const auto &pair : pairs) {
				const auto &flags = pair.flags;
				if (flags.isSet(PxContactPairFlag::eREMOVED_SHAPE_0) ||
				    flags.isSet(PxContactPairFlag::eREMOVED_SHAPE_1) ||
				    !flags.isSet(PxContactPairFlag::eINTERNAL_HAS_IMPULSES) ||
				    !pair.contactPatches) {
					continue;
				}
				std::span<const PxContactPatch> patches{
				    reinterpret_cast<const PxContactPatch *>(
				        pair.contactPatches),
				    pair.patchCount};
				std::invoke(fn, pair, patches);
			}
		}

		static void
		iterate_contact_pairs(const physx::PxContactPairHeader &header,
		                      const physx::PxContactPair *pairs,
		                      physx::PxU32 nbPairs, auto &&fn) {
			using namespace physx;
			iterate_contact_itemsets(
			    header, pairs, nbPairs,
			    [&](std::span<const PxContactPair> pairs,
			        const physx::PxTransform &pose0,
			        const physx::PxTransform &pose1) {
				    iterate_contact_pairs(
				        pairs, [&](const PxContactPair &pair,
				                   std::span<const PxContactPatch> patches) {
					        std::invoke(fn, pair, patches, pose0, pose1);
				        });
			    });
		}

		static std::span<const physx::PxReal>
		impulses_view(const physx::PxContactPair &pair,
		              const physx::PxContactPatch &patch) {
			return std::span<const physx::PxReal>{
			    &pair.contactImpulses[patch.startContactIndex],
			    patch.nbContacts};
		}

		// Returns total wrenches applied to each actor,
		// in the world frame w.r.t. center of mass
		std::array<std::tuple<physx::PxVec3, physx::PxVec3>, 2>
		accumulate_wrenches(const physx::PxContactPairHeader &header,
		                    const physx::PxContactPair *pairs,
		                    physx::PxU32 nbPairs) {
			using namespace physx;

			auto get_com = [](PxActor *actor) {
				if (actor->is<PxRigidBody>()) {
					return static_cast<PxRigidBody *>(actor)
					    ->getCMassLocalPose()
					    .p;
				} else {
					return PxVec3{0, 0, 0};
				}
			};
			PxVec3 p_0_com0 = get_com(header.actors[0]);
			PxVec3 p_1_com1 = get_com(header.actors[1]);

			PxVec3 total_J0{0, 0, 0};
			PxVec3 total_J1{0, 0, 0};
			PxVec3 total_tau0{0, 0, 0};
			PxVec3 total_tau1{0, 0, 0};
			iterate_contact_itemsets(
			    header, pairs, nbPairs,
			    [&](std::span<const PxContactPair> pairs,
			        const physx::PxTransform &T_world_0,
			        const physx::PxTransform &T_world_1) {
				    PxVec3 p_world_com0 = T_world_0.transform(p_0_com0);
				    PxVec3 p_world_com1 = T_world_1.transform(p_1_com1);
				    for (const auto &pair : pairs) {
					    const auto &flags = pair.flags;
					    if (flags.isSet(PxContactPairFlag::eREMOVED_SHAPE_0) ||
					        flags.isSet(PxContactPairFlag::eREMOVED_SHAPE_1) ||
					        !flags.isSet(
					            PxContactPairFlag::eINTERNAL_HAS_IMPULSES) ||
					        !pair.contactPatches || !pair.contactPoints ||
					        !pair.contactImpulses) {
						    continue;
					    }

					    PxContactStreamIterator it{
					        pair.contactPatches, pair.contactPoints,
					        pair.getInternalFaceIndices(), pair.patchCount,
					        pair.contactCount};
					    PxU32 k = 0;
					    while (it.hasNextPatch()) {
						    it.nextPatch();
						    while (it.hasNextContact()) {
							    it.nextContact();
							    PxVec3 x = it.getContactPoint();
							    PxVec3 n = it.getContactNormal();
							    PxReal j = pair.contactImpulses[k++];
							    PxVec3 J = n * j;
							    total_J0 += J;
							    total_J1 += -J;
							    total_tau0 += (x - p_world_com0).cross(J);
							    total_tau1 += (x - p_world_com1).cross(-J);
						    }
					    }

					    PxFrictionAnchorStreamIterator faIt{
					        pair.contactPatches, pair.frictionPatches,
					        pair.patchCount};
					    while (faIt.hasNextPatch()) {
						    faIt.nextPatch();
						    while (faIt.hasNextFrictionAnchor()) {
							    faIt.nextFrictionAnchor();
							    PxVec3 x = faIt.getPosition();
							    PxVec3 J = faIt.getImpulse();
							    total_J0 += J;
							    total_J1 += -J;
							    total_tau0 += (x - p_world_com0).cross(J);
							    total_tau1 += (x - p_world_com1).cross(-J);
						    }
					    }
				    }
			    });
			double dt = getPxSceneElapsedTime(px_scene_);
			return {std::make_tuple(total_J0 / dt, total_tau0 / dt),
			        std::make_tuple(total_J1 / dt, total_tau1 / dt)};
		}

		const PartId *lookup_part_id(const physx::PxActor *ca) {
			auto *rb = cast_rigid_actor(const_cast<physx::PxActor *>(ca));
			if (rb == nullptr) {
				return nullptr;
			}
			return cc_index_.ids().template find_key<PartId>(rb);
		};

		const ComponentId *lookup_cc(const physx::PxActor *ca) {
			auto *rb = cast_rigid_actor(const_cast<physx::PxActor *>(ca));
			if (rb == nullptr) {
				return nullptr;
			}
			return cc_index_.ids().find_value(rb);
		};

		void process_assembly_contacts(const physx::PxContactPairHeader &header,
		                               const physx::PxContactPair *pairs,
		                               physx::PxU32 nbPairs) {
			using namespace physx;
			PxRigidActor *rb0 = cast_rigid_actor(header.actors[0]);
			PxRigidActor *rb1 = cast_rigid_actor(header.actors[1]);
			if (rb0 == nullptr || rb1 == nullptr) {
				return;
			}
			double dt = getPxSceneElapsedTime(px_scene_);
			iterate_contact_pairs(
			    header, pairs, nbPairs,
			    [&](const PxContactPair &pair,
			        std::span<const PxContactPatch> patches,
			        const PxTransform &pose0, const PxTransform &pose1) {
				    // For each contact pair in each CCD pass

				    const auto &[shape0, shape1] = pair.shapes;
				    const InterfaceSpec *if0 =
				        shape_map_.find_value(ActorShapePair{rb0, shape0});
				    const InterfaceSpec *if1 =
				        shape_map_.find_value(ActorShapePair{rb1, shape1});
				    if (!if0 || !if1) {
					    return;
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
					    return;
				    }

				    // Sum up contact impulses
				    PxVec3 total_impulse{0, 0, 0};
				    for (const auto &patch : patches) {
					    PxReal patch_impulse = 0;
					    for (const auto &impulse : impulses_view(pair, patch)) {
						    patch_impulse += impulse;
					    }
					    total_impulse += patch.normal * patch_impulse;
				    }

				    if (to_swap) {
					    process_assembly_contact(rb1, shape1, *if1, pose1, rb0,
					                             shape0, *if0, pose0,
					                             -total_impulse, dt);
				    } else {
					    process_assembly_contact(rb0, shape0, *if0, pose0, rb1,
					                             shape1, *if1, pose1,
					                             total_impulse, dt);
				    }
			    });
		}

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
				    shape_map_.template find_key<InterfaceRef>(
				        ActorShapePair{rb0, s0});
				const InterfaceRef *ifref1 =
				    shape_map_.template find_key<InterfaceRef>(
				        ActorShapePair{rb1, s1});
				if (ifref0 && ifref1) {
					debug_info.csref = {*ifref0, *ifref1};
					sim_out_.enqueue_assembly_debug_info(debug_info);
				}
			}

			if (!result.has_value()) {
				return;
			}

			const InterfaceRef &ifref0 =
			    shape_map_.template key_of<InterfaceRef>(
			        ActorShapePair{rb0, s0});
			const InterfaceRef &ifref1 =
			    shape_map_.template key_of<InterfaceRef>(
			        ActorShapePair{rb1, s1});

			sim_out_.enqueue_assembly(ConnSegRef{ifref0, ifref1}, *result);
		}
	};

  public:
	explicit PhysicsLegoGraph(const MetricSystem &metrics, physx::PxPhysics *px,
	                          Hooks *hooks = nullptr,
	                          AssemblyThresholds thresholds = {},
	                          bool collect_assembly_debug_info = true)
	    : metrics_{metrics}, px_{px}, hooks_{hooks},
	      collect_assembly_debug_info_{collect_assembly_debug_info},
	      topology_{}, assembly_checker_{thresholds}, shape_mapping_{},
	      cc_index_{topology_},
	      constraint_scheduler_{
	          &topology_, ConstraintSchedulingPolicy{},
	          std::bind_front(&PhysicsLegoGraph::create_constraint, this),
	          std::bind_front(&PhysicsLegoGraph::release_constraint, this)},
	      filtering_reset_{topology_}, sim_out_{}, topology_hooks_{this} {
		topology_.set_hooks(&topology_hooks_);
	}
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
		auto external_wrenches = sim_out_.consume_external_wrenches();
		(void)external_wrenches; // TODO: use external wrenches

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
	bool collect_assembly_debug_info_;

	TopologyGraph topology_;
	AssemblyChecker assembly_checker_;
	PhysicsShapeMapping shape_mapping_;
	PhysicsComponentIndex cc_index_;
	PhysicsConstraintScheduler constraint_scheduler_;
	PhysicsFilteringResetAggregator filtering_reset_;
	SimOutputData sim_out_;

	TopologyHooks topology_hooks_;
	std::unique_ptr<PhysxBinding> physx_binding_;

	void flush_physx_ops() {
		shape_mapping_.commit();
		cc_index_.commit(
		    std::bind_front(&PhysicsLegoGraph::build_cc_data, this),
		    std::bind_front(&PhysicsLegoGraph::update_cc_data, this));
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

	ComponentData build_cc_data(ComponentId cc_id, PartId rep) {
		// TODO
		(void)cc_id;
		return ComponentData{rep};
	}

	void update_cc_data(ComponentId cc_id, ComponentData &data) {
		// TODO
		(void)cc_id;
		(void)data;
	}

	static_assert(TopologyGraph::HasOnPartAddedHook);
	static_assert(TopologyGraph::HasOnPartRemovingHook);
	static_assert(TopologyGraph::HasOnPartRemovedHook);
	static_assert(TopologyGraph::HasOnBundleCreatedHook);
	static_assert(TopologyGraph::HasOnBundleRemovedHook);
	static_assert(TopologyGraph::HasOnConnectedHook);
	static_assert(TopologyGraph::HasOnDisconnectedHook);
};

} // namespace lego_assemble
