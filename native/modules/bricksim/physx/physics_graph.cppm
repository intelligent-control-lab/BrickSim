export module bricksim.physx.physics_graph;

import std;
import bricksim.core.specs;
import bricksim.core.graph;
import bricksim.core.connections;
import bricksim.core.component_index;
import bricksim.physx.assembly;
import bricksim.physx.constraint_scheduler;
import bricksim.physx.filtering_reset;
import bricksim.physx.shape_mapping;
import bricksim.physx.weld_constraint;
import bricksim.physx.patcher;
import bricksim.physx.breakage;
import bricksim.utils.type_list;
import bricksim.utils.poly_store;
import bricksim.utils.transforms;
import bricksim.utils.conversions;
import bricksim.utils.multi_key_map;
import bricksim.utils.pair;
import bricksim.utils.unordered_pair;
import bricksim.utils.metric_system;
import bricksim.utils.eigen_format;
import bricksim.utils.logging;
import bricksim.vendor;

namespace bricksim {

constexpr bool EnableBreakage = true;

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

export struct PhysicsConnectionSegmentWrapper
    : SimpleWrapper<ConnectionSegment> {
	template <class... Args>
	explicit PhysicsConnectionSegmentWrapper(Args &&...args)
	    : SimpleWrapper<ConnectionSegment>(std::forward<Args>(args)...) {}

	double utilization() const {
		return utilization_;
	}

  private:
	template <class, class> friend class PhysicsLegoGraph;

	double utilization_{-1.0};
};

export using PhysicsConnectionBundleWrapper = SimpleWrapper<ConnectionBundle>;

using ContactExclusionPair = UnorderedPair<ActorShapePair>;
using ContactExclusionPairHash =
    typename ContactExclusionPair::Hasher<ActorShapePairHash>;
using ContactExclusionSet =
    std::unordered_set<ContactExclusionPair, ContactExclusionPairHash>;

struct ComponentData {
	PartId representative;
	std::unique_ptr<BreakageSystem> breakage_system;
	std::unique_ptr<BreakageState> breakage_state;
};

export struct PhysicsAssemblyDebugInfo : AssemblyDebugInfo {
	ConnSegRef csref;
};

using PartValidationQueue = std::unordered_set<PartId>;

struct PartRigidInfo {
	physx::PxVec3 com_pos{physx::PxZero};
	physx::PxQuat body_rot{physx::PxIdentity};
	physx::PxVec3 linear_velocity{physx::PxZero};
	physx::PxVec3 angular_velocity{physx::PxZero};
	physx::PxVec3 linear_impulse{physx::PxZero};
	physx::PxVec3 angular_impulse{physx::PxZero};
};

using PartRigidInfoMap = std::unordered_map<PartId, PartRigidInfo>;

physx::PxVec3 compute_linear_momentum(physx::PxRigidDynamic *rb) {
	return rb->getLinearVelocity() * rb->getMass();
}

physx::PxVec3 compute_angular_momentum(physx::PxRigidDynamic *rb) {
	physx::PxQuat q_WA = rb->getGlobalPose().q;
	physx::PxQuat q_AM = rb->getCMassLocalPose().q;
	physx::PxQuat q_WM = q_WA * q_AM;
	physx::PxVec3 omega_W = rb->getAngularVelocity();
	physx::PxVec3 omega_M = q_WM.rotateInv(omega_W);
	return q_WM.rotate(rb->getMassSpaceInertiaTensor().multiply(omega_M));
}

export template <class Ps, class Hooks> class PhysicsLegoGraph;

export template <PartLike... Ps, class Hooks>
class PhysicsLegoGraph<type_list<Ps...>, Hooks> {
  private:
	class TopologyHooks;

  public:
	using Self = PhysicsLegoGraph<type_list<Ps...>, Hooks>;

	using TopologyGraph =
	    LegoGraph<type_list<Ps...>, PhysicsPartWrapper,
	              type_list<physx::PxRigidDynamic *>,
	              type_list<std::hash<physx::PxRigidDynamic *>>,
	              type_list<std::equal_to<>>, PhysicsConnectionSegmentWrapper,
	              type_list<>, type_list<>, type_list<>,
	              PhysicsConnectionBundleWrapper, TopologyHooks>;
	using ConstraintSchedulingPolicy =
	    CombinedSchedulingPolicy<TopologyGraph, TreeOnlySchedulingPolicy,
	                             RandomRegularGraphSchedulingPolicy>;
	// using ConstraintSchedulingPolicy = CombinedSchedulingPolicy<TopologyGraph, FullGraphSchedulingPolicy, RandomRegularGraphSchedulingPolicy>;
	// using ConstraintSchedulingPolicy = RandomRegularGraphSchedulingPolicy<TopologyGraph>;
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
	    ComponentIndex<TopologyGraph,
	                   type_list<PartId, physx::PxRigidDynamic *>,
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
	    requires(Hooks &hooks, ConnSegId csid, const ConnSegRef &csref,
	             const ConnectionSegment &conn_seg) {
		    { // Called after a connection segment is disassembled
			    hooks.on_disassembled(csid, csref, conn_seg)
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
		PartValidationQueue &parts_to_validate_;
		bool &in_simulation_step_;

		void check_not_in_simulation_step() const {
			if (in_simulation_step_) {
				throw std::runtime_error(
				    "Topology modification not allowed during simulation step");
			}
		}

	  public:
		TopologyHooks(Self *owner)
		    : shape_mapping_{owner->shape_mapping_},
		      cc_index_{owner->cc_index_},
		      constraint_scheduler_{owner->constraint_scheduler_},
		      filtering_reset_{owner->filtering_reset_},
		      parts_to_validate_{owner->parts_to_validate_},
		      in_simulation_step_{owner->in_simulation_step_} {}

		~TopologyHooks() = default;
		TopologyHooks(const TopologyHooks &) = delete;
		TopologyHooks &operator=(const TopologyHooks &) = delete;
		TopologyHooks(TopologyHooks &&) = delete;
		TopologyHooks &operator=(TopologyHooks &&) = delete;

		void on_part_added(G::PartEntry entry) {
			check_not_in_simulation_step();
			PartId pid = entry.template key<PartId>();
			parts_to_validate_.emplace(pid);
			shape_mapping_.add_part(entry);
			cc_index_.mark_dirty_topological(pid);
			constraint_scheduler_.notify_part_added(pid);
		}

		void on_part_removing(G::PartEntry entry) {
			check_not_in_simulation_step();
			PartId pid = entry.template key<PartId>();
			parts_to_validate_.erase(pid);
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
			check_not_in_simulation_step();
			const auto &[stud_if_ref, hole_if_ref] =
			    cs_entry.template key<ConnSegRef>();
			const auto &[stud_pid, stud_ifid] = stud_if_ref;
			const auto &[hole_pid, hole_ifid] = hole_if_ref;
			cc_index_.mark_dirty_data_by_part(stud_pid);
			cc_index_.mark_dirty_data_by_part(hole_pid);
		}

		void on_disconnected([[maybe_unused]] ConnSegId csid,
		                     const ConnSegRef &csref) {
			check_not_in_simulation_step();
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
		std::vector<PhysicsAssemblyDebugInfo> assembly_debug_infos_cur_;
		std::vector<PhysicsAssemblyDebugInfo> assembly_debug_infos_prev_;

	  public:
		void enqueue_assembly(auto &&...args) {
			pending_assemblies_.emplace_back(
			    std::forward<decltype(args)>(args)...);
		}

		void enqueue_assembly_debug_info(auto &&...args) {
			assembly_debug_infos_cur_.emplace_back(
			    std::forward<decltype(args)>(args)...);
		}

		std::vector<PendingAssembly> consume_assemblies() {
			std::vector<PendingAssembly> res = std::move(pending_assemblies_);
			pending_assemblies_.clear();
			return res;
		}

		std::vector<PhysicsAssemblyDebugInfo> get_assembly_debug_infos() const {
			return assembly_debug_infos_prev_;
		}

		void swap_debug_info_buffers() {
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

			pairFlags |= PxPairFlag::eSOLVE_CONTACT;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_PERSISTS;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_CCD;
			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
			pairFlags |= PxPairFlag::eDETECT_DISCRETE_CONTACT;
			pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;
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
			const ComponentId *cc0 = lookup_cc(header.actors[0]);
			const ComponentId *cc1 = lookup_cc(header.actors[1]);
			if (cc0 != nullptr && cc1 != nullptr && *cc0 != *cc1) {
				// Lego vs. lego contact, different connected components
				process_assembly_contacts(header, pairs, nbPairs);
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

		const ComponentId *lookup_cc(const physx::PxActor *actor) {
			if (const physx::PxRigidDynamic *ra =
			        actor->is<physx::PxRigidDynamic>()) {
				return cc_index_.ids().find_value(
				    const_cast<physx::PxRigidDynamic *>(ra));
			} else {
				return nullptr;
			}
		};

		void process_assembly_contacts(const physx::PxContactPairHeader &header,
		                               const physx::PxContactPair *pairs,
		                               physx::PxU32 nbPairs) {
			using namespace physx;
			PxRigidDynamic *rb0 = header.actors[0]->is<PxRigidDynamic>();
			PxRigidDynamic *rb1 = header.actors[1]->is<PxRigidDynamic>();
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
		    physx::PxRigidDynamic *rb0, physx::PxShape *s0,
		    const InterfaceSpec &if0, const physx::PxTransform &pose0_px,
		    physx::PxRigidDynamic *rb1, physx::PxShape *s1,
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
	                          AssemblyThresholds assembly_thresholds = {},
	                          BreakageThresholds breakage_thresholds = {},
	                          bool collect_assembly_debug_info = true)
	    : metrics_{metrics}, px_{px}, hooks_{hooks},
	      collect_assembly_debug_info_{collect_assembly_debug_info},
	      topology_{}, assembly_checker_{assembly_thresholds},
	      breakage_checker_{breakage_thresholds}, shape_mapping_{},
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

	BreakageChecker &breakage_checker() {
		return breakage_checker_;
	}

	const BreakageChecker &breakage_checker() const {
		return breakage_checker_;
	}

	std::vector<PhysicsAssemblyDebugInfo> get_assembly_debug_infos() const {
		return sim_out_.get_assembly_debug_infos();
	}

	void do_pre_step() {
		for (PartId pid : parts_to_validate_) {
			validate_part_actor(pid);
		}
		parts_to_validate_.clear();
		shape_mapping_.commit();
		prepare_rigid_info_map();
		cc_index_.commit(
		    std::bind_front(&PhysicsLegoGraph::build_cc_data, this),
		    std::bind_front(&PhysicsLegoGraph::update_cc_data, this));
		constraint_scheduler_.commit();
		filtering_reset_.commit();

		// Do not check in_simulation_step_ here because sometimes do_post_step is not called,
		// e.g. when Isaaclab is resumed from pause
		in_simulation_step_ = true;
	}

	void do_post_step() {
		if (!in_simulation_step_) {
			throw std::runtime_error(
			    "do_post_step called without matching do_pre_step");
		}
		auto pending_assemblies = sim_out_.consume_assemblies();
		sim_out_.swap_debug_info_buffers();
		populate_rigid_info_map();
		in_simulation_step_ = false;
		auto pending_disassemblies = compute_breakages();

		if constexpr (EnableBreakage) {
			for (const auto &[csid] : pending_disassemblies) {
				const auto *cs_entry =
				    topology_.connection_segments().find(csid);
				ConnSegRef csref = cs_entry->template key<ConnSegRef>();
				ConnectionSegment conn_seg = cs_entry->value().wrapped();
				bool disconnected = topology_.disconnect(csid).has_value();
				if (disconnected) {
					log_info("Connection {} breaks", csid);
					if constexpr (HasOnDisassembledHook) {
						if (hooks_) {
							hooks_->on_disassembled(csid, csref, conn_seg);
						}
					}
				}
			}
		}
		for (const auto &[csref, conn_seg] : pending_assemblies) {
			const auto &[stud_if, hole_if] = csref;
			std::optional<ConnSegId> csid =
			    topology_.connect(stud_if, hole_if, conn_seg);
			if (csid.has_value()) {
				log_info("Connection {} assembles", *csid);
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
	BreakageChecker breakage_checker_;
	PhysicsShapeMapping shape_mapping_;
	PhysicsComponentIndex cc_index_;
	PhysicsConstraintScheduler constraint_scheduler_;
	PhysicsFilteringResetAggregator filtering_reset_;
	SimOutputData sim_out_;
	PartValidationQueue parts_to_validate_;
	PartRigidInfoMap part_rigid_info_map_;

	TopologyHooks topology_hooks_;
	std::unique_ptr<PhysxBinding> physx_binding_;

	bool in_simulation_step_{false};

	void validate_part_actor(PartId pid) {
		typename TopologyGraph::PartConstEntry entry =
		    topology_.parts().entry_of(pid);
		physx::PxRigidDynamic *actor =
		    entry.template key<physx::PxRigidDynamic *>();
		auto [mass, com, inertia_tensor] = entry.visit([](const auto &pw) {
			const auto &part = pw.wrapped();
			return std::make_tuple(part.mass(), part.com(),
			                       part.inertia_tensor());
		});
		double px_mass = actor->getMass();
		if (px_mass <= 0.0 || std::abs(px_mass - mass) > 1e-6) {
			log_error("PhysicsLegoGraph::validate_part_actor: mass mismatch "
			          "for {} (expected {:.6e}, got {:.6e})",
			          pid, mass, px_mass);
		}
		physx::PxTransform Tcm = actor->getCMassLocalPose();
		Eigen::Vector3d px_com = as<Eigen::Vector3d>(Tcm.p);
		if ((px_com - com).norm() > 1e-6) {
			log_error("PhysicsLegoGraph::validate_part_actor: center of mass "
			          "mismatch for {} (expected {:.6e}, got {:.6e})",
			          pid, com, px_com);
		}
		physx::PxVec3 I_diag = actor->getMassSpaceInertiaTensor();
		physx::PxMat33 I_mass{physx::PxVec3{I_diag.x, 0, 0},
		                      physx::PxVec3{0, I_diag.y, 0},
		                      physx::PxVec3{0, 0, I_diag.z}};
		physx::PxMat33 R(Tcm.q);
		physx::PxMat33 I_actor = R * I_mass * R.getTranspose();
		Eigen::Matrix3d px_inertia_tensor = as<Eigen::Matrix3d>(I_actor);
		if ((px_inertia_tensor - inertia_tensor).norm() > 1e-6) {
			log_error("PhysicsLegoGraph::validate_part_actor: inertia tensor "
			          "mismatch for {} (expected {:.6e}, got {:.6e})",
			          pid, inertia_tensor, px_inertia_tensor);
		}
	}

	physx::PxConstraint *create_constraint(PartId a_id, PartId b_id,
	                                       const Transformd &T_a_b) {
		// PxConstraint shader uses body frames (COM frames) bA2w/bB2w.
		// Our T_a_b (from Python) is defined between actor-local origins (bottom centers).
		// Convert to COM-local frames so the weld aligns the intended anchor points.
		//
		// parentLocal (A_com -> B_com) = (A_com -> A_orig) * (A_orig -> B_orig) * (B_orig -> B_com)
		// childLocal is identity so cB2w = bB2w (B_com).
		physx::PxRigidDynamic *actor_a =
		    topology_.parts().template key_of<physx::PxRigidDynamic *>(a_id);
		physx::PxRigidDynamic *actor_b =
		    topology_.parts().template key_of<physx::PxRigidDynamic *>(b_id);
		Transformd T_a_acom =
		    metrics_.to_m(as<Transformd>(actor_a->getCMassLocalPose()));
		Transformd T_b_bcom =
		    metrics_.to_m(as<Transformd>(actor_b->getCMassLocalPose()));
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
		ComponentData data;
		data.representative = rep;
		initialize_cc_data(cc_id, data);
		return data;
	}

	void update_cc_data(ComponentId cc_id, ComponentData &data) {
		initialize_cc_data(cc_id, data);
	}

	void initialize_cc_data(ComponentId cc_id, ComponentData &cc_data) {
		if (cc_index_.cc_sizes().at(cc_id) <= 1) {
			cc_data.breakage_system = nullptr;
			cc_data.breakage_state = nullptr;
			return;
		}
		cc_data.breakage_system = std::make_unique<BreakageSystem>(
		    breakage_checker_.build_system(topology_, cc_data.representative));
		BreakageInitialInput initial_input =
		    prepare_breakage_input<BreakageInitialInput>(cc_data);
		cc_data.breakage_state = std::make_unique<BreakageState>(
		    breakage_checker_.build_initial_state(*cc_data.breakage_system,
		                                          initial_input));
	}

	std::vector<PendingDisassembly> compute_breakages() {
		std::vector<PendingDisassembly> pending_disassemblies;
		for (const auto &[cc_id, _] : cc_index_.data()) {
			ComponentData &cc_data = cc_index_.data_at(cc_id);
			if (!cc_data.breakage_system || !cc_data.breakage_state) {
				continue;
			}
			BreakageSystem &sys = *cc_data.breakage_system;
			BreakageState &state = *cc_data.breakage_state;
			BreakageInput in = prepare_breakage_input<BreakageInput>(cc_data);
			BreakageSolution sol = breakage_checker_.solve(sys, in, state);
			if (sol.utilization.size() > 0) {
				// Break the most utilized connection
				std::optional<ConnSegId> max_csid;
				double max_u = -1.0;
				for (int i = 0; i < sol.utilization.size(); ++i) {
					double u = sol.utilization(i);
					ConnSegId csid = sys.clutch_ids().at(i);
					auto &csw = topology_.connection_segment_at(csid);
					csw.utilization_ = u;
					if (!max_csid.has_value() || u > max_u) {
						max_csid = csid;
						max_u = u;
					}
				}
				if (max_u > 1.0) {
					pending_disassemblies.emplace_back(*max_csid);
				}
			}
		}
		return pending_disassemblies;
	}

	template <class T>
	    requires std::same_as<T, BreakageInitialInput> ||
	             std::same_as<T, BreakageInput>
	T prepare_breakage_input(const ComponentData &cc_data) {
		T input{};
		const BreakageSystem &sys = *cc_data.breakage_system;
		int num_parts = sys.num_parts();
		input.w.resize(num_parts, 3);
		input.v.resize(num_parts, 3);
		input.q.resize(num_parts, 4);
		input.c.resize(num_parts, 3);
		if constexpr (std::same_as<T, BreakageInput>) {
			input.J.resize(num_parts, 3);
			input.H.resize(num_parts, 3);
			physx::PxRigidDynamic *actor_rep =
			    topology_.parts().template key_of<physx::PxRigidDynamic *>(
			        cc_data.representative);
			physx::PxScene *px_scene = actor_rep->getScene();
			input.dt = getPxSceneElapsedTime(px_scene);
		}

		for (int index = 0; index < num_parts; ++index) {
			PartId pid = sys.part_ids().at(index);
			PartRigidInfo info = part_rigid_info_map_.at(pid);
			input.w.row(index) =
			    metrics_.to_rps(as<Eigen::Vector3d>(info.angular_velocity));
			input.v.row(index) =
			    metrics_.to_mps(as<Eigen::Vector3d>(info.linear_velocity));
			input.q.row(index) =
			    as<Eigen::Quaterniond>(info.body_rot).normalized().coeffs();
			input.c.row(index) =
			    metrics_.to_m(as<Eigen::Vector3d>(info.com_pos));
			if constexpr (std::same_as<T, BreakageInput>) {
				input.J.row(index) =
				    metrics_.to_Ns(as<Eigen::Vector3d>(info.linear_impulse));
				input.H.row(index) =
				    metrics_.to_Nms(as<Eigen::Vector3d>(info.angular_impulse));
			}
		}
		return input;
	}

	void prepare_rigid_info_map() {
		part_rigid_info_map_.clear();
		topology_.parts().for_each([&](const auto &keys) {
			PartId pid =
			    std::get<TopologyGraph::PartKeys::template index_of<PartId>>(
			        keys);
			physx::PxRigidDynamic *actor =
			    std::get<TopologyGraph::PartKeys::template index_of<
			        physx::PxRigidDynamic *>>(keys);
			PartRigidInfo info;
			physx::PxTransform pose = actor->getGlobalPose();
			info.body_rot = pose.q;
			info.com_pos = pose.transform(actor->getCMassLocalPose().p);
			info.linear_velocity = actor->getLinearVelocity();
			info.angular_velocity = actor->getAngularVelocity();
			// Pre-fill impulses with negative momentum
			info.linear_impulse = -compute_linear_momentum(actor);
			info.angular_impulse = -compute_angular_momentum(actor);
			part_rigid_info_map_.emplace(pid, info);
		});
	}

	void populate_rigid_info_map() {
		topology_.parts().for_each([&](const auto &keys) {
			PartId pid =
			    std::get<TopologyGraph::PartKeys::template index_of<PartId>>(
			        keys);
			physx::PxRigidDynamic *actor =
			    std::get<TopologyGraph::PartKeys::template index_of<
			        physx::PxRigidDynamic *>>(keys);
			PartRigidInfo &info = part_rigid_info_map_.at(pid);
			physx::PxTransform pose = actor->getGlobalPose();
			info.body_rot = pose.q;
			info.com_pos = pose.transform(actor->getCMassLocalPose().p);
			info.linear_velocity = actor->getLinearVelocity();
			info.angular_velocity = actor->getAngularVelocity();
			info.linear_impulse += compute_linear_momentum(actor);
			info.angular_impulse += compute_angular_momentum(actor);
		});
		for (const auto &[edge, handle] : constraint_scheduler_.constraints()) {
			const auto &[constraint, T] = handle;
			physx::PxU32 typeID = physx::PxConstraintExtIDs::eINVALID_ID;
			const WeldConstraintData *weld_data =
			    reinterpret_cast<const WeldConstraintData *>(
			        constraint->getExternalReference(typeID));
			if (!weld_data || typeID != kWeldConstraintExtID) {
				log_error("PhysicsLegoGraph::populate_rigid_info_map: "
				          "unexpected constraint external reference");
				continue;
			}
			physx::PxRigidActor *actor_a = nullptr;
			physx::PxRigidActor *actor_b = nullptr;
			constraint->getActors(actor_a, actor_b);
			if (!actor_a || !actor_b) {
				log_error("PhysicsLegoGraph::populate_rigid_info_map: "
				          "constraint has null actors");
				continue;
			}
			physx::PxRigidDynamic *rb_a = actor_a->is<physx::PxRigidDynamic>();
			physx::PxRigidDynamic *rb_b = actor_b->is<physx::PxRigidDynamic>();
			if (!rb_a || !rb_b) {
				log_error("PhysicsLegoGraph::populate_rigid_info_map: "
				          "constraint actors are not RigidDynamic");
				continue;
			}
			auto [pid_a, pid_b] = edge;
			PartRigidInfo &info_a = part_rigid_info_map_.at(pid_a);
			PartRigidInfo &info_b = part_rigid_info_map_.at(pid_b);
			physx::PxReal dt = getPxSceneElapsedTime(constraint->getScene());
			physx::PxTransform bA2w =
			    rb_a->getGlobalPose() * rb_a->getCMassLocalPose();
			physx::PxTransform bB2w =
			    rb_b->getGlobalPose() * rb_b->getCMassLocalPose();
			physx::PxVec3 F0_W, tau0_W_atP;
			constraint->getForce(F0_W, tau0_W_atP);
			physx::PxVec3 P_W = bB2w.transform(weld_data->childLocal).p;
			physx::PxVec3 com0_W = bA2w.p;
			physx::PxVec3 r0_W = P_W - com0_W;
			physx::PxVec3 tau0_W_atCOM0 = tau0_W_atP + r0_W.cross(F0_W);
			physx::PxVec3 F1_W = -F0_W;
			physx::PxVec3 tau1_W_atP = -tau0_W_atP;
			physx::PxVec3 com1_W = bB2w.p;
			physx::PxVec3 r1_W = P_W - com1_W;
			physx::PxVec3 tau1_W_atCOM1 = tau1_W_atP + r1_W.cross(F1_W);
			info_a.linear_impulse -= F0_W * dt;
			info_a.angular_impulse -= tau0_W_atCOM0 * dt;
			info_b.linear_impulse -= F1_W * dt;
			info_b.angular_impulse -= tau1_W_atCOM1 * dt;
		}
	}

	static_assert(TopologyGraph::HasOnPartAddedHook);
	static_assert(TopologyGraph::HasOnPartRemovingHook);
	static_assert(TopologyGraph::HasOnPartRemovedHook);
	static_assert(TopologyGraph::HasOnBundleCreatedHook);
	static_assert(TopologyGraph::HasOnBundleRemovedHook);
	static_assert(TopologyGraph::HasOnConnectedHook);
	static_assert(TopologyGraph::HasOnDisconnectedHook);
};

} // namespace bricksim
