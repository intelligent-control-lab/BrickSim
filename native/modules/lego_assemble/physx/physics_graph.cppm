export module lego_assemble.physx.physics_graph;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.core.assembly;
import lego_assemble.physx.weld_constraint;
import lego_assemble.physx.scene_patcher;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;
import lego_assemble.utils.skip_graph;
import lego_assemble.utils.transforms;
import lego_assemble.utils.conversions;
import lego_assemble.utils.multi_key_map;
import lego_assemble.utils.pair;
import lego_assemble.utils.unordered_pair;
import lego_assemble.utils.metric_system;
import lego_assemble.vendor.physx;
import lego_assemble.vendor.eigen;

namespace lego_assemble {

export struct PendingAssembly {
	ConnSegRef csref;
	ConnectionSegment conn_seg;
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
	    std::initializer_list<InterfaceShapePair> if_shapes, Args &&...args)
	    : SimplePartWrapper<P>(std::forward<Args>(args)...),
	      interface_shapes_{if_shapes, tail_mr(args...)} {}

	std::span<const InterfaceShapePair> interface_shapes() const {
		return interface_shapes_;
	}

  private:
	std::pmr::vector<InterfaceShapePair> interface_shapes_;
};

export struct PhysicsConnectionSegmentWrapper
    : SimpleWrapper<ConnectionSegment> {
	ActorShapePair px_stud_shape{nullptr, nullptr};
	ActorShapePair px_hole_shape{nullptr, nullptr};
	template <class... Args>
	explicit PhysicsConnectionSegmentWrapper(Args &&...args)
	    : SimpleWrapper<ConnectionSegment>(std::forward<Args>(args)...) {}
};

export struct PhysicsConnectionBundleWrapper : SimpleWrapper<ConnectionBundle> {
	physx::PxConstraint *px_constraint{nullptr};

	template <class... Args>
	explicit PhysicsConnectionBundleWrapper(Args &&...args)
	    : SimpleWrapper<ConnectionBundle>(std::forward<Args>(args)...) {}
};

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

export template <PartLike... Ps> class PhysicsLegoGraph {
  public:
	using Self = PhysicsLegoGraph<Ps...>;

  private:
	struct TopologyHooks {
		Self *owner_;

		TopologyHooks(Self *owner) : owner_{owner} {}

		template <class P>
		void on_part_added(PartId pid, PhysicsPartWrapper<P> &pw) {
			// Update shape <-> interface mapping
			physx::PxRigidActor *px_actor =
			    owner_->topology_.parts()
			        .template project_key<PartId, physx::PxRigidActor *>(pid);
			if (!px_actor) {
				throw std::runtime_error(std::format(
				    "Part id {} has no associated PxRigidActor", pid));
			}
			for (const auto &[if_id, shape] : pw.interface_shapes()) {
				if (shape == nullptr) {
					throw std::runtime_error(std::format(
					    "Interface {} of part id {} has null PxShape", if_id,
					    pid));
				}
				if (!owner_->shape_mapping_.emplace(
				        {{pid, if_id}, {px_actor, shape}},
				        get_interface_at(pw.wrapped(), if_id))) {
					throw std::runtime_error(std::format(
					    "Failed to map shape to interface {} of part id {}",
					    if_id, pid));
				}
			}
		}

		template <class P>
		void on_part_removing(PartId pid, PhysicsPartWrapper<P> &pw) {
			// Update shape <-> interface mapping
			for (const auto &[if_id, shape] : pw.interface_shapes()) {
				if (!owner_->shape_mapping_.erase_by_key<InterfaceRef>(
				        {pid, if_id})) {
					throw std::runtime_error(std::format(
					    "Failed to unmap shape from interface {} of part id {}",
					    if_id, pid));
				}
			}
		}

		void on_connected(ConnSegId csid, const ConnSegRef &csref,
		                  const InterfaceSpec &stud_spec,
		                  const InterfaceSpec &hole_spec,
		                  PhysicsConnectionSegmentWrapper &csw,
		                  PhysicsConnectionBundleWrapper &cbw) {
			const auto &[stud_if_ref, hole_if_ref] = csref;

			// Set up PhysX shapes for connection segment
			if (auto stud_shape_mapping =
			        owner_->shape_mapping_
			            .project<InterfaceRef, ActorShapePair>(stud_if_ref)) {
				csw.px_stud_shape = *stud_shape_mapping;
			}
			if (auto hole_shape_mapping =
			        owner_->shape_mapping_
			            .project<InterfaceRef, ActorShapePair>(hole_if_ref)) {
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

		void on_disconnecting(ConnSegId csid, const ConnSegRef &csref,
		                      PhysicsConnectionSegmentWrapper &csw,
		                      PhysicsConnectionBundleWrapper &cbw) {
			// Remove contact exclusion
			if (csw.px_stud_shape != NullActorShapePair &&
			    csw.px_hole_shape != NullActorShapePair) {
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
		}

		void on_bundle_created(const ConnectionEndpoint &ep,
		                       PhysicsConnectionBundleWrapper &cbw) {
			auto [a_id, b_id] = ep;

			// Create PhysX constraint
			const auto &T_a_b = cbw.wrapped().T_a_b;
			cbw.px_constraint = owner_->create_constraint(a_id, b_id, T_a_b);

			// Update skip graph
			if (!owner_->skip_graph_.connect(a_id, b_id)) {
				throw std::runtime_error(std::format(
				    "Failed to connect parts {} and {} in skip graph "
				    "during bundle creation",
				    a_id, b_id));
			}
		}

		void on_bundle_removing(const ConnectionEndpoint &ep,
		                        PhysicsConnectionBundleWrapper &cbw) {
			auto [a_id, b_id] = ep;

			// Destroy PhysX constraint
			owner_->destroy_constraint(cbw.px_constraint);
			cbw.px_constraint = nullptr;

			// Update skip graph
			if (!owner_->skip_graph_.disconnect(a_id, b_id)) {
				throw std::runtime_error(
				    std::format("Failed to disconnect parts {} and {} in skip "
				                "graph during bundle removal",
				                a_id, b_id));
			}
		}
	};

	class PhysxBinding : private PxSimulationFilterCallbackProxy,
	                     private PxSimulationEventCallbackProxy {
	  public:
		explicit PhysxBinding(Self *owner, physx::PxScene *px_scene)
		    : owner_{owner}, px_scene_{px_scene} {
			// Register callbacks

			if (!patchPxScene(px_scene_))
				goto err_patchPxScene;

			if (!setPxSimulationFilterCallback(this))
				goto err_setPxSimulationFilterCallback;

			if (!setPxSimulationEventCallback(this))
				goto err_setPxSimulationEventCallback;

			return;

		err_setPxSimulationEventCallback:

			clearPxSimulationFilterCallback();
		err_setPxSimulationFilterCallback:

			unpatchPxScene(px_scene_);
		err_patchPxScene:
			throw std::runtime_error(
			    "PhysxBinding: failed to set up PhysX simulation callbacks");
		}
		~PhysxBinding() {
			unpatchPxScene(true);
		}
		PhysxBinding(const PhysxBinding &) = delete;
		PhysxBinding &operator=(const PhysxBinding &) = delete;

		virtual physx::PxFilterFlags pairFound(
		    physx::PxU64 pairID, physx::PxFilterObjectAttributes attributes0,
		    physx::PxFilterData filterData0, const physx::PxActor *a0,
		    const physx::PxShape *s0,
		    physx::PxFilterObjectAttributes attributes1,
		    physx::PxFilterData filterData1, const physx::PxActor *a1,
		    const physx::PxShape *s1, physx::PxPairFlags &pairFlags) override {

			physx::PxFilterFlags result =
			    PxSimulationFilterCallbackProxy::pairFound(
			        pairID, attributes0, filterData0, a0, s0, attributes1,
			        filterData1, a1, s1, pairFlags);

			// Only if both actors are rigid actors
			auto *rb0 = cast_rigid_actor(const_cast<physx::PxActor *>(a0));
			if (rb0 == nullptr) {
				return result;
			}
			auto *rb1 = cast_rigid_actor(const_cast<physx::PxActor *>(a1));
			if (rb1 == nullptr) {
				return result;
			}

			// Check contact exclusion
			ContactExclusionPair exclusion_pair{
			    {rb0, const_cast<physx::PxShape *>(s0)},
			    {rb1, const_cast<physx::PxShape *>(s1)}};
			if (owner_->contact_exclusions_.contains(exclusion_pair)) {
				return physx::PxFilterFlag::eKILL;
			}

			// Enable contact pose report collision between parts
			bool is_part0 =
			    owner_->topology_.parts().template alive<physx::PxRigidActor *>(
			        rb0);
			bool is_part1 =
			    owner_->topology_.parts().template alive<physx::PxRigidActor *>(
			        rb1);
			if (is_part0 && is_part1) {
				pairFlags |= physx::PxPairFlag::eCONTACT_EVENT_POSE;
			}
			return result;
		}

		virtual void onContact(const physx::PxContactPairHeader &header,
		                       const physx::PxContactPair *pairs,
		                       physx::PxU32 nbPairs) override {

			PxSimulationEventCallbackProxy::onContact(header, pairs, nbPairs);

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

			physx::PxReal dt = getElapsedTime(px_scene_);

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
	static_assert(TopologyGraph::HasAllOnPartAddedHooks);
	static_assert(TopologyGraph::HasAllOnPartRemovingHooks);
	static_assert(TopologyGraph::HasOnConnectedHook);
	static_assert(TopologyGraph::HasOnBundleCreatedHook);
	static_assert(TopologyGraph::HasOnBundleRemovingHook);

	using SkipGraph = SimpleSkipGraphScheduler<
	    PartId, physx::PxConstraint *,
	    std::function<physx::PxConstraint *(PartId, PartId)>,
	    std::function<void(physx::PxConstraint *)>>;
	using ShapeMapping =
	    MultiKeyMap<type_list<InterfaceRef, ActorShapePair>, InterfaceSpec,
	                type_list<InterfaceRefHash, ActorShapePairHash>>;

	explicit PhysicsLegoGraph(
	    const MetricSystem &metrics, physx::PxPhysics *px,
	    std::pmr::memory_resource *mr = std::pmr::get_default_resource())
	    : res_{mr}, metrics_{metrics}, px_{px}, hooks_{this},
	      topology_{&hooks_, mr},
	      skip_graph_{std::bind(&PhysicsLegoGraph::create_aux_constraint, this,
	                            std::placeholders::_1, std::placeholders::_2),
	                  std::bind(&PhysicsLegoGraph::destroy_constraint, this,
	                            std::placeholders::_1),
	                  mr},
	      shape_mapping_{mr}, contact_exclusions_{mr}, pending_assemblies_{mr} {
	}
	~PhysicsLegoGraph() = default;
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

	std::pmr::vector<PendingAssembly> drain_pending_assemblies() {
		std::pmr::vector<PendingAssembly> batch(res_);
		{
			std::lock_guard lock{pending_assemblies_mutex_};
			batch.swap(pending_assemblies_);
		}
		return batch;
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

  private:
	std::pmr::memory_resource *res_;
	MetricSystem metrics_;
	physx::PxPhysics *px_;
	TopologyHooks hooks_;
	TopologyGraph topology_;
	SkipGraph skip_graph_;
	ShapeMapping shape_mapping_;
	ContactExclusionSet contact_exclusions_;
	std::unique_ptr<PhysxBinding> physx_binding_;
	AssemblyChecker assembly_checker_;
	std::pmr::vector<PendingAssembly> pending_assemblies_;
	std::mutex pending_assemblies_mutex_;

	void enqueue_pending_assembly(auto &&...args) {
		std::lock_guard lock{pending_assemblies_mutex_};
		pending_assemblies_.emplace_back(std::forward<decltype(args)>(args)...);
	}

	physx::PxConstraint *create_aux_constraint(PartId a_id, PartId b_id) {
		std::optional<Transformd> T_a_b =
		    topology_.lookup_transform(a_id, b_id);
		if (!T_a_b.has_value()) {
			throw std::runtime_error(
			    std::format("Cannot find graph transform from part id {} to {}",
			                a_id, b_id));
		}
		return create_constraint(a_id, b_id, *T_a_b);
	}

	physx::PxConstraint *create_constraint(PartId a_id, PartId b_id,
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

		const auto &[q, t] = metrics_.from_m(T_a_b); // Unit conversion
		auto T_a_b_px = as<physx::PxTransform>(q, t);
		physx::PxTransform A_o2com(physx::PxIdentity);
		physx::PxTransform B_o2com(physx::PxIdentity);
		if (auto *rbA = actor_a->is<physx::PxRigidBody>()) {
			A_o2com = rbA->getCMassLocalPose();
		}
		if (auto *rbB = actor_b->is<physx::PxRigidBody>()) {
			B_o2com = rbB->getCMassLocalPose();
		}
		physx::PxTransform A_com2o = A_o2com.getInverse();
		physx::PxTransform parentLocal = A_com2o * T_a_b_px * B_o2com;
		physx::PxTransform childLocal(physx::PxIdentity);
		return createWeldConstraint(*px_, actor_a, actor_b,
		                            {
		                                .parentLocal = parentLocal,
		                                .childLocal = childLocal,
		                            });
	}

	void destroy_constraint(physx::PxConstraint *constraint) {
		if (!constraint) {
			throw std::runtime_error("Cannot destroy null constraint");
		}
		constraint->release();
	}
};

} // namespace lego_assemble
