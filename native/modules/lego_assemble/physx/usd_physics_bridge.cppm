export module lego_assemble.physx.usd_physics_bridge;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.core.assembly;
import lego_assemble.physx.physics_graph;
import lego_assemble.usd.usd_graph;
import lego_assemble.usd.author;
import lego_assemble.usd.parse;
import lego_assemble.usd.specs;
import lego_assemble.utils.type_list;
import lego_assemble.utils.multi_key_set;
import lego_assemble.utils.typed_id;
import lego_assemble.utils.poly_store;
import lego_assemble.vendor.omni;
import lego_assemble.vendor.physx;
import lego_assemble.vendor.carb;
import lego_assemble.vendor.pxr;

namespace lego_assemble {

export template <class Parts, class PartAuthors, class PartParsers>
class UsdPhysicsBridge;

export template <class... Ps, class... PAs, class... PPs>
class UsdPhysicsBridge<type_list<Ps...>, type_list<PAs...>, type_list<PPs...>> {
  public:
	using PartTypeList = type_list<Ps...>;
	using PartAuthorList = type_list<PAs...>;
	using PartParserList = type_list<PPs...>;
	using Self = UsdPhysicsBridge<PartTypeList, PartAuthorList, PartParserList>;

	class Hooks;
	using PhysicsGraph = PhysicsLegoGraph<PartTypeList, Hooks>;
	using UsdGraph =
	    UsdLegoGraph<PartTypeList, PartAuthorList, PartParserList, Hooks>;

	class Hooks {
	  private:
		Self *owner_;
		explicit Hooks(Self *owner) : owner_(owner) {}
		~Hooks() = default;
		Hooks(const Hooks &) = delete;
		Hooks &operator=(const Hooks &) = delete;
		Hooks(Hooks &&) = delete;
		Hooks &operator=(Hooks &&) = delete;

		friend Self;
		friend PhysicsGraph;
		friend UsdGraph::TopologyGraph;

		// ==== Listens PhysicsGraph ====

		// Thead safety: called by PhysicsGraph.do_post_step() on USD/Kit thread

		void on_assembled(ConnSegId csid, const ConnSegRef &csref,
		                  const ConnectionSegment &conn_seg) {
			if (owner_->sync_conns_to_usd_) {
				owner_->writeback_conn_creation(csid, csref, conn_seg);
			}
		}

		void on_disassembled(ConnSegId csid) {
			if (owner_->sync_conns_to_usd_) {
				owner_->writeback_conn_removal(csid);
			}
		}

		// ==== Listens UsdGraph ====

		// Thead safety: called on USD/Kit thread

		void
		on_connected(ConnSegId csid, const ConnSegRef &csref,
		             [[maybe_unused]] const InterfaceSpec &stud_spec,
		             [[maybe_unused]] const InterfaceSpec &hole_spec,
		             SimpleWrapper<ConnectionSegment> &csw,
		             [[maybe_unused]] SimpleWrapper<ConnectionBundle> &cbw) {
			if (owner_->suppress_usd_callbacks_) {
				return;
			}
			if (owner_->sync_conns_to_physics_) {
				owner_->bind_connection(csid, csref, csw);
			}
		}

		void on_disconnecting(
		    ConnSegId csid, [[maybe_unused]] const ConnSegRef &csref,
		    [[maybe_unused]] SimpleWrapper<ConnectionSegment> &csw,
		    [[maybe_unused]] SimpleWrapper<ConnectionBundle> &cbw) {
			if (owner_->suppress_usd_callbacks_) {
				return;
			}
			if (owner_->sync_conns_to_physics_) {
				owner_->unbind_connection(csid);
			}
		}
	};

	using PhysicsPartId = TypedId<struct PhysicsPartTag, PartId>;
	using UsdPartId = TypedId<struct UsdPartTag, PartId>;
	using PhysicsConnSegId = TypedId<struct PhysicsConnSegTag, ConnSegId>;
	using UsdConnSegId = TypedId<struct UsdConnSegTag, ConnSegId>;

	using PartIdMapping = MultiKeySet<type_list<PhysicsPartId, UsdPartId>>;
	using ConnSegIdMapping =
	    MultiKeySet<type_list<PhysicsConnSegId, UsdConnSegId>>;

	explicit UsdPhysicsBridge(
	    PhysicsGraph *physics_graph, UsdGraph *usd_graph,
	    omni::physx::IPhysx *omni_px, bool sync_conns_to_usd = true,
	    bool sync_conns_to_physics = true, bool warn_divergence = true,
	    AlignPolicy align_policy = AlignPolicy::MoveHoleCC)
	    : hooks_{this}, physics_graph_{physics_graph}, usd_graph_{usd_graph},
	      omni_px_{omni_px}, sync_conns_to_usd_{sync_conns_to_usd},
	      sync_conns_to_physics_{sync_conns_to_physics},
	      warn_divergence_{warn_divergence}, align_policy_{align_policy} {
		if (physics_graph_->get_hooks() != nullptr) [[unlikely]] {
			throw std::runtime_error(
			    "UsdPhysicsBridge: Physics graph already has hooks set");
		}
		if (usd_graph_->get_hooks() != nullptr) [[unlikely]] {
			throw std::runtime_error(
			    "UsdPhysicsBridge: USD graph already has hooks set");
		}
		initial_sync();
		physics_graph_->set_hooks(&hooks_);
		usd_graph_->set_hooks(&hooks_);
		physx_obj_sub_ = omni_px_->subscribeObjectChangeNotifications({
		    // Thead safety: called on USD/Kit thread
		    .objectCreationNotifyFn =
		        std::bind_front(&Self::on_object_creation_notify, this),
		    .objectDestructionNotifyFn =
		        std::bind_front(&Self::on_object_destruction_notify, this),
		    .allObjectsDestructionNotifyFn =
		        std::bind_front(&Self::on_all_objects_destruction_notify, this),
		    .userData = nullptr,
		    .stopCallbackWhenSimStopped = false,
		});
	}
	~UsdPhysicsBridge() {
		tear_down();
	}
	UsdPhysicsBridge(const UsdPhysicsBridge &) = delete;
	UsdPhysicsBridge &operator=(const UsdPhysicsBridge &) = delete;
	UsdPhysicsBridge(UsdPhysicsBridge &&) = delete;
	UsdPhysicsBridge &operator=(UsdPhysicsBridge &&) = delete;

	const PartIdMapping &part_mapping() const {
		return pid_mapping_;
	}

	const ConnSegIdMapping &connection_mapping() const {
		return csid_mapping_;
	}

	bool is_active() const {
		return active_;
	}

  private:
	Hooks hooks_;
	PhysicsGraph *physics_graph_;
	UsdGraph *usd_graph_;
	omni::physx::IPhysx *omni_px_;
	bool sync_conns_to_usd_;
	bool sync_conns_to_physics_;
	bool warn_divergence_;
	AlignPolicy align_policy_;
	PartIdMapping pid_mapping_;
	ConnSegIdMapping csid_mapping_;
	std::optional<omni::physx::SubscriptionId> physx_obj_sub_;
	bool active_ = true;

	std::size_t suppress_usd_callbacks_ = 0;
	class SuppressUsdCallbacks {
	  public:
		explicit SuppressUsdCallbacks(Self *owner)
		    : counter_(owner->suppress_usd_callbacks_) {
			counter_++;
		}
		~SuppressUsdCallbacks() {
			counter_--;
		}
		SuppressUsdCallbacks(const SuppressUsdCallbacks &) = delete;
		SuppressUsdCallbacks &operator=(const SuppressUsdCallbacks &) = delete;
		SuppressUsdCallbacks(SuppressUsdCallbacks &&) = delete;
		SuppressUsdCallbacks &operator=(SuppressUsdCallbacks &&) = delete;

	  private:
		std::size_t &counter_;
	};

	bool writeback_conn_creation(ConnSegId physics_csid,
	                             const ConnSegRef &physics_csref,
	                             const ConnectionSegment &conn_seg) {
		// Write a created connection in PhysicsGraph back to UsdGraph.
		PhysicsConnSegId t_physics_csid{physics_csid};
		if (csid_mapping_.contains(t_physics_csid)) [[unlikely]] {
			log_error("UsdPhysicsBridge: physics conn seg id {} is already "
			          "mapped during connection creation writeback",
			          physics_csid);
			return false;
		}
		const auto &[physics_stud_ref, physics_hole_ref] = physics_csref;
		const auto &[physics_stud_pid, stud_ifid] = physics_stud_ref;
		const auto &[physics_hole_pid, hole_ifid] = physics_hole_ref;
		PhysicsPartId t_physics_stud_pid{physics_stud_pid};
		PhysicsPartId t_physics_hole_pid{physics_hole_pid};
		const UsdPartId *t_usd_stud_pid_ptr =
		    pid_mapping_.template project<PhysicsPartId, UsdPartId>(
		        t_physics_stud_pid);
		const UsdPartId *t_usd_hole_pid_ptr =
		    pid_mapping_.template project<PhysicsPartId, UsdPartId>(
		        t_physics_hole_pid);
		if (!t_usd_stud_pid_ptr || !t_usd_hole_pid_ptr) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to find USD part ids for "
			          "physics part ids {} and {} during connection "
			          "creation writeback",
			          physics_stud_pid, physics_hole_pid);
			return false;
		}
		PartId usd_stud_pid = t_usd_stud_pid_ptr->value();
		PartId usd_hole_pid = t_usd_hole_pid_ptr->value();
		SuppressUsdCallbacks _suppress_cbk{this};
		auto usd_conn_opt = usd_graph_->connect({usd_stud_pid, stud_ifid},
		                                        {usd_hole_pid, hole_ifid},
		                                        conn_seg, align_policy_);
		if (!usd_conn_opt) {
			// This happens on graph divergence
			if (warn_divergence_) {
				log_warn("UsdPhysicsBridge: failed to add connection for "
				         "USD part ids {} and {} during connection creation "
				         "writeback",
				         usd_stud_pid, usd_hole_pid);
			}
			return false;
		}
		const auto &[usd_csid, usd_conn_path] = *usd_conn_opt;
		UsdConnSegId t_usd_csid{usd_csid};
		if (!csid_mapping_.emplace(t_physics_csid, t_usd_csid)) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to map physics conn seg id {} "
			          "and USD conn seg id {} during connection creation "
			          "writeback",
			          physics_csid, usd_csid);
			// Fallthrough
		}
		return true;
	}

	bool writeback_conn_removal(ConnSegId physics_csid) {
		// Delete a removed connection in PhysicsGraph from UsdGraph.
		PhysicsConnSegId t_physics_csid{physics_csid};
		const UsdConnSegId *t_usd_csid_ptr =
		    csid_mapping_.template project<PhysicsConnSegId, UsdConnSegId>(
		        t_physics_csid);
		if (!t_usd_csid_ptr) {
			// This happens on graph divergence
			if (warn_divergence_) {
				log_warn("UsdPhysicsBridge: failed to find USD conn seg id for "
				         "physics conn seg id {} during connection removal "
				         "writeback",
				         physics_csid);
			}
			return false;
		}
		ConnSegId usd_csid = t_usd_csid_ptr->value();
		const pxr::SdfPath *usd_conn_path_ptr =
		    usd_graph_->topology()
		        .connection_segments()
		        .template project<ConnSegId, pxr::SdfPath>(usd_csid);
		if (!usd_conn_path_ptr) {
			log_error("UsdPhysicsBridge: failed to find USD conn path for "
			          "USD conn seg id {} during connection removal "
			          "writeback",
			          usd_csid);
			return false;
		}
		pxr::SdfPath usd_conn_path = *usd_conn_path_ptr;
		SuppressUsdCallbacks _suppress_cbk{this};
		bool disconnected = usd_graph_->disconnect(usd_conn_path);
		if (!disconnected) {
			// This happens when it's not a managed connection
			// Fallthrough
		}
		// Remove from mapping
		bool erased = csid_mapping_.erase_by_key(t_physics_csid);
		if (!erased) {
			log_error("UsdPhysicsBridge: failed to erase mapping for "
			          "physics conn seg id {} during connection removal "
			          "writeback",
			          physics_csid);
			// Fallthrough
		}
		return true;
	}

	std::vector<InterfaceShapePair> resolve_collider_shapes(
	    std::span<const InterfaceColliderPair> collider_paths) {
		std::vector<InterfaceShapePair> shapes;
		shapes.reserve(collider_paths.size());
		for (const auto &[if_id, path] : collider_paths) {
			physx::PxShape *px_shape = static_cast<physx::PxShape *>(
			    omni_px_->getPhysXPtr(path, omni::physx::ePTShape));
			if (px_shape == nullptr) [[unlikely]] {
				log_error("UsdPhysicsBridge: failed to get PxShape for "
				          "collider path {}",
				          path.GetText());
				continue;
			}
			shapes.emplace_back(if_id, px_shape);
		}
		return shapes;
	}

	physx::PxRigidActor *resolve_rigid_actor(const pxr::SdfPath &part_path) {
		return static_cast<physx::PxRigidActor *>(
		    omni_px_->getPhysXPtr(part_path, omni::physx::ePTActor));
	}

	template <PartLike P>
	bool bind_part(PartId usd_pid, const UsdPartWrapper<P> &usd_pw,
	               physx::PxRigidActor *px_actor) {
		// Bind a USD part to PhysicsGraph. The corresponding actor must already exist.
		UsdPartId t_usd_pid{usd_pid};
		if (pid_mapping_.contains(t_usd_pid)) [[unlikely]] {
			log_error("UsdPhysicsBridge: USD part id {} is already bound",
			          usd_pid);
			return false;
		}
		const P &part = usd_pw.wrapped();
		auto colliders = resolve_collider_shapes(usd_pw.colliders());
		std::optional<PartId> physics_pid_opt =
		    physics_graph_->topology().template add_part<P>(
		        std::forward_as_tuple(px_actor), std::move(colliders), part);
		if (!physics_pid_opt) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to add part id {} to physics "
			          "graph",
			          usd_pid);
			return false;
		}
		PartId physics_pid = *physics_pid_opt;
		PhysicsPartId t_physics_pid{physics_pid};
		if (!pid_mapping_.emplace(t_physics_pid, t_usd_pid)) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to map physics part id {} and "
			          "USD part id {}",
			          physics_pid, usd_pid);
			// Fallthrough
		}

		// Connect relevant connections
		for (ConnSegId csid : usd_pw.outgoings()) {
			bind_connection(csid);
		}
		for (ConnSegId csid : usd_pw.incomings()) {
			bind_connection(csid);
		}
		return true;
	}

	bool unbind_part(PartId physics_pid) {
		// Unbind a part in PhysicsGraph.
		// First, remove all connections in bookkeeping table.
		// No need to remove connections from PhysicsGraph here,
		// remove_part(...) will do that.
		auto remove_conn_entry = [&](ConnSegId physics_csid) {
			PhysicsConnSegId t_physics_csid{physics_csid};
			[[maybe_unused]] bool erased =
			    csid_mapping_.erase_by_key(t_physics_csid);
			// Return value can be false if the entry doesn't exist,
			// this happens on graph divergence.
			if (!erased) {
				if (warn_divergence_) {
					log_warn("UsdPhysicsBridge: failed to erase mapping for "
					         "physics conn seg id {} during unbind",
					         physics_csid);
				}
			}
		};
		bool visited = physics_graph_->topology().parts().visit(
		    physics_pid, [&]<PartLike P>(const PhysicsPartWrapper<P> &pw) {
			    for (ConnSegId csid : pw.outgoings()) {
				    remove_conn_entry(csid);
			    }
			    for (ConnSegId csid : pw.incomings()) {
				    remove_conn_entry(csid);
			    }
		    });
		if (!visited) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to find physics part id {} "
			          "during unbind",
			          physics_pid);
			return false;
		}
		// Then, remove the part itself
		bool removed =
		    physics_graph_->topology().remove_part(physics_pid).has_value();
		if (!removed) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to remove physics part id {} "
			          "during unbind",
			          physics_pid);
			// Fallthrough
		}
		// Finally, remove from mapping
		PhysicsPartId t_physics_pid{physics_pid};
		bool erased = pid_mapping_.erase_by_key(t_physics_pid);
		if (!erased) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to erase mapping for physics "
			          "part id {} during unbind",
			          physics_pid);
			// Fallthrough
		}
		return true;
	}

	bool bind_connection(ConnSegId usd_csid) {
		const ConnSegRef *usd_csref_ptr =
		    usd_graph_->topology()
		        .connection_segments()
		        .template project<ConnSegId, ConnSegRef>(usd_csid);
		if (usd_csref_ptr == nullptr) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to find ConnSegRef for USD "
			          "conn seg id {}",
			          usd_csid);
			return false;
		}
		const ConnSegRef &usd_csref = *usd_csref_ptr;
		const SimpleWrapper<ConnectionSegment> *usd_csw_ptr =
		    usd_graph_->topology().connection_segments().find(usd_csid);
		if (usd_csw_ptr == nullptr) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to find ConnectionSegment "
			          "wrapper for USD conn seg id {}",
			          usd_csid);
			return false;
		}
		const SimpleWrapper<ConnectionSegment> &usd_csw = *usd_csw_ptr;
		return bind_connection(usd_csid, usd_csref, usd_csw);
	}

	bool bind_connection(ConnSegId usd_csid, const ConnSegRef &usd_csref,
	                     const SimpleWrapper<ConnectionSegment> &usd_csw) {
		// Bind a USD connection segment to PhysicsGraph.
		// Requires both endpoint parts to be already bound.
		UsdConnSegId t_usd_csid{usd_csid};
		if (csid_mapping_.contains(t_usd_csid)) [[unlikely]] {
			log_error("UsdPhysicsBridge: USD conn seg id {} is already bound",
			          usd_csid);
			return false;
		}
		const auto &[usd_stud_ref, usd_hole_ref] = usd_csref;
		const auto &[usd_stud_pid, stud_ifid] = usd_stud_ref;
		const auto &[usd_hole_pid, hole_ifid] = usd_hole_ref;
		UsdPartId t_usd_stud_pid{usd_stud_pid};
		UsdPartId t_usd_hole_pid{usd_hole_pid};
		const PhysicsPartId *t_physics_stud_pid =
		    pid_mapping_.template project<UsdPartId, PhysicsPartId>(
		        t_usd_stud_pid);
		const PhysicsPartId *t_physics_hole_pid =
		    pid_mapping_.template project<UsdPartId, PhysicsPartId>(
		        t_usd_hole_pid);
		if (t_physics_stud_pid == nullptr || t_physics_hole_pid == nullptr) {
			// Endpoint parts not bound yet
			return false;
		}
		std::optional<ConnSegId> physics_csid_opt =
		    physics_graph_->topology().connect(
		        {t_physics_stud_pid->value(), stud_ifid},
		        {t_physics_hole_pid->value(), hole_ifid}, {},
		        usd_csw.wrapped());
		if (!physics_csid_opt) {
			// Failed to connect. This could happen when two graphs diverge.
			if (warn_divergence_) {
				log_warn("UsdPhysicsBridge: failed to add connection seg id {} "
				         "to physics graph",
				         usd_csid);
			}
			return false;
		}
		ConnSegId physics_csid = *physics_csid_opt;
		PhysicsConnSegId t_physics_csid{physics_csid};
		if (!csid_mapping_.emplace(t_physics_csid, t_usd_csid)) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to map physics conn seg id {} "
			          "and USD conn seg id {}",
			          physics_csid, usd_csid);
			// Fallthrough
		}
		return true;
	}

	bool unbind_connection(ConnSegId usd_csid) {
		// Unbind a USD connection from PhysicsGraph.
		UsdConnSegId t_usd_csid{usd_csid};
		const PhysicsConnSegId *t_physics_csid_ptr =
		    csid_mapping_.template project<UsdConnSegId, PhysicsConnSegId>(
		        t_usd_csid);
		if (t_physics_csid_ptr == nullptr) {
			// The connection is not bound at all.
			// This could happen on graph divergence.
			if (warn_divergence_) {
				log_warn("UsdPhysicsBridge: USD conn seg id {} is not bound "
				         "during unbind",
				         usd_csid);
			}
			return false;
		}
		ConnSegId physics_csid = t_physics_csid_ptr->value();
		bool disconnected =
		    physics_graph_->topology().disconnect(physics_csid).has_value();
		if (!disconnected) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to disconnect physics conn "
			          "seg id {} during unbind of USD conn seg id {}",
			          physics_csid, usd_csid);
			// Fallthrough
		}
		bool erased = csid_mapping_.erase_by_key(t_usd_csid);
		if (!erased) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to erase mapping for USD "
			          "conn seg id {} during unbind",
			          usd_csid);
			// Fallthrough
		}
		return true;
	}

	void initial_sync() {
		(
		    [&]<PartLike P>(
		        const pmr_vector_storage<UsdPartWrapper<P>, PartId> &storage) {
			    for (const auto &[pid, pw] :
			         std::views::zip(storage.ids, storage.data)) {
				    const pxr::SdfPath *part_path_ptr =
				        usd_graph_->topology()
				            .parts()
				            .template project_key<PartId, pxr::SdfPath>(pid);
				    if (!part_path_ptr) [[unlikely]] {
					    log_error("UsdPhysicsBridge: failed to find part path "
					              "for USD part id {} during initial sync",
					              pid);
					    continue;
				    }
				    pxr::SdfPath part_path = *part_path_ptr;
				    physx::PxRigidActor *px_actor =
				        resolve_rigid_actor(part_path);
				    if (px_actor == nullptr) {
					    // Omni PhysX hasn't created the actor yet
					    continue;
				    }
				    bind_part<P>(pid, pw, px_actor);
			    }
		    }(usd_graph_->topology()
		          .parts()
		          .template storage_for<UsdPartWrapper<Ps>>()),
		    ...);
	}

	void on_object_creation_notify(const pxr::SdfPath &sdf_path,
	                               omni::physx::usdparser::ObjectId object_id,
	                               omni::physx::PhysXType type,
	                               [[maybe_unused]] void *user_data) {
		// If this is a rigid actor for a part, we can bind it.
		if (type != omni::physx::ePTActor) {
			return;
		}
		const PartId *usd_pid_ptr =
		    usd_graph_->topology()
		        .parts()
		        .template project_key<pxr::SdfPath, PartId>(sdf_path);
		if (usd_pid_ptr == nullptr) {
			return;
		}
		physx::PxRigidActor *px_actor = static_cast<physx::PxRigidActor *>(
		    omni_px_->getPhysXPtrFast(object_id));
		PartId usd_pid = *usd_pid_ptr;
		bool visited = usd_graph_->topology().parts().visit(
		    usd_pid, [&]<PartLike P>(const UsdPartWrapper<P> &pw) {
			    bind_part(usd_pid, pw, px_actor);
		    });
		if (!visited) [[unlikely]] {
			log_error("UsdPhysicsBridge: failed to find USD part id {} "
			          "during object creation notify",
			          usd_pid);
		}
	}

	void
	on_object_destruction_notify([[maybe_unused]] const pxr::SdfPath &sdf_path,
	                             omni::physx::usdparser::ObjectId object_id,
	                             omni::physx::PhysXType type,
	                             [[maybe_unused]] void *userData) {
		// If this is a rigid actor for a bound part, we need to unbind it.
		if (type != omni::physx::ePTActor) {
			return;
		}
		physx::PxRigidActor *px_actor = static_cast<physx::PxRigidActor *>(
		    omni_px_->getPhysXPtrFast(object_id));
		const PartId *physics_pid_ptr =
		    physics_graph_->topology()
		        .parts()
		        .template project_key<physx::PxRigidActor *, PartId>(px_actor);
		if (physics_pid_ptr == nullptr) {
			return;
		}
		PartId physics_pid = *physics_pid_ptr;
		unbind_part(physics_pid);
	}

	void on_all_objects_destruction_notify([[maybe_unused]] void *user_data) {
		tear_down();
	}

	void tear_down() {
		if (physx_obj_sub_.has_value()) {
			omni_px_->unsubscribeObjectChangeNotifications(*physx_obj_sub_);
			physx_obj_sub_.reset();
		}
		if (usd_graph_->get_hooks() == &hooks_) {
			usd_graph_->set_hooks(nullptr);
		}
		if (physics_graph_->get_hooks() == &hooks_) {
			physics_graph_->set_hooks(nullptr);
		}
		pid_mapping_.clear();
		csid_mapping_.clear();
		active_ = false;
	}

	static_assert(PhysicsGraph::HasOnAssembledHook);
	static_assert(PhysicsGraph::HasOnDisassembledHook);
	static_assert(UsdGraph::HasOnConnectedHook);
	static_assert(UsdGraph::HasOnDisconnectingHook);
};

} // namespace lego_assemble
