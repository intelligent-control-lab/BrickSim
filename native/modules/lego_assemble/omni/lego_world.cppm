export module lego_assemble.omni.lego_world;

import std;
import lego_assemble.core.specs;
import lego_assemble.physx.assembly;
import lego_assemble.physx.breakage;
import lego_assemble.physx.physics_graph;
import lego_assemble.usd.usd_graph;
import lego_assemble.usd.author;
import lego_assemble.usd.parse;
import lego_assemble.usd.specs;
import lego_assemble.omni.usd_physics_bridge;
import lego_assemble.utils.type_list;
import lego_assemble.utils.metric_system;
import lego_assemble.utils.logging;
import lego_assemble.vendor;

namespace lego_assemble {

export struct LegoConfig {
	bool sync_conns_to_usd{true};
	bool sync_conns_to_physics{true};
	bool warn_divergence{false};
	AlignPolicy align_policy{AlignPolicy::MoveHoleCC};
	AssemblyThresholds assembly_thresholds{};
	BreakageThresholds breakage_thresholds{
	    .Enabled = false, // disable now because it's still experimental
	};
	std::string breakage_debug_dump_dir{};
};

export template <class Parts, class PartAuthors, class PartParsers>
class LegoWorld;

export template <class... Ps, class... PAs, class... PPs>
class LegoWorld<type_list<Ps...>, type_list<PAs...>, type_list<PPs...>> {
  public:
	using PartTypeList = type_list<Ps...>;
	using PartAuthorList = type_list<PAs...>;
	using PartParserList = type_list<PPs...>;
	using Self = LegoWorld<PartTypeList, PartAuthorList, PartParserList>;

	using Bridge =
	    UsdPhysicsBridge<PartTypeList, PartAuthorList, PartParserList>;
	using PhysicsGraph = Bridge::PhysicsGraph;
	using UsdGraph = Bridge::UsdGraph;

	explicit LegoWorld(pxr::UsdStageRefPtr stage, LegoConfig cfg = {})
	    : stage_{std::move(stage)}, cfg_{std::move(cfg)} {
		omni_px_ = carb::getCachedInterface<omni::physx::IPhysx>();
		if (!omni_px_) {
			throw std::runtime_error("IPhysx unavailable");
		}

		// Register Omni PhysX callbacks
		physx_obj_sub_ = omni_px_->subscribeObjectChangeNotifications({
		    .objectCreationNotifyFn =
		        std::bind_front(&Self::on_object_creation_notify, this),
		    .objectDestructionNotifyFn = nullptr,
		    .allObjectsDestructionNotifyFn =
		        std::bind_front(&Self::on_all_objects_destruction_notify, this),
		    .userData = nullptr,
		    .stopCallbackWhenSimStopped = false,
		});
		physx_prestep_sub_ = omni_px_->subscribePhysicsOnStepEvents(
		    true, 0,
		    [](float elapsedTime, void *userData) {
			    static_cast<Self *>(userData)->on_physics_prestep(elapsedTime);
		    },
		    this);
		physx_events_sub_ = omni_px_->subscribePhysicsSimulationEvents(
		    [](omni::physx::SimulationStatusEvent eventStatus, void *userData) {
			    static_cast<Self *>(userData)->on_physics_simulation_event(
			        eventStatus);
		    },
		    this);

		// Initialize UsdGraph
		usd_graph_ = std::make_unique<UsdGraph>(stage_);
	}
	~LegoWorld() {
		// Destroy simulation objects if any
		if (bridge_) {
			bridge_.reset();
		}
		if (physics_graph_) {
			physics_graph_->unbind_physx_scene();
			physics_graph_.reset();
		}
		if (px_scene_) {
			px_scene_ = nullptr;
		}

		// Destroy UsdGraph
		usd_graph_.reset();

		// Unsubscribe Omni PhysX callbacks
		if (omni_px_ && physx_obj_sub_) {
			omni_px_->unsubscribeObjectChangeNotifications(*physx_obj_sub_);
			physx_obj_sub_.reset();
		}
		if (omni_px_ && physx_prestep_sub_) {
			omni_px_->unsubscribePhysicsOnStepEvents(*physx_prestep_sub_);
			physx_prestep_sub_.reset();
		}
		if (omni_px_ && physx_events_sub_) {
			omni_px_->unsubscribePhysicsSimulationEvents(*physx_events_sub_);
			physx_events_sub_.reset();
		}

		// Unset interfaces
		omni_px_ = nullptr;
	}

	LegoWorld(const LegoWorld &) = delete;
	LegoWorld &operator=(const LegoWorld &) = delete;
	LegoWorld(LegoWorld &&) = delete;
	LegoWorld &operator=(LegoWorld &&) = delete;

	UsdGraph &usd_graph() {
		return *usd_graph_;
	}

	PhysicsGraph *physics_graph() {
		return physics_graph_.get();
	}

	Bridge *bridge() {
		return bridge_.get();
	}

	physx::PxScene *px_scene() {
		return px_scene_;
	}

	bool is_simulation_active() const {
		return px_scene_ != nullptr;
	}

	AssemblyThresholds get_assembly_thresholds() const {
		if (physics_graph_) {
			return physics_graph_->assembly_checker().thresholds;
		} else {
			return cfg_.assembly_thresholds;
		}
	}

	void set_assembly_thresholds(const AssemblyThresholds &thresholds) {
		cfg_.assembly_thresholds = thresholds;
		if (physics_graph_) {
			physics_graph_->assembly_checker().thresholds = thresholds;
		}
	}

	BreakageThresholds get_breakage_thresholds() const {
		if (physics_graph_) {
			return physics_graph_->breakage_checker().thresholds;
		} else {
			return cfg_.breakage_thresholds;
		}
	}

	void set_breakage_thresholds(const BreakageThresholds &thresholds) {
		cfg_.breakage_thresholds = thresholds;
		if (physics_graph_) {
			physics_graph_->breakage_checker().thresholds = thresholds;
		}
	}

  private:
	pxr::UsdStageRefPtr stage_;
	LegoConfig cfg_;

	omni::physx::IPhysx *omni_px_ = nullptr;

	std::optional<omni::physx::SubscriptionId> physx_obj_sub_;
	std::optional<omni::physx::SubscriptionId> physx_prestep_sub_;
	std::optional<omni::physx::SubscriptionId> physx_events_sub_;

	// UsdGraph is always available
	std::unique_ptr<UsdGraph> usd_graph_;

	// These are only available if simulation is active
	physx::PxScene *px_scene_ = nullptr;
	std::unique_ptr<PhysicsGraph> physics_graph_;
	std::unique_ptr<Bridge> bridge_;

	void setup_simulation(physx::PxScene *new_scene) {
		if (px_scene_ != nullptr) {
			if (new_scene == px_scene_) {
				log_warn("LegoWorld: simulation already set up with the same "
				         "PxScene");
			} else {
				log_error(
				    "LegoWorld: simulation already set up with a PxScene");
			}
			return;
		}
		px_scene_ = new_scene;
		physics_graph_ = std::make_unique<PhysicsGraph>(
		    MetricSystem(stage_), &new_scene->getPhysics(), nullptr,
		    cfg_.assembly_thresholds, cfg_.breakage_thresholds);
		physics_graph_->breakage_checker().set_debug_dump_dir(
		    cfg_.breakage_debug_dump_dir);
		bool physics_graph_bound = physics_graph_->bind_physx_scene(px_scene_);
		if (!physics_graph_bound) {
			throw std::runtime_error(
			    "LegoWorld: failed to bind PhysicsGraph to PxScene");
		}
		bridge_ = std::make_unique<Bridge>(
		    physics_graph_.get(), usd_graph_.get(), omni_px_,
		    cfg_.sync_conns_to_usd, cfg_.sync_conns_to_physics,
		    cfg_.warn_divergence, cfg_.align_policy);
		log_info("LegoWorld: simulation set up successfully");
	}

	void stop_simulation() {
		if (px_scene_ == nullptr) {
			// This could happen if LegoWorld is constructed while simulation is active
			return;
		}
		bridge_.reset();
		if (!physics_graph_->unbind_physx_scene()) {
			log_warn("LegoWorld: failed to unbind PhysicsGraph from PxScene");
		}
		physics_graph_.reset();
		px_scene_ = nullptr;
		log_info("LegoWorld: simulation stopped");
	}

	void
	on_object_creation_notify([[maybe_unused]] const pxr::SdfPath &sdf_path,
	                          omni::physx::usdparser::ObjectId object_id,
	                          omni::physx::PhysXType type,
	                          [[maybe_unused]] void *user_data) {
		if (type != omni::physx::ePTScene) {
			return;
		}
		auto *new_scene =
		    static_cast<physx::PxScene *>(omni_px_->getPhysXPtrFast(object_id));
		if (!new_scene) {
			log_error("LegoWorld: failed to retrieve PxScene pointer for {}",
			          sdf_path.GetText());
			return;
		}
		pxr::UsdPrim stage_prim = stage_->GetPrimAtPath(sdf_path);
		if (!stage_prim.IsValid()) {
			log_error("LegoWorld: invalid prim for {}", sdf_path.GetText());
			return;
		}
		pxr::TfToken update_type;
		if (stage_prim
		        .GetAttribute(pxr::PhysxSchemaTokens->physxSceneUpdateType)
		        .Get(&update_type) &&
		    !update_type.IsEmpty()) {
			if (update_type == pxr::PhysxSchemaTokens->Synchronous) {
				// Continue
			} else if (update_type == pxr::PhysxSchemaTokens->Asynchronous) {
				log_error("LegoWorld: Asynchronous PhysX update type is "
				          "unsupported for {}",
				          sdf_path.GetText());
				return;
			} else if (update_type == pxr::PhysxSchemaTokens->Disabled) {
				log_info("LegoWorld: PhysX update disabled for {}",
				         sdf_path.GetText());
				return;
			} else {
				log_error("LegoWorld: unknown PhysX update type {} for {}",
				          update_type.GetText(), sdf_path.GetText());
				return;
			}
		} else {
			// PhysX defaults to Synchronous (see PhysXScene ctor).
		}

		log_info("LegoWorld: detected PhysX scene creation for {}",
		         sdf_path.GetText());
		setup_simulation(new_scene);
	}

	void on_all_objects_destruction_notify([[maybe_unused]] void *user_data) {
		stop_simulation();
	}

	void on_physics_prestep([[maybe_unused]] float elapsedTime) {
		if (physics_graph_) {
			physics_graph_->do_pre_step();
		}
	}

	void on_physics_simulation_event(
	    omni::physx::SimulationStatusEvent eventStatus) {
		if (eventStatus != omni::physx::eSimulationComplete) {
			return;
		}
		if (bridge_) {
			bridge_->clear_connection_logs();
		}
		if (physics_graph_) {
			physics_graph_->do_post_step();
		}
	}
};

} // namespace lego_assemble
