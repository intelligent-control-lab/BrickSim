export module lego_assemble.omni.lego_world;

import std;
import lego_assemble.core.specs;
import lego_assemble.physx.physics_graph;
import lego_assemble.usd.usd_graph;
import lego_assemble.usd.author;
import lego_assemble.usd.parse;
import lego_assemble.usd.specs;
import lego_assemble.omni.usd_physics_bridge;
import lego_assemble.utils.type_list;
import lego_assemble.utils.metric_system;
import lego_assemble.vendor.pxr;
import lego_assemble.vendor.physx;
import lego_assemble.vendor.omni;
import lego_assemble.vendor.carb;

namespace lego_assemble {

export struct LegoConfig {
	bool sync_conns_to_usd = true;
	bool sync_conns_to_physics = false;
	bool warn_divergence = true;
	AlignPolicy align_policy = AlignPolicy::MoveHoleCC;
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
	using PhysicsGraph = typename Bridge::PhysicsGraph;
	using UsdGraph = typename Bridge::UsdGraph;

	explicit LegoWorld(pxr::UsdStageRefPtr stage, LegoConfig cfg)
	    : stage_{std::move(stage)}, cfg_{std::move(cfg)} {

		// Retrieve interfaces
		auto *stage_update_iface =
		    carb::getCachedInterface<omni::kit::IStageUpdate>();
		if (!stage_update_iface) {
			throw std::runtime_error("IStageUpdate unavailable");
		}
		stage_update_ = stage_update_iface->getStageUpdate();
		omni_px_ = carb::getCachedInterface<omni::physx::IPhysx>();
		if (!omni_px_) {
			throw std::runtime_error("IPhysx unavailable");
		}

		// Register Omni PhysX callbacks
		physx_obj_sub_ = omni_px_->subscribeObjectChangeNotifications({
		    // Thead safety: called on USD/Kit thread
		    .objectCreationNotifyFn =
		        std::bind_front(&Self::on_object_creation_notify, this),
		    .objectDestructionNotifyFn = nullptr,
		    .allObjectsDestructionNotifyFn =
		        std::bind_front(&Self::on_all_objects_destruction_notify, this),
		    .userData = nullptr,
		    .stopCallbackWhenSimStopped = false,
		});

		// Register StageUpdateNodes
		stage_update_pre_ = stage_update_->createStageUpdateNode({
		    .displayName = "lego_assemble (pre-physics)",
		    .userData = this,
		    .order = 9, // "Physics"'s order is 10
		    .onAttach = nullptr,
		    .onDetach = nullptr,
		    .onPause = nullptr,
		    .onStop = nullptr,
		    .onResume = nullptr,
		    .onUpdate =
		        []([[maybe_unused]] float currentTime,
		           [[maybe_unused]] float elapsedSecs,
		           [[maybe_unused]] const omni::kit::StageUpdateSettings
		               *settings,
		           void *userData) {
			        if (auto *self = static_cast<Self *>(userData)) {
				        if (self->physics_graph_) {
					        self->physics_graph_->do_pre_step();
				        }
			        }
		        },
		    .onPrimAdd = nullptr,
		    .onPrimOrPropertyChange = nullptr,
		    .onPrimRemove = nullptr,
		    .onRaycast = nullptr,
		});
		stage_update_post_ = stage_update_->createStageUpdateNode({
		    .displayName = "lego_assemble (post-physics)",
		    .userData = this,
		    .order = 11, // "Physics"'s order is 10
		    .onAttach = nullptr,
		    .onDetach = nullptr,
		    .onPause = nullptr,
		    .onStop = nullptr,
		    .onResume = nullptr,
		    .onUpdate =
		        []([[maybe_unused]] float currentTime,
		           [[maybe_unused]] float elapsedSecs,
		           [[maybe_unused]] const omni::kit::StageUpdateSettings
		               *settings,
		           void *userData) {
			        if (auto *self = static_cast<Self *>(userData)) {
				        if (self->physics_graph_) {
					        self->physics_graph_->do_post_step();
				        }
			        }
		        },
		    .onPrimAdd = nullptr,
		    .onPrimOrPropertyChange = nullptr,
		    .onPrimRemove = nullptr,
		    .onRaycast = nullptr,
		});

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

		// Remove StageUpdateNodes
		if (stage_update_ && stage_update_pre_) {
			stage_update_->destroyStageUpdateNode(stage_update_pre_);
			stage_update_pre_ = nullptr;
		}
		if (stage_update_ && stage_update_post_) {
			stage_update_->destroyStageUpdateNode(stage_update_post_);
			stage_update_post_ = nullptr;
		}

		// Unsubscribe Omni PhysX callbacks
		if (omni_px_ && physx_obj_sub_) {
			omni_px_->unsubscribeObjectChangeNotifications(*physx_obj_sub_);
			physx_obj_sub_.reset();
		}

		// Unset interfaces
		omni_px_ = nullptr;
		stage_update_.reset();
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

  private:
	pxr::UsdStageRefPtr stage_;
	LegoConfig cfg_;

	std::shared_ptr<omni::kit::StageUpdate> stage_update_;
	omni::physx::IPhysx *omni_px_ = nullptr;

	std::optional<omni::physx::SubscriptionId> physx_obj_sub_;
	omni::kit::StageUpdateNode *stage_update_pre_ = nullptr;
	omni::kit::StageUpdateNode *stage_update_post_ = nullptr;

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
				return;
			} else {
				throw std::runtime_error(
				    "LegoWorld: simulation already set up with a PxScene");
			}
		}
		px_scene_ = new_scene;
		physics_graph_ = std::make_unique<PhysicsGraph>(
		    MetricSystem(stage_), new_scene->getPhysics());
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
			log_warn("LegoWorld: simulation is not set up; cannot stop");
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
		if (type == omni::physx::ePTScene) {
			auto *new_scene = static_cast<physx::PxScene *>(
			    omni_px_->getPhysXPtrFast(object_id));
			if (new_scene) {
				setup_simulation(new_scene);
			}
		}
	}

	void on_all_objects_destruction_notify([[maybe_unused]] void *user_data) {
		stop_simulation();
	}
};

} // namespace lego_assemble
