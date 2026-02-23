export module bricksim.omni.lego_runtime;

import std;
import bricksim.core.specs;
import bricksim.physx.assembly;
import bricksim.physx.breakage;
import bricksim.usd.author;
import bricksim.usd.parse;
import bricksim.utils.type_list;
import bricksim.omni.lego_world;
import bricksim.io.topology;
import bricksim.utils.logging;
import bricksim.vendor;

namespace bricksim {

export class LegoRuntime {
  public:
	using Parts = PartList<BrickPart>;
	using PartAuthors = type_list<PrototypeBrickAuthor>;
	using PartParsers = type_list<BrickParser>;
	using World = LegoWorld<Parts, PartAuthors, PartParsers>;
	using Serializer = TopologySerializer<BrickSerializer>;

	static LegoRuntime &instance() {
		static LegoRuntime inst;
		return inst;
	}

	World *world() {
		return world_.get();
	}

	AssemblyThresholds get_assembly_thresholds() const {
		if (world_) {
			return world_->get_assembly_thresholds();
		} else {
			return cfg_.assembly_thresholds;
		}
	}

	void set_assembly_thresholds(const AssemblyThresholds &thresholds) {
		cfg_.assembly_thresholds = thresholds;
		if (world_) {
			world_->set_assembly_thresholds(thresholds);
		}
	}

	BreakageThresholds get_breakage_thresholds() const {
		if (world_) {
			return world_->get_breakage_thresholds();
		} else {
			return cfg_.breakage_thresholds;
		}
	}

	void set_breakage_thresholds(const BreakageThresholds &thresholds) {
		cfg_.breakage_thresholds = thresholds;
		if (world_) {
			world_->set_breakage_thresholds(thresholds);
		}
	}

  private:
	LegoConfig cfg_{
	    .breakage_debug_dump_dir =
	        std::filesystem::temp_directory_path().string(),
	};

	std::shared_ptr<omni::kit::StageUpdate> stage_update_;
	omni::kit::StageUpdateNode *stage_update_node_ = nullptr;

	long stage_id_ = -1;
	std::unique_ptr<World> world_;

	void on_stage_attach(long stage_id, [[maybe_unused]] double mpu) {
		stage_id_ = stage_id;

		// Tear down any existing world before creating a new one.
		if (world_) {
			log_info("LegoRuntime: destroying existing LegoWorld for new stage "
			         "attach (stage id {})",
			         stage_id_);
			world_.reset();
		}

		pxr::UsdStageRefPtr stage = pxr::UsdUtilsStageCache::Get().Find(
		    pxr::UsdStageCache::Id::FromLongInt(stage_id_));
		if (!stage) {
			throw std::runtime_error(
			    "LegoRuntime: failed to get UsdStage from UsdStageCache on "
			    "attach");
		}

		world_ = std::make_unique<World>(std::move(stage), cfg_);
		log_info("LegoRuntime: LegoWorld created for stage id {}", stage_id_);
	}

	void on_stage_detach() {
		stage_id_ = -1;
		if (world_) {
			log_info("LegoRuntime: destroying LegoWorld on stage detach");
			world_.reset();
		}
	}

	LegoRuntime() {
		auto *stage_update_iface =
		    carb::getCachedInterface<omni::kit::IStageUpdate>();
		if (!stage_update_iface) {
			throw std::runtime_error("LegoRuntime: IStageUpdate unavailable");
		}
		stage_update_ = stage_update_iface->getStageUpdate();

		stage_update_node_ = stage_update_->createStageUpdateNode({
		    .displayName = "BrickSim (runtime)",
		    .userData = this,
		    .order = 200, // After all nodes
		    .onAttach =
		        [](long stage_id, double meters_per_unit, void *user_data) {
			        if (auto *self = static_cast<LegoRuntime *>(user_data)) {
				        self->on_stage_attach(stage_id, meters_per_unit);
			        }
		        },
		    .onDetach =
		        [](void *user_data) {
			        if (auto *self = static_cast<LegoRuntime *>(user_data)) {
				        self->on_stage_detach();
			        }
		        },
		    .onPause = nullptr,
		    .onStop = nullptr,
		    .onResume = nullptr,
		    .onUpdate = nullptr,
		    .onPrimAdd = nullptr,
		    .onPrimOrPropertyChange = nullptr,
		    .onPrimRemove = nullptr,
		    .onRaycast = nullptr,
		});
	}

	~LegoRuntime() {
		// Destroy stage-scoped world first so it can clean up PhysX
		// subscriptions and USD hooks while the stage is still alive.
		if (world_) {
			log_info("LegoRuntime: destroying LegoWorld in destructor");
			world_.reset();
		}

		// Unregister from StageUpdate before releasing the interface.
		if (stage_update_ && stage_update_node_) {
			stage_update_->destroyStageUpdateNode(stage_update_node_);
			stage_update_node_ = nullptr;
		}
		stage_update_.reset();
	}

	LegoRuntime(const LegoRuntime &) = delete;
	LegoRuntime &operator=(const LegoRuntime &) = delete;
	LegoRuntime(LegoRuntime &&) = delete;
	LegoRuntime &operator=(LegoRuntime &&) = delete;
};

} // namespace bricksim
