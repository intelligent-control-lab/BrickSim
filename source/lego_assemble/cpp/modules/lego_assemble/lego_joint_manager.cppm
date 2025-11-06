export module lego_assemble.lego_joint_manager;

import std;
import lego_assemble.lego_graph;
import lego_assemble.usd_bridge;
import lego_assemble.physx.scene_patcher;
import lego_assemble.vendor.physx;
import lego_assemble.vendor.carb;
import lego_assemble.vendor.pxr;
import lego_assemble.vendor.omni;

namespace lego_assemble {

static LegoGraph::Thresholds g_thresholds;
static omni::physx::IPhysx *g_physx = nullptr;
static std::unique_ptr<LegoUsdBridge> g_lego;
static long int g_stageId = -1;

static omni::physx::SubscriptionId g_physxObjSub =
    omni_physx::kInvalidSubscriptionId;
static omni::physx::SubscriptionId g_physxObjFullSub =
    omni_physx::kInvalidSubscriptionId;
static omni::physx::SubscriptionId g_preStepSub =
    omni_physx::kInvalidSubscriptionId;
static omni::physx::SubscriptionId g_postStepSub =
    omni_physx::kInvalidSubscriptionId;
static omni::kit::StageUpdatePtr g_stageUpdate;
static omni::kit::StageUpdateNode *g_stageUpdateNode = nullptr;

export bool initLegoJointManager() {
	g_physx = carb::getCachedInterface<omni::physx::IPhysx>();
	if (!g_physx) {
		log_error("IPhysx unavailable");
		return false;
	}

	// ==== Subscribe to PhysX object creation/destruction
	omni::physx::IPhysicsObjectChangeCallback physxObjCb;
	physxObjCb.objectCreationNotifyFn =
	    [](const pxr::SdfPath &sdfPath,
	       omni::physx::usdparser::ObjectId objectId,
	       omni::physx::PhysXType type, void *userData) {
		    if (!g_lego)
			    return;
		    if (type == omni::physx::PhysXType::ePTActor) {
			    auto *actor = static_cast<physx::PxRigidActor *>(
			        g_physx->getPhysXPtrFast(objectId));
			    if (actor) {
				    g_lego->onRigidCreated(actor, sdfPath);
			    }
		    }
	    };
	physxObjCb.objectDestructionNotifyFn =
	    [](const pxr::SdfPath &sdfPath,
	       omni::physx::usdparser::ObjectId objectId,
	       omni::physx::PhysXType type, void *userData) {
		    if (!g_lego)
			    return;
		    if (type == omni::physx::PhysXType::ePTActor) {
			    auto *actor = static_cast<physx::PxRigidActor *>(
			        g_physx->getPhysXPtrFast(objectId));
			    if (actor) {
				    g_lego->onRigidDestroyed(actor, sdfPath);
			    }
		    }
	    };
	physxObjCb.stopCallbackWhenSimStopped = true;
	g_physxObjSub = g_physx->subscribeObjectChangeNotifications(physxObjCb);

	// ==== Subscribe to ALL PhysX object creation/destruction
	omni::physx::IPhysicsObjectChangeCallback physxObjFullCb;
	physxObjFullCb.objectCreationNotifyFn =
	    [](const pxr::SdfPath &sdfPath,
	       omni::physx::usdparser::ObjectId objectId,
	       omni::physx::PhysXType type, void *userData) {
		    if (type == omni::physx::PhysXType::ePTScene) {
			    auto *scene = static_cast<physx::PxScene *>(
			        g_physx->getPhysXPtrFast(objectId));
			    if (scene) {
				    if (patchPxScene(scene)) {
					    log_info("PxScene %p patched successfully", scene);
				    } else {
					    log_error("Failed to patch PxScene %p", scene);
				    }
			    }
		    }
	    };
	physxObjFullCb.allObjectsDestructionNotifyFn = [](void *userData) {
		if (g_lego) {
			g_lego = nullptr;
			log_info("Lego state destroyed (allObjectsDestructionNotify)");
		}
		unpatchPxScene(true);
	};
	physxObjFullCb.stopCallbackWhenSimStopped = false;
	g_physxObjFullSub =
	    g_physx->subscribeObjectChangeNotifications(physxObjFullCb);

	// ==== Subscribe to PhysX pre-step event
	if (g_preStepSub == omni_physx::kInvalidSubscriptionId) {
		g_preStepSub = g_physx->subscribePhysicsOnStepEvents(
		    true, 0,
		    [](float elapsedTime, void *userData) {
			    if (!g_lego) {
				    auto *px = static_cast<physx::PxPhysics *>(
				        g_physx->getPhysXPtr({}, omni::physx::ePTPhysics));
				    if (!px) {
					    log_fatal("ePTPhysics is nullptr");
					    return;
				    }
				    auto stageRef = pxr::UsdUtilsStageCache::Get().Find(
				        pxr::UsdStageCache::Id::FromLongInt(g_stageId));
				    if (!stageRef) {
					    log_fatal(
					        "Failed to get stage from stage cache (pre-step)");
					    return;
				    }
				    g_lego = std::make_unique<LegoUsdBridge>(
				        std::move(stageRef), px, g_physx);
				    g_lego->getGraph().setThresholds(g_thresholds);
				    log_info("Lego state created (pre-step)");
			    }
			    g_lego->onPreStep();
		    },
		    nullptr);
	}

	// ==== Subscribe to PhysX post-step event
	if (g_postStepSub == omni_physx::kInvalidSubscriptionId) {
		g_postStepSub = g_physx->subscribePhysicsOnStepEvents(
		    false, 0,
		    [](float elapsedTime, void *userData) {
			    if (g_lego) {
				    g_lego->onPostStep();
			    }
		    },
		    nullptr);
	}

	// ==== Subscribe to stage update events (stage attach/detach + USD edits)
	auto *suIface = carb::getCachedInterface<omni::kit::IStageUpdate>();
	if (!suIface) {
		log_error("IStageUpdate unavailable");
		return false;
	}
	g_stageUpdate = suIface->getStageUpdate();
	if (!g_stageUpdate) {
		log_error("StageUpdate is nullptr");
		return false;
	}
	omni::kit::StageUpdateNodeDesc desc = {nullptr};
	desc.displayName = "lego_assemble";
	desc.userData = nullptr;
	desc.order = 200; // After almost all other nodes, before debug (1000)
	desc.onAttach = [](long stageId, double metersPerUnit, void *userData) {
		// Called when USD attach
		g_stageId = stageId;
	};
	desc.onDetach = [](void *userData) {
		// Called when USD detach
		g_stageId = -1;
	};
	desc.onPrimAdd = [](const pxr::SdfPath &primPath, void *userData) {
		if (g_lego) {
			g_lego->enqueuePrimChange(primPath);
		}
	};
	desc.onPrimOrPropertyChange = [](const pxr::SdfPath &primOrPropertyPath,
	                                 void *userData) {
		if (g_lego) {
			g_lego->enqueuePrimChange(primOrPropertyPath);
		}
	};
	desc.onPrimRemove = [](const pxr::SdfPath &primPath, void *userData) {
		if (g_lego) {
			g_lego->enqueuePrimChange(primPath);
		}
	};
	g_stageUpdateNode = g_stageUpdate->createStageUpdateNode(desc);

	return true;
}

export bool deinitLegoJointManager() {
	bool success = true;

	// ==== Unsubscribe from stage update events
	if (g_stageUpdate && g_stageUpdateNode) {
		g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
	} else {
		log_warn("Cannot destroy StageUpdateNode");
		success = false;
	}
	g_stageUpdateNode = nullptr;
	g_stageUpdate = nullptr;

	if (g_physx) {
		// ==== Unsubscribe from PhysX post-step event
		if (g_postStepSub != omni_physx::kInvalidSubscriptionId) {
			g_physx->unsubscribePhysicsOnStepEvents(g_postStepSub);
			g_postStepSub = omni_physx::kInvalidSubscriptionId;
		} else {
			log_warn("Post-step event not subscribed");
			success = false;
		}

		// ==== Unsubscribe from PhysX pre-step event
		if (g_preStepSub != omni_physx::kInvalidSubscriptionId) {
			g_physx->unsubscribePhysicsOnStepEvents(g_preStepSub);
			g_preStepSub = omni_physx::kInvalidSubscriptionId;
		} else {
			log_warn("Pre-step event not subscribed");
			success = false;
		}

		// ==== Unsubscribe from PhysX object creation/destruction
		if (g_physxObjSub != omni_physx::kInvalidSubscriptionId) {
			g_physx->unsubscribeObjectChangeNotifications(g_physxObjSub);
			g_physxObjSub = omni_physx::kInvalidSubscriptionId;
		} else {
			log_warn("PhysX object change not subscribed");
			success = false;
		}

		// ==== Unsubscribe from ALL PhysX object creation/destruction
		if (g_physxObjFullSub != omni_physx::kInvalidSubscriptionId) {
			g_physx->unsubscribeObjectChangeNotifications(g_physxObjFullSub);
			g_physxObjFullSub = omni_physx::kInvalidSubscriptionId;
		} else {
			log_warn("PhysX full object change not subscribed");
			success = false;
		}
	} else {
		log_error("IPhysx unavailable");
		success = false;
	}

	g_lego = nullptr;
	g_physx = nullptr;
	return success;
}

export void setLegoThresholds(const LegoGraph::Thresholds &thresholds) {
	g_thresholds = thresholds;
	if (g_lego) {
		g_lego->getGraph().setThresholds(g_thresholds);
	}
}

export void getLegoThresholds(LegoGraph::Thresholds &out) {
	out = g_thresholds;
}

} // namespace lego_assemble
