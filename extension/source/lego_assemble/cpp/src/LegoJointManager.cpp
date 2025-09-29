#include "LegoJointManager.h"

#include "LegoUsdBridge.h"
#include "ScenePatcher.h"

#include <carb/InterfaceUtils.h>
#include <carb/events/EventsUtils.h>
#include <carb/logging/Log.h>

#include <pxr/usd/usdUtils/stageCache.h>

#include <omni/kit/IStageUpdate.h>
#include <omni/physx/IPhysx.h>

namespace lego_assemble {

static omni::physx::IPhysx *g_physx = nullptr;
static std::unique_ptr<LegoUsdBridge> g_lego;
static long int g_stageId = -1;

static omni::physx::SubscriptionId g_physxObjSub =
    omni::physx::kInvalidSubscriptionId;
static omni::physx::SubscriptionId g_physxObjFullSub =
    omni::physx::kInvalidSubscriptionId;
static omni::physx::SubscriptionId g_preStepSub =
    omni::physx::kInvalidSubscriptionId;
static omni::kit::StageUpdatePtr g_stageUpdate;
static omni::kit::StageUpdateNode *g_stageUpdateNode = nullptr;

bool initLegoJointManager() {
	g_physx = carb::getCachedInterface<omni::physx::IPhysx>();
	if (!g_physx) {
		CARB_LOG_ERROR("IPhysx unavailable");
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
					    CARB_LOG_INFO("PxScene %p patched successfully", scene);
				    } else {
					    CARB_LOG_ERROR("Failed to patch PxScene %p", scene);
				    }
			    }
		    }
	    };
	physxObjFullCb.allObjectsDestructionNotifyFn = [](void *userData) {
		if (g_lego) {
			g_lego = nullptr;
			CARB_LOG_INFO("Lego state destroyed (allObjectsDestructionNotify)");
		}
		unpatchPxScene(true);
	};
	physxObjFullCb.stopCallbackWhenSimStopped = false;
	g_physxObjFullSub =
	    g_physx->subscribeObjectChangeNotifications(physxObjFullCb);

	// ==== Subscribe to PhysX pre-step event
	if (g_preStepSub == omni::physx::kInvalidSubscriptionId) {
		g_preStepSub = g_physx->subscribePhysicsOnStepEvents(
		    true, 0,
		    [](float elapsedTime, void *userData) {
			    if (!g_lego) {
				    auto *px = static_cast<physx::PxPhysics *>(
				        g_physx->getPhysXPtr({}, omni::physx::ePTPhysics));
				    if (!px) {
					    CARB_LOG_FATAL("ePTPhysics is nullptr");
					    return;
				    }
				    auto stageRef = pxr::UsdUtilsStageCache::Get().Find(
				        pxr::UsdStageCache::Id::FromLongInt(g_stageId));
				    if (!stageRef) {
					    CARB_LOG_FATAL(
					        "Failed to get stage from stage cache (pre-step)");
					    return;
				    }
				    g_lego = std::make_unique<LegoUsdBridge>(
				        std::move(stageRef), px, g_physx);
				    CARB_LOG_INFO("Lego state created (pre-step)");
			    }
			    g_lego->processPrimChanges();
		    },
		    nullptr);
	}

	// ==== Subscribe to stage update events (stage attach/detach + USD edits)
	auto *suIface = carb::getCachedInterface<omni::kit::IStageUpdate>();
	if (!suIface) {
		CARB_LOG_ERROR("IStageUpdate unavailable");
		return false;
	}
	g_stageUpdate = suIface->getStageUpdate();
	if (!g_stageUpdate) {
		CARB_LOG_ERROR("StageUpdate is nullptr");
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

bool deinitLegoJointManager() {
	bool success = true;

	// ==== Unsubscribe from stage update events
	if (g_stageUpdate && g_stageUpdateNode) {
		g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
	} else {
		CARB_LOG_WARN("Cannot destroy StageUpdateNode");
		success = false;
	}
	g_stageUpdateNode = nullptr;
	g_stageUpdate = nullptr;

	if (g_physx) {
		// ==== Unsubscribe from PhysX pre-step event
		if (g_preStepSub != omni::physx::kInvalidSubscriptionId) {
			g_physx->unsubscribePhysicsOnStepEvents(g_preStepSub);
			g_preStepSub = omni::physx::kInvalidSubscriptionId;
		} else {
			CARB_LOG_WARN("Pre-step event not subscribed");
			success = false;
		}

		// ==== Unsubscribe from PhysX object creation/destruction
		if (g_physxObjSub != omni::physx::kInvalidSubscriptionId) {
			g_physx->unsubscribeObjectChangeNotifications(g_physxObjSub);
			g_physxObjSub = omni::physx::kInvalidSubscriptionId;
		} else {
			CARB_LOG_WARN("PhysX object change not subscribed");
			success = false;
		}

		// ==== Unsubscribe from ALL PhysX object creation/destruction
		if (g_physxObjFullSub != omni::physx::kInvalidSubscriptionId) {
			g_physx->unsubscribeObjectChangeNotifications(g_physxObjFullSub);
			g_physxObjFullSub = omni::physx::kInvalidSubscriptionId;
		} else {
			CARB_LOG_WARN("PhysX full object change not subscribed");
			success = false;
		}
	} else {
		CARB_LOG_ERROR("IPhysx unavailable");
		success = false;
	}

	g_lego = nullptr;
	g_physx = nullptr;
	return success;
}

} // namespace lego_assemble
