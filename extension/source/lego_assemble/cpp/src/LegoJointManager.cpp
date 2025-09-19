#include "LegoJointManager.h"
#include "tokens.h"

#include <PxPhysicsAPI.h>
#include <carb/InterfaceUtils.h>
#include <carb/events/EventsUtils.h>
#include <carb/logging/Log.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/physx/IPhysx.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdUtils/stageCache.h>

namespace lego_assemble {

static omni::physx::IPhysx *g_physx = nullptr;
static pxr::UsdStageWeakPtr g_stage;

static omni::physx::SubscriptionId g_preStepSub =
    omni::physx::kInvalidSubscriptionId;
static omni::kit::StageUpdatePtr g_stageUpdate;
static omni::kit::StageUpdateNode *g_stageUpdateNode = nullptr;

static carb::events::ISubscriptionPtr g_simulationResumedSub;
static std::unique_ptr<carb::events::LambdaEventListener>
    g_simulationResumedListener;

static void onPhysxPreStep(float dt, void *userData) {
	// TODO
}

static void processPrimChange(const pxr::SdfPath &primPath) {
	if (!g_stage)
		return;
	pxr::UsdPrim prim = g_stage->GetPrimAtPath(primPath);
	if (!prim || !prim.IsA<pxr::UsdPhysicsFixedJoint>())
		return;
	pxr::UsdAttribute lego_conn_attr = prim.GetAttribute(LegoTokens->lego_conn);
	if (!lego_conn_attr)
		return;
	bool is_lego_conn;
	if (!lego_conn_attr.Get<bool>(&is_lego_conn))
		return;

	// TODO
}

static void onAttach(long stageId, double metersPerUnit, void *userData) {
	g_stage = pxr::UsdUtilsStageCache::Get().Find(
	    pxr::UsdStageCache::Id::FromLongInt(stageId));
	if (!g_stage)
		CARB_LOG_ERROR("PhysX could not find USD stage");
}

static void onDetach(void *userData) {
	g_stage = nullptr;
}

static void onPrimAdd(const pxr::SdfPath &primPath, void *userData) {
	processPrimChange(primPath);
}

static void onPrimOrPropertyChange(const pxr::SdfPath &primOrPropertyPath,
                                   void *userData) {
	processPrimChange(primOrPropertyPath);
}

bool initLegoJointManager() {
	g_physx = carb::getCachedInterface<omni::physx::IPhysx>();
	if (!g_physx) {
		CARB_LOG_ERROR("LegoJointManager: IPhysx unavailable");
		return false;
	}

	// Subscribe to PhysX pre-step event
	if (g_preStepSub == omni::physx::kInvalidSubscriptionId) {
		g_preStepSub = g_physx->subscribePhysicsOnStepEvents(
		    true /*preStep*/, 0 /*order*/, onPhysxPreStep, nullptr);
	}

	// Subscribe to PhysX simulation resumed event
	g_simulationResumedListener =
	    std::make_unique<carb::events::LambdaEventListener>(
	        [](carb::events::IEvent *e) { /* TODO */ });
	g_simulationResumedSub =
	    g_physx->getSimulationEventStreamV2()->createSubscriptionToPopByType(
	        omni::physx::SimulationEvent::eResumed,
	        g_simulationResumedListener.get());

	// Subscribe to stage update events
	auto *suIface = carb::getCachedInterface<omni::kit::IStageUpdate>();
	if (!suIface) {
		CARB_LOG_ERROR("LegoJointManager: IStageUpdate unavailable");
		return false;
	}
	g_stageUpdate = suIface->getStageUpdate();
	if (!g_stageUpdate) {
		CARB_LOG_ERROR("LegoJointManager: StageUpdate is nullptr");
		return false;
	}
	omni::kit::StageUpdateNodeDesc desc = {nullptr};
	desc.displayName = "lego_assemble";
	desc.userData = nullptr;
	desc.order = 200; // After almost all other nodes, before debug (1000)
	desc.onAttach = &onAttach;
	desc.onDetach = &onDetach;
	desc.onPrimAdd = &onPrimAdd;
	desc.onPrimOrPropertyChange = &onPrimOrPropertyChange;
	g_stageUpdateNode = g_stageUpdate->createStageUpdateNode(desc);

	return true;
}

bool deinitLegoJointManager() {
	bool success = true;

	// Unsubscribe from stage update events
	if (g_stageUpdate && g_stageUpdateNode) {
		g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
	} else {
		CARB_LOG_WARN("LegoJointManager: cannot destroy StageUpdateNode");
		success = false;
	}
	g_stageUpdateNode = nullptr;
	g_stageUpdate = nullptr;

	if (g_physx) {
		// Unsubscribe from PhysX simulation resumed event
		if (g_simulationResumedSub) {
			g_simulationResumedSub->unsubscribe();
		} else {
			CARB_LOG_WARN(
			    "LegoJointManager: simulation resumed event not subscribed");
			success = false;
		}
		g_simulationResumedSub = nullptr;
		g_simulationResumedListener = nullptr;

		// Unsubscribe from PhysX pre-step event
		if (g_preStepSub != omni::physx::kInvalidSubscriptionId) {
			g_physx->unsubscribePhysicsOnStepEvents(g_preStepSub);
			g_preStepSub = omni::physx::kInvalidSubscriptionId;
		} else {
			CARB_LOG_WARN("LegoJointManager: pre-step event not subscribed");
			success = false;
		}
	} else {
		CARB_LOG_ERROR("LegoJointManager: IPhysx unavailable");
		success = false;
	}

	g_stage = nullptr;
	g_physx = nullptr;
	return success;
}

std::optional<std::tuple<physx::PxVec3, physx::PxVec3>>
getPhysxJointForceTorque(const pxr::SdfPath &jointPath) {
	if (!g_physx)
		return std::nullopt;
	void *jointPtr =
	    g_physx->getPhysXPtr(jointPath, omni::physx::PhysXType::ePTJoint);
	if (!jointPtr)
		return std::nullopt;
	physx::PxJoint *joint = static_cast<physx::PxJoint *>(jointPtr);
	physx::PxConstraint *constraint = joint->getConstraint();
	if (!constraint)
		return std::nullopt;

	physx::PxVec3 force, torque;
	constraint->getForce(force, torque);
	return std::make_tuple(force, torque);
}

} // namespace lego_assemble
