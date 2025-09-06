#include "JointInvMassInertiaSetter.h"

#include <PxPhysicsAPI.h>
#include <carb/InterfaceUtils.h>
#include <carb/logging/Log.h>
#include <mutex>
#include <omni/physx/IPhysx.h>

namespace {

struct SetInvMassInertiaReq {
	pxr::SdfPath path;
	float invMass0;
	float invInertia0;
	float invMass1;
	float invInertia1;
};

static std::vector<SetInvMassInertiaReq> g_pending;
static std::mutex g_pendingMutex;

static void processReqQueue() {
	auto *iPhysx = carb::getCachedInterface<omni::physx::IPhysx>();
	if (!iPhysx) {
		CARB_LOG_ERROR("JointInvMassInertiaSetter: IPhysx unavailable");
		return;
	}

	std::vector<SetInvMassInertiaReq> local;
	{
		std::scoped_lock lock(g_pendingMutex);
		if (g_pending.empty())
			return;
		local.swap(g_pending);
	}

	for (auto &req : local) {
		void *jointPtr =
		    iPhysx->getPhysXPtr(req.path, omni::physx::PhysXType::ePTJoint);
		if (jointPtr) {
			physx::PxJoint *joint = static_cast<physx::PxJoint *>(jointPtr);
			if (req.invMass0 >= 0.0f)
				joint->setInvMassScale0(req.invMass0);
			if (req.invInertia0 >= 0.0f)
				joint->setInvInertiaScale0(req.invInertia0);
			if (req.invMass1 >= 0.0f)
				joint->setInvMassScale1(req.invMass1);
			if (req.invInertia1 >= 0.0f)
				joint->setInvInertiaScale1(req.invInertia1);
			CARB_LOG_INFO("Set inv-mass/inertia for %s", req.path.GetText());

		} else {
			CARB_LOG_WARN("Failed to set inv-mass/inertia for %s",
			              req.path.GetText());
		}
	}
}

static omni::physx::SubscriptionId g_preStepSub =
    omni::physx::kInvalidSubscriptionId;

static void onPhysxPreStep(float dt, void * /*userData*/) {
	processReqQueue();
}

} // namespace

namespace lego_assemble {

bool initJointInvMassInertiaSetter() {
	auto *iPhysx = carb::getCachedInterface<omni::physx::IPhysx>();
	if (!iPhysx) {
		CARB_LOG_ERROR("JointInvMassInertiaSetter: IPhysx unavailable");
		return false;
	}

	if (g_preStepSub == omni::physx::kInvalidSubscriptionId) {
		g_preStepSub = iPhysx->subscribePhysicsOnStepEvents(
		    true /*preStep*/, 0 /*order*/, onPhysxPreStep, nullptr);
	}

	return true;
}

bool deinitJointInvMassInertiaSetter() {
	auto *iPhysx = carb::getCachedInterface<omni::physx::IPhysx>();
	if (!iPhysx) {
		CARB_LOG_ERROR("JointInvMassInertiaSetter: IPhysx unavailable");
		return false;
	}

	if (g_preStepSub != omni::physx::kInvalidSubscriptionId) {
		iPhysx->unsubscribePhysicsOnStepEvents(g_preStepSub);
		g_preStepSub = omni::physx::kInvalidSubscriptionId;
	}

	return true;
}

bool setPhysxJointInvMassInertia(const pxr::SdfPath &jointPath,
                                 float invMassScale0, float invInertiaScale0,
                                 float invMassScale1, float invInertiaScale1) {
	SetInvMassInertiaReq req{jointPath, invMassScale0, invInertiaScale0,
	                         invMassScale1, invInertiaScale1};
	std::scoped_lock lock(g_pendingMutex);
	g_pending.emplace_back(std::move(req));
	return true;
}

} // namespace lego_assemble
