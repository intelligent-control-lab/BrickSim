#include <pxr/usd/sdf/path.h>

#include <PxPhysicsAPI.h>
#include <carb/InterfaceUtils.h>
#include <carb/logging/Log.h>
#include <mutex>
#include <omni/kit/IStageUpdate.h>
#include <omni/physx/IPhysx.h>

namespace {
static omni::kit::StageUpdatePtr g_stageUpdate;
static omni::kit::StageUpdateNode *g_stageUpdateNode = nullptr;

static constexpr int MAX_QUEUED_ATTEMPTS = 3;

struct JointScaleReq {
	pxr::SdfPath path;
	float invMass0;
	float invInertia0;
	float invMass1;
	float invInertia1;
	uint32_t attempts = 0; // number of failed tries so far
};

static std::vector<JointScaleReq> g_pending;
static std::mutex g_pendingMutex;

static void ProcessJointInvMassInertiaQueue() {
	auto *iPhysx = carb::getCachedInterface<omni::physx::IPhysx>();
	if (!iPhysx) {
		CARB_LOG_ERROR("JointInvMassInertiaQueue: IPhysx unavailable.");
		return;
	}

	std::vector<JointScaleReq> local;
	{
		std::scoped_lock lock(g_pendingMutex);
		if (g_pending.empty())
			return;
		local.swap(g_pending);
	}

	std::vector<JointScaleReq> retry;
	retry.reserve(local.size());

	for (auto &req : local) {

		void *jointPtr =
		    iPhysx->getPhysXPtr(req.path, omni::physx::PhysXType::ePTJoint);
		if (jointPtr) {
			::physx::PxJoint *joint = static_cast<::physx::PxJoint *>(jointPtr);
			if (req.invMass0 >= 0.0f)
				joint->setInvMassScale0(req.invMass0);
			if (req.invInertia0 >= 0.0f)
				joint->setInvInertiaScale0(req.invInertia0);
			if (req.invMass1 >= 0.0f)
				joint->setInvMassScale1(req.invMass1);
			if (req.invInertia1 >= 0.0f)
				joint->setInvInertiaScale1(req.invInertia1);
			CARB_LOG_INFO(
			    "StageUpdate: set invMass/Inertia scales for %s (retries=%u)",
			    req.path.GetText(), req.attempts);

		} else {
			req.attempts++;
			if (req.attempts < MAX_QUEUED_ATTEMPTS) {
				retry.push_back(std::move(req));
			} else {
				CARB_LOG_WARN(
				    "StageUpdate: failed to set invMass/Inertia scales for %s "
				    "after %u attempts; giving up.",
				    req.path.GetText(), req.attempts);
			}
		}
	}

	if (!retry.empty()) {
		std::scoped_lock lock(g_pendingMutex);
		g_pending.insert(g_pending.begin(), retry.begin(), retry.end());
	}
}

// No-op callbacks — listener only
static void onAttach(long int /*stageId*/, double /*metersPerUnit*/,
                     void * /*userData*/) {}
static void onDetach(void * /*userData*/) {}
static void onPause(void * /*userData*/) {}
static void onStop(void * /*userData*/) {}
static void onResume(float /*currentTime*/, void * /*userData*/) {}
static void onUpdate(float /*currentTime*/, float /*elapsedSecs*/,
                     const omni::kit::StageUpdateSettings * /*settings*/,
                     void * /*userData*/) {
	ProcessJointInvMassInertiaQueue();
}
} // namespace

namespace lego_assemble {

bool CreateStageUpdateListener() {
	if (g_stageUpdateNode)
		return false;

	auto *suIface = carb::getCachedInterface<omni::kit::IStageUpdate>();
	if (!suIface)
		return false;

	g_stageUpdate = suIface->getStageUpdate();
	if (!g_stageUpdate)
		return false;

	omni::kit::StageUpdateNodeDesc desc = {nullptr};
	desc.displayName = "lego_assemble";
	desc.userData = nullptr;
	desc.order = 11; // Right after physics (order 10)
	desc.onAttach = onAttach;
	desc.onDetach = onDetach;
	desc.onPause = onPause;
	desc.onStop = onStop;
	desc.onResume = onResume;
	desc.onUpdate = onUpdate;

	g_stageUpdateNode = g_stageUpdate->createStageUpdateNode(desc);
	return g_stageUpdateNode != nullptr;
}

bool DestroyStageUpdateListener() {
	if (g_stageUpdate && g_stageUpdateNode) {
		g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
		g_stageUpdateNode = nullptr;
		return true;
	}
	return false;
}

bool EnqueueSetJointInvMassInertia(const pxr::SdfPath &jointPath,
                                   float invMassScale0, float invInertiaScale0,
                                   float invMassScale1,
                                   float invInertiaScale1) {
	JointScaleReq req{jointPath, invMassScale0, invInertiaScale0, invMassScale1,
	                  invInertiaScale1};
	std::scoped_lock lock(g_pendingMutex);
	g_pending.emplace_back(std::move(req));
	CARB_LOG_INFO("Queued joint inv-mass/inertia set: %s", jointPath.GetText());
	return true;
}

void ClearQueuedJointInvMassInertia() {
	std::scoped_lock lock(g_pendingMutex);
	g_pending.clear();
}

} // namespace lego_assemble
