#pragma once

#include <functional>

#include <PxFiltering.h>
#include <PxSimulationEventCallback.h>

namespace lego_assemble {

using PxSimulationFilterShaderProxy = std::function<physx::PxFilterFlags(
    physx::PxSimulationFilterShader originalShader,
    physx::PxFilterObjectAttributes attributes0,
    physx::PxFilterData filterData0,
    physx::PxFilterObjectAttributes attributes1,
    physx::PxFilterData filterData1, physx::PxPairFlags &pairFlags,
    const void *constantBlock, physx::PxU32 constantBlockSize)>;

class PxSimulationFilterCallbackProxy
    : public physx::PxSimulationFilterCallback {
  public:
	physx::PxSimulationFilterCallback *wrapped = nullptr;
	virtual ~PxSimulationFilterCallbackProxy() override {};
	virtual void setWrapped(physx::PxSimulationFilterCallback *w) {
		wrapped = w;
	}
	virtual physx::PxFilterFlags pairFound(
	    physx::PxU64 pairID, physx::PxFilterObjectAttributes attributes0,
	    physx::PxFilterData filterData0, const physx::PxActor *a0,
	    const physx::PxShape *s0, physx::PxFilterObjectAttributes attributes1,
	    physx::PxFilterData filterData1, const physx::PxActor *a1,
	    const physx::PxShape *s1, physx::PxPairFlags &pairFlags) override {
		if (wrapped != nullptr) {
			return wrapped->pairFound(pairID, attributes0, filterData0, a0, s0,
			                          attributes1, filterData1, a1, s1,
			                          pairFlags);
		} else {
			return {};
		}
	}
	virtual void pairLost(physx::PxU64 pairID,
	                      physx::PxFilterObjectAttributes attributes0,
	                      physx::PxFilterData filterData0,
	                      physx::PxFilterObjectAttributes attributes1,
	                      physx::PxFilterData filterData1,
	                      bool objectRemoved) override {
		if (wrapped != nullptr) {
			wrapped->pairLost(pairID, attributes0, filterData0, attributes1,
			                  filterData1, objectRemoved);
		}
	}
	virtual bool statusChange(physx::PxU64 &pairID,
	                          physx::PxPairFlags &pairFlags,
	                          physx::PxFilterFlags &filterFlags) override {
		if (wrapped) {
			return wrapped->statusChange(pairID, pairFlags, filterFlags);
		} else {
			return false;
		}
	}
};

class PxSimulationEventCallbackProxy : public physx::PxSimulationEventCallback {
  public:
	physx::PxSimulationEventCallback *wrapped = nullptr;
	virtual ~PxSimulationEventCallbackProxy() override {};
	virtual void setWrapped(physx::PxSimulationEventCallback *w) {
		wrapped = w;
	}
	virtual void onConstraintBreak(physx::PxConstraintInfo *constraints,
	                               physx::PxU32 count) override {
		if (wrapped != nullptr) {
			wrapped->onConstraintBreak(constraints, count);
		}
	}
	virtual void onWake(physx::PxActor **actors, physx::PxU32 count) override {
		if (wrapped != nullptr) {
			wrapped->onWake(actors, count);
		}
	}
	virtual void onSleep(physx::PxActor **actors, physx::PxU32 count) override {
		if (wrapped != nullptr) {
			wrapped->onSleep(actors, count);
		}
	}
	virtual void onContact(const physx::PxContactPairHeader &pairHeader,
	                       const physx::PxContactPair *pairs,
	                       physx::PxU32 nbPairs) override {
		if (wrapped != nullptr) {
			wrapped->onContact(pairHeader, pairs, nbPairs);
		}
	}
	virtual void onTrigger(physx::PxTriggerPair *pairs,
	                       physx::PxU32 count) override {
		if (wrapped != nullptr) {
			wrapped->onTrigger(pairs, count);
		}
	}
	virtual void onAdvance(const physx::PxRigidBody *const *bodyBuffer,
	                       const physx::PxTransform *poseBuffer,
	                       const physx::PxU32 count) override {
		if (wrapped != nullptr) {
			wrapped->onAdvance(bodyBuffer, poseBuffer, count);
		}
	}
};

bool patchPxScene(physx::PxScene *scene);

// The current PxScene must be valid when restoreCallbacks = true.
bool unpatchPxScene(bool restoreCallbacks);

bool setPxSimulationFilterShader(PxSimulationFilterShaderProxy cb);
bool clearPxSimulationFilterShader();
bool setPxSimulationFilterCallback(PxSimulationFilterCallbackProxy *cb);
bool clearPxSimulationFilterCallback();
bool setPxSimulationEventCallback(PxSimulationEventCallbackProxy *cb);
bool clearPxSimulationEventCallback();

} // namespace lego_assemble
