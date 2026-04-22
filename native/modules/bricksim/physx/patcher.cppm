export module bricksim.physx.patcher;

import std;
import bricksim.utils.logging;
import bricksim.vendor;

namespace bricksim {

// ========== PhysX Internals ===========
// Identified offsets from Isaac Sim v6.0.0-dev2 binaries
// PhysX 110.0 / PhysX SDK 5.8.0, Linux x86_64

constexpr std::size_t offset_NpScene_mScene = 1424;
constexpr std::size_t offset_NpScene_mScene_mFilterCallback =
    offset_NpScene_mScene + 1016;
constexpr std::size_t offset_NpScene_mElapsedTime = 952;
physx::PxSimulationFilterCallback **
locate_mFilterCallback(physx::PxScene *scene) {
	return reinterpret_cast<physx::PxSimulationFilterCallback **>(
	    reinterpret_cast<unsigned char *>(scene) +
	    offset_NpScene_mScene_mFilterCallback);
}
physx::PxReal *locate_mElapsedTime(physx::PxScene *scene) {
	return reinterpret_cast<physx::PxReal *>(
	    reinterpret_cast<unsigned char *>(scene) + offset_NpScene_mElapsedTime);
}
export physx::PxReal getPxSceneElapsedTime(const physx::PxScene *scene) {
	return *locate_mElapsedTime(const_cast<physx::PxScene *>(scene));
}
// =====================

export class PxSimulationFilterPatch
    : public physx::PxSimulationFilterCallback {
  public:
	explicit PxSimulationFilterPatch(physx::PxScene *scene) : scene_{scene} {
		auto **cb_ptr = locate_mFilterCallback(scene_);
		wrapped_ = *cb_ptr;
		*cb_ptr = this;
		log_info("PxSimulationFilterPatch: patched PxSimulationFilterCallback");
	}

	virtual ~PxSimulationFilterPatch() {
		log_info("PxSimulationFilterPatch: unpatch on destruction");
		auto **cb_ptr = locate_mFilterCallback(scene_);
		if (*cb_ptr == this) {
			*cb_ptr = wrapped_;
		} else {
			log_error("PxSimulationFilterCallback changed since set, not "
			          "restoring, continuing");
		}
		wrapped_ = nullptr;
		scene_ = nullptr;
	}

	PxSimulationFilterPatch(const PxSimulationFilterPatch &) = delete;
	PxSimulationFilterPatch &
	operator=(const PxSimulationFilterPatch &) = delete;
	PxSimulationFilterPatch(PxSimulationFilterPatch &&) = delete;
	PxSimulationFilterPatch &operator=(PxSimulationFilterPatch &&) = delete;

	physx::PxSimulationFilterCallback *wrapped() const {
		return wrapped_;
	}

	virtual physx::PxFilterFlags pairFound(
	    physx::PxU64 pairID, physx::PxFilterObjectAttributes attributes0,
	    physx::PxFilterData filterData0, const physx::PxActor *a0,
	    const physx::PxShape *s0, physx::PxFilterObjectAttributes attributes1,
	    physx::PxFilterData filterData1, const physx::PxActor *a1,
	    const physx::PxShape *s1, physx::PxPairFlags &pairFlags) override {
		if (wrapped_ != nullptr) {
			return wrapped_->pairFound(pairID, attributes0, filterData0, a0, s0,
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
		if (wrapped_ != nullptr) {
			wrapped_->pairLost(pairID, attributes0, filterData0, attributes1,
			                   filterData1, objectRemoved);
		}
	}
	virtual bool statusChange(physx::PxU64 &pairID,
	                          physx::PxPairFlags &pairFlags,
	                          physx::PxFilterFlags &filterFlags) override {
		if (wrapped_) {
			return wrapped_->statusChange(pairID, pairFlags, filterFlags);
		} else {
			return false;
		}
	}

  private:
	physx::PxScene *scene_;
	physx::PxSimulationFilterCallback *wrapped_;
};

export class PxSimulationEventPatch : public physx::PxSimulationEventCallback {
  public:
	explicit PxSimulationEventPatch(physx::PxScene *scene) : scene_{scene} {
		wrapped_ = scene->getSimulationEventCallback();
		scene->setSimulationEventCallback(this);
		log_info("PxSimulationEventPatch: patched PxSimulationEventCallback");
	}
	virtual ~PxSimulationEventPatch() {
		log_info("PxSimulationEventPatch: unpatch on destruction");
		if (scene_->getSimulationEventCallback() == this) {
			scene_->setSimulationEventCallback(wrapped_);
		} else {
			log_error("PxSimulationEventCallback changed since set, not "
			          "restoring, continuing");
		}
		wrapped_ = nullptr;
		scene_ = nullptr;
	}

	PxSimulationEventPatch(const PxSimulationEventPatch &) = delete;
	PxSimulationEventPatch &operator=(const PxSimulationEventPatch &) = delete;
	PxSimulationEventPatch(PxSimulationEventPatch &&) = delete;
	PxSimulationEventPatch &operator=(PxSimulationEventPatch &&) = delete;

	physx::PxSimulationEventCallback *wrapped() const {
		return wrapped_;
	}

	virtual void onConstraintBreak(physx::PxConstraintInfo *constraints,
	                               physx::PxU32 count) override {
		if (wrapped_ != nullptr) {
			wrapped_->onConstraintBreak(constraints, count);
		}
	}
	virtual void onWake(physx::PxActor **actors, physx::PxU32 count) override {
		if (wrapped_ != nullptr) {
			wrapped_->onWake(actors, count);
		}
	}
	virtual void onSleep(physx::PxActor **actors, physx::PxU32 count) override {
		if (wrapped_ != nullptr) {
			wrapped_->onSleep(actors, count);
		}
	}
	virtual void onContact(const physx::PxContactPairHeader &pairHeader,
	                       const physx::PxContactPair *pairs,
	                       physx::PxU32 nbPairs) override {
		if (wrapped_ != nullptr) {
			wrapped_->onContact(pairHeader, pairs, nbPairs);
		}
	}
	virtual void onTrigger(physx::PxTriggerPair *pairs,
	                       physx::PxU32 count) override {
		if (wrapped_ != nullptr) {
			wrapped_->onTrigger(pairs, count);
		}
	}
	virtual void onAdvance(const physx::PxRigidBody *const *bodyBuffer,
	                       const physx::PxTransform *poseBuffer,
	                       const physx::PxU32 count) override {
		if (wrapped_ != nullptr) {
			wrapped_->onAdvance(bodyBuffer, poseBuffer, count);
		}
	}

  private:
	physx::PxScene *scene_;
	physx::PxSimulationEventCallback *wrapped_;
};

} // namespace bricksim
