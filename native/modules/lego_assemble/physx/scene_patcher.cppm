export module lego_assemble.physx.scene_patcher;

import std;
import lego_assemble.vendor;

namespace lego_assemble {

export using PxSimulationFilterShaderProxy = std::function<physx::PxFilterFlags(
    physx::PxSimulationFilterShader originalShader,
    physx::PxFilterObjectAttributes attributes0,
    physx::PxFilterData filterData0,
    physx::PxFilterObjectAttributes attributes1,
    physx::PxFilterData filterData1, physx::PxPairFlags &pairFlags,
    const void *constantBlock, physx::PxU32 constantBlockSize)>;

export class PxSimulationFilterCallbackProxy
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

export class PxSimulationEventCallbackProxy
    : public physx::PxSimulationEventCallback {
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

static std::mutex g_mutex;
static physx::PxScene *g_scene = nullptr;
static physx::PxSimulationFilterShader g_filterShader_wrapped = nullptr;
static PxSimulationFilterShaderProxy g_filterShader_proxy = nullptr;
static PxSimulationFilterCallbackProxy *g_filterCallback_proxy = nullptr;
static PxSimulationEventCallbackProxy *g_eventCallback_proxy = nullptr;

// PhysX 107.3-physx-5.6.1, Linux x86_64, gcc-11, with GPU, PUBLIC_RELEASE=0
//
// physx/source/physx/src/NpScene.h:747
// field mScene
// Type: Sc::Scene
// Offset: 1440 bytes
// Size: 5440 bytes, alignment 16 byte
//
// physx/source/simulationcontroller/include/ScScene.h:783
// field mFilterShader
// Type: PxSimulationFilterShader (aka PxFlags<physx::PxFilterFlag::Enum, unsigned short> (*)(unsigned int, physx::PxFilterData, unsigned int, physx::PxFilterData, PxFlags<physx::PxPairFlag::Enum, unsigned short> &, const void *, unsigned int))
// Offset: 1008 bytes
// Size: 8 bytes, alignment 8 bytes
//
// physx/source/simulationcontroller/include/ScScene.h:784
// field mFilterCallback
// Type: PxSimulationFilterCallback *
// Offset: 1016 bytes
// Size: 8 bytes, alignment 8 bytes
//
// physx/source/simulationcontroller/include/ScScene.h:805
// field mSimulationEventCallback
// Type: PxSimulationEventCallback *
// Offset: 1216 bytes
// Size: 8 bytes, alignment 8 bytes
//
// /home/yushijinhun/repos/PhysX/physx/source/physx/src/NpScene.h:653
// field mElapsedTime
// Type: PxReal (aka float)
// Offset: 968 bytes
// Size: 4 bytes, alignment 4 bytes
// needed to transfer the elapsed time param from the user to the sim thread.
static constexpr std::size_t offset_NpScene_mScene = 1440;
static constexpr std::size_t offset_NpScene_mScene_mFilterShader =
    offset_NpScene_mScene + 1008;
static constexpr std::size_t offset_NpScene_mScene_mFilterCallback =
    offset_NpScene_mScene + 1016;
static constexpr std::size_t offset_NpScene_mScene_mSimulationEventCallback =
    offset_NpScene_mScene + 1216;
static constexpr std::size_t offset_NpScene_mElapsedTime = 968;
static physx::PxSimulationFilterShader *
locate_mFilterShader(physx::PxScene *scene) {
	return reinterpret_cast<physx::PxSimulationFilterShader *>(
	    reinterpret_cast<unsigned char *>(scene) +
	    offset_NpScene_mScene_mFilterShader);
}
static physx::PxSimulationFilterCallback **
locate_mFilterCallback(physx::PxScene *scene) {
	return reinterpret_cast<physx::PxSimulationFilterCallback **>(
	    reinterpret_cast<unsigned char *>(scene) +
	    offset_NpScene_mScene_mFilterCallback);
}
static physx::PxSimulationEventCallback **
locate_mSimulationEventCallback(physx::PxScene *scene) {
	return reinterpret_cast<physx::PxSimulationEventCallback **>(
	    reinterpret_cast<unsigned char *>(scene) +
	    offset_NpScene_mScene_mSimulationEventCallback);
}
static physx::PxReal *locate_mElapsedTime(physx::PxScene *scene) {
	return reinterpret_cast<physx::PxReal *>(
	    reinterpret_cast<unsigned char *>(scene) + offset_NpScene_mElapsedTime);
}

static physx::PxFilterFlags entry_SimulationFilterShader(
    physx::PxFilterObjectAttributes attributes0,
    physx::PxFilterData filterData0,
    physx::PxFilterObjectAttributes attributes1,
    physx::PxFilterData filterData1, physx::PxPairFlags &pairFlags,
    const void *constantBlock, physx::PxU32 constantBlockSize) {
	if (g_filterShader_proxy) {
		return g_filterShader_proxy(
		    g_filterShader_wrapped, attributes0, filterData0, attributes1,
		    filterData1, pairFlags, constantBlock, constantBlockSize);
	} else {
		return g_filterShader_wrapped(attributes0, filterData0, attributes1,
		                              filterData1, pairFlags, constantBlock,
		                              constantBlockSize);
	}
}

export bool patchPxScene(physx::PxScene *scene) {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_scene != nullptr) {
		// Note g_scene can be the same as scene, but this doesn't mean they are the same object
		// it might be another object allocated at the same address.
		log_error("Previous PxScene {:p} hasn't been unpatched yet when "
		          "patching new PxScene {:p}",
		          static_cast<const void *>(g_scene),
		          static_cast<const void *>(scene));
		return false;
	}
	scene->lockWrite();

	auto *filterShader_ptr = locate_mFilterShader(scene);

	// Pre-patch checks
	bool ok = true;
	if (*filterShader_ptr == nullptr) {
		ok = false;
		log_error("NpScene->mScene->mFilterShader is null");
	} else if (*filterShader_ptr == &entry_SimulationFilterShader) {
		ok = false;
		log_error("NpScene->mScene->mFilterShader already patched");
	}

	// Perform patching
	if (ok) {
		g_scene = scene;
		g_filterShader_wrapped = *filterShader_ptr;
		*filterShader_ptr = entry_SimulationFilterShader;
		log_info(
		    "NpScene->mScene->mFilterShader patched, original {:p}, now {:p}",
		    reinterpret_cast<const void *>(g_filterShader_wrapped),
		    reinterpret_cast<const void *>(*filterShader_ptr));
	}

	scene->unlockWrite();
	return true;
}

export bool unpatchPxScene(bool restoreCallbacks) {
	std::lock_guard<std::mutex> lock(g_mutex);

	if (g_scene == nullptr) {
		return false;
	}

	if (restoreCallbacks) {
		g_scene->lockWrite();
		auto *filterShader_ptr = locate_mFilterShader(g_scene);
		auto *filterCallback_ptr = locate_mFilterCallback(g_scene);
		auto *eventCallback_ptr = locate_mSimulationEventCallback(g_scene);
		if (*filterShader_ptr == &entry_SimulationFilterShader) {
			*filterShader_ptr = g_filterShader_wrapped;
			log_info("NpScene->mScene->mFilterShader restored, now {:p}",
			         reinterpret_cast<const void *>(*filterShader_ptr));
		} else {
			log_warn("NpScene->mScene->mFilterShader changed since patched, "
			         "not restoring, continuing");
		}
		if (g_filterCallback_proxy != nullptr) {
			if (*filterCallback_ptr == g_filterCallback_proxy) {
				*filterCallback_ptr = g_filterCallback_proxy->wrapped;
				g_filterCallback_proxy->setWrapped(nullptr);
				log_info("NpScene->mScene->mFilterCallback restored, now {:p}",
				         static_cast<const void *>(*filterCallback_ptr));
			} else {
				log_warn(
				    "NpScene->mScene->mFilterCallback changed since patched, "
				    "not restoring, continuing");
			}
		}
		if (g_eventCallback_proxy != nullptr) {
			if (*eventCallback_ptr == g_eventCallback_proxy) {
				*eventCallback_ptr = g_eventCallback_proxy->wrapped;
				g_eventCallback_proxy->setWrapped(nullptr);
				log_info("NpScene->mScene->mSimulationEventCallback "
				         "restored, now {:p}",
				         static_cast<const void *>(*eventCallback_ptr));
			} else {
				log_warn("NpScene->mScene->mSimulationEventCallback changed "
				         "since patched, "
				         "not restoring, continuing");
			}
		}
		g_scene->unlockWrite();
	}

	g_filterShader_wrapped = nullptr;
	g_filterShader_proxy = nullptr;
	g_filterCallback_proxy = nullptr;
	g_eventCallback_proxy = nullptr;
	log_info("PxScene {:p} unpatched", static_cast<const void *>(g_scene));
	g_scene = nullptr;
	return true;
}

export bool setPxSimulationFilterShader(PxSimulationFilterShaderProxy cb) {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_scene == nullptr) {
		log_error("No PxScene patched");
		return false;
	}
	if (g_filterShader_proxy != nullptr) {
		log_error("PxSimulationFilterShaderProxy already set");
		return false;
	}
	g_filterShader_proxy = std::move(cb);
	log_info("PxSimulationFilterShaderProxy set");
	return true;
}
export bool clearPxSimulationFilterShader() {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_scene == nullptr) {
		log_error("No PxScene patched");
		return false;
	}
	if (g_filterShader_proxy == nullptr) {
		log_error("PxSimulationFilterShaderProxy not set");
		return false;
	}
	g_filterShader_proxy = nullptr;
	log_info("PxSimulationFilterShaderProxy cleared");
	return true;
}
export bool setPxSimulationFilterCallback(PxSimulationFilterCallbackProxy *cb) {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_scene == nullptr) {
		log_error("No PxScene patched");
		return false;
	}
	if (g_filterCallback_proxy != nullptr) {
		log_error("PxSimulationFilterCallbackProxy already set");
		return false;
	}
	auto *filterCallback_ptr = locate_mFilterCallback(g_scene);
	cb->setWrapped(*filterCallback_ptr);
	*filterCallback_ptr = cb;
	g_filterCallback_proxy = cb;
	log_info("PxSimulationFilterCallbackProxy set, now {:p}",
	         static_cast<const void *>(*filterCallback_ptr));
	return true;
}
export bool clearPxSimulationFilterCallback() {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_scene == nullptr) {
		log_error("No PxScene patched");
		return false;
	}
	if (g_filterCallback_proxy == nullptr) {
		log_error("PxSimulationFilterCallbackProxy not set");
		return false;
	}
	auto *filterCallback_ptr = locate_mFilterCallback(g_scene);
	if (*filterCallback_ptr == g_filterCallback_proxy) {
		*filterCallback_ptr = g_filterCallback_proxy->wrapped;
		g_filterCallback_proxy->setWrapped(nullptr);
		g_filterCallback_proxy = nullptr;
		log_info("PxSimulationFilterCallbackProxy cleared, now {:p}",
		         static_cast<const void *>(*filterCallback_ptr));
		return true;
	} else {
		log_warn(
		    "NpScene->mScene->mFilterCallback changed since set, not clearing, "
		    "continuing");
		return false;
	}
}
export bool setPxSimulationEventCallback(PxSimulationEventCallbackProxy *cb) {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_scene == nullptr) {
		log_error("No PxScene patched");
		return false;
	}
	if (g_eventCallback_proxy != nullptr) {
		log_error("PxSimulationEventCallbackProxy already set");
		return false;
	}
	auto *simulationEventCallback_ptr =
	    locate_mSimulationEventCallback(g_scene);
	cb->setWrapped(*simulationEventCallback_ptr);
	*simulationEventCallback_ptr = cb;
	g_eventCallback_proxy = cb;
	log_info("PxSimulationEventCallbackProxy set, now {:p}",
	         static_cast<const void *>(*simulationEventCallback_ptr));
	return true;
}
export bool clearPxSimulationEventCallback() {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_scene == nullptr) {
		log_error("No PxScene patched");
		return false;
	}
	if (g_eventCallback_proxy == nullptr) {
		log_error("PxSimulationEventCallbackProxy not set");
		return false;
	}
	auto *simulationEventCallback_ptr =
	    locate_mSimulationEventCallback(g_scene);
	if (*simulationEventCallback_ptr == g_eventCallback_proxy) {
		*simulationEventCallback_ptr = g_eventCallback_proxy->wrapped;
		g_eventCallback_proxy->setWrapped(nullptr);
		g_eventCallback_proxy = nullptr;
		log_info("PxSimulationEventCallbackProxy cleared, now {:p}",
		         static_cast<const void *>(*simulationEventCallback_ptr));
		return true;
	} else {
		log_warn(
		    "NpScene->mScene->mSimulationEventCallback changed since set, not "
		    "clearing, continuing");
		return false;
	}
}

export physx::PxReal getElapsedTime(const physx::PxScene *scene) {
	return *locate_mElapsedTime(const_cast<physx::PxScene *>(scene));
}

} // namespace lego_assemble
