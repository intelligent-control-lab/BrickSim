#include "ScenePatcher.h"

#include <cassert>
#include <unordered_set>

#include <carb/logging/Log.h>

#include <PxSimulationEventCallback.h>

namespace lego_assemble {

struct CollisionExcludePair {
	const physx::PxActor *a0;
	const physx::PxShape *s0;
	const physx::PxActor *a1;
	const physx::PxShape *s1;

	CollisionExcludePair(const physx::PxActor *A0, const physx::PxShape *S0,
	                     const physx::PxActor *A1, const physx::PxShape *S1) {
		std::less<const void *> less;
		const bool swap_needed = less(static_cast<const void *>(A1),
		                              static_cast<const void *>(A0)) ||
		                         (!(less(static_cast<const void *>(A0),
		                                 static_cast<const void *>(A1)) ||
		                            less(static_cast<const void *>(A1),
		                                 static_cast<const void *>(A0))) &&
		                          less(static_cast<const void *>(S1),
		                               static_cast<const void *>(S0)));

		if (swap_needed) {
			a0 = A1;
			s0 = S1;
			a1 = A0;
			s1 = S0;
		} else {
			a0 = A0;
			s0 = S0;
			a1 = A1;
			s1 = S1;
		}
	}

	bool operator==(const CollisionExcludePair &o) const {
		return (a0 == o.a0 && s0 == o.s0 && a1 == o.a1 && s1 == o.s1) ||
		       (a0 == o.a1 && s0 == o.s1 && a1 == o.a0 && s1 == o.s0);
	}

	struct Hasher {
		static std::size_t hashPtr(const void *p) {
			return std::hash<const void *>{}(p);
		}
		static std::size_t mix(std::size_t seed, std::size_t v) {
			seed ^= v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
			return seed;
		}
		static std::size_t hashSide(const physx::PxActor *a,
		                            const physx::PxShape *s) {
			std::size_t seed = 0;
			seed = mix(seed, hashPtr(a));
			seed = mix(seed, hashPtr(s));
			return seed;
		}
		std::size_t operator()(const CollisionExcludePair &p) const {
			const std::size_t h0 = hashSide(p.a0, p.s0);
			const std::size_t h1 = hashSide(p.a1, p.s1);
			return h0 ^ h1; // commutative
		}
	};
};

class PxSimulationFilterCallbackWrapper final
    : public physx::PxSimulationFilterCallback {
  public:
	physx::PxSimulationFilterCallback *const wrapped;
	std::unordered_set<CollisionExcludePair, CollisionExcludePair::Hasher>
	    exclusions;

	explicit PxSimulationFilterCallbackWrapper(
	    physx::PxSimulationFilterCallback *wrapped)
	    : wrapped(wrapped) {}
	virtual ~PxSimulationFilterCallbackWrapper() override {
		// mFilterCallback is not owned by ScScene, do not delete it here.
	}
	PxSimulationFilterCallbackWrapper(
	    const PxSimulationFilterCallbackWrapper &) = delete;
	PxSimulationFilterCallbackWrapper &
	operator=(const PxSimulationFilterCallbackWrapper &) = delete;

	virtual physx::PxFilterFlags pairFound(
	    physx::PxU64 pairID, physx::PxFilterObjectAttributes attributes0,
	    physx::PxFilterData filterData0, const physx::PxActor *a0,
	    const physx::PxShape *s0, physx::PxFilterObjectAttributes attributes1,
	    physx::PxFilterData filterData1, const physx::PxActor *a1,
	    const physx::PxShape *s1, physx::PxPairFlags &pairFlags) override {
		auto result =
		    wrapped->pairFound(pairID, attributes0, filterData0, a0, s0,
		                       attributes1, filterData1, a1, s1, pairFlags);
		if (exclusions.contains({a0, s0, a1, s1}) ||
		    exclusions.contains({a0, nullptr, a1, nullptr}) ||
		    exclusions.contains({a0, s0, a1, nullptr}) ||
		    exclusions.contains({a0, nullptr, a1, s1})) {
			result = physx::PxFilterFlag::eKILL;
		}
		return result;
	}

	virtual void pairLost(physx::PxU64 pairID,
	                      physx::PxFilterObjectAttributes attributes0,
	                      physx::PxFilterData filterData0,
	                      physx::PxFilterObjectAttributes attributes1,
	                      physx::PxFilterData filterData1,
	                      bool objectRemoved) override {
		wrapped->pairLost(pairID, attributes0, filterData0, attributes1,
		                  filterData1, objectRemoved);
	}

	virtual bool statusChange(physx::PxU64 &pairID,
	                          physx::PxPairFlags &pairFlags,
	                          physx::PxFilterFlags &filterFlags) override {
		return wrapped->statusChange(pairID, pairFlags, filterFlags);
	}
};

class PxSimulationEventCallbackWrapper final
    : public physx::PxSimulationEventCallback {
  public:
	physx::PxSimulationEventCallback *const wrapped;
	explicit PxSimulationEventCallbackWrapper(
	    physx::PxSimulationEventCallback *wrapped)
	    : wrapped(wrapped) {}
	virtual ~PxSimulationEventCallbackWrapper() override {
		// mSimulationEventCallback is not owned by ScScene, do not delete it here
	}
	PxSimulationEventCallbackWrapper(const PxSimulationEventCallbackWrapper &) =
	    delete;
	PxSimulationEventCallbackWrapper &
	operator=(const PxSimulationEventCallbackWrapper &) = delete;

	virtual void onConstraintBreak(physx::PxConstraintInfo *constraints,
	                               physx::PxU32 count) override {
		if (wrapped) {
			wrapped->onConstraintBreak(constraints, count);
		}
	}

	virtual void onWake(physx::PxActor **actors, physx::PxU32 count) override {
		if (wrapped) {
			wrapped->onWake(actors, count);
		}
	}

	virtual void onSleep(physx::PxActor **actors, physx::PxU32 count) override {
		if (wrapped) {
			wrapped->onSleep(actors, count);
		}
	}

	virtual void onContact(const physx::PxContactPairHeader &pairHeader,
	                       const physx::PxContactPair *pairs,
	                       physx::PxU32 nbPairs) override {
		if (wrapped) {
			wrapped->onContact(pairHeader, pairs, nbPairs);
		}
	}

	virtual void onTrigger(physx::PxTriggerPair *pairs,
	                       physx::PxU32 count) override {
		if (wrapped) {
			wrapped->onTrigger(pairs, count);
		}
	}

	virtual void onAdvance(const physx::PxRigidBody *const *bodyBuffer,
	                       const physx::PxTransform *poseBuffer,
	                       const physx::PxU32 count) override {
		if (wrapped) {
			wrapped->onAdvance(bodyBuffer, poseBuffer, count);
		}
	}
};

static std::mutex g_mutex;
static physx::PxScene *g_scene = nullptr;
static physx::PxSimulationFilterShader g_filterShader_wrapped = nullptr;
static PxSimulationFilterCallbackWrapper *g_filterCallback_wrapper = nullptr;
static PxSimulationEventCallbackWrapper *g_event_callback_wrapper = nullptr;

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
static constexpr size_t offset_NpScene_mScene = 1440;
static constexpr size_t offset_NpScene_mScene_mFilterShader =
    offset_NpScene_mScene + 1008;
static constexpr size_t offset_NpScene_mScene_mFilterCallback =
    offset_NpScene_mScene + 1016;
static constexpr size_t offset_NpScene_mScene_mSimulationEventCallback =
    offset_NpScene_mScene + 1216;
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

static physx::PxFilterFlags entry_SimulationFilterShader(
    physx::PxFilterObjectAttributes attributes0,
    physx::PxFilterData filterData0,
    physx::PxFilterObjectAttributes attributes1,
    physx::PxFilterData filterData1, physx::PxPairFlags &pairFlags,
    const void *constantBlock, physx::PxU32 constantBlockSize) {
	return g_filterShader_wrapped(attributes0, filterData0, attributes1,
	                              filterData1, pairFlags, constantBlock,
	                              constantBlockSize);
}

bool patchPxScene(physx::PxScene *scene) {
	std::lock_guard<std::mutex> lock(g_mutex);

	if (g_scene != nullptr) {
		// Note g_scene can be the same as scene, but this doesn't mean they are the same object
		// it might be another object allocated at the same address.
		CARB_LOG_ERROR("Previous PxScene %p hasn't been unpatched yet when "
		               "patching new PxScene %p",
		               g_scene, scene);
		return false;
	}
	assert(g_filterShader_wrapped == nullptr);
	assert(g_filterCallback_wrapper == nullptr);

	scene->lockWrite();

	auto *filterShader_ptr = locate_mFilterShader(scene);
	auto **filterCallback_ptr = locate_mFilterCallback(scene);
	auto **simulationEventCallback_ptr = locate_mSimulationEventCallback(scene);

	// Pre-patch checks
	bool ok = true;
	if (*filterShader_ptr == nullptr) {
		ok = false;
		CARB_LOG_ERROR("NpScene->mScene->mFilterShader is null");
	} else if (*filterShader_ptr == &entry_SimulationFilterShader) {
		ok = false;
		CARB_LOG_ERROR("NpScene->mScene->mFilterShader already patched");
	}
	if (*filterCallback_ptr == nullptr) {
		ok = false;
		CARB_LOG_ERROR("NpScene->mScene->mFilterCallback is null");
	}

	// Perform patching
	if (ok) {
		g_scene = scene;
		g_filterShader_wrapped = *filterShader_ptr;
		*filterShader_ptr = entry_SimulationFilterShader;
		CARB_LOG_INFO(
		    "NpScene->mScene->mFilterShader patched, original %p, now %p",
		    g_filterShader_wrapped, *filterShader_ptr);

		g_filterCallback_wrapper =
		    new PxSimulationFilterCallbackWrapper(*filterCallback_ptr);
		*filterCallback_ptr = g_filterCallback_wrapper;
		CARB_LOG_INFO(
		    "NpScene->mScene->mFilterCallback patched, original %p, now %p",
		    g_filterCallback_wrapper->wrapped, *filterCallback_ptr);
		g_event_callback_wrapper =
		    new PxSimulationEventCallbackWrapper(*simulationEventCallback_ptr);
		*simulationEventCallback_ptr = g_event_callback_wrapper;
		CARB_LOG_INFO("NpScene->mScene->mSimulationEventCallback patched, "
		              "original %p, now %p",
		              g_event_callback_wrapper->wrapped,
		              *simulationEventCallback_ptr);
	}

	scene->unlockWrite();

	return ok;
}

bool unpatchPxScene(bool restoreCallbacks) {
	std::lock_guard<std::mutex> lock(g_mutex);

	if (g_scene == nullptr) {
		return false;
	}
	assert(g_filterShader_wrapped != nullptr);
	assert(g_filterCallback_wrapper != nullptr);

	if (restoreCallbacks) {
		g_scene->lockWrite();
		auto *filterShader_ptr = locate_mFilterShader(g_scene);
		auto **filterCallback_ptr = locate_mFilterCallback(g_scene);
		auto **simulationEventCallback_ptr =
		    locate_mSimulationEventCallback(g_scene);
		if (*filterShader_ptr == &entry_SimulationFilterShader) {
			*filterShader_ptr = g_filterShader_wrapped;
			CARB_LOG_INFO("NpScene->mScene->mFilterShader restored, now %p",
			              *filterShader_ptr);
		} else {
			CARB_LOG_WARN(
			    "NpScene->mScene->mFilterShader changed since patched, "
			    "not restoring, continuing");
		}
		if (*filterCallback_ptr == g_filterCallback_wrapper) {
			*filterCallback_ptr = g_filterCallback_wrapper->wrapped;
			CARB_LOG_INFO("NpScene->mScene->mFilterCallback restored, now %p",
			              *filterCallback_ptr);
		} else {
			CARB_LOG_WARN(
			    "NpScene->mScene->mFilterCallback changed since patched, "
			    "not restoring, continuing");
		}
		if (*simulationEventCallback_ptr == g_event_callback_wrapper) {
			*simulationEventCallback_ptr = g_event_callback_wrapper->wrapped;
			CARB_LOG_INFO(
			    "NpScene->mScene->mSimulationEventCallback restored, now %p",
			    *simulationEventCallback_ptr);
		} else {
			CARB_LOG_WARN("NpScene->mScene->mSimulationEventCallback changed "
			              "since patched, "
			              "not restoring, continuing");
		}
		g_scene->unlockWrite();
	}

	delete g_filterCallback_wrapper;
	g_filterCallback_wrapper = nullptr;
	g_filterShader_wrapped = nullptr;
	delete g_event_callback_wrapper;
	g_event_callback_wrapper = nullptr;
	CARB_LOG_INFO("PxScene %p unpatched", g_scene);
	g_scene = nullptr;
	return true;
}

bool addContactExclusion(const physx::PxActor *a0, const physx::PxShape *s0,
                         const physx::PxActor *a1, const physx::PxShape *s1) {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_filterCallback_wrapper == nullptr) {
		CARB_LOG_ERROR("No PxScene is patched");
		return false;
	}
	return g_filterCallback_wrapper->exclusions.insert({a0, s0, a1, s1}).second;
}

bool removeContactExclusion(const physx::PxActor *a0, const physx::PxShape *s0,
                            const physx::PxActor *a1,
                            const physx::PxShape *s1) {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_filterCallback_wrapper == nullptr) {
		CARB_LOG_ERROR("No PxScene is patched");
		return false;
	}
	return g_filterCallback_wrapper->exclusions.erase({a0, s0, a1, s1}) > 0;
}

} // namespace lego_assemble
