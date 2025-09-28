#include "ScenePatcher.h"

#include <cassert>
#include <unordered_set>

#include <carb/logging/Log.h>

#include <PxPhysics.h>
#include <PxScene.h>
#include <foundation/PxArray.h>
#include <foundation/PxMutex.h>

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
		if (exclusions.contains({a0, s0, a1, s1})) {
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

static std::mutex g_mutex;
static physx::PxScene *g_scene = nullptr;
static physx::PxSimulationFilterShader g_shader_wrapped = nullptr;
static PxSimulationFilterCallbackWrapper *g_callback_wrapper = nullptr;

static physx::PxFilterFlags entry_SimulationFilterShader(
    physx::PxFilterObjectAttributes attributes0,
    physx::PxFilterData filterData0,
    physx::PxFilterObjectAttributes attributes1,
    physx::PxFilterData filterData1, physx::PxPairFlags &pairFlags,
    const void *constantBlock, physx::PxU32 constantBlockSize) {
	return g_shader_wrapped(attributes0, filterData0, attributes1, filterData1,
	                        pairFlags, constantBlock, constantBlockSize);
}

bool patchPxScene(physx::PxScene *scene) {
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
	// physx/source/physx/src/NpPhysics.h:311
	// field mSceneAndMaterialMutex
	// Type: PxMutex (aka PxMutexT<>)
	// Offset: 296 bytes
	// Size: 8 bytes, alignment 8 bytes
	//
	// physx/source/physx/src/NpPhysics.h:286
	// field mSceneArray
	// Type: PxArray<NpScene *>
	// Offset: 24 bytes
	// Size: 16 bytes, alignment 8 bytes
	constexpr size_t offset_mScene = 1440;
	constexpr size_t offset_mFilterShader = offset_mScene + 1008;
	constexpr size_t offset_mFilterCallback = offset_mScene + 1016;
	constexpr size_t offset_mSceneAndMaterialMutex = 296;
	constexpr size_t offset_mSceneArray = 24;

	std::lock_guard<std::mutex> lock(g_mutex);

	if (g_scene != nullptr) {
		if (g_scene == scene) {
			CARB_LOG_INFO("PxScene %p is already patched", scene);
			return true;
		}

		// Check if previous scene is already released
		auto &px = scene->getPhysics();
		auto *mutex_ptr = reinterpret_cast<physx::PxMutex *>(
		    reinterpret_cast<unsigned char *>(&px) +
		    offset_mSceneAndMaterialMutex);
		auto *scene_array_ptr = reinterpret_cast<physx::PxArray<void *> *>(
		    reinterpret_cast<unsigned char *>(&px) + offset_mSceneArray);
		{
			physx::PxMutex::ScopedLock lock(*mutex_ptr);
			if (scene_array_ptr->find(g_scene) != scene_array_ptr->end()) {
				CARB_LOG_ERROR("Previous patched PxScene %p is still alive "
				               "when patching new PxScene %p",
				               g_scene, scene);
				return false;
			}
		}
		// Previous scene is released, we can clear the wrapped shader and callback
		CARB_LOG_INFO("Previous patched PxScene %p is released", g_scene);
		g_scene = nullptr;
		if (g_shader_wrapped != nullptr) {
			CARB_LOG_INFO(
			    "Clearing previous wrapped PxSimulationFilterShader %p",
			    g_shader_wrapped);
			g_shader_wrapped = nullptr;
		}
		if (g_callback_wrapper != nullptr) {
			CARB_LOG_INFO(
			    "Destroying previous PxSimulationFilterCallbackWrapper %p",
			    g_callback_wrapper);
			delete g_callback_wrapper;
			g_callback_wrapper = nullptr;
		}
	}
	assert(g_shader_wrapped == nullptr);
	assert(g_callback_wrapper == nullptr);

	scene->lockWrite();

	auto *shader_ptr = reinterpret_cast<physx::PxSimulationFilterShader *>(
	    reinterpret_cast<unsigned char *>(scene) + offset_mFilterShader);
	auto **callback_ptr =
	    reinterpret_cast<physx::PxSimulationFilterCallback **>(
	        reinterpret_cast<unsigned char *>(scene) + offset_mFilterCallback);

	// Pre-patch checks
	bool ok = true;
	if (*shader_ptr == nullptr) {
		ok = false;
		CARB_LOG_ERROR("NpScene->mScene->mFilterShader is null");
	} else if (*shader_ptr == &entry_SimulationFilterShader) {
		ok = false;
		CARB_LOG_ERROR("NpScene->mScene->mFilterShader already patched");
	}
	if (*callback_ptr == nullptr) {
		ok = false;
		CARB_LOG_ERROR("NpScene->mScene->mFilterCallback is null");
	}

	// Perform patching
	if (ok) {
		g_scene = scene;
		g_shader_wrapped = *shader_ptr;
		*shader_ptr = entry_SimulationFilterShader;
		CARB_LOG_INFO(
		    "NpScene->mScene->mFilterShader patched, original %p, now %p",
		    g_shader_wrapped, *shader_ptr);

		g_callback_wrapper =
		    new PxSimulationFilterCallbackWrapper(*callback_ptr);
		*callback_ptr = g_callback_wrapper;
		CARB_LOG_INFO(
		    "NpScene->mScene->mFilterCallback patched, original %p, now %p",
		    g_callback_wrapper->wrapped, *callback_ptr);
	}

	scene->unlockWrite();

	return ok;
}

bool addContactExclusion(const physx::PxActor *a0, const physx::PxShape *s0,
                         const physx::PxActor *a1, const physx::PxShape *s1) {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_callback_wrapper == nullptr) {
		CARB_LOG_ERROR("No PxScene is patched");
		return false;
	}
	g_callback_wrapper->exclusions.insert({a0, s0, a1, s1});
	return true;
}

bool removeContactExclusion(const physx::PxActor *a0, const physx::PxShape *s0,
                            const physx::PxActor *a1,
                            const physx::PxShape *s1) {
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_callback_wrapper == nullptr) {
		CARB_LOG_ERROR("No PxScene is patched");
		return false;
	}
	return g_callback_wrapper->exclusions.erase({a0, s0, a1, s1}) > 0;
}

} // namespace lego_assemble
