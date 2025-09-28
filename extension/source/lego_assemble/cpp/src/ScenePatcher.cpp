#include <cassert>

#include <carb/logging/Log.h>

#include <PxPhysics.h>
#include <PxScene.h>
#include <foundation/PxArray.h>
#include <foundation/PxMutex.h>

namespace lego_assemble {

class PxSimulationFilterCallbackWrapper final
    : public physx::PxSimulationFilterCallback {
  public:
	physx::PxSimulationFilterCallback *const wrapped_;

	explicit PxSimulationFilterCallbackWrapper(
	    physx::PxSimulationFilterCallback *wrapped)
	    : wrapped_(wrapped) {}
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
		// TODO
		return wrapped_->pairFound(pairID, attributes0, filterData0, a0, s0,
		                           attributes1, filterData1, a1, s1, pairFlags);
	}

	virtual void pairLost(physx::PxU64 pairID,
	                      physx::PxFilterObjectAttributes attributes0,
	                      physx::PxFilterData filterData0,
	                      physx::PxFilterObjectAttributes attributes1,
	                      physx::PxFilterData filterData1,
	                      bool objectRemoved) override {
		// TODO
		wrapped_->pairLost(pairID, attributes0, filterData0, attributes1,
		                   filterData1, objectRemoved);
	}

	virtual bool statusChange(physx::PxU64 &pairID,
	                          physx::PxPairFlags &pairFlags,
	                          physx::PxFilterFlags &filterFlags) override {
		// TODO
		return wrapped_->statusChange(pairID, pairFlags, filterFlags);
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
	// TODO
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
		    g_callback_wrapper->wrapped_, *callback_ptr);
	}

	scene->unlockWrite();

	return ok;
}

} // namespace lego_assemble
