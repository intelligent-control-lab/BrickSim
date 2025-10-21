#pragma once

#include <lego_assemble/Utils/PairHelpers.h>

#include "LegoGraph.h"

#include <mutex>
#include <vector>

#include <PxRigidActor.h>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/pathTable.h>
#include <pxr/usd/usd/stage.h>

#include <omni/physx/IPhysx.h>

namespace lego_assemble {

class LegoUsdBridge {
  public:
	explicit LegoUsdBridge(pxr::UsdStageRefPtr stage, physx::PxPhysics *px,
	                       omni::physx::IPhysx *omni_px);
	LegoUsdBridge(const LegoUsdBridge &) = delete;
	LegoUsdBridge &operator=(const LegoUsdBridge &) = delete;
	~LegoUsdBridge();

	void onRigidCreated(physx::PxRigidActor *actor,
	                    const pxr::SdfPath &primPath);
	void onRigidDestroyed(physx::PxRigidActor *actor,
	                      const pxr::SdfPath &primPath);
	void enqueuePrimChange(const pxr::SdfPath &primPath);
	void onPreStep();
	void onPostStep();
	LegoGraph &getGraph();

  private:
	struct ConnDesc {
		pxr::SdfPath parent_path;
		pxr::SdfPath child_path;
		std::uint64_t creation_time;
	};

	pxr::UsdStageRefPtr stage_;
	omni::physx::IPhysx *omni_px_;
	std::uint64_t current_time;

	pxr::SdfPathTable<physx::PxRigidActor *> bodies_;
	pxr::SdfPathTable<ConnDesc> conns_;
	std::unordered_map<physx::PxRigidActor *, pxr::SdfPath> body_rev_;
	std::unordered_map<std::pair<physx::PxRigidActor *, physx::PxRigidActor *>,
	                   pxr::SdfPath, PairHash<physx::PxRigidActor *>>
	    conn_rev_;
	LegoGraph graph_;
	std::vector<pxr::SdfPath> pendingChanges_;
	std::mutex mutex_;
};

} // namespace lego_assemble
