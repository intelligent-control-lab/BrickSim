#pragma once

#include "LegoGraph.h"

#include <mutex>
#include <vector>

#include <PxRigidActor.h>
#include <foundation/PxTransform.h>

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
	void processPrimChanges();

  private:
	struct ConnInfo {
		pxr::SdfPath parent;
		pxr::SdfPath child;
		physx::PxTransform T_parent_local;
		physx::PxTransform T_child_local;
		float overlap_xy[2];
	};
	struct BrickInfo {
		pxr::SdfPath top_collider;
		pxr::SdfPath body_collider;
	};

	omni::physx::IPhysx *omni_px_;
	pxr::UsdStageRefPtr stage_;

	pxr::SdfPathTable<physx::PxRigidActor *> bodies_;
	pxr::SdfPathTable<std::pair<pxr::SdfPath, pxr::SdfPath>> conns_;
	LegoGraph graph_;
	std::vector<pxr::SdfPath> pendingChanges_;
	std::mutex mutex_;

	void loadFromStage_();
	bool getBrickInfo_(const pxr::UsdPrim &prim, BrickInfo &out) const;
	bool getConnInfo_(const pxr::UsdPrim &prim, ConnInfo &out) const;
};

} // namespace lego_assemble
