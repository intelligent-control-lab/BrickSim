#pragma once

#include "LegoBricks.h"

#include <memory>
#include <vector>

#include <PxRigidActor.h>
#include <PxShape.h>
#include <foundation/PxTransform.h>

namespace lego_assemble {

class LegoGraph {
  public:
	struct BrickInfo {
		std::array<BrickUnit, 3> dimensions;
		physx::PxShape *body_collider;
		physx::PxShape *top_collider;
	};
	struct ConnInfo {
		physx::PxTransform T_parent_local;
		physx::PxTransform T_child_local;
		float overlap_xy[2];
	};

	explicit LegoGraph(physx::PxPhysics *px);
	LegoGraph(const LegoGraph &) = delete;
	LegoGraph &operator=(const LegoGraph &) = delete;
	~LegoGraph();

	bool addRigidBody(physx::PxRigidActor *actor, const BrickInfo &info);
	bool removeRigidBody(physx::PxRigidActor *actor);
	bool connect(physx::PxRigidActor *a, physx::PxRigidActor *b, const ConnInfo &info);
	bool disconnect(physx::PxRigidActor *a, physx::PxRigidActor *b);
	std::vector<std::pair<physx::PxRigidActor *, physx::PxRigidActor *>> solveLimits();
	void clear();

  private:
	class Impl;
	std::unique_ptr<Impl> impl_;
};

}; // namespace lego_assemble
