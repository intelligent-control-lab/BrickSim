#pragma once

#include <memory>

#include <PxRigidActor.h>
#include <PxShape.h>
#include <foundation/PxTransform.h>

namespace lego_assemble {

class LegoGraph {
  public:
	explicit LegoGraph(physx::PxPhysics *px);
	LegoGraph(const LegoGraph &) = delete;
	LegoGraph &operator=(const LegoGraph &) = delete;
	~LegoGraph();

	bool addRigidBody(physx::PxRigidActor *actor, physx::PxShape *body_collider,
	                  physx::PxShape *top_collider);
	bool removeRigidBody(physx::PxRigidActor *actor);
	bool connect(physx::PxRigidActor *a, physx::PxRigidActor *b,
	             const physx::PxTransform &T_a_b);
	bool disconnect(physx::PxRigidActor *a, physx::PxRigidActor *b);
	void clear();

  private:
	class Impl;
	std::unique_ptr<Impl> impl_;
};

}; // namespace lego_assemble
