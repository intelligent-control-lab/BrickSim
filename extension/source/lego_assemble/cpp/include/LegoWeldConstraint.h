#pragma once

#include <PxConstraint.h>
#include <PxPhysics.h>
#include <foundation/PxTransform.h>

namespace lego_assemble {

struct LegoWeldData {
	physx::PxTransform parentLocal; // joint frame in parent actor local space
	physx::PxTransform childLocal;  // joint frame in child actor local space
};

extern physx::PxConstraint *CreateLegoWeld(physx::PxPhysics &physics,
                                           physx::PxRigidActor *a,
                                           physx::PxRigidActor *b,
                                           LegoWeldData data);

} // namespace lego_assemble
