#pragma once

#include <PxActor.h>
#include <PxScene.h>
#include <PxShape.h>

namespace lego_assemble {

bool patchPxScene(physx::PxScene *scene);

bool addContactExclusion(const physx::PxActor *a0, const physx::PxShape *s0,
                         const physx::PxActor *a1, const physx::PxShape *s1);

bool removeContactExclusion(const physx::PxActor *a0, const physx::PxShape *s0,
                            const physx::PxActor *a1, const physx::PxShape *s1);

} // namespace lego_assemble
