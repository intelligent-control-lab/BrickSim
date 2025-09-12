#pragma once

#include <PxPhysicsAPI.h>
#include <optional>
#include <pxr/usd/sdf/path.h>
#include <tuple>

namespace lego_assemble {

bool initLegoJointManager();

bool deinitLegoJointManager();

bool setPhysxJointInvMassInertia(const pxr::SdfPath &jointPath,
                                 float invMassScale0, float invInertiaScale0,
                                 float invMassScale1, float invInertiaScale1);

bool setDefaultLegoJointInvMassInertia(float invMassScale0,
                                       float invInertiaScale0,
                                       float invMassScale1,
                                       float invInertiaScale1);

std::optional<std::tuple<physx::PxVec3, physx::PxVec3>>
getPhysxJointForceTorque(const pxr::SdfPath &jointPath);

} // namespace lego_assemble
