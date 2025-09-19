#pragma once

#include <PxPhysicsAPI.h>
#include <optional>
#include <pxr/usd/sdf/path.h>
#include <tuple>

namespace lego_assemble {

bool initLegoJointManager();

bool deinitLegoJointManager();

std::optional<std::tuple<physx::PxVec3, physx::PxVec3>>
getPhysxJointForceTorque(const pxr::SdfPath &jointPath);

} // namespace lego_assemble
