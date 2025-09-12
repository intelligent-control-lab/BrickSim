#pragma once

#include <pxr/usd/sdf/path.h>

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

} // namespace lego_assemble
