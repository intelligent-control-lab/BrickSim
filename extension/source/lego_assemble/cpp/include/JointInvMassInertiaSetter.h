#pragma once

#include <pxr/usd/sdf/path.h>

namespace lego_assemble {

bool initJointInvMassInertiaSetter();

bool deinitJointInvMassInertiaSetter();

bool setJointInvMassInertia(const pxr::SdfPath &jointPath, float invMassScale0,
                            float invInertiaScale0, float invMassScale1,
                            float invInertiaScale1);

} // namespace lego_assemble
