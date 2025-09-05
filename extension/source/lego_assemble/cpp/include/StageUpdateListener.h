#pragma once

#include <pxr/usd/sdf/path.h>

namespace lego_assemble {

bool CreateStageUpdateListener();

bool DestroyStageUpdateListener();

// Queue a request to set joint inverse mass/inertia scales once the USD->PhysX
// mapping is ready. Returns true if queued.
bool EnqueueSetJointInvMassInertia(const pxr::SdfPath &jointPath,
                                   float invMassScale0,
                                   float invInertiaScale0,
                                   float invMassScale1,
                                   float invInertiaScale1);

// Clear any queued requests (useful on shutdown or reset).
void ClearQueuedJointInvMassInertia();

} // namespace lego_assemble
