module;

// Order matters, this must be included before other USD headers
#include <pxr/usd/usd/schemaRegistry.h>

#include <omni/usd/UsdContextIncludes.h>

#include <omni/kit/IStageUpdate.h>
#include <omni/physx/IPhysx.h>
#include <omni/usd/UsdContext.h>

export module lego_assemble.vendor.omni;

export namespace omni {

namespace kit {
using omni::kit::IStageUpdate;
using omni::kit::StageUpdate;
using omni::kit::StageUpdateNode;
using omni::kit::StageUpdateNodeDesc;
using omni::kit::StageUpdatePtr;
using omni::kit::StageUpdateSettings;
} // namespace kit

namespace physx {
using omni::physx::ePTActor;
using omni::physx::ePTPhysics;
using omni::physx::ePTShape;
using omni::physx::IPhysicsObjectChangeCallback;
using omni::physx::IPhysx;
using omni::physx::PhysXType;
using omni::physx::SubscriptionId;

namespace usdparser {
using omni::physx::usdparser::ObjectId;
} // namespace usdparser

} // namespace physx

namespace usd {
using omni::usd::UsdContext;
}
} // namespace omni

namespace lego_assemble::omni_physx {
using SubscriptionId = omni::physx::SubscriptionId;
export constexpr SubscriptionId kInvalidSubscriptionId(0xFFffFFffFF);
} // namespace lego_assemble::omni_physx
