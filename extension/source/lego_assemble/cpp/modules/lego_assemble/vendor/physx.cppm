module;
#include <PxPhysicsAPI.h>
#include <foundation/PxMat34.h>

export module lego_assemble.vendor.physx;

// Re-export a curated PhysX API
export namespace physx {
using physx::Px1DConstraint;
using physx::Px1DConstraintFlag;
using physx::PxActor;
using physx::PxActorType;
using physx::PxBase;
using physx::PxConstraint;
using physx::PxConstraintConnector;
using physx::PxConstraintExtIDs;
using physx::PxConstraintFlag;
using physx::PxConstraintInfo;
using physx::PxConstraintInvMassScale;
using physx::PxConstraintShaderTable;
using physx::PxConstraintSolveHint;
using physx::PxConstraintSolverPrep;
using physx::PxContactPair;
using physx::PxContactPairExtraDataIterator;
using physx::PxContactPairFlag;
using physx::PxContactPairHeader;
using physx::PxContactPairHeaderFlag;
using physx::PxContactPairPose;
using physx::PxContactPatch;
using physx::PxFilterData;
using physx::PxFilterFlag;
using physx::PxFilterFlags;
using physx::PxFilterObjectAttributes;
using physx::PxIdentity;
using physx::PxMat33;
using physx::PxMat33T;
using physx::PxMat34;
using physx::PxMat34T;
using physx::PxMat44T;
using physx::PxPairFlag;
using physx::PxPairFlags;
using physx::PxPhysics;
using physx::PxPvdUpdateType;
using physx::PxQuat;
using physx::PxQuatT;
using physx::PxReal;
using physx::PxRigidActor;
using physx::PxRigidBody;
using physx::PxScene;
using physx::PxShape;
using physx::PxSimulationEventCallback;
using physx::PxSimulationFilterCallback;
using physx::PxSimulationFilterShader;
using physx::PxTransform;
using physx::PxTransform32;
using physx::PxTransformT;
using physx::PxTriggerPair;
using physx::PxU16;
using physx::PxU32;
using physx::PxU64;
using physx::PxVec2;
using physx::PxVec2d;
using physx::PxVec2T;
using physx::PxVec3;
using physx::PxVec3d;
using physx::PxVec3p;
using physx::PxVec3T;
using physx::PxVec4;
using physx::PxVec4d;
using physx::PxVec4T;

namespace pvdsdk {
using pvdsdk::PvdDataStream;
}

} // namespace physx
