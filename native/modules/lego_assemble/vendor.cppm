module;

// See pxr/base/arch/defines.h
#define _GLIBCXX_PERMIT_BACKWARD_HASH
#define ARCH_HAS_GNU_STL_EXTENSIONS

// ==== STL ====
#include <format>
#include <source_location>

// ==== Eigen ====
#include <Eigen/Eigen>

// ==== nlohmann_json ====
#include <nlohmann/json.hpp>

// ==== PhysX ====
#include <PxPhysicsAPI.h>
#include <foundation/PxMat34.h>

// ==== Carb ====
#include <carb/BindingsUtils.h>
#include <carb/InterfaceUtils.h>
#include <carb/logging/Log.h>

// ==== pxr ====
#include <pxr/base/gf/matrix2d.h>
#include <pxr/base/gf/matrix2f.h>
#include <pxr/base/gf/matrix3d.h>
#include <pxr/base/gf/matrix3f.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/matrix4f.h>
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/traits.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec4d.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/tf/staticTokens.h>

// Order matters, this must be included before other USD headers
#include <pxr/usd/usd/schemaRegistry.h>

#include <pxr/usd/kind/registry.h>
#include <pxr/usd/sdf/attributeSpec.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/pathTable.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/sdf/relationshipSpec.h>
#include <pxr/usd/sdf/schema.h>
#include <pxr/usd/usd/notice.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/tokens.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformCache.h>
#include <pxr/usd/usdGeom/xformOp.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/usd/usdPhysics/metrics.h>
#include <pxr/usd/usdPhysics/tokens.h>
#include <pxr/usd/usdUtils/stageCache.h>

#include <physxSchema/tokens.h>

// ==== omni ====
// Order matters, this must be included before other USD headers
#include <pxr/usd/usd/schemaRegistry.h>

#include <omni/usd/UsdContextIncludes.h>

#include <omni/kit/IStageUpdate.h>
#include <omni/physx/IPhysx.h>
#include <omni/usd/UsdContext.h>

// ==== Carb binding definition ====
// Declare as a Carbonite bindings module for Python so logging and builtins
// are registered even when imported outside Kit, and to define CARB globals.
CARB_BINDINGS("lego_assemble", "python")

export module lego_assemble.vendor;

// ==== Eigen ====
export namespace Eigen {
using Eigen::Affine;
using Eigen::Affine2d;
using Eigen::Affine2f;
using Eigen::Affine3d;
using Eigen::Affine3f;
using Eigen::AngleAxis;
using Eigen::AngleAxisd;
using Eigen::AngleAxisf;
using Eigen::Isometry;
using Eigen::Isometry2d;
using Eigen::Isometry2f;
using Eigen::Isometry3d;
using Eigen::Isometry3f;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Matrix4d;
using Eigen::Matrix4f;
using Eigen::Quaternion;
using Eigen::Quaterniond;
using Eigen::Quaternionf;
using Eigen::Rotation2D;
using Eigen::Rotation2Dd;
using Eigen::Rotation2Df;
using Eigen::SimplicialLDLT;
using Eigen::SparseMatrix;
using Eigen::Success;
using Eigen::Transform;
using Eigen::Triplet;
using Eigen::Vector;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::Vector3i;
using Eigen::Vector4d;
using Eigen::Vector4f;
using Eigen::Vector4i;
using Eigen::VectorXd;
using Eigen::VectorXf;
using Eigen::VectorXi;
} // namespace Eigen

// ==== nlohmann_json ====
export namespace nlohmann {
using nlohmann::json;
using nlohmann::ordered_json;
} // namespace nlohmann

// ==== PhysX ====
export namespace physx {
using physx::Px1DConstraint;
using physx::Px1DConstraintFlag;
using physx::PxActor;
using physx::PxActorFlag;
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
using physx::PxDeletionEventFlag;
using physx::PxDeletionEventFlags;
using physx::PxDeletionListener;
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
using physx::PxRigidStatic;
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

// ==== carb ====
export namespace carb {
using carb::getCachedInterface;
} // namespace carb

// ==== pxr ====
export namespace pxr {
using pxr::GfBBox3d;
using pxr::GfIsGfMatrix;
using pxr::GfIsGfQuat;
using pxr::GfIsGfVec;
using pxr::GfMatrix2d;
using pxr::GfMatrix2f;
using pxr::GfMatrix3d;
using pxr::GfMatrix3f;
using pxr::GfMatrix4d;
using pxr::GfMatrix4f;
using pxr::GfQuatd;
using pxr::GfQuatf;
using pxr::GfRange1d;
using pxr::GfRange1f;
using pxr::GfRange2d;
using pxr::GfRange2f;
using pxr::GfRange3d;
using pxr::GfRotation;
using pxr::GfTransform;
using pxr::GfVec2d;
using pxr::GfVec2f;
using pxr::GfVec2i;
using pxr::GfVec3d;
using pxr::GfVec3f;
using pxr::GfVec3i;
using pxr::GfVec4d;
using pxr::GfVec4f;
using pxr::GfVec4i;
using pxr::KindTokens;
using pxr::PhysxSchemaTokens;
using pxr::SdfAttributeSpec;
using pxr::SdfAttributeSpecHandle;
using pxr::SdfChangeBlock;
using pxr::SdfCreatePrimInLayer;
using pxr::SdfFieldKeys;
using pxr::SdfLayerHandle;
using pxr::SdfPath;
using pxr::SdfPathTable;
using pxr::SdfPathVector;
using pxr::SdfPrimSpecHandle;
using pxr::SdfRelationshipSpec;
using pxr::SdfRelationshipSpecHandle;
using pxr::SdfSchema;
using pxr::SdfSpecHandle;
using pxr::SdfSpecifierClass;
using pxr::SdfSpecifierDef;
using pxr::SdfTokenListOp;
using pxr::SdfValueRoleNames;
using pxr::SdfValueTypeNames;
using pxr::SdfVariability;
using pxr::SdfVariabilityUniform;
using pxr::SdfVariabilityVarying;
using pxr::TfCreateNonConstWeakPtr;
using pxr::TfCreateRefPtr;
using pxr::TfCreateRefPtrFromProtectedWeakPtr;
using pxr::TfCreateWeakPtr;
using pxr::TfNotice;
using pxr::TfRefPtr;
using pxr::TfStaticData;
using pxr::TfToken;
using pxr::TfTokenVector;
using pxr::TfWeakBase;
using pxr::TfWeakPtr;
using pxr::UsdGeomBBoxCache;
using pxr::UsdGeomGetStageMetersPerUnit;
using pxr::UsdGeomTokens;
using pxr::UsdGeomXformable;
using pxr::UsdGeomXformCache;
using pxr::UsdGeomXformOp;
using pxr::UsdNotice;
using pxr::UsdPhysicsGetStageKilogramsPerUnit;
using pxr::UsdPhysicsTokens;
using pxr::UsdPrim;
using pxr::UsdPrimRange;
using pxr::UsdStage;
using pxr::UsdStageCache;
using pxr::UsdStageRefPtr;
using pxr::UsdStageWeakPtr;
using pxr::UsdTimeCode;
using pxr::UsdTimeCodeTokens;
using pxr::UsdTokens;
using pxr::UsdUtilsStageCache;
using pxr::VtIntArray;
using pxr::VtTokenArray;
using pxr::VtValue;
using pxr::VtVec3fArray;
} // namespace pxr

// ==== omni ====
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

// ==== Carb Logging Utils ====
namespace lego_assemble {

template <class... Args>
void _log(int level, std::source_location loc, std::format_string<Args...> fmt,
          Args &&...args) {
	if (!(g_carbLogFn && g_carbLogLevel <= level))
		return;
	g_carbLogFn(g_carbClientName, level, loc.file_name(), loc.function_name(),
	            static_cast<int>(loc.line()), "%s",
	            std::format(fmt, std::forward<Args>(args)...).c_str());
}

export template <class... Args>
void log_verbose(std::format_string<Args...> fmt, Args &&...args) {
	_log(carb::logging::kLevelVerbose, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args>
void log_info(std::format_string<Args...> fmt, Args &&...args) {
	_log(carb::logging::kLevelInfo, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args>
void log_warn(std::format_string<Args...> fmt, Args &&...args) {
	_log(carb::logging::kLevelWarn, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args>
void log_error(std::format_string<Args...> fmt, Args &&...args) {
	_log(carb::logging::kLevelError, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args>
void log_fatal(std::format_string<Args...> fmt, Args &&...args) {
	_log(carb::logging::kLevelFatal, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

} // namespace lego_assemble
