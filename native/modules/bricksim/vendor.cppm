module;

// ==== Eigen ====
#include <Eigen/Eigen>

// ==== nlohmann_json ====
#include <nlohmann/json.hpp>

// ==== osqp ====
#include <osqp.h>

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
CARB_BINDINGS("bricksim", "python")

export module bricksim.vendor;

// NOLINTBEGIN(misc-unused-using-decls)

// ==== Eigen ====
export namespace Eigen {
using Eigen::Affine;
using Eigen::Affine2d;
using Eigen::Affine2f;
using Eigen::Affine3d;
using Eigen::Affine3f;
using Eigen::aligned_allocator;
using Eigen::AngleAxis;
using Eigen::AngleAxisd;
using Eigen::AngleAxisf;
using Eigen::COLAMDOrdering;
using Eigen::ColMajor;
using Eigen::ComputeFullU;
using Eigen::ComputeFullV;
using Eigen::ComputeThinU;
using Eigen::ComputeThinV;
using Eigen::Dynamic;
using Eigen::DynamicIndex;
using Eigen::Index;
using Eigen::Infinity;
using Eigen::Isometry;
using Eigen::Isometry2d;
using Eigen::Isometry2f;
using Eigen::Isometry3d;
using Eigen::Isometry3f;
using Eigen::JacobiSVD;
using Eigen::LLT;
using Eigen::Lower;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Matrix4d;
using Eigen::Matrix4f;
using Eigen::MatrixBase;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::Quaternion;
using Eigen::Quaterniond;
using Eigen::Quaternionf;
using Eigen::Ref;
using Eigen::Rotation2D;
using Eigen::Rotation2Dd;
using Eigen::Rotation2Df;
using Eigen::RowMajor;
using Eigen::SelfAdjointEigenSolver;
using Eigen::SimplicialLDLT;
using Eigen::SparseLU;
using Eigen::SparseMatrix;
using Eigen::SparseMatrixBase;
using Eigen::SparseQR;
using Eigen::StrictlyLower;
using Eigen::StrictlyUpper;
using Eigen::Success;
using Eigen::Transform;
using Eigen::Triplet;
using Eigen::Undefined;
using Eigen::Upper;
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
using Eigen::VectorX;
using Eigen::VectorXd;
using Eigen::VectorXf;
using Eigen::VectorXi;
} // namespace Eigen

// ==== nlohmann_json ====
export namespace nlohmann {
using nlohmann::adl_serializer;
using nlohmann::json;
using nlohmann::ordered_json;
} // namespace nlohmann

// ==== osqp ====
export namespace bricksim::vendor::osqp {
using ::osqp_adjoint_derivative_compute;
using ::osqp_adjoint_derivative_get_mat;
using ::osqp_adjoint_derivative_get_vec;
using ::OSQP_ALGEBRA_LOAD_ERROR;
using ::osqp_capabilities;
using ::osqp_capabilities_type;
using ::OSQP_CAPABILITY_CODEGEN;
using ::OSQP_CAPABILITY_DERIVATIVES;
using ::OSQP_CAPABILITY_DIRECT_SOLVER;
using ::OSQP_CAPABILITY_INDIRECT_SOLVER;
using ::OSQP_CAPABILITY_UPDATE_MATRICES;
using ::osqp_cleanup;
using ::osqp_codegen;
using ::OSQP_CODEGEN_DEFINES_ERROR;
using ::osqp_cold_start;
using ::OSQP_DATA_NOT_INITIALIZED;
using ::OSQP_DATA_VALIDATION_ERROR;
using ::OSQP_DIAGONAL_PRECONDITIONER;
using ::OSQP_DIRECT_SOLVER;
using ::OSQP_DUAL_INFEASIBLE;
using ::OSQP_DUAL_INFEASIBLE_INACCURATE;
using ::osqp_error_message;
using ::osqp_error_type;
using ::OSQP_FOPEN_ERROR;
using ::OSQP_FUNC_NOT_IMPLEMENTED;
using ::osqp_get_dimensions;
using ::osqp_get_solution;
using ::OSQP_INDIRECT_SOLVER;
using ::OSQP_LAST_ERROR_PLACE;
using ::OSQP_LINSYS_SOLVER_INIT_ERROR;
using ::osqp_linsys_solver_type;
using ::OSQP_MAX_ITER_REACHED;
using ::OSQP_MEM_ALLOC_ERROR;
using ::OSQP_NO_ERROR;
using ::OSQP_NO_PRECONDITIONER;
using ::OSQP_NON_CVX;
using ::OSQP_NONCVX_ERROR;
using ::OSQP_POLISH_FAILED;
using ::OSQP_POLISH_LINSYS_ERROR;
using ::OSQP_POLISH_NO_ACTIVE_SET_FOUND;
using ::OSQP_POLISH_NOT_PERFORMED;
using ::osqp_polish_status_type;
using ::OSQP_POLISH_SUCCESS;
using ::osqp_precond_type;
using ::OSQP_PRIMAL_INFEASIBLE;
using ::OSQP_PRIMAL_INFEASIBLE_INACCURATE;
using ::osqp_set_default_codegen_defines;
using ::osqp_set_default_settings;
using ::OSQP_SETTINGS_VALIDATION_ERROR;
using ::osqp_setup;
using ::OSQP_SIGINT;
using ::osqp_solve;
using ::OSQP_SOLVED;
using ::OSQP_SOLVED_INACCURATE;
using ::osqp_status_type;
using ::OSQP_TIME_LIMIT_REACHED;
using ::OSQP_UNKNOWN_SOLVER;
using ::OSQP_UNSOLVED;
using ::osqp_update_data_mat;
using ::osqp_update_data_vec;
using ::osqp_update_rho;
using ::osqp_update_settings;
using ::osqp_version;
using ::osqp_warm_start;
using ::OSQP_WORKSPACE_NOT_INIT_ERROR;
using ::OSQPCodegenDefines;
using ::OSQPCodegenDefines_free;
using ::OSQPCodegenDefines_new;
using ::OSQPCscMatrix;
using ::OSQPCscMatrix_diag_scalar;
using ::OSQPCscMatrix_diag_vec;
using ::OSQPCscMatrix_free;
using ::OSQPCscMatrix_identity;
using ::OSQPCscMatrix_new;
using ::OSQPCscMatrix_set_data;
using ::OSQPCscMatrix_zeros;
using ::OSQPFloat;
using ::OSQPInfo;
using ::OSQPInt;
using ::OSQPSettings;
using ::OSQPSettings_free;
using ::OSQPSettings_new;
using ::OSQPSolution;
using ::OSQPSolver;
using ::OSQPWorkspace;
}; // namespace bricksim::vendor::osqp

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
using physx::PxContact;
using physx::PxContactPair;
using physx::PxContactPairExtraDataIterator;
using physx::PxContactPairFlag;
using physx::PxContactPairFlags;
using physx::PxContactPairHeader;
using physx::PxContactPairHeaderFlag;
using physx::PxContactPairPose;
using physx::PxContactPairVelocity;
using physx::PxContactPatch;
using physx::PxContactStreamIterator;
using physx::PxDeletionEventFlag;
using physx::PxDeletionEventFlags;
using physx::PxDeletionListener;
using physx::PxExtendedContact;
using physx::PxFilterData;
using physx::PxFilterFlag;
using physx::PxFilterFlags;
using physx::PxFilterObjectAttributes;
using physx::PxFrictionAnchorStreamIterator;
using physx::PxFrictionPatch;
using physx::PxGpuContactPair;
using physx::PxIdentity;
using physx::PxMat33;
using physx::PxMat33T;
using physx::PxMat34;
using physx::PxMat34T;
using physx::PxMat44T;
using physx::PxMaterialFlags;
using physx::PxModifiableContact;
using physx::PxPairFlag;
using physx::PxPairFlags;
using physx::PxPhysics;
using physx::PxPvdUpdateType;
using physx::PxQuat;
using physx::PxQuatT;
using physx::PxReal;
using physx::PxRigidActor;
using physx::PxRigidBody;
using physx::PxRigidDynamic;
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
using physx::PxU8;
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
using physx::PxZero;

namespace pvdsdk {
using pvdsdk::PvdDataStream;
}

} // namespace physx

// ==== carb ====
export namespace carb {
using carb::getCachedInterface;

namespace logging {
using carb::logging::LogFn;
} // namespace logging
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
using pxr::UsdTraverseInstanceProxies;
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
using omni::physx::eSimulationComplete;
using omni::physx::eSimulationEnded;
using omni::physx::eSimulationStarting;
using omni::physx::IPhysicsObjectChangeCallback;
using omni::physx::IPhysx;
using omni::physx::PhysXType;
using omni::physx::SimulationStatusEvent;
using omni::physx::SubscriptionId;

namespace usdparser {
using omni::physx::usdparser::ObjectId;
} // namespace usdparser

} // namespace physx

namespace usd {
using omni::usd::UsdContext;
}
} // namespace omni

// NOLINTEND(misc-unused-using-decls)

namespace bricksim::vendor::carb {
export auto &g_carbLogFn() {
	return ::g_carbLogFn;
}
export auto &g_carbLogLevel() {
	return ::g_carbLogLevel;
}
export auto &g_carbClientName() {
	return ::g_carbClientName;
}
} // namespace bricksim::vendor::carb

namespace bricksim::vendor::osqp {
export constexpr ::OSQPFloat infinity = OSQP_INFTY;
export auto &get_status_message(osqp_status_type status) {
	return ::OSQP_STATUS_MESSAGE[status];
}
} // namespace bricksim::vendor::osqp
