#include "BrickSpawner.h"
#include "LegoTokens.h"
#include "SdfUtils.h"

#include <format>

#include <physxSchema/tokens.h>
#include <pxr/usd/kind/registry.h>
#include <pxr/usd/usd/tokens.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformOp.h>
#include <pxr/usd/usdPhysics/metrics.h>
#include <pxr/usd/usdPhysics/tokens.h>

namespace lego_assemble {

static void constructBrickClass(const pxr::UsdStageRefPtr &stage,
                                const pxr::SdfPath &root_path,
                                const std::array<BrickUnit, 3> &dimensions,
                                const BrickColor &color) {
	auto mpu = pxr::UsdGeomGetStageMetersPerUnit(stage);
	auto kpu = pxr::UsdPhysicsGetStageKilogramsPerUnit(stage);
	auto realDimensions = brickDimensionsToMeters(dimensions);
	std::array<float, 3> fColor = {
	    color[0] / 255.0f,
	    color[1] / 255.0f,
	    color[2] / 255.0f,
	};

	auto layer = stage->GetEditTarget().GetLayer();
	pxr::SdfChangeBlock _changes;

	auto root = pxr::SdfCreatePrimInLayer(layer, root_path);
	root->SetSpecifier(pxr::SdfSpecifierClass);
	root->SetTypeName(pxr::UsdGeomTokens->Xform);
	SetInfo(root, pxr::UsdTokens->apiSchemas,
	        pxr::SdfTokenListOp::Create({
	            pxr::UsdPhysicsTokens->PhysicsRigidBodyAPI,
	            pxr::PhysxSchemaTokens->PhysxRigidBodyAPI,
	            pxr::PhysxSchemaTokens->PhysxContactReportAPI,
	        }));
	SetInfo(root, pxr::SdfFieldKeys->Kind, pxr::KindTokens->component);
	NewAttr<pxr::GfVec3i>(root, LegoTokens->brick_dimensions, dimensions);
	NewAttr<pxr::GfVec3i>(root, LegoTokens->brick_color, color);
	NewAttr<float>(root, pxr::PhysxSchemaTokens->physxContactReportThreshold,
	               0.0f);
	NewAttr<bool>(root, pxr::UsdPhysicsTokens->physicsRigidBodyEnabled, true);

	auto bodyCollider = pxr::SdfCreatePrimInLayer(
	    layer, root_path.AppendChild(LegoTokens->BodyCollider));
	bodyCollider->SetSpecifier(pxr::SdfSpecifierDef);
	bodyCollider->SetTypeName(pxr::UsdGeomTokens->Cube);
	SetInfo(bodyCollider, pxr::UsdTokens->apiSchemas,
	        pxr::SdfTokenListOp::Create({
	            pxr::UsdPhysicsTokens->PhysicsCollisionAPI,
	            pxr::UsdPhysicsTokens->PhysicsMassAPI,
	        }));
	NewAttr<double>(bodyCollider, pxr::UsdGeomTokens->size, 1.0);
	NewAttr<pxr::TfToken>(bodyCollider, pxr::UsdGeomTokens->visibility,
	                      pxr::UsdGeomTokens->invisible);
	NewAttr<pxr::GfVec3f>(bodyCollider, xformOpScale,
	                      {
	                          realDimensions[0] / mpu,
	                          realDimensions[1] / mpu,
	                          realDimensions[2] / mpu,
	                      });
	NewAttr<pxr::GfVec3d>(bodyCollider, xformOpTranslate,
	                      {
	                          0.0,
	                          0.0,
	                          realDimensions[2] / 2 / mpu,
	                      });
	NewAttr<pxr::VtTokenArray>(bodyCollider, pxr::UsdGeomTokens->xformOpOrder,
	                           {xformOpTranslate, xformOpScale});
	NewAttr<bool>(bodyCollider, pxr::UsdPhysicsTokens->physicsCollisionEnabled,
	              true);
	NewAttr<float>(bodyCollider, pxr::UsdPhysicsTokens->physicsMass,
	               brickMassInKg(dimensions) / kpu);

	auto topCollider = pxr::SdfCreatePrimInLayer(
	    layer, root_path.AppendChild(LegoTokens->TopCollider));
	topCollider->SetSpecifier(pxr::SdfSpecifierDef);
	topCollider->SetTypeName(pxr::UsdGeomTokens->Cube);
	SetInfo(topCollider, pxr::UsdTokens->apiSchemas,
	        pxr::SdfTokenListOp::Create({
	            pxr::UsdPhysicsTokens->PhysicsCollisionAPI,
	            pxr::UsdPhysicsTokens->PhysicsMassAPI,
	        }));
	NewAttr<double>(topCollider, pxr::UsdGeomTokens->size, 1.0);
	NewAttr<pxr::TfToken>(topCollider, pxr::UsdGeomTokens->visibility,
	                      pxr::UsdGeomTokens->invisible);
	NewAttr<pxr::GfVec3f>(topCollider, xformOpScale,
	                      {
	                          realDimensions[0] / mpu,
	                          realDimensions[1] / mpu,
	                          StudHeight / mpu,
	                      });
	NewAttr<pxr::GfVec3d>(topCollider, xformOpTranslate,
	                      {
	                          0.0,
	                          0.0,
	                          (realDimensions[2] + StudHeight / 2) / mpu,
	                      });
	NewAttr<pxr::VtTokenArray>(topCollider, pxr::UsdGeomTokens->xformOpOrder,
	                           {xformOpTranslate, xformOpScale});
	NewAttr<bool>(topCollider, pxr::UsdPhysicsTokens->physicsCollisionEnabled,
	              true);
	NewAttr<float>(topCollider, pxr::UsdPhysicsTokens->physicsMass, 0.0f);

	auto body = pxr::SdfCreatePrimInLayer(
	    layer, root_path.AppendChild(LegoTokens->Body));
	body->SetSpecifier(pxr::SdfSpecifierDef);
	body->SetTypeName(pxr::UsdGeomTokens->Cube);
	NewAttr<double>(body, pxr::UsdGeomTokens->size, 1.0);
	NewAttr<pxr::GfVec3f>(body, xformOpScale,
	                      {
	                          realDimensions[0] / mpu,
	                          realDimensions[1] / mpu,
	                          realDimensions[2] / mpu,
	                      });
	NewAttr<pxr::GfVec3d>(body, xformOpTranslate,
	                      {
	                          0.0,
	                          0.0,
	                          realDimensions[2] / 2 / mpu,
	                      });
	NewAttr<pxr::VtTokenArray>(body, pxr::UsdGeomTokens->xformOpOrder,
	                           {xformOpTranslate, xformOpScale});
	NewAttr<pxr::VtArray<pxr::GfVec3f>>(
	    body, pxr::UsdGeomTokens->primvarsDisplayColor,
	    {{fColor[0], fColor[1], fColor[2]}}, pxr::SdfValueRoleNames->Color);

	auto studPrototypePath = root_path.AppendChild(LegoTokens->StudPrototype);
	auto studPrototype = pxr::SdfCreatePrimInLayer(layer, studPrototypePath);
	studPrototype->SetSpecifier(pxr::SdfSpecifierClass);
	studPrototype->SetTypeName(pxr::UsdGeomTokens->Cylinder);
	NewAttr<double>(studPrototype, pxr::UsdGeomTokens->height, 1.0);
	NewAttr<pxr::VtVec3fArray>(
	    studPrototype, pxr::UsdGeomTokens->primvarsDisplayColor,
	    {{fColor[0], fColor[1], fColor[2]}}, pxr::SdfValueRoleNames->Color);
	NewAttr<pxr::GfVec3f>(studPrototype, xformOpScale,
	                      {
	                          StudDiameter / 2.0 / mpu,
	                          StudDiameter / 2.0 / mpu,
	                          StudHeight / mpu,
	                      });
	NewAttr<pxr::VtTokenArray>(studPrototype, pxr::UsdGeomTokens->xformOpOrder,
	                           {xformOpTranslate, xformOpScale});

	pxr::VtVec3fArray positions;
	positions.resize(dimensions[0] * dimensions[1]);
	for (int i = 0; i < dimensions[0]; i++) {
		for (int j = 0; j < dimensions[1]; j++) {
			double x_offset = (i - (dimensions[0] - 1) / 2.0) * BrickLength;
			double y_offset = (j - (dimensions[1] - 1) / 2.0) * BrickLength;
			double z_offset = realDimensions[2] + StudHeight / 2.0;
			positions[i * dimensions[1] + j] = {
			    static_cast<float>(x_offset / mpu),
			    static_cast<float>(y_offset / mpu),
			    static_cast<float>(z_offset / mpu),
			};
		}
	}

	auto studs = pxr::SdfCreatePrimInLayer(
	    layer, root_path.AppendChild(LegoTokens->Studs));
	studs->SetSpecifier(pxr::SdfSpecifierDef);
	studs->SetTypeName(pxr::UsdGeomTokens->PointInstancer);
	pxr::SdfRelationshipSpec::New(studs, pxr::UsdGeomTokens->prototypes)
	    ->GetTargetPathList()
	    .Add(studPrototypePath);
	NewAttr<pxr::VtVec3fArray>(studs, pxr::UsdGeomTokens->positions, positions,
	                           pxr::SdfValueRoleNames->Point);
	NewAttr<pxr::VtIntArray>(studs, pxr::UsdGeomTokens->protoIndices,
	                         pxr::VtIntArray(positions.size(), 0));
}

// This path MUST be sorted after /World due to Omni PhysX's bug.
// Defected functions:
// - omni::physics::schema::PrimIteratorMapRange::reset(...) (BUG)
// - omni::physx::usdparser::loadPhysicsFromPrimitive(...) (Calls above function)
// PrimIteratorMapRange::reset(...) does the following:
// 1. Builds a UsdPrimRange over the per-frame "physics change map" roots (lexical SdfPath order, RB-tree).
// 2. Advances to the first prim that passes USD's DefaultPredicate:
//     Active && Loaded && Defined && !Abstract
// 3. If the first root fails the predicate, it prunes once; if there's no next acceptable root,
//    the range is empty (mAtEnd stays 1) and the "heavy load" (PhysX rigid/shape creation) is skipped.
// Why path name order mattered:
// * Brick prototype prims are authored as Class (SdfSpecifierClass) -> Abstract=true -> fail DefaultPredicate.
// * With prototypes under "/BrickPrototypes", that path sorts before "/World". The first changed root
//   the loader sees is the Class prototype -> it prunes; often no next acceptable root that frame ->
//   empty range -> heavy load skipped -> no rigid.
// * Moving prototypes to "/_BrickPrototypes" makes them sort after "/World"
//   (ASCII 'W'=0x57, ''=0x5F). The first root becomes the /World instance (a Def Xform), which
//   passes DefaultPredicate, so loadPhysicsFromPrimitive runs the heavy path and creates the rigid.
static const pxr::SdfPath BricksPrototypesPath("/_BrickPrototypes");
static pxr::SdfPath ensureBrickClass(const pxr::UsdStageRefPtr &stage,
                                     const std::array<BrickUnit, 3> &dimensions,
                                     const BrickColor &color) {
	auto layer = stage->GetEditTarget().GetLayer();

	if (!layer->GetPrimAtPath(BricksPrototypesPath)) {
		pxr::SdfChangeBlock _changes;
		auto classRoot = pxr::SdfCreatePrimInLayer(layer, BricksPrototypesPath);
		classRoot->SetSpecifier(pxr::SdfSpecifierDef);
	}

	auto brickName =
	    std::format("Brick_{0}x{1}x{2}_{3:02x}{4:02x}{5:02x}", dimensions[0],
	                dimensions[1], dimensions[2], color[0], color[1], color[2]);
	auto brickPath = BricksPrototypesPath.AppendChild(pxr::TfToken(brickName));
	if (!layer->GetPrimAtPath(brickPath)) {
		constructBrickClass(stage, brickPath, dimensions, color);
	}
	return brickPath;
}

void createBrickPrim(const pxr::UsdStageRefPtr &stage, const pxr::SdfPath &path,
                     const std::array<BrickUnit, 3> &dimensions,
                     const BrickColor &color) {
	auto layer = stage->GetEditTarget().GetLayer();
	pxr::SdfChangeBlock _changes;
	auto classPath = ensureBrickClass(stage, dimensions, color);
	auto prim = pxr::SdfCreatePrimInLayer(layer, path);
	prim->SetSpecifier(pxr::SdfSpecifierDef);
	prim->GetInheritPathList().Add(classPath);
}

} // namespace lego_assemble
