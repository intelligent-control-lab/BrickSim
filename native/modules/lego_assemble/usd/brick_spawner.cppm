export module lego_assemble.usd.brick_spawner;

import std;
import lego_assemble.core.specs;
import lego_assemble.usd.tokens;
import lego_assemble.utils.conversions;
import lego_assemble.utils.sdf;
import lego_assemble.vendor.pxr;

namespace lego_assemble {

static void constructBrickClass(const pxr::UsdStageRefPtr &stage,
                                const pxr::SdfPath &root_path,
                                const BrickPart &part) {
	auto mpu = pxr::UsdGeomGetStageMetersPerUnit(stage);
	auto kpu = pxr::UsdPhysicsGetStageKilogramsPerUnit(stage);
	std::array<BrickUnit, 3> dimensions{part.L(), part.W(), part.H()};
	std::array<double, 3> realDimensions{
	    dimensions[0] * BrickUnitLength,
	    dimensions[1] * BrickUnitLength,
	    dimensions[2] * PlateUnitHeight,
	};
	BrickColor color = part.color();
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
	SetAttr<pxr::GfVec3i>(root, LegoTokens->brick_dimensions, dimensions);
	SetAttr<pxr::GfVec3i>(root, LegoTokens->brick_color, color);
	SetAttr<float>(root, pxr::PhysxSchemaTokens->physxContactReportThreshold,
	               0.0f);
	SetAttr<bool>(root, pxr::UsdPhysicsTokens->physicsRigidBodyEnabled, true);

	auto bodyCollider = pxr::SdfCreatePrimInLayer(
	    layer, root_path.AppendChild(LegoTokens->BodyCollider));
	bodyCollider->SetSpecifier(pxr::SdfSpecifierDef);
	bodyCollider->SetTypeName(pxr::UsdGeomTokens->Cube);
	SetInfo(bodyCollider, pxr::UsdTokens->apiSchemas,
	        pxr::SdfTokenListOp::Create({
	            pxr::UsdPhysicsTokens->PhysicsCollisionAPI,
	            pxr::UsdPhysicsTokens->PhysicsMassAPI,
	        }));
	SetAttr<double>(bodyCollider, pxr::UsdGeomTokens->size, 1.0);
	SetAttr<pxr::TfToken>(bodyCollider, pxr::UsdGeomTokens->visibility,
	                      pxr::UsdGeomTokens->invisible);
	SetAttr<pxr::GfVec3f>(bodyCollider, xformOpScale,
	                      {
	                          realDimensions[0] / mpu,
	                          realDimensions[1] / mpu,
	                          realDimensions[2] / mpu,
	                      });
	SetAttr<pxr::GfVec3d>(bodyCollider, xformOpTranslate,
	                      {
	                          0.0,
	                          0.0,
	                          realDimensions[2] / 2 / mpu,
	                      });
	SetAttr<pxr::VtTokenArray>(bodyCollider, pxr::UsdGeomTokens->xformOpOrder,
	                           {xformOpTranslate, xformOpScale});
	SetAttr<bool>(bodyCollider, pxr::UsdPhysicsTokens->physicsCollisionEnabled,
	              true);
	SetAttr<float>(bodyCollider, pxr::UsdPhysicsTokens->physicsMass,
	               part.mass() / kpu);

	auto topCollider = pxr::SdfCreatePrimInLayer(
	    layer, root_path.AppendChild(LegoTokens->TopCollider));
	topCollider->SetSpecifier(pxr::SdfSpecifierDef);
	topCollider->SetTypeName(pxr::UsdGeomTokens->Cube);
	SetInfo(topCollider, pxr::UsdTokens->apiSchemas,
	        pxr::SdfTokenListOp::Create({
	            pxr::UsdPhysicsTokens->PhysicsCollisionAPI,
	            pxr::UsdPhysicsTokens->PhysicsMassAPI,
	        }));
	SetAttr<double>(topCollider, pxr::UsdGeomTokens->size, 1.0);
	SetAttr<pxr::TfToken>(topCollider, pxr::UsdGeomTokens->visibility,
	                      pxr::UsdGeomTokens->invisible);
	SetAttr<pxr::GfVec3f>(topCollider, xformOpScale,
	                      {
	                          realDimensions[0] / mpu,
	                          realDimensions[1] / mpu,
	                          StudHeight / mpu,
	                      });
	SetAttr<pxr::GfVec3d>(topCollider, xformOpTranslate,
	                      {
	                          0.0,
	                          0.0,
	                          (realDimensions[2] + StudHeight / 2) / mpu,
	                      });
	SetAttr<pxr::VtTokenArray>(topCollider, pxr::UsdGeomTokens->xformOpOrder,
	                           {xformOpTranslate, xformOpScale});
	SetAttr<bool>(topCollider, pxr::UsdPhysicsTokens->physicsCollisionEnabled,
	              true);
	SetAttr<float>(topCollider, pxr::UsdPhysicsTokens->physicsMass, 0.0f);

	auto body = pxr::SdfCreatePrimInLayer(
	    layer, root_path.AppendChild(LegoTokens->Body));
	body->SetSpecifier(pxr::SdfSpecifierDef);
	body->SetTypeName(pxr::UsdGeomTokens->Cube);
	SetAttr<double>(body, pxr::UsdGeomTokens->size, 1.0);
	SetAttr<pxr::GfVec3f>(body, xformOpScale,
	                      {
	                          realDimensions[0] / mpu,
	                          realDimensions[1] / mpu,
	                          realDimensions[2] / mpu,
	                      });
	SetAttr<pxr::GfVec3d>(body, xformOpTranslate,
	                      {
	                          0.0,
	                          0.0,
	                          realDimensions[2] / 2 / mpu,
	                      });
	SetAttr<pxr::VtTokenArray>(body, pxr::UsdGeomTokens->xformOpOrder,
	                           {xformOpTranslate, xformOpScale});
	SetAttr<pxr::VtVec3fArray>(body, pxr::UsdGeomTokens->primvarsDisplayColor,
	                           {as<pxr::GfVec3f>(fColor)},
	                           pxr::SdfValueRoleNames->Color);

	auto studPrototypePath = root_path.AppendChild(LegoTokens->StudPrototype);
	auto studPrototype = pxr::SdfCreatePrimInLayer(layer, studPrototypePath);
	studPrototype->SetSpecifier(pxr::SdfSpecifierClass);
	studPrototype->SetTypeName(pxr::UsdGeomTokens->Cylinder);
	SetAttr<double>(studPrototype, pxr::UsdGeomTokens->height, 1.0);
	SetAttr<pxr::VtVec3fArray>(
	    studPrototype, pxr::UsdGeomTokens->primvarsDisplayColor,
	    {as<pxr::GfVec3f>(fColor)}, pxr::SdfValueRoleNames->Color);
	SetAttr<pxr::GfVec3f>(studPrototype, xformOpScale,
	                      {
	                          StudDiameter / 2.0 / mpu,
	                          StudDiameter / 2.0 / mpu,
	                          StudHeight / mpu,
	                      });
	SetAttr<pxr::VtTokenArray>(studPrototype, pxr::UsdGeomTokens->xformOpOrder,
	                           {xformOpTranslate, xformOpScale});

	pxr::VtVec3fArray positions;
	positions.resize(dimensions[0] * dimensions[1]);
	for (int i = 0; i < dimensions[0]; i++) {
		for (int j = 0; j < dimensions[1]; j++) {
			double x_offset = (i - (dimensions[0] - 1) / 2.0) * BrickUnitLength;
			double y_offset = (j - (dimensions[1] - 1) / 2.0) * BrickUnitLength;
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
	SetAttr<pxr::VtVec3fArray>(studs, pxr::UsdGeomTokens->positions, positions,
	                           pxr::SdfValueRoleNames->Point);
	SetAttr<pxr::VtIntArray>(studs, pxr::UsdGeomTokens->protoIndices,
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
                                     const BrickPart &part) {
	auto layer = stage->GetEditTarget().GetLayer();

	if (!layer->GetPrimAtPath(BricksPrototypesPath)) {
		pxr::SdfChangeBlock _changes;
		auto classRoot = pxr::SdfCreatePrimInLayer(layer, BricksPrototypesPath);
		classRoot->SetSpecifier(pxr::SdfSpecifierDef);
	}

	auto color = part.color();
	auto brickName =
	    std::format("Brick_{0}x{1}x{2}_{3:02x}{4:02x}{5:02x}", part.L(),
	                part.W(), part.H(), color[0], color[1], color[2]);
	auto brickPath = BricksPrototypesPath.AppendChild(pxr::TfToken(brickName));
	if (!layer->GetPrimAtPath(brickPath)) {
		constructBrickClass(stage, brickPath, part);
	}
	return brickPath;
}

export void createBrickPrim(const pxr::UsdStageRefPtr &stage,
                            const pxr::SdfPath &path, const BrickPart &part) {
	auto layer = stage->GetEditTarget().GetLayer();
	pxr::SdfChangeBlock _changes;
	auto classPath = ensureBrickClass(stage, part);
	auto prim = pxr::SdfCreatePrimInLayer(layer, path);
	prim->SetSpecifier(pxr::SdfSpecifierDef);
	prim->GetInheritPathList().Add(classPath);
}

} // namespace lego_assemble
