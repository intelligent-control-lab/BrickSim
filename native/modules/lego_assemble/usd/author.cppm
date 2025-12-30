export module lego_assemble.usd.author;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.usd.tokens;
import lego_assemble.usd.specs;
import lego_assemble.utils.conversions;
import lego_assemble.utils.sdf;
import lego_assemble.utils.metric_system;
import lego_assemble.vendor;

namespace lego_assemble {

// ==== Parts ====

export template <class T>
concept PartAuthor =
    requires {
	    typename T::PartType;
	    requires PartLike<typename T::PartType>;
    } && requires(T t, const pxr::UsdStageRefPtr &stage,
                  const pxr::SdfPath &path, const typename T::PartType &part) {
	    { t(stage, path, part) } -> std::same_as<InterfaceCollidersVector>;
    };

export template <class T, class P>
concept PartPrototypeNaming =
    PartLike<P> && requires(T t, const P &part, const pxr::SdfPath &path) {
	    { t.get_name(part) } -> std::convertible_to<std::string>;
	    {
		    t.get_colliders(part, path)
	    } -> std::same_as<InterfaceCollidersVector>;
    };

const pxr::SdfPath PrototypesPath{"/_Prototypes"};

export template <PartAuthor PA, PartPrototypeNaming<typename PA::PartType> PN>
struct PrototypePartAuthor {
  public:
	using PartType = typename PA::PartType;

	InterfaceCollidersVector operator()(const pxr::UsdStageRefPtr &stage,
	                                    const pxr::SdfPath &path,
	                                    const PartType &part) {
		auto layer = stage->GetEditTarget().GetLayer();
		pxr::SdfChangeBlock _changes;
		auto class_path = ensure_prototype(stage, part);
		auto prim = pxr::SdfCreatePrimInLayer(layer, path);
		prim->SetSpecifier(pxr::SdfSpecifierDef);
		prim->GetInheritPathList().Add(class_path);
		return PN{}.get_colliders(part, path);
	}

  private:
	pxr::SdfPath ensure_prototype(const pxr::UsdStageRefPtr &stage,
	                              const PartType &part) {
		auto layer = stage->GetEditTarget().GetLayer();
		if (!layer->GetPrimAtPath(PrototypesPath)) {
			auto class_root = pxr::SdfCreatePrimInLayer(layer, PrototypesPath);
			class_root->SetSpecifier(pxr::SdfSpecifierDef);
			class_root->SetTypeName(pxr::UsdGeomTokens->Scope);
		}
		auto proto_name = PN{}.get_name(part);
		auto proto_path = PrototypesPath.AppendChild(pxr::TfToken(proto_name));
		if (!layer->GetPrimAtPath(proto_path)) {
			PA{}(stage, proto_path, part);
		}
		return proto_path;
	}
};

export struct SimpleBrickAuthor {
	using PartType = BrickPart;

	InterfaceCollidersVector operator()(const pxr::UsdStageRefPtr &stage,
	                                    const pxr::SdfPath &root_path,
	                                    const BrickPart &part) {
		MetricSystem metrics(stage);
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
		SetAttr<pxr::TfToken>(root, LegoTokens->PartKind,
		                      LegoTokens->PartKindBrick);
		SetAttr<pxr::GfVec3i>(root, LegoTokens->BrickDimensions, dimensions);
		SetAttr<pxr::GfVec3i>(root, LegoTokens->BrickColor, color);
		SetAttr<float>(
		    root, pxr::PhysxSchemaTokens->physxContactReportThreshold, 0.0f);
		SetAttr<bool>(root, pxr::UsdPhysicsTokens->physicsRigidBodyEnabled,
		              true);

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
		                          metrics.from_m(realDimensions[0]),
		                          metrics.from_m(realDimensions[1]),
		                          metrics.from_m(realDimensions[2]),
		                      });
		SetAttr<pxr::GfVec3d>(bodyCollider, xformOpTranslate,
		                      {
		                          metrics.from_m(0.0),
		                          metrics.from_m(0.0),
		                          metrics.from_m(realDimensions[2] / 2),
		                      });
		SetAttr<pxr::VtTokenArray>(bodyCollider,
		                           pxr::UsdGeomTokens->xformOpOrder,
		                           {xformOpTranslate, xformOpScale});
		SetAttr<bool>(bodyCollider,
		              pxr::UsdPhysicsTokens->physicsCollisionEnabled, true);
		SetAttr<float>(bodyCollider, pxr::UsdPhysicsTokens->physicsMass,
		               metrics.from_kg(part.mass()));
		SetAttr<float>(bodyCollider,
		               pxr::UsdPhysicsTokens->physicsStaticFriction, 0.5f);
		SetAttr<float>(bodyCollider,
		               pxr::UsdPhysicsTokens->physicsDynamicFriction, 0.4f);
		SetAttr<float>(bodyCollider, pxr::UsdPhysicsTokens->physicsRestitution,
		               0.1f);

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
		                          metrics.from_m(realDimensions[0]),
		                          metrics.from_m(realDimensions[1]),
		                          metrics.from_m(StudHeight),
		                      });
		SetAttr<pxr::GfVec3d>(
		    topCollider, xformOpTranslate,
		    {
		        metrics.from_m(0.0),
		        metrics.from_m(0.0),
		        metrics.from_m(realDimensions[2] + StudHeight / 2),
		    });
		SetAttr<pxr::VtTokenArray>(topCollider,
		                           pxr::UsdGeomTokens->xformOpOrder,
		                           {xformOpTranslate, xformOpScale});
		SetAttr<bool>(topCollider,
		              pxr::UsdPhysicsTokens->physicsCollisionEnabled, true);
		SetAttr<float>(topCollider, pxr::UsdPhysicsTokens->physicsMass, 0.0f);
		SetAttr<float>(topCollider,
		               pxr::UsdPhysicsTokens->physicsStaticFriction, 1.0f);
		SetAttr<float>(topCollider,
		               pxr::UsdPhysicsTokens->physicsDynamicFriction, 0.8f);
		SetAttr<float>(topCollider, pxr::UsdPhysicsTokens->physicsRestitution,
		               0.2f);

		auto body = pxr::SdfCreatePrimInLayer(
		    layer, root_path.AppendChild(LegoTokens->Body));
		body->SetSpecifier(pxr::SdfSpecifierDef);
		body->SetTypeName(pxr::UsdGeomTokens->Cube);
		SetAttr<double>(body, pxr::UsdGeomTokens->size, 1.0);
		SetAttr<pxr::GfVec3f>(body, xformOpScale,
		                      {
		                          metrics.from_m(realDimensions[0]),
		                          metrics.from_m(realDimensions[1]),
		                          metrics.from_m(realDimensions[2]),
		                      });
		SetAttr<pxr::GfVec3d>(body, xformOpTranslate,
		                      {
		                          metrics.from_m(0.0),
		                          metrics.from_m(0.0),
		                          metrics.from_m(realDimensions[2] / 2),
		                      });
		SetAttr<pxr::VtTokenArray>(body, pxr::UsdGeomTokens->xformOpOrder,
		                           {xformOpTranslate, xformOpScale});
		SetAttr<pxr::VtVec3fArray>(
		    body, pxr::UsdGeomTokens->primvarsDisplayColor,
		    {as<pxr::GfVec3f>(fColor)}, pxr::SdfValueRoleNames->Color);

		auto studPrototypePath =
		    root_path.AppendChild(LegoTokens->StudPrototype);
		auto studPrototype =
		    pxr::SdfCreatePrimInLayer(layer, studPrototypePath);
		studPrototype->SetSpecifier(pxr::SdfSpecifierClass);
		studPrototype->SetTypeName(pxr::UsdGeomTokens->Cylinder);
		SetAttr<double>(studPrototype, pxr::UsdGeomTokens->height, 1.0);
		SetAttr<pxr::VtVec3fArray>(
		    studPrototype, pxr::UsdGeomTokens->primvarsDisplayColor,
		    {as<pxr::GfVec3f>(fColor)}, pxr::SdfValueRoleNames->Color);
		SetAttr<pxr::GfVec3f>(studPrototype, xformOpScale,
		                      {
		                          metrics.from_m(StudDiameter / 2.0),
		                          metrics.from_m(StudDiameter / 2.0),
		                          metrics.from_m(StudHeight),
		                      });
		SetAttr<pxr::VtTokenArray>(studPrototype,
		                           pxr::UsdGeomTokens->xformOpOrder,
		                           {xformOpTranslate, xformOpScale});

		pxr::VtVec3fArray positions;
		positions.resize(dimensions[0] * dimensions[1]);
		for (int i = 0; i < dimensions[0]; i++) {
			for (int j = 0; j < dimensions[1]; j++) {
				double x_offset =
				    (i - (dimensions[0] - 1) / 2.0) * BrickUnitLength;
				double y_offset =
				    (j - (dimensions[1] - 1) / 2.0) * BrickUnitLength;
				double z_offset = realDimensions[2] + StudHeight / 2.0;
				positions[i * dimensions[1] + j] = {
				    static_cast<float>(metrics.from_m(x_offset)),
				    static_cast<float>(metrics.from_m(y_offset)),
				    static_cast<float>(metrics.from_m(z_offset)),
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
		SetAttr<pxr::VtVec3fArray>(studs, pxr::UsdGeomTokens->positions,
		                           positions, pxr::SdfValueRoleNames->Point);
		SetAttr<pxr::VtIntArray>(studs, pxr::UsdGeomTokens->protoIndices,
		                         pxr::VtIntArray(positions.size(), 0));

		return {
		    {BrickPart::HoleId, bodyCollider->GetPath()},
		    {BrickPart::StudId, topCollider->GetPath()},
		};
	}
};
static_assert(PartAuthor<SimpleBrickAuthor>);

export struct BrickPrototypeNaming {
	std::string get_name(const BrickPart &part) {
		auto color = part.color();
		auto L = part.L();
		auto W = part.W();
		auto H = part.H();
		return std::format("Brick_{0}x{1}x{2}_{3:02x}{4:02x}{5:02x}", L, W, H,
		                   color[0], color[1], color[2]);
	}
	InterfaceCollidersVector
	get_colliders([[maybe_unused]] const BrickPart &part,
	              const pxr::SdfPath &path) {
		return {
		    {BrickPart::HoleId, path.AppendChild(LegoTokens->BodyCollider)},
		    {BrickPart::StudId, path.AppendChild(LegoTokens->TopCollider)},
		};
	}
};
static_assert(PartPrototypeNaming<BrickPrototypeNaming, BrickPart>);

export using PrototypeBrickAuthor =
    PrototypePartAuthor<SimpleBrickAuthor, BrickPrototypeNaming>;
static_assert(PartAuthor<PrototypeBrickAuthor>);

// ==== Connections ====
export void author_connection(const pxr::UsdStageRefPtr &stage,
                              const pxr::SdfPath &path,
                              const pxr::SdfPath &stud, InterfaceId stud_if,
                              const pxr::SdfPath &hole, InterfaceId hole_if,
                              const ConnectionSegment &conn_seg) {
	auto layer = stage->GetEditTarget().GetLayer();
	pxr::SdfChangeBlock _changes;
	auto prim = pxr::SdfCreatePrimInLayer(layer, path);
	prim->SetSpecifier(pxr::SdfSpecifierDef);
	prim->SetTypeName(LegoTokens->Connection);
	SetRelationship(prim, LegoTokens->ConnStud, stud);
	SetRelationship(prim, LegoTokens->ConnHole, hole);
	SetAttr<int>(prim, LegoTokens->ConnStudInterface,
	             static_cast<int>(stud_if));
	SetAttr<int>(prim, LegoTokens->ConnHoleInterface,
	             static_cast<int>(hole_if));
	SetAttr<pxr::GfVec2i>(prim, LegoTokens->ConnOffset, conn_seg.offset);
	SetAttr<int>(prim, LegoTokens->ConnYaw, static_cast<int>(conn_seg.yaw));
}

} // namespace lego_assemble
