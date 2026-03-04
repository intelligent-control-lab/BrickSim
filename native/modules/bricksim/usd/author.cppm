export module bricksim.usd.author;

import std;
import bricksim.core.specs;
import bricksim.core.connections;
import bricksim.usd.tokens;
import bricksim.usd.specs;
import bricksim.usd.geometry;
import bricksim.usd.material;
import bricksim.utils.conversions;
import bricksim.utils.sdf;
import bricksim.utils.metric_system;
import bricksim.utils.strings;
import bricksim.utils.logging;
import bricksim.vendor;

namespace bricksim {

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
	    { t.parse_name(std::string{}) } -> std::same_as<std::optional<P>>;
	    {
		    t.get_colliders(part, path)
	    } -> std::same_as<InterfaceCollidersVector>;
    };

export template <class T>
concept PrototypePartAuthorLike =
    PartAuthor<T> && requires(T t, const pxr::UsdStageRefPtr &stage) {
	    { t.update_prototypes(stage) };
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
		auto inherits_list = prim->GetInheritPathList();
		inherits_list.ClearEditsAndMakeExplicit();
		inherits_list.Add(class_path);
		return PN{}.get_colliders(part, path);
	}

	void update_prototypes(const pxr::UsdStageRefPtr &stage) {
		pxr::SdfChangeBlock _changes;
		auto layer = stage->GetEditTarget().GetLayer();
		auto prototypes_root = layer->GetPrimAtPath(PrototypesPath);
		if (!prototypes_root) {
			return;
		}
		auto prototypes = prototypes_root->GetNameChildren();
		for (const auto &prototype : prototypes) {
			auto part = PN{}.parse_name(prototype->GetName());
			if (!part) {
				continue;
			}
			pxr::SdfPath prototype_path = prototype->GetPath();
			prototype->SetNameChildren({});
			PA{}(stage, prototype_path, *part);
			log_info("Updated prototype {}", prototype_path.GetAsString());
		}
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

void setup_mass_properties(const MetricSystem &metrics,
                           const pxr::SdfPrimSpecHandle &prim,
                           const PartLike auto &part) {
	double mass = part.mass();
	Eigen::Vector3d com = part.com();
	Eigen::Matrix3d I = part.inertia_tensor();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver{I};
	if (solver.info() != Eigen::Success) {
		throw std::runtime_error(
		    "setup_mass_properties: eigen decomposition failed");
	}

	Eigen::Vector3d com_u = metrics.from_m(com);
	Eigen::Vector3d diag_u = metrics.from_kgm2(solver.eigenvalues());

	Eigen::Matrix3d Q = solver.eigenvectors();
	if (Q.determinant() < 0.0) {
		Q.col(0) *= -1.0;
	}
	Eigen::Matrix3d R = Q.transpose();
	Eigen::Quaterniond q_pa = Eigen::Quaterniond(R).normalized();
	if (q_pa.w() < 0.0) {
		q_pa.coeffs() *= -1.0;
	}

	SetAttr<float>(prim, pxr::UsdPhysicsTokens->physicsMass,
	               static_cast<float>(metrics.from_kg(mass)));
	SetAttr<pxr::GfVec3f>(prim, pxr::UsdPhysicsTokens->physicsCenterOfMass,
	                      as<pxr::GfVec3f>(com_u));
	SetAttr<pxr::GfVec3f>(prim, pxr::UsdPhysicsTokens->physicsDiagonalInertia,
	                      as<pxr::GfVec3f>(diag_u));
	SetAttr<pxr::GfQuatf>(prim, pxr::UsdPhysicsTokens->physicsPrincipalAxes,
	                      as<pxr::GfQuatf>(q_pa));
}

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
		            pxr::UsdPhysicsTokens->PhysicsMassAPI,
		        }));
		SetInfo(root, pxr::SdfFieldKeys->Kind, pxr::KindTokens->component);
		SetAttr<pxr::TfToken>(root, LegoTokens->PartKind,
		                      LegoTokens->PartKindBrick);
		SetAttr<pxr::GfVec3i>(root, LegoTokens->BrickDimensions, dimensions);
		SetAttr<pxr::GfVec3i>(root, LegoTokens->BrickColor, color);
		SetAttr<float>(
		    root, pxr::PhysxSchemaTokens->physxContactReportThreshold, 0.0f);
		SetAttr<float>(
		    root, pxr::PhysxSchemaTokens->physxRigidBodySleepThreshold, 0.0f);
		SetAttr<bool>(root, pxr::UsdPhysicsTokens->physicsRigidBodyEnabled,
		              true);
		setup_mass_properties(metrics, root, part);

		auto bodyCollider = pxr::SdfCreatePrimInLayer(
		    layer, root_path.AppendChild(LegoTokens->BodyCollider));
		bodyCollider->SetSpecifier(pxr::SdfSpecifierDef);
		bodyCollider->SetTypeName(pxr::UsdGeomTokens->Cube);
		SetInfo(bodyCollider, pxr::UsdTokens->apiSchemas,
		        pxr::SdfTokenListOp::Create(
		            {pxr::UsdPhysicsTokens->PhysicsCollisionAPI}));
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
		        pxr::SdfTokenListOp::Create(
		            {pxr::UsdPhysicsTokens->PhysicsCollisionAPI}));
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
		SetAttr<float>(topCollider,
		               pxr::UsdPhysicsTokens->physicsStaticFriction, 1.0f);
		SetAttr<float>(topCollider,
		               pxr::UsdPhysicsTokens->physicsDynamicFriction, 0.8f);
		SetAttr<float>(topCollider, pxr::UsdPhysicsTokens->physicsRestitution,
		               0.2f);

		auto materialPath = root_path.AppendChild(LegoTokens->LegoMaterial);
		create_lego_material(layer, materialPath, color);

		constexpr double TolerancePerSide = 5e-5;
		constexpr double ToleranceUpFace = 5e-5;
		bool is1xN = dimensions[0] == 1 || dimensions[1] == 1;
		double body_wall_thickness = is1xN ? 1.5e-3 : 1.2e-3;
		double body_visual_height = realDimensions[2] - ToleranceUpFace;
		auto bodyPath = root_path.AppendChild(LegoTokens->Body);
		make_open_box(
		    layer, bodyPath,
		    {
		        metrics.from_m(realDimensions[0] - 2 * TolerancePerSide),
		        metrics.from_m(realDimensions[1] - 2 * TolerancePerSide),
		        metrics.from_m(body_visual_height),
		    },
		    metrics.from_m(body_wall_thickness), OpenFace::NegZ);
		auto body = layer->GetPrimAtPath(bodyPath);
		SetAttr<pxr::GfVec3d>(body, xformOpTranslate,
		                      {
		                          0.0,
		                          0.0,
		                          metrics.from_m(body_visual_height / 2.0),
		                      });
		SetAttr<pxr::VtTokenArray>(body, pxr::UsdGeomTokens->xformOpOrder,
		                           {xformOpTranslate, xformOpOrient});
		SetRelationship(body, pxr::UsdShadeTokens->materialBinding,
		                materialPath);

		if (is1xN) {
			if (dimensions[0] > 1 || dimensions[1] > 1) {
				constexpr double PillarRadius = 1.5e-3;
				constexpr double PillarHeightTolerance = 1e-4;
				double pillar_height = body_visual_height -
				                       body_wall_thickness -
				                       PillarHeightTolerance;
				auto pillarPrototypePath =
				    root_path.AppendChild(LegoTokens->PillarPrototype);
				auto pillarPrototype =
				    pxr::SdfCreatePrimInLayer(layer, pillarPrototypePath);
				pillarPrototype->SetSpecifier(pxr::SdfSpecifierClass);
				pillarPrototype->SetTypeName(pxr::UsdGeomTokens->Cylinder);
				SetAttr<double>(pillarPrototype, pxr::UsdGeomTokens->height,
				                1.0);
				SetAttr<pxr::GfVec3f>(pillarPrototype, xformOpScale,
				                      {
				                          metrics.from_m(PillarRadius),
				                          metrics.from_m(PillarRadius),
				                          metrics.from_m(pillar_height),
				                      });
				SetAttr<pxr::VtTokenArray>(pillarPrototype,
				                           pxr::UsdGeomTokens->xformOpOrder,
				                           {xformOpTranslate, xformOpScale});
				SetRelationship(pillarPrototype,
				                pxr::UsdShadeTokens->materialBinding,
				                materialPath);
				pxr::VtVec3fArray positions;
				int pillar_count = std::max(dimensions[0], dimensions[1]) - 1;
				positions.resize(pillar_count);
				for (int k = 0; k < pillar_count; k++) {
					double x_offset =
					    dimensions[0] == 1
					        ? 0.0
					        : (k + 1 - dimensions[0] / 2.0) * BrickUnitLength;
					double y_offset =
					    dimensions[1] == 1
					        ? 0.0
					        : (k + 1 - dimensions[1] / 2.0) * BrickUnitLength;
					double z_offset =
					    pillar_height / 2.0 + PillarHeightTolerance;
					positions[k] = {
					    static_cast<float>(metrics.from_m(x_offset)),
					    static_cast<float>(metrics.from_m(y_offset)),
					    static_cast<float>(metrics.from_m(z_offset)),
					};
				}
				auto pillars = pxr::SdfCreatePrimInLayer(
				    layer, root_path.AppendChild(LegoTokens->Pillars));
				pillars->SetSpecifier(pxr::SdfSpecifierDef);
				pillars->SetTypeName(pxr::UsdGeomTokens->PointInstancer);
				pxr::SdfRelationshipSpec::New(pillars,
				                              pxr::UsdGeomTokens->prototypes)
				    ->GetTargetPathList()
				    .Add(pillarPrototypePath);
				SetAttr<pxr::VtVec3fArray>(
				    pillars, pxr::UsdGeomTokens->positions, positions,
				    pxr::SdfValueRoleNames->Point);
				SetAttr<pxr::VtIntArray>(pillars,
				                         pxr::UsdGeomTokens->protoIndices,
				                         pxr::VtIntArray(positions.size(), 0));
			}
		} else {
			constexpr double TubeThickness = 8e-4;
			constexpr double TubeOuterRadius =
			    StudDiameter / 2.0 + TubeThickness;
			constexpr double TubeHeightTolerance = 1e-4;
			double tube_height =
			    body_visual_height - body_wall_thickness - TubeHeightTolerance;
			auto tubePrototypePath =
			    root_path.AppendChild(LegoTokens->TubePrototype);
			make_hollow_cylinder(layer, tubePrototypePath, TubeOuterRadius,
			                     TubeThickness, tube_height, 64, false, true);
			auto tubePrototype = layer->GetPrimAtPath(tubePrototypePath);
			tubePrototype->SetSpecifier(pxr::SdfSpecifierClass);
			SetAttr<pxr::VtTokenArray>(tubePrototype,
			                           pxr::UsdGeomTokens->xformOpOrder,
			                           {xformOpTranslate});
			SetRelationship(tubePrototype, pxr::UsdShadeTokens->materialBinding,
			                materialPath);
			pxr::VtVec3fArray positions;
			positions.resize((dimensions[0] - 1) * (dimensions[1] - 1));
			for (int i = 0; i < dimensions[0] - 1; i++) {
				for (int j = 0; j < dimensions[1] - 1; j++) {
					double x_offset =
					    (i + 1 - dimensions[0] / 2.0) * BrickUnitLength;
					double y_offset =
					    (j + 1 - dimensions[1] / 2.0) * BrickUnitLength;
					double z_offset = tube_height / 2.0 + TubeHeightTolerance;
					positions[i * (dimensions[1] - 1) + j] = {
					    static_cast<float>(metrics.from_m(x_offset)),
					    static_cast<float>(metrics.from_m(y_offset)),
					    static_cast<float>(metrics.from_m(z_offset)),
					};
				}
			}
			auto tubes = pxr::SdfCreatePrimInLayer(
			    layer, root_path.AppendChild(LegoTokens->Tubes));
			tubes->SetSpecifier(pxr::SdfSpecifierDef);
			tubes->SetTypeName(pxr::UsdGeomTokens->PointInstancer);
			pxr::SdfRelationshipSpec::New(tubes, pxr::UsdGeomTokens->prototypes)
			    ->GetTargetPathList()
			    .Add(tubePrototypePath);
			SetAttr<pxr::VtVec3fArray>(tubes, pxr::UsdGeomTokens->positions,
			                           positions,
			                           pxr::SdfValueRoleNames->Point);
			SetAttr<pxr::VtIntArray>(tubes, pxr::UsdGeomTokens->protoIndices,
			                         pxr::VtIntArray(positions.size(), 0));
		}

		constexpr double StudVisualHeight = StudHeight + ToleranceUpFace;
		auto studPrototypePath =
		    root_path.AppendChild(LegoTokens->StudPrototype);
		auto studPrototype =
		    pxr::SdfCreatePrimInLayer(layer, studPrototypePath);
		studPrototype->SetSpecifier(pxr::SdfSpecifierClass);
		studPrototype->SetTypeName(pxr::UsdGeomTokens->Cylinder);
		SetAttr<double>(studPrototype, pxr::UsdGeomTokens->height, 1.0);
		SetRelationship(studPrototype, pxr::UsdShadeTokens->materialBinding,
		                materialPath);
		SetAttr<pxr::GfVec3f>(studPrototype, xformOpScale,
		                      {
		                          metrics.from_m(StudDiameter / 2.0),
		                          metrics.from_m(StudDiameter / 2.0),
		                          metrics.from_m(StudVisualHeight),
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
				double z_offset = body_visual_height + StudVisualHeight / 2.0;
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
	std::optional<BrickPart> parse_name(const std::string &name) {
		std::string_view sv{name};
		if (!sv.starts_with("Brick_"))
			return std::nullopt;
		sv.remove_prefix(6);
		auto x1 = sv.find('x');
		if (x1 == std::string_view::npos)
			return std::nullopt;
		auto L = parse_int<BrickUnit>(sv.substr(0, x1));
		if (!L)
			return std::nullopt;
		sv.remove_prefix(x1 + 1);
		auto x2 = sv.find('x');
		if (x2 == std::string_view::npos)
			return std::nullopt;
		auto W = parse_int<BrickUnit>(sv.substr(0, x2));
		if (!W)
			return std::nullopt;
		sv.remove_prefix(x2 + 1);
		auto x3 = sv.find('_');
		if (x3 == std::string_view::npos)
			return std::nullopt;
		auto H = parse_int<BrickUnit>(sv.substr(0, x3));
		if (!H)
			return std::nullopt;
		sv.remove_prefix(x3 + 1);
		if (sv.size() != 6)
			return std::nullopt;
		auto r = parse_int<std::uint8_t>(sv.substr(0, 2), 16);
		auto g = parse_int<std::uint8_t>(sv.substr(2, 2), 16);
		auto b = parse_int<std::uint8_t>(sv.substr(4, 2), 16);
		if (!r || !g || !b)
			return std::nullopt;

		return BrickPart(*L, *W, *H, {*r, *g, *b});
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

} // namespace bricksim
