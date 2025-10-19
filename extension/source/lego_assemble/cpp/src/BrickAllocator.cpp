#include "BrickAllocator.h"
#include "BrickSpawner.h"

#include <carb/logging/Log.h>

namespace lego_assemble {

// Collision Groups
static const pxr::SdfPath CollisionRootPath("/World/collisions");
static const pxr::SdfPath GlobalCollisionGroupPath =
    CollisionRootPath.AppendChild(pxr::TfToken("global_group"));
static const pxr::TfToken FilteredGroupsProp("physics:filteredGroups");

static pxr::SdfRelationshipSpecHandle
getCollisionGroupRel(const pxr::SdfLayerHandle &layer, EnvId envId) {
	pxr::SdfPath colGrpPath;
	if (envId == kNoEnv) {
		colGrpPath = GlobalCollisionGroupPath;
	} else {
		auto envName = std::format("group{}", envId);
		colGrpPath = CollisionRootPath.AppendChild(pxr::TfToken(envName));
	}
	return layer->GetRelationshipAtPath(
	    colGrpPath.AppendProperty(FilteredGroupsProp));
}

static const pxr::SdfPath LegoStatePath("/_LegoState");
static const pxr::SdfPath LegoStateNextIdProp =
    LegoStatePath.AppendProperty(pxr::TfToken("next_id"));

std::tuple<BrickId, pxr::SdfPath>
allocateBrick(const pxr::UsdStageRefPtr &stage,
              const std::array<BrickUnit, 3> &dimensions,
              const BrickColor &color, EnvId env_id) {
	auto layer = stage->GetEditTarget().GetLayer();
	pxr::SdfChangeBlock _changes;

	BrickId nextId;
	auto nextIdAttr = layer->GetAttributeAtPath(LegoStateNextIdProp);
	if (nextIdAttr) {
		nextId = nextIdAttr->GetDefaultValue().Get<std::int64_t>();
	} else {
		auto statePrim = layer->GetPrimAtPath(LegoStatePath);
		if (!statePrim) {
			statePrim = pxr::SdfCreatePrimInLayer(layer, LegoStatePath);
			statePrim->SetSpecifier(pxr::SdfSpecifierDef);
		}
		nextIdAttr = pxr::SdfAttributeSpec::New(
		    statePrim, LegoStateNextIdProp.GetNameToken(),
		    pxr::SdfValueTypeNames->Int64, pxr::SdfVariabilityUniform, true);
		nextId = 0;
	}

	pxr::SdfPath path;
	while (layer->GetPrimAtPath(
	    path = pxr::SdfPath(pathForBrick(env_id, nextId)))) {
		nextId++;
	};

	if (!nextIdAttr->SetDefaultValue(pxr::VtValue(nextId + 1))) {
		CARB_LOG_FATAL("allocateBrick: Failed to update next_id.");
	}

	createBrickPrim(stage, path, dimensions, color);

	if (auto colGrpRel = getCollisionGroupRel(layer, env_id)) {
		colGrpRel->GetTargetPathList().Append(path);
		// TODO: do we really need this?
	}

	return {nextId, path};
}

bool deallocateBrick(const pxr::UsdStageRefPtr &stage, EnvId env_id,
                     BrickId brick_id) {
	auto layer = stage->GetEditTarget().GetLayer();
	pxr::SdfChangeBlock _changes;
	auto path = pathForBrick(env_id, brick_id);
	auto brick = layer->GetPrimAtPath(path);
	if (brick) {
		auto parent = brick->GetNameParent();
		if (parent) {
			if (auto colGrpRel = getCollisionGroupRel(layer, env_id)) {
				colGrpRel->GetTargetPathList().Remove(path);
			}
			if (!parent->RemoveNameChild(brick)) {
				CARB_LOG_WARN("deallocateBrick: Failed to remove brick prim %s",
				              path.GetText());
			}
			return true;
		} else {
			CARB_LOG_WARN("deallocateBrick: Brick prim has no parent!");
		}
	}
	return false;
}

static bool isBrickOrConn(const pxr::SdfPrimSpecHandle &prim) {
	auto name = prim->GetName();
	if (name.starts_with("Brick_") || name.starts_with("Conn_")) {
		return true;
	}
	return false;
}

void deallocateAllBricksInEnv(const pxr::UsdStageRefPtr &stage, EnvId env_id) {
	auto layer = stage->GetEditTarget().GetLayer();
	pxr::SdfChangeBlock _changes;
	auto env = layer->GetPrimAtPath(pathForEnv(env_id));
	if (!env) {
		return;
	}

	auto colGrpRel = getCollisionGroupRel(layer, env_id);
	std::vector<pxr::SdfPrimSpecHandle> toKeep;
	for (const auto &child : env->GetNameChildren()) {
		if (isBrickOrConn(child)) {
			if (colGrpRel) {
				colGrpRel->GetTargetPathList().Erase(child->GetPath());
			}
		} else {
			toKeep.push_back(child);
		}
	}
	env->SetNameChildren(toKeep);
}

} // namespace lego_assemble
