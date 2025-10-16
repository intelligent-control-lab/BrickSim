#include "LegoBricks.h"
#include "LegoTokens.h"

#include <ranges>

#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/xformable.h>

namespace lego_assemble {

static bool parseBrick(const pxr::UsdPrim &prim, LegoTopology::Brick &out) {
	pxr::GfVec3i dimensions;
	if (!prim.GetAttribute(LegoTokens->brick_dimensions).Get(&dimensions)) {
		return false;
	}
	pxr::GfVec3i color;
	if (!prim.GetAttribute(LegoTokens->brick_color).Get(&color)) {
		return false;
	}
	out.dimensions[0] = dimensions[0];
	out.dimensions[1] = dimensions[1];
	out.dimensions[2] = dimensions[2];
	out.color[0] = color[0];
	out.color[1] = color[1];
	out.color[2] = color[2];
	return true;
}

static bool
parseConnection(const pxr::UsdPrim &prim,
                const pxr::SdfPathTable<LegoTopology::BrickID> &path2id,
                LegoTopology::Connection &out) {
	bool enabled;
	if (!prim.GetAttribute(LegoTokens->conn_enabled).Get(&enabled) ||
	    !enabled) {
		return false;
	}
	pxr::SdfPathVector body0;
	if (!prim.GetRelationship(LegoTokens->conn_body0).GetTargets(&body0) ||
	    body0.size() != 1) {
		return false;
	}
	auto id0_it = path2id.find(body0.front());
	if (id0_it == path2id.end()) {
		return false;
	}
	pxr::SdfPathVector body1;
	if (!prim.GetRelationship(LegoTokens->conn_body1).GetTargets(&body1) ||
	    body1.size() != 1) {
		return false;
	}
	auto id1_it = path2id.find(body1.front());
	if (id1_it == path2id.end()) {
		return false;
	}
	pxr::GfVec2i offset_studs;
	if (!prim.GetAttribute(LegoTokens->conn_offset_studs).Get(&offset_studs)) {
		return false;
	}
	int yaw_index;
	if (!prim.GetAttribute(LegoTokens->conn_yaw_index).Get(&yaw_index)) {
		return false;
	}
	out.parent = id0_it->second;
	out.child = id1_it->second;
	out.offset[0] = offset_studs[0];
	out.offset[1] = offset_studs[1];
	out.orientation = static_cast<BrickOrientation>(yaw_index);
	return true;
}

static bool getPose(const pxr::UsdPrim &prim, pxr::GfVec3d &outPos,
                    pxr::GfQuatd &outRot) {
	pxr::UsdGeomXformable xform(prim);
	if (!xform) {
		return false;
	}
	pxr::GfMatrix4d mat;
	bool resetsXformStack;
	if (!xform.GetLocalTransformation(&mat, &resetsXformStack)) {
		return false;
	}
	outPos = mat.ExtractTranslation();
	outRot = mat.ExtractRotationQuat();
	return true;
}

template <std::unsigned_integral T = size_t> struct DisjointSet {
	std::vector<T> parent;

	DisjointSet(T n) : parent(n) {
		for (auto i = T(0); i < n; i++) {
			parent[i] = i;
		}
	}

	T find(T x) {
		auto root = x;
		while (parent[root] != root) {
			root = parent[root];
		}
		while (parent[x] != x) {
			auto next = parent[x];
			parent[x] = root;
			x = next;
		}
		return root;
	}

	void unite(T x, T y) {
		auto rx = find(x);
		auto ry = find(y);
		if (rx != ry) {
			parent[ry] = rx;
		}
	}

	auto roots() const {
		return std::views::iota(T(0), static_cast<T>(parent.size())) |
		       std::views::filter([this](T x) { return parent[x] == x; });
	}
};

LegoTopology exportLegoTopology(const pxr::UsdStageRefPtr &stage,
                                const pxr::SdfPath &rootPath) {
	LegoTopology result;
	LegoTopology::BrickID bricksCnt = 0;
	pxr::SdfPathTable<LegoTopology::BrickID> path2id;
	std::vector<pxr::SdfPath> id2path;
	auto rootPrim = stage->GetPrimAtPath(rootPath);
	if (!rootPrim) {
		return result;
	}
	for (const auto &child : rootPrim.GetChildren()) {
		LegoTopology::Brick brick;
		if (parseBrick(child, brick)) {
			brick.id = bricksCnt++;
			result.bricks.push_back(std::move(brick));
			path2id[child.GetPath()] = brick.id;
			id2path.push_back(child.GetPath());
		}
	}
	DisjointSet<LegoTopology::BrickID> ds(bricksCnt);
	for (const auto &child : rootPrim.GetChildren()) {
		LegoTopology::Connection conn;
		if (parseConnection(child, path2id, conn)) {
			ds.unite(conn.parent, conn.child);
			result.connections.push_back(std::move(conn));
		}
	}
	auto mpu = pxr::UsdGeomGetStageMetersPerUnit(stage);
	bool first = true;
	pxr::GfVec3d pos0;
	pxr::GfQuatd rot0inv;
	for (auto brickId : ds.roots()) {
		pxr::GfVec3d pos;
		pxr::GfQuatd rot;
		if (!getPose(stage->GetPrimAtPath(id2path[brickId]), pos, rot)) {
			continue;
		}
		LegoTopology::PoseHint hint;
		hint.brick = brickId;
		if (first) {
			first = false;
			pos0 = pos;
			rot0inv = rot.GetInverse();
			hint.pos = {0.0, 0.0, 0.0};
			hint.rot = {1.0, 0.0, 0.0, 0.0};
		} else {
			auto relPos = rot0inv.Transform(pos - pos0);
			auto relPosMeters = relPos * mpu;
			auto relRot = rot0inv * rot;
			hint.pos = {relPosMeters[0], relPosMeters[1], relPosMeters[2]};
			hint.rot = {relRot.GetReal(), relRot.GetImaginary()[0],
			            relRot.GetImaginary()[1], relRot.GetImaginary()[2]};
		}
		result.pose_hints.push_back(std::move(hint));
	}
	return result;
}
} // namespace lego_assemble
