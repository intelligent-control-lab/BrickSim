export module lego_assemble.usd.parse;

import std;
import lego_assemble.usd.tokens;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.usd.specs;
import lego_assemble.utils.conversions;
import lego_assemble.utils.c4_rotation;
import lego_assemble.vendor.pxr;
import lego_assemble.vendor.eigen;

namespace lego_assemble {

// ==== Parts ====

export template <PartLike P>
using PartPrimParseResult = std::pair<P, InterfaceCollidersVector>;

export template <class T>
concept PartParser = requires {
	typename T::PartType;
	requires PartLike<typename T::PartType>;
} && requires(T t, const pxr::UsdPrim &prim) {
	{
		t(prim)
	} -> std::convertible_to<
	    std::optional<PartPrimParseResult<typename T::PartType>>>;
};

export class BrickParser {
  public:
	using PartType = BrickPart;

	std::optional<PartPrimParseResult<BrickPart>>
	operator()(const pxr::UsdPrim &prim) {
		pxr::TfToken part_kind;
		if (!prim.GetAttribute(LegoTokens->PartKind).Get(&part_kind) ||
		    part_kind != LegoTokens->PartKindBrick) {
			return std::nullopt;
		}

		pxr::GfVec3i dimensions_gf;
		if (!prim.GetAttribute(LegoTokens->BrickDimensions)
		         .Get(&dimensions_gf)) {
			return std::nullopt;
		}

		pxr::GfVec3i color_gf;
		if (!prim.GetAttribute(LegoTokens->BrickColor).Get(&color_gf)) {
			return std::nullopt;
		}

		auto hole_collider_prim = prim.GetChild(LegoTokens->BodyCollider);
		auto stud_collider_prim = prim.GetChild(LegoTokens->TopCollider);
		if (!hole_collider_prim || !stud_collider_prim) {
			return std::nullopt;
		}

		BrickUnit L = dimensions_gf[0];
		BrickUnit W = dimensions_gf[1];
		PlateUnit H = dimensions_gf[2];
		BrickColor brick_color = as<BrickColor>(color_gf);
		BrickPart part(L, W, H, brick_color);
		InterfaceCollidersVector colliders{
		    {BrickPart::HoleId, hole_collider_prim.GetPath()},
		    {BrickPart::StudId, stud_collider_prim.GetPath()},
		};
		return {{part, colliders}};
	}
};

// ==== Connections ====
export struct ConnectionPrimParseResult {
	pxr::SdfPath stud_path;
	InterfaceId stud_interface;
	pxr::SdfPath hole_path;
	InterfaceId hole_interface;
	ConnectionSegment conn_seg;
};

export std::optional<ConnectionPrimParseResult>
parse_connection_prim(const pxr::UsdPrim &prim) {
	pxr::SdfPathVector stud_v, hole_v;
	prim.GetRelationship(LegoTokens->ConnStud).GetTargets(&stud_v);
	prim.GetRelationship(LegoTokens->ConnHole).GetTargets(&hole_v);
	if (stud_v.empty() || hole_v.empty()) {
		return std::nullopt;
	}
	pxr::SdfPath stud_path = stud_v.front();
	pxr::SdfPath hole_path = hole_v.front();

	int stud_interface;
	if (!prim.GetAttribute(LegoTokens->ConnStudInterface)
	         .Get(&stud_interface)) {
		return std::nullopt;
	}
	int hole_interface;
	if (!prim.GetAttribute(LegoTokens->ConnHoleInterface)
	         .Get(&hole_interface)) {
		return std::nullopt;
	}
	pxr::GfVec2i offset_gf;
	if (!prim.GetAttribute(LegoTokens->ConnOffset).Get(&offset_gf)) {
		return std::nullopt;
	}
	int yaw_index;
	if (!prim.GetAttribute(LegoTokens->ConnYaw).Get(&yaw_index)) {
		return std::nullopt;
	}
	return {{.stud_path = stud_path,
	         .stud_interface = static_cast<InterfaceId>(stud_interface),
	         .hole_path = hole_path,
	         .hole_interface = static_cast<InterfaceId>(hole_interface),
	         .conn_seg = {
	             .offset = as<Eigen::Vector2i>(offset_gf),
	             .yaw = to_c4(yaw_index),
	         }}};
}

} // namespace lego_assemble
