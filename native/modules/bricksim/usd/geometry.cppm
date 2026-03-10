export module bricksim.usd.geometry;

import std;
import bricksim.utils.sdf;
import bricksim.vendor;

namespace bricksim {

export void make_cylinder(const pxr::SdfLayerHandle &layer,
                          const pxr::SdfPath &path, double radius,
                          double height, int segments) {
	pxr::SdfChangeBlock _changes;
	auto prim = pxr::SdfCreatePrimInLayer(layer, path);
	prim->SetSpecifier(pxr::SdfSpecifierDef);
	prim->SetTypeName(pxr::UsdGeomTokens->Mesh);

	double z0 = -height / 2.0;
	double z1 = +height / 2.0;

	pxr::VtVec3fArray points;
	points.resize(4 * segments + 2);
	float f_radius = static_cast<float>(radius);
	float fz0 = static_cast<float>(z0);
	float fz1 = static_cast<float>(z1);

	// [0..N-1] bottom cap ring, [N..2N-1] side bottom ring,
	// [2N..3N-1] side top ring, [3N..4N-1] top cap ring,
	// [4N] bottom center, [4N+1] top center
	for (int i = 0; i < segments; ++i) {
		double a = 2.0 * std::numbers::pi * static_cast<double>(i) /
		           static_cast<double>(segments);
		float c = static_cast<float>(std::cos(a));
		float s = static_cast<float>(std::sin(a));
		float x = f_radius * c;
		float y = f_radius * s;

		points[i] = pxr::GfVec3f(x, y, fz0);
		points[i + segments] = pxr::GfVec3f(x, y, fz0);
		points[i + 2 * segments] = pxr::GfVec3f(x, y, fz1);
		points[i + 3 * segments] = pxr::GfVec3f(x, y, fz1);
	}
	points[4 * segments] = pxr::GfVec3f(0.0f, 0.0f, fz0);
	points[4 * segments + 1] = pxr::GfVec3f(0.0f, 0.0f, fz1);

	pxr::VtIntArray face_counts;
	pxr::VtIntArray face_indices;
	face_counts.reserve(3 * segments);
	face_indices.reserve(10 * segments);

	auto add_quad = [&](int a, int b, int c, int d) {
		face_counts.push_back(4);
		face_indices.push_back(a);
		face_indices.push_back(b);
		face_indices.push_back(c);
		face_indices.push_back(d);
	};
	auto add_tri = [&](int a, int b, int c) {
		face_counts.push_back(3);
		face_indices.push_back(a);
		face_indices.push_back(b);
		face_indices.push_back(c);
	};

	int bottom_center = 4 * segments;
	int top_center = bottom_center + 1;
	for (int i = 0; i < segments; ++i) {
		int j = (i + 1) % segments;

		int cb_i = i;
		int cb_j = j;
		int sb_i = i + segments;
		int sb_j = j + segments;
		int st_i = i + 2 * segments;
		int st_j = j + 2 * segments;
		int ct_i = i + 3 * segments;
		int ct_j = j + 3 * segments;

		add_quad(sb_i, sb_j, st_j, st_i);
		add_tri(bottom_center, cb_j, cb_i);
		add_tri(top_center, ct_i, ct_j);
	}

	SetAttr<pxr::VtVec3fArray>(prim, pxr::UsdGeomTokens->points, points,
	                           pxr::SdfValueRoleNames->Point);
	SetAttr<pxr::VtIntArray>(prim, pxr::UsdGeomTokens->faceVertexCounts,
	                         face_counts);
	SetAttr<pxr::VtIntArray>(prim, pxr::UsdGeomTokens->faceVertexIndices,
	                         face_indices);
	SetAttr<pxr::TfToken>(prim, pxr::UsdGeomTokens->subdivisionScheme,
	                      pxr::UsdGeomTokens->none);

	SetAttr<pxr::VtVec3fArray>(
	    prim, pxr::UsdGeomTokens->extent,
	    {
	        pxr::GfVec3f(static_cast<float>(-radius),
	                     static_cast<float>(-radius), static_cast<float>(z0)),
	        pxr::GfVec3f(static_cast<float>(radius), static_cast<float>(radius),
	                     static_cast<float>(z1)),
	    },
	    pxr::SdfValueRoleNames->Point);
}

export void make_hollow_cylinder(const pxr::SdfLayerHandle &layer,
                                 const pxr::SdfPath &path, double outer_radius,
                                 double thickness, double height, int segments,
                                 bool top_cap, bool bottom_cap) {
	pxr::SdfChangeBlock _changes;
	auto prim = pxr::SdfCreatePrimInLayer(layer, path);
	prim->SetSpecifier(pxr::SdfSpecifierDef);
	prim->SetTypeName(pxr::UsdGeomTokens->Mesh);

	double inner_radius = outer_radius - thickness;
	double z0 = -height / 2.0;
	double z1 = +height / 2.0;

	pxr::VtVec3fArray points;
	points.resize(4 * segments);
	float f_outer_radius = static_cast<float>(outer_radius);
	float f_inner_radius = static_cast<float>(inner_radius);
	float fz0 = static_cast<float>(z0);
	float fz1 = static_cast<float>(z1);

	// [0..N-1] outer bottom, [N..2N-1] outer top,
	// [2N..3N-1] inner bottom, [3N..4N-1] inner top
	for (int i = 0; i < segments; ++i) {
		double a = 2.0 * std::numbers::pi * static_cast<double>(i) /
		           static_cast<double>(segments);
		float c = static_cast<float>(std::cos(a));
		float s = static_cast<float>(std::sin(a));
		float ox = f_outer_radius * c;
		float oy = f_outer_radius * s;
		float ix = f_inner_radius * c;
		float iy = f_inner_radius * s;

		points[i] = pxr::GfVec3f(ox, oy, fz0);
		points[i + segments] = pxr::GfVec3f(ox, oy, fz1);
		points[i + 2 * segments] = pxr::GfVec3f(ix, iy, fz0);
		points[i + 3 * segments] = pxr::GfVec3f(ix, iy, fz1);
	}

	pxr::VtIntArray face_counts;
	pxr::VtIntArray face_indices;
	std::size_t per_seg_faces = 2;
	if (top_cap)
		per_seg_faces += 1;
	if (bottom_cap)
		per_seg_faces += 1;
	face_counts.reserve(per_seg_faces * segments);
	face_indices.reserve(4 * per_seg_faces * segments);

	auto add_quad = [&](int a, int b, int c, int d) {
		face_counts.push_back(4);
		face_indices.push_back(a);
		face_indices.push_back(b);
		face_indices.push_back(c);
		face_indices.push_back(d);
	};

	for (int i = 0; i < segments; ++i) {
		int j = (i + 1) % segments;

		int ob_i = i;
		int ob_j = j;
		int ot_i = i + segments;
		int ot_j = j + segments;
		int ib_i = i + 2 * segments;
		int ib_j = j + 2 * segments;
		int it_i = i + 3 * segments;
		int it_j = j + 3 * segments;

		add_quad(ob_i, ob_j, ot_j, ot_i); // outer wall
		add_quad(it_i, it_j, ib_j, ib_i); // inner wall

		if (top_cap) {
			add_quad(ot_i, ot_j, it_j, it_i);
		}
		if (bottom_cap) {
			add_quad(ib_i, ib_j, ob_j, ob_i);
		}
	}

	SetAttr<pxr::VtVec3fArray>(prim, pxr::UsdGeomTokens->points, points,
	                           pxr::SdfValueRoleNames->Point);
	SetAttr<pxr::VtIntArray>(prim, pxr::UsdGeomTokens->faceVertexCounts,
	                         face_counts);
	SetAttr<pxr::VtIntArray>(prim, pxr::UsdGeomTokens->faceVertexIndices,
	                         face_indices);
	SetAttr<pxr::TfToken>(prim, pxr::UsdGeomTokens->subdivisionScheme,
	                      pxr::UsdGeomTokens->none);

	SetAttr<pxr::VtVec3fArray>(
	    prim, pxr::UsdGeomTokens->extent,
	    {
	        pxr::GfVec3f(static_cast<float>(-outer_radius),
	                     static_cast<float>(-outer_radius),
	                     static_cast<float>(z0)),
	        pxr::GfVec3f(static_cast<float>(outer_radius),
	                     static_cast<float>(outer_radius),
	                     static_cast<float>(z1)),
	    },
	    pxr::SdfValueRoleNames->Point);
}

export enum class OpenFace { PosX, NegX, PosY, NegY, PosZ, NegZ };

export void make_open_box(const pxr::SdfLayerHandle &layer,
                          const pxr::SdfPath &path,
                          std::array<double, 3> outer_size, double thickness,
                          OpenFace open_face = OpenFace::PosZ) {
	pxr::SdfChangeBlock _changes;
	auto prim = pxr::SdfCreatePrimInLayer(layer, path);
	prim->SetSpecifier(pxr::SdfSpecifierDef);
	prim->SetTypeName(pxr::UsdGeomTokens->Mesh);

	auto [sx, sy, sz] = outer_size;
	switch (open_face) {
	case OpenFace::PosX:
	case OpenFace::NegX:
		std::swap(sx, sz);
		break;
	case OpenFace::PosY:
	case OpenFace::NegY:
		std::swap(sy, sz);
		break;
	default:
		break;
	}

	double x0 = -sx / 2.0;
	double x1 = +sx / 2.0;
	double y0 = -sy / 2.0;
	double y1 = +sy / 2.0;
	double z0 = -sz / 2.0;
	double z1 = +sz / 2.0;

	double xi0 = x0 + thickness;
	double xi1 = x1 - thickness;
	double yi0 = y0 + thickness;
	double yi1 = y1 - thickness;
	double zi0 = z0 + thickness;
	double zi1 = z1;

	std::array<double, 4> xs{x0, xi0, xi1, x1};
	std::array<double, 4> ys{y0, yi0, yi1, y1};
	std::array<double, 3> zs{z0, zi0, zi1};

	auto vid = [](int ix, int iy, int iz) { return ix + 4 * iy + 16 * iz; };

	pxr::VtVec3fArray points;
	points.reserve(4 * 4 * 3);
	for (double z : zs) {
		for (double y : ys) {
			for (double x : xs) {
				points.emplace_back(static_cast<float>(x),
				                    static_cast<float>(y),
				                    static_cast<float>(z));
			}
		}
	}

	pxr::VtIntArray face_counts;
	pxr::VtIntArray face_indices;
	face_counts.reserve(46);
	face_indices.reserve(46 * 4);
	auto add_quad = [&](int a, int b, int c, int d) {
		face_counts.push_back(4);
		face_indices.push_back(a);
		face_indices.push_back(b);
		face_indices.push_back(c);
		face_indices.push_back(d);
	};

	for (int iy = 0; iy < 3; ++iy) {
		for (int ix = 0; ix < 3; ++ix) {
			add_quad(vid(ix, iy, 0), vid(ix, iy + 1, 0), vid(ix + 1, iy + 1, 0),
			         vid(ix + 1, iy, 0));
		}
	}

	for (int iy = 0; iy < 3; ++iy) {
		for (int ix = 0; ix < 3; ++ix) {
			if (ix == 1 && iy == 1) {
				continue;
			}
			add_quad(vid(ix, iy, 2), vid(ix + 1, iy, 2), vid(ix + 1, iy + 1, 2),
			         vid(ix, iy + 1, 2));
		}
	}

	add_quad(vid(1, 1, 1), vid(2, 1, 1), vid(2, 2, 1), vid(1, 2, 1));

	std::array<std::pair<int, int>, 2> z_segs{{{0, 1}, {1, 2}}};

	for (int iy = 0; iy < 3; ++iy) {
		for (auto [zlo, zhi] : z_segs) {
			add_quad(vid(3, iy, zlo), vid(3, iy + 1, zlo), vid(3, iy + 1, zhi),
			         vid(3, iy, zhi));
		}
	}

	for (int iy = 0; iy < 3; ++iy) {
		for (auto [zlo, zhi] : z_segs) {
			add_quad(vid(0, iy, zlo), vid(0, iy, zhi), vid(0, iy + 1, zhi),
			         vid(0, iy + 1, zlo));
		}
	}

	for (int ix = 0; ix < 3; ++ix) {
		for (auto [zlo, zhi] : z_segs) {
			add_quad(vid(ix, 3, zlo), vid(ix, 3, zhi), vid(ix + 1, 3, zhi),
			         vid(ix + 1, 3, zlo));
		}
	}

	for (int ix = 0; ix < 3; ++ix) {
		for (auto [zlo, zhi] : z_segs) {
			add_quad(vid(ix, 0, zlo), vid(ix + 1, 0, zlo), vid(ix + 1, 0, zhi),
			         vid(ix, 0, zhi));
		}
	}

	add_quad(vid(2, 1, 1), vid(2, 1, 2), vid(2, 2, 2), vid(2, 2, 1));
	add_quad(vid(1, 1, 1), vid(1, 2, 1), vid(1, 2, 2), vid(1, 1, 2));
	add_quad(vid(1, 2, 1), vid(2, 2, 1), vid(2, 2, 2), vid(1, 2, 2));
	add_quad(vid(1, 1, 1), vid(1, 1, 2), vid(2, 1, 2), vid(2, 1, 1));

	SetAttr<pxr::VtVec3fArray>(prim, pxr::UsdGeomTokens->points, points,
	                           pxr::SdfValueRoleNames->Point);
	SetAttr<pxr::VtIntArray>(prim, pxr::UsdGeomTokens->faceVertexCounts,
	                         face_counts);
	SetAttr<pxr::VtIntArray>(prim, pxr::UsdGeomTokens->faceVertexIndices,
	                         face_indices);
	SetAttr<pxr::TfToken>(prim, pxr::UsdGeomTokens->subdivisionScheme,
	                      pxr::UsdGeomTokens->none);
	SetAttr<pxr::VtVec3fArray>(
	    prim, pxr::UsdGeomTokens->extent,
	    {
	        pxr::GfVec3f(static_cast<float>(x0), static_cast<float>(y0),
	                     static_cast<float>(z0)),
	        pxr::GfVec3f(static_cast<float>(x1), static_cast<float>(y1),
	                     static_cast<float>(z1)),
	    },
	    pxr::SdfValueRoleNames->Point);

	float s = std::numbers::sqrt2_v<float> / 2.0f;
	pxr::GfQuatf orient{1.0f, pxr::GfVec3f{0.0f, 0.0f, 0.0f}};
	switch (open_face) {
	case OpenFace::PosZ:
		break;
	case OpenFace::NegZ:
		orient = pxr::GfQuatf{0.0f, pxr::GfVec3f{1.0f, 0.0f, 0.0f}};
		break;
	case OpenFace::PosX:
		orient = pxr::GfQuatf{s, pxr::GfVec3f{0.0f, s, 0.0f}};
		break;
	case OpenFace::NegX:
		orient = pxr::GfQuatf{s, pxr::GfVec3f{0.0f, -s, 0.0f}};
		break;
	case OpenFace::PosY:
		orient = pxr::GfQuatf{s, pxr::GfVec3f{-s, 0.0f, 0.0f}};
		break;
	case OpenFace::NegY:
		orient = pxr::GfQuatf{s, pxr::GfVec3f{s, 0.0f, 0.0f}};
		break;
	}

	if (open_face != OpenFace::PosZ) {
		SetAttr<pxr::GfQuatf>(prim, xformOpOrient, orient);
		SetAttr<pxr::VtTokenArray>(prim, pxr::UsdGeomTokens->xformOpOrder,
		                           {xformOpOrient});
	}
}

} // namespace bricksim
