export module lego_assemble.usd.arrange;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.usd.tokens;
import lego_assemble.utils.transforms;
import lego_assemble.utils.pack2d_maxrect;
import lego_assemble.utils.metric_system;
import lego_assemble.utils.usd_envs;
import lego_assemble.utils.conversions;
import lego_assemble.vendor;

namespace lego_assemble {

export struct TableRect {
	double x_min{};
	double y_min{};
	double x_max{};
	double y_max{};
	double z{};
};

export struct ArrangeConfig {
	// Target region on the table (env frame, meters).
	TableRect region{};

	// Extra clearance around each brick's 2D footprint (meters).
	// This is added on all sides before packing, so actual spacing
	// between bricks is >= 2 * clearance_xy in both axes.
	double clearance_xy{0.008};

	// Discretization step for the 2D packer grid (meters per cell).
	double grid_resolution{0.008};

	// Whether the 2D packer is allowed to rotate rectangles
	// (swap width/height).
	bool allow_rotation{true};

	// Whether to avoid all other parts in the environment,
	// or only those in parts_to_avoid. If true, parts_to_avoid is ignored.
	bool avoid_all_other_parts{true};
};

export struct ArrangeResult {
	std::vector<PartId> placed;
	std::vector<PartId> not_placed;

	[[nodiscard]] bool all_placed() const noexcept {
		return not_placed.empty();
	}
};

BBox3d transform_bbox_3d(const BBox3d &box, const Transformd &T) {
	const auto &[q, t] = T;
	constexpr double inf = std::numeric_limits<double>::infinity();
	Eigen::Vector3d min{inf, inf, inf};
	Eigen::Vector3d max{-inf, -inf, -inf};
	for (int ix = 0; ix < 2; ++ix) {
		double cx = (ix == 0) ? box.min.x() : box.max.x();
		for (int iy = 0; iy < 2; ++iy) {
			double cy = (iy == 0) ? box.min.y() : box.max.y();
			for (int iz = 0; iz < 2; ++iz) {
				double cz = (iz == 0) ? box.min.z() : box.max.z();
				Eigen::Vector3d p_local{cx, cy, cz};
				Eigen::Vector3d p = q * p_local + t;
				min = min.cwiseMin(p);
				max = max.cwiseMax(p);
			}
		}
	}
	return {.min = min, .max = max};
}

void expand_to_include(BBox3d &box, const BBox3d &other) {
	box.min = box.min.cwiseMin(other.min);
	box.max = box.max.cwiseMax(other.max);
}

BBox2d to_bbox_2d(const BBox3d &box) {
	return {
	    .min = box.min.head<2>(),
	    .max = box.max.head<2>(),
	};
}

pack2d::Rect discretize_bbox_2d(const BBox2d &box, const TableRect &region,
                                double grid) {
	auto discretize =
	    [&](double box_min, double box_max, double region_min,
	        double region_max) -> std::tuple<std::int32_t, std::int32_t> {
		double x0 = std::max(box_min, region_min);
		double x1 = std::min(box_max, region_max);
		if (x1 <= x0) {
			return {0, 0};
		}
		std::int32_t cell_min =
		    static_cast<std::int32_t>(std::floor((x0 - region_min) / grid));
		std::int32_t cell_max =
		    static_cast<std::int32_t>(std::ceil((x1 - region_min) / grid));
		return {cell_min, cell_max - cell_min};
	};
	auto [cell_x, cell_w] =
	    discretize(box.min.x(), box.max.x(), region.x_min, region.x_max);
	auto [cell_y, cell_h] =
	    discretize(box.min.y(), box.max.y(), region.y_min, region.y_max);
	return {.x = cell_x, .y = cell_y, .w = cell_w, .h = cell_h};
}

BBox3d get_part_bbox(auto &g, PartId pid) {
	return g.topology().parts().visit(
	    pid, [&](const auto &pw) { return pw.wrapped().bbox(); });
}

export template <class UsdGraph>
ArrangeResult
arrange_parts_on_table(UsdGraph &g, const ArrangeConfig &config,
                       std::span<const PartId> parts_to_arrange,
                       std::span<const PartId> parts_to_avoid = {},
                       std::span<const BBox2d> regions_to_avoid = {},
                       std::span<const std::int32_t> structure_ids = {}) {
	if (!structure_ids.empty() &&
	    structure_ids.size() != parts_to_arrange.size()) {
		throw std::invalid_argument(
		    "arrange_parts_on_table: structure_ids size must match "
		    "parts_to_arrange size.");
	}

	// Compute bin size in cells.
	const auto &region = config.region;
	double grid = config.grid_resolution;
	double region_w = region.x_max - region.x_min;
	double region_h = region.y_max - region.y_min;
	std::int32_t Nx =
	    std::max(1, static_cast<std::int32_t>(std::ceil(region_w / grid)));
	std::int32_t Ny =
	    std::max(1, static_cast<std::int32_t>(std::ceil(region_h / grid)));
	pack2d::Bin bin{Nx, Ny};

	// Determine environment id, ensure all parts are in the same env
	std::optional<std::int64_t> env_id;
	auto ensure_env = [&](PartId pid) {
		auto e = g.part_env_id(pid);
		if (!e) {
			throw std::runtime_error(std::format(
			    "arrange_parts_on_table: part id {} does not exist in graph",
			    pid));
		}
		if (env_id) {
			if (*e != *env_id) {
				throw std::runtime_error(std::format(
				    "arrange_parts_on_table: part id {} is in env {}, "
				    "different from other parts in env {}",
				    pid, *e, *env_id));
			}
		} else {
			env_id = *e;
		}
	};

	// Collect parts to arrange
	struct ComponentInfo {
		PartId root_pid;
		std::int32_t structure_id;
		Transformd T_s_root;
		std::vector<PartId> arranged_part_ids;
	};
	struct StructureInfo {
		std::size_t root_cc_idx;

		// BBox in root CC's root part frame
		BBox3d bbox;

		// Transform from env to structure root CC root part frame
		Transformd T_env_s;

		// Whether the structure is valid for packing
		bool valid;

		std::optional<Transformd> arranged_T_env_s;
	};
	std::vector<ComponentInfo> components;
	std::unordered_map<std::int32_t, StructureInfo> structures;

	// Map from part id to index in components
	std::unordered_map<PartId, std::size_t> arranged_parts;

	// Collect connected components
	for (std::size_t idx_u = 0; idx_u < parts_to_arrange.size(); idx_u++) {
		PartId u = parts_to_arrange[idx_u];
		auto arranged_parts_it = arranged_parts.find(u);
		if (arranged_parts_it != arranged_parts.end()) {
			// Already arranged as part of another component
			ComponentInfo &cc_info = components[arranged_parts_it->second];
			if (!structure_ids.empty() &&
			    cc_info.structure_id != structure_ids[idx_u]) {
				throw std::runtime_error(std::format(
				    "arrange_parts_on_table: part id {} is specified to be in "
				    "{}, but it's already in structure {}",
				    u, structure_ids[idx_u], cc_info.structure_id));
			}
			cc_info.arranged_part_ids.emplace_back(u);
			continue;
		}

		// New component
		ensure_env(u);
		std::size_t cc_idx = components.size();
		ComponentInfo &cc = components.emplace_back();
		cc.root_pid = u;
		cc.arranged_part_ids.emplace_back(u);

		// Lookup root part pose
		auto T_env_u = g.part_pose_relative_to_env(u);
		if (!T_env_u) {
			throw std::runtime_error(std::format(
			    "arrange_parts_on_table: part id {} has no pose", u));
		}

		cc.structure_id = structure_ids.empty() ? idx_u : structure_ids[idx_u];
		StructureInfo *structure;
		auto structures_it = structures.find(cc.structure_id);
		if (structures_it == structures.end()) {
			// First time seeing this structure id
			// Make current CC its root
			constexpr double inf = std::numeric_limits<double>::infinity();
			auto [new_it, inserted] = structures.emplace(
			    cc.structure_id,
			    StructureInfo{
			        .root_cc_idx = cc_idx,
			        .bbox = {.min = {inf, inf, inf}, .max = {-inf, -inf, -inf}},
			        .T_env_s = *T_env_u,
			        .valid = false,
			        .arranged_T_env_s = std::nullopt,
			    });
			if (!inserted) {
				throw std::runtime_error(
				    "Internal error: structure insertion failed");
			}
			structure = &new_it->second;
			cc.T_s_root = SE3d{}.identity();
		} else {
			// Compute relative transform to existing structure root
			structure = &structures_it->second;
			cc.T_s_root = inverse(structure->T_env_s) * (*T_env_u);
		}

		// Expand the structure's bbox
		const auto &T_s_u = cc.T_s_root;
		for (auto [v, T_u_v] : g.topology().component_view(u).transforms()) {
			arranged_parts.emplace(v, cc_idx);
			Transformd T_s_v = T_s_u * T_u_v;
			BBox3d bbox_local = get_part_bbox(g, v);
			BBox3d bbox_in_s = transform_bbox_3d(bbox_local, T_s_v);
			expand_to_include(structure->bbox, bbox_in_s);
		}
	}

	// Prepare structure rectangles for packing
	std::vector<pack2d::RectInput> structure_rects;
	for (auto &[structure_id, structure] : structures) {
		BBox2d bbox_2d = to_bbox_2d(structure.bbox);
		double w = bbox_2d.max.x() - bbox_2d.min.x();
		double h = bbox_2d.max.y() - bbox_2d.min.y();
		structure.valid = w > 0.0 && h > 0.0;
		if (!structure.valid) {
			continue;
		}
		w += 2.0 * config.clearance_xy;
		h += 2.0 * config.clearance_xy;
		std::int32_t w_cells = static_cast<std::int32_t>(std::ceil(w / grid));
		std::int32_t h_cells = static_cast<std::int32_t>(std::ceil(h / grid));
		structure_rects.push_back({
		    .id = structure_id,
		    .width = w_cells,
		    .height = h_cells,
		});
	}

	if (!env_id) {
		// No parts to arrange
		return {};
	}

	// Build obstacle list
	std::vector<pack2d::Rect> obstacles;
	auto add_obstacle_bbox = [&](const BBox2d &bbox) {
		pack2d::Rect rect =
		    discretize_bbox_2d(bbox, config.region, config.grid_resolution);
		if (rect.w > 0 && rect.h > 0) {
			obstacles.emplace_back(rect);
		}
	};
	auto add_obstacle_part = [&](PartId pid) {
		BBox3d bbox_local = get_part_bbox(g, pid);
		auto T_env_part = g.part_pose_relative_to_env(pid);
		if (!T_env_part) {
			throw std::runtime_error(std::format(
			    "arrange_parts_on_table: part id {} has no pose", pid));
		}
		BBox3d bbox_env = transform_bbox_3d(bbox_local, *T_env_part);
		BBox2d bbox_env_2d = to_bbox_2d(bbox_env);
		add_obstacle_bbox(bbox_env_2d);
	};
	for (const auto &zone : regions_to_avoid) {
		add_obstacle_bbox(zone);
	}
	if (config.avoid_all_other_parts) {
		for (auto [pid, path] : g.parts_in_env(*env_id)) {
			if (!arranged_parts.contains(pid)) {
				add_obstacle_part(pid);
			}
		}
	} else {
		for (PartId pid : parts_to_avoid) {
			ensure_env(pid);
			if (!arranged_parts.contains(pid)) {
				add_obstacle_part(pid);
			}
		}
	}

	// Solve packing
	pack2d::PackResult pack_result = pack2d::pack_all(
	    bin, structure_rects, obstacles, pack2d::Heuristic::BestShortSideFit,
	    config.allow_rotation);

	// Compute arranged transforms
	for (auto &packed_rect : pack_result.packed) {
		StructureInfo &structure = structures.at(packed_rect.id);

		// Determine rotation
		Eigen::Quaterniond q;
		if (packed_rect.rotated) {
			// 90 deg around +z
			q = Eigen::Quaterniond(Eigen::AngleAxisd(std::numbers::pi / 2.0,
			                                         Eigen::Vector3d::UnitZ()));
		} else {
			q = Eigen::Quaterniond::Identity();
		}

		// Calculate the target center in the env frame.
		// The packer returns integer coordinates for the bottom-left corner.
		Eigen::Vector2d world_xy_min{region.x_min + (packed_rect.x * grid),
		                             region.y_min + (packed_rect.y * grid)};

		// The allocated size in meters
		Eigen::Vector2d alloc_size_xy{packed_rect.width * grid,
		                              packed_rect.height * grid};

		// The geometric center of the allocated space on the table
		Eigen::Vector2d target_center_xy = world_xy_min + 0.5 * alloc_size_xy;

		// Calculate geometric center
		Eigen::Vector3d local_center =
		    0.5 * (structure.bbox.min + structure.bbox.max);

		// Calculate translation
		Eigen::Vector3d t;
		t.head<2>() = target_center_xy - (q * local_center).head<2>();
		t.z() = region.z - structure.bbox.min.z();

		structure.arranged_T_env_s = {q, t};
	}

	// Apply arranged transforms to parts
	ArrangeResult result;
	for (ComponentInfo &cc : components) {
		StructureInfo &structure = structures.at(cc.structure_id);
		if (!structure.arranged_T_env_s) {
			// Structure was not packed
			result.not_placed.insert(result.not_placed.end(),
			                         cc.arranged_part_ids.begin(),
			                         cc.arranged_part_ids.end());
			continue;
		}
		Transformd T_env_s = *structure.arranged_T_env_s;
		Transformd T_env_root = T_env_s * cc.T_s_root;
		g.set_component_transform(cc.root_pid, T_env_root);
		result.placed.insert(result.placed.end(), cc.arranged_part_ids.begin(),
		                     cc.arranged_part_ids.end());
	}
	return result;
}

export std::vector<BBox2d>
compute_regions_to_avoid(const pxr::UsdStageRefPtr &stage,
                         const TableRect &table_region, double clearance_height,
                         std::span<const pxr::SdfPath> obstacle_paths) {
	MetricSystem metrics{stage};

	double z_band_min = table_region.z;
	double z_band_max = table_region.z + clearance_height;
	if (z_band_min > z_band_max) {
		// Negative clearance
		std::swap(z_band_min, z_band_max);
	}

	// Configure a bbox cache for the env, using common purposes.
	pxr::TfTokenVector purposes;
	purposes.push_back(pxr::UsdGeomTokens->default_);
	purposes.push_back(pxr::UsdGeomTokens->render);
	purposes.push_back(pxr::UsdGeomTokens->proxy);
	purposes.push_back(pxr::UsdGeomTokens->guide);
	pxr::UsdGeomBBoxCache bbox_cache(pxr::UsdTimeCode::Default(), purposes,
	                                 /*useExtentsHint=*/true);

	std::vector<BBox2d> zones;
	zones.reserve(obstacle_paths.size());

	std::optional<std::int64_t> env_id;
	pxr::UsdPrim env_prim;

	for (const pxr::SdfPath &path : obstacle_paths) {
		pxr::UsdPrim prim = stage->GetPrimAtPath(path);
		if (!prim.IsValid()) {
			continue;
		}

		// Determine env prim
		auto e = env_id_from_path(path);
		if (!e) {
			continue;
		}
		if (env_id) {
			if (*e != *env_id) {
				continue;
			}
		} else {
			env_id = *e;
			pxr::SdfPath env_path = path_for_env(*env_id);
			env_prim = stage->GetPrimAtPath(env_path);
			if (!env_prim.IsValid()) {
				throw std::runtime_error(
				    std::format("compute_avoid_zones: env prim {} not found",
				                env_path.GetText()));
			}
		}

		// Compute the obstacle's bbox in the env frame (STAGE UNITS).
		pxr::GfBBox3d bbox_stage =
		    bbox_cache.ComputeRelativeBound(prim, env_prim);
		pxr::GfRange3d range_stage = bbox_stage.ComputeAlignedRange();
		if (range_stage.IsEmpty()) {
			continue;
		}

		auto bbox_min = metrics.to_m(as<Eigen::Vector3d>(range_stage.GetMin()));
		auto bbox_max = metrics.to_m(as<Eigen::Vector3d>(range_stage.GetMax()));

		double bbox_z_min = bbox_min.z();
		double bbox_z_max = bbox_max.z();
		if (bbox_z_max < z_band_min || bbox_z_min > z_band_max) {
			// No overlap in z
			continue;
		}

		zones.push_back({
		    .min = bbox_min.head<2>(),
		    .max = bbox_max.head<2>(),
		});
	}

	return zones;
}

export struct WorkspaceConfig {
	std::int64_t env_id{kNoEnv};
	Eigen::Vector3d bbox_min{0.0, 0.0, 0.0};
	Eigen::Vector3d bbox_max{0.0, 0.0, 0.0};
	double clearance_xy{0.008};
	double grid_resolution{0.008};
	double allow_rotation{true};
	std::vector<pxr::SdfPath> obstacle_paths;

	TableRect table_rect() const {
		return {
		    .x_min = bbox_min.x(),
		    .y_min = bbox_min.y(),
		    .x_max = bbox_max.x(),
		    .y_max = bbox_max.y(),
		    .z = bbox_min.z(),
		};
	}

	ArrangeConfig arrange_config() const {
		ArrangeConfig config;
		config.region = table_rect();
		config.clearance_xy = clearance_xy;
		config.grid_resolution = grid_resolution;
		config.allow_rotation = static_cast<bool>(allow_rotation);
		return config;
	}

	double clearance_height() const {
		return bbox_max.z() - bbox_min.z();
	}

	static std::optional<WorkspaceConfig> from_prim(const pxr::UsdPrim &prim) {
		if (!prim.IsValid()) {
			return std::nullopt;
		}
		pxr::SdfPath prim_path = prim.GetPath();
		std::optional<std::int64_t> env_id = env_id_from_path(prim_path);
		if (!env_id) {
			return std::nullopt;
		}
		pxr::UsdPrim env_prim =
		    prim.GetStage()->GetPrimAtPath(path_for_env(*env_id));
		if (!env_prim.IsValid()) {
			return std::nullopt;
		}
		MetricSystem metrics{prim.GetStage()};

		pxr::TfTokenVector purposes;
		purposes.push_back(pxr::UsdGeomTokens->default_);
		purposes.push_back(pxr::UsdGeomTokens->render);
		purposes.push_back(pxr::UsdGeomTokens->proxy);
		purposes.push_back(pxr::UsdGeomTokens->guide);
		pxr::UsdGeomBBoxCache bbox_cache(pxr::UsdTimeCode::Default(), purposes,
		                                 /*useExtentsHint=*/true);
		pxr::GfBBox3d bbox_stage =
		    bbox_cache.ComputeRelativeBound(prim, env_prim);
		pxr::GfRange3d range_stage = bbox_stage.ComputeAlignedRange();
		if (range_stage.IsEmpty()) {
			return std::nullopt;
		}

		WorkspaceConfig result;
		result.env_id = *env_id;
		result.bbox_min =
		    metrics.to_m(as<Eigen::Vector3d>(range_stage.GetMin()));
		result.bbox_max =
		    metrics.to_m(as<Eigen::Vector3d>(range_stage.GetMax()));

		prim.GetAttribute(LegoTokens->LegoWorkspaceClearance)
		    .Get(&result.clearance_xy);
		prim.GetAttribute(LegoTokens->LegoWorkspaceGridResolution)
		    .Get(&result.grid_resolution);
		prim.GetAttribute(LegoTokens->LegoWorkspaceAllowRotation)
		    .Get(&result.allow_rotation);

		pxr::SdfPathVector obstacle_paths;
		prim.GetRelationship(LegoTokens->LegoWorkspaceObstacles)
		    .GetTargets(&obstacle_paths);
		result.obstacle_paths.reserve(obstacle_paths.size());
		for (const pxr::SdfPath &path : obstacle_paths) {
			result.obstacle_paths.emplace_back(
			    path.MakeAbsolutePath(prim_path));
		}
		return result;
	}
};

export template <class UsdGraph>
ArrangeResult
arrange_parts_in_workspace(UsdGraph &g, const WorkspaceConfig &workspace,
                           std::span<const PartId> parts_to_arrange,
                           std::span<const std::int32_t> structure_ids = {}) {
	std::vector<BBox2d> regions_to_avoid = compute_regions_to_avoid(
	    g.stage(), workspace.table_rect(), workspace.clearance_height(),
	    workspace.obstacle_paths);
	return arrange_parts_on_table(g, workspace.arrange_config(),
	                              parts_to_arrange, {}, regions_to_avoid,
	                              structure_ids);
}

} // namespace lego_assemble
