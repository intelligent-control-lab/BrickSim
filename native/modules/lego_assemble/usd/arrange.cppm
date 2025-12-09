export module lego_assemble.usd.arrange;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.utils.transforms;
import lego_assemble.utils.pack2d_maxrect;
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
	bool allow_rotation{false};

	// Whether to avoid all other parts in the environment,
	// or only those in parts_to_avoid. If true, parts_to_avoid is ignored.
	bool avoid_all_other_parts{false};
};

export struct ArrangeResult {
	std::vector<PartId> placed;
	std::vector<PartId> not_placed;

	[[nodiscard]] bool all_placed() const noexcept {
		return not_placed.empty();
	}
};

BBox2d project_bbox_xy(const BBox3d &local_box, const Transformd &T_env_part) {
	const auto &[q, t] = T_env_part;

	Eigen::Vector2d min_xy{std::numeric_limits<double>::infinity(),
	                       std::numeric_limits<double>::infinity()};
	Eigen::Vector2d max_xy{-std::numeric_limits<double>::infinity(),
	                       -std::numeric_limits<double>::infinity()};

	// Enumerate 8 corners of the local AABB
	for (int ix = 0; ix < 2; ++ix) {
		double cx = (ix == 0) ? local_box.min.x() : local_box.max.x();
		for (int iy = 0; iy < 2; ++iy) {
			double cy = (iy == 0) ? local_box.min.y() : local_box.max.y();
			for (int iz = 0; iz < 2; ++iz) {
				double cz = (iz == 0) ? local_box.min.z() : local_box.max.z();
				Eigen::Vector3d p_local{cx, cy, cz};

				// env-frame position
				Eigen::Vector3d p_env = q * p_local + t;

				min_xy.x() = std::min(min_xy.x(), p_env.x());
				min_xy.y() = std::min(min_xy.y(), p_env.y());
				max_xy.x() = std::max(max_xy.x(), p_env.x());
				max_xy.y() = std::max(max_xy.y(), p_env.y());
			}
		}
	}

	return {.min = min_xy, .max = max_xy};
}

BBox2d bbox_xy(const BBox3d &box) {
	return {
	    .min = box.min.head<2>(),
	    .max = box.max.head<2>(),
	};
}

struct BBox2i {
	// Half-open integer ranges: [min, max)
	Eigen::Vector2i min;
	Eigen::Vector2i max;

	bool operator==(const BBox2i &other) const = default;

	int width() const {
		return max.x() - min.x();
	}
	int height() const {
		return max.y() - min.y();
	}
	bool empty() const {
		return width() <= 0 || height() <= 0;
	}
	operator lego_assemble::pack2d::Rect() const {
		return {
		    .x = min.x(),
		    .y = min.y(),
		    .w = width(),
		    .h = height(),
		};
	}
};

// Returns the smallest half-open integer AABB of grid cells that covers the
// intersection of `box` with `region`.
//
// - `grid_resolution` is meters per cell.
// - The logical grid origin (cell 0,0) is at (region.x_min, region.y_min).
// - If there is no intersection, returns min==max (empty box).
BBox2i discretize_bbox_xy(const BBox2d &box, const TableRect &region,
                          double grid_resolution) noexcept {
	double gx = region.x_min;
	double gy = region.y_min;

	// Clip to region first
	double x0 = std::max(box.min.x(), region.x_min);
	double x1 = std::min(box.max.x(), region.x_max);
	double y0 = std::max(box.min.y(), region.y_min);
	double y1 = std::min(box.max.y(), region.y_max);

	// No overlap with region -> empty
	if (x1 <= x0 || y1 <= y0) {
		return {.min = {0, 0}, .max = {0, 0}};
	}

	double region_w = region.x_max - region.x_min;
	double region_h = region.y_max - region.y_min;

	// Total grid size; we use ceil so the entire region is covered.
	int Nx =
	    std::max(1, static_cast<int>(std::ceil(region_w / grid_resolution)));
	int Ny =
	    std::max(1, static_cast<int>(std::ceil(region_h / grid_resolution)));

	auto cell_min = [&](double coord, double origin) -> int {
		return static_cast<int>(std::floor((coord - origin) / grid_resolution));
	};
	auto cell_max = [&](double coord, double origin) -> int {
		return static_cast<int>(std::ceil((coord - origin) / grid_resolution));
	};

	int ix0 = cell_min(x0, gx);
	int ix1 = cell_max(x1, gx);
	int iy0 = cell_min(y0, gy);
	int iy1 = cell_max(y1, gy);

	ix0 = std::clamp(ix0, 0, Nx);
	ix1 = std::clamp(ix1, 0, Nx);
	iy0 = std::clamp(iy0, 0, Ny);
	iy1 = std::clamp(iy1, 0, Ny);

	// Ensure half-open ordering
	if (ix1 < ix0)
		std::swap(ix0, ix1);
	if (iy1 < iy0)
		std::swap(iy0, iy1);

	return {.min = {ix0, iy0}, .max = {ix1, iy1}};
}

export template <class UsdGraph>
ArrangeResult
arrange_bricks_on_table(UsdGraph &g, const ArrangeConfig &config,
                        std::span<const PartId> parts_to_arrange,
                        std::span<const PartId> parts_to_avoid,
                        std::span<const BBox2d> avoid_zones = {}) {
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
	std::vector<pack2d::Rect> obstacles;

	// Helpers
	std::optional<std::int64_t> env_id;
	auto ensure_env = [&](PartId pid) {
		auto e = g.part_env_id(pid);
		if (!e) {
			throw std::runtime_error(std::format(
			    "arrange_bricks_on_table: part id {} does not exist in graph",
			    pid));
		}
		if (env_id) {
			if (*e != *env_id) {
				throw std::runtime_error(std::format(
				    "arrange_bricks_on_table: part id {} is in env {}, "
				    "different from other parts in env {}",
				    pid, *e, *env_id));
			}
		} else {
			env_id = *e;
		}
	};
	auto get_part_bbox = [&](PartId pid) -> BBox3d {
		bool visited = false;
		BBox3d bbox;
		g.topology().parts().visit(pid, [&](const auto &pw) {
			bbox = pw.wrapped().bbox();
			visited = true;
		});
		if (!visited) {
			throw std::runtime_error(std::format(
			    "arrange_bricks_on_table: part id {} does not exist in graph",
			    pid));
		}
		return bbox;
	};
	auto to_cells = [grid](double len) -> std::int32_t {
		if (len <= 0.0)
			return 1;
		double cells_f = std::ceil(len / grid);
		auto cells_i = static_cast<std::int64_t>(cells_f);
		if (cells_i <= 0)
			return 1;
		if (cells_i > std::numeric_limits<std::int32_t>::max())
			return std::numeric_limits<std::int32_t>::max();
		return static_cast<std::int32_t>(cells_i);
	};
	auto add_obstacle_box = [&](const BBox2d &box) {
		BBox2i grid_box =
		    discretize_bbox_xy(box, config.region, config.grid_resolution);
		if (!grid_box.empty()) {
			obstacles.push_back(static_cast<pack2d::Rect>(grid_box));
		}
	};
	auto add_obstacle_part = [&](PartId pid) {
		BBox3d bbox = get_part_bbox(pid);
		auto T_env_part = g.part_pose_relative_to_env(pid);
		if (!T_env_part) {
			throw std::runtime_error(std::format(
			    "arrange_bricks_on_table: part id {} has no pose", pid));
		}
		BBox2d box_xy = project_bbox_xy(bbox, *T_env_part);
		add_obstacle_box(box_xy);
	};

	// Result object
	ArrangeResult result;
	if (parts_to_arrange.empty()) {
		// Ensure ensure_env is called at least once and env_id is set
		return result;
	}

	// Collect parts to arrange
	std::vector<PartId> part_ids;
	std::vector<BBox3d> part_bboxes;
	std::vector<pack2d::RectInput> rects;
	std::size_t N = parts_to_arrange.size();
	part_ids.reserve(N);
	part_bboxes.reserve(N);
	rects.reserve(N);
	for (PartId pid : parts_to_arrange) {
		ensure_env(pid);
		BBox3d bbox = get_part_bbox(pid);
		BBox2d box_xy = bbox_xy(bbox);
		double w = box_xy.max.x() - box_xy.min.x();
		double h = box_xy.max.y() - box_xy.min.y();
		if (w <= 0.0 || h <= 0.0) {
			result.not_placed.push_back(pid);
			continue;
		}

		w += 2.0 * config.clearance_xy;
		h += 2.0 * config.clearance_xy;

		std::int32_t w_cells = to_cells(w);
		std::int32_t h_cells = to_cells(h);
		std::size_t idx = rects.size();
		part_ids.push_back(pid);
		part_bboxes.push_back(bbox);
		rects.push_back(pack2d::RectInput{
		    .id = static_cast<std::int32_t>(idx),
		    .width = w_cells,
		    .height = h_cells,
		});
	}

	// Build obstacle list
	for (const auto &zone : avoid_zones) {
		add_obstacle_box(zone);
	}
	if (config.avoid_all_other_parts) {
		std::unordered_set<PartId> arrange_set(parts_to_arrange.begin(),
		                                       parts_to_arrange.end());
		for (auto [pid, path] : g.parts_in_env(*env_id)) {
			if (!arrange_set.contains(pid)) {
				add_obstacle_part(pid);
			}
		}
	} else {
		for (PartId pid : parts_to_avoid) {
			ensure_env(pid);
			add_obstacle_part(pid);
		}
	}

	// Solve packing
	pack2d::PackResult pack_result = pack2d::pack_all(
	    bin, rects, obstacles, pack2d::Heuristic::BestShortSideFit,
	    config.allow_rotation);

	for (const auto &packed : pack_result.packed) {
		std::size_t idx = static_cast<std::size_t>(packed.id);
		PartId pid = part_ids[idx];
		const BBox3d &bbox = part_bboxes[idx];

		// Determine rotation
		Eigen::Quaterniond q;
		if (packed.rotated) {
			// 90 deg around +z
			q = Eigen::Quaterniond(Eigen::AngleAxisd(std::numbers::pi / 2.0,
			                                         Eigen::Vector3d::UnitZ()));
		} else {
			q = Eigen::Quaterniond::Identity();
		}

		// Calculate geometric center
		Eigen::Vector3d local_center = 0.5 * (bbox.min + bbox.max);

		// Calculate the target center in the env frame.
		// The packer returns integer coordinates for the bottom-left corner.
		Eigen::Vector2d world_xy_min{region.x_min + (packed.x * grid),
		                             region.y_min + (packed.y * grid)};

		// The allocated size in meters
		Eigen::Vector2d alloc_size_xy{packed.width * grid,
		                              packed.height * grid};

		// The geometric center of the allocated space on the table
		Eigen::Vector2d target_center_xy = world_xy_min + 0.5 * alloc_size_xy;

		// Calculate translation
		Eigen::Vector3d t;
		t.head<2>() = target_center_xy - (q * local_center).head<2>();
		t.z() = region.z - bbox.min.z();

		// Set part transform
		Transformd T_env_part{q, t};
		g.set_component_transform(pid, T_env_part);
		result.placed.push_back(pid);
	}

	for (std::int32_t id : pack_result.not_packed_ids) {
		result.not_placed.push_back(part_ids[static_cast<std::size_t>(id)]);
	}
	return result;
}

} // namespace lego_assemble
