export module bricksim.api;

import std;
import bricksim.core.specs;
import bricksim.core.graph;
import bricksim.core.connections;
import bricksim.physx.assembly;
import bricksim.physx.breakage;
import bricksim.physx.physics_graph;
import bricksim.usd.arrange;
import bricksim.omni.usd_physics_bridge;
import bricksim.omni.lego_runtime;
import bricksim.utils.conversions;
import bricksim.utils.transforms;
import bricksim.utils.c4_rotation;
import bricksim.utils.bbox;
import bricksim.io.topology;
import bricksim.vendor;

namespace bricksim::api {

using World = LegoRuntime::World;
using UsdPartId = World::Bridge::UsdPartId;
using PhysicsPartId = World::Bridge::PhysicsPartId;
using UsdConnSegId = World::Bridge::UsdConnSegId;
using PhysicsConnSegId = World::Bridge::PhysicsConnSegId;
using UsdGraph = World::UsdGraph;

using QuatArray = std::array<double, 4>; // w, x, y, z
using PosArray = std::array<double, 3>;  // x, y, z
using PathStr = std::string;
using BBox2dArray = std::array<double, 4>; // min_x, min_y, max_x, max_y
using TransformTuple = std::tuple<QuatArray, PosArray>;

World &lego_world() {
	World *world = LegoRuntime::instance().world();
	if (!world) {
		throw std::runtime_error("No stage is currently attached");
	}
	return *world;
}

auto &usd_graph() {
	return lego_world().usd_graph();
}

auto &usd_topology() {
	return usd_graph().topology();
}

PartId usd_part_id(const PathStr &part_path) {
	return usd_topology().parts().template key_of<PartId>(
	    pxr::SdfPath{part_path});
}

std::vector<PartId> usd_part_ids(const std::vector<PathStr> &part_paths) {
	std::vector<PartId> part_ids;
	part_ids.reserve(part_paths.size());
	for (const auto &path : part_paths) {
		part_ids.emplace_back(usd_part_id(path));
	}
	return part_ids;
}

PathStr usd_part_path(PartId pid) {
	return usd_topology()
	    .parts()
	    .template key_of<pxr::SdfPath>(pid)
	    .GetAsString();
}

std::vector<PathStr> usd_part_paths(const std::vector<PartId> &part_ids) {
	std::vector<PathStr> part_paths;
	part_paths.reserve(part_ids.size());
	for (PartId pid : part_ids) {
		part_paths.emplace_back(usd_part_path(pid));
	}
	return part_paths;
}

PathStr usd_conn_path(ConnSegId csid) {
	return usd_topology()
	    .connection_segments()
	    .template key_of<pxr::SdfPath>(csid)
	    .GetAsString();
}

std::vector<PathStr> usd_conn_paths(const std::vector<ConnSegId> &conn_ids) {
	std::vector<PathStr> conn_paths;
	conn_paths.reserve(conn_ids.size());
	for (ConnSegId csid : conn_ids) {
		conn_paths.emplace_back(usd_conn_path(csid));
	}
	return conn_paths;
}

Transformd transform_from_arrays(const std::optional<QuatArray> &rot,
                                 const std::optional<PosArray> &pos) {
	return {
	    as<Eigen::Quaterniond>(rot.value_or({1.0, 0.0, 0.0, 0.0})),
	    as<Eigen::Vector3d>(pos.value_or({0.0, 0.0, 0.0})),
	};
}

TransformTuple transform_to_arrays(const Transformd &T) {
	const auto &[q, t] = T;
	return {as_array<double>(q), as_array<double, 3>(t)};
}

export PathStr allocate_brick_part(const std::array<BrickUnit, 3> &dimensions,
                                   const BrickColor &color, std::int64_t env_id,
                                   const std::optional<QuatArray> &rot,
                                   const std::optional<PosArray> &pos) {
	BrickPart part{dimensions[0], dimensions[1], dimensions[2], color};
	auto added =
	    usd_graph().template add_part<BrickPart>(env_id, std::move(part));
	if (!added) {
		throw std::runtime_error(std::format(
		    "Failed to allocate brick part of dimensions {}x{}x{} in env {}",
		    dimensions[0], dimensions[1], dimensions[2], env_id));
	}
	const auto &[pid, path] = *added;
	if (rot || pos) {
		Transformd T_env_part = transform_from_arrays(rot, pos);
		usd_graph().set_component_transform(pid, T_env_part);
	}
	return path.GetAsString();
}

export void
allocate_unmanaged_brick_part(const std::array<BrickUnit, 3> &dimensions,
                              const BrickColor &color, const PathStr &path) {
	BrickPart part{dimensions[0], dimensions[1], dimensions[2], color};
	usd_graph()
	    .allocator()
	    .allocate_part_unmanaged<UsdGraph::PartAuthorFor<BrickPart>>(
	        pxr::SdfPath{path}, part);
}

export bool deallocate_part(const PathStr &part_path) {
	return usd_graph().remove_part(pxr::SdfPath{part_path});
}

export TransformTuple compute_graph_transform(const PathStr &a_path,
                                              const PathStr &b_path) {
	if (auto result = usd_topology().lookup_transform(pxr::SdfPath{a_path},
	                                                  pxr::SdfPath{b_path})) {
		return transform_to_arrays(*result);
	}
	throw std::runtime_error(std::format(
	    "No connection path found between parts {} and {}", a_path, b_path));
}

export TransformTuple
compute_connection_transform(const PathStr &stud_path, InterfaceId stud_if,
                             const PathStr &hole_path, InterfaceId hole_if,
                             const std::array<BrickUnit, 2> &offset, int yaw) {
	InterfaceRef stud_ref{usd_part_id(stud_path), stud_if};
	InterfaceRef hole_ref{usd_part_id(hole_path), hole_if};
	ConnectionSegment seg{
	    .offset = as<Eigen::Vector2i>(offset),
	    .yaw = to_c4(yaw),
	};
	InterfaceSpec stud_spec = usd_topology().interface_spec_at(stud_ref);
	InterfaceSpec hole_spec = usd_topology().interface_spec_at(hole_ref);
	Transformd T = seg.compute_transform(stud_spec, hole_spec);
	return transform_to_arrays(T);
}

export std::tuple<TransformTuple, TransformTuple>
compute_connection_local_transform(
    const PathStr &stud_path, InterfaceId stud_if, const PathStr &hole_path,
    InterfaceId hole_if, const std::array<BrickUnit, 2> &offset, int yaw) {
	InterfaceRef stud_ref{usd_part_id(stud_path), stud_if};
	InterfaceRef hole_ref{usd_part_id(hole_path), hole_if};
	ConnectionSegment seg{
	    .offset = as<Eigen::Vector2i>(offset),
	    .yaw = to_c4(yaw),
	};
	InterfaceSpec stud_spec = usd_topology().interface_spec_at(stud_ref);
	InterfaceSpec hole_spec = usd_topology().interface_spec_at(hole_ref);
	ConnectionLocalTransform T =
	    seg.compute_local_transform(stud_spec, hole_spec);
	return {
	    transform_to_arrays(T.T_stud_local),
	    transform_to_arrays(T.T_hole_local),
	};
}

export PathStr create_connection(const PathStr &stud_path, InterfaceId stud_if,
                                 const PathStr &hole_path, InterfaceId hole_if,
                                 const std::array<BrickUnit, 2> &offset,
                                 int yaw) {
	InterfaceRef stud_ref{usd_part_id(stud_path), stud_if};
	InterfaceRef hole_ref{usd_part_id(hole_path), hole_if};
	ConnectionSegment seg{
	    .offset = as<Eigen::Vector2i>(offset),
	    .yaw = to_c4(yaw),
	};
	auto connected = usd_graph().connect(stud_ref, hole_ref, seg);
	if (!connected) {
		throw std::runtime_error(std::format(
		    "Failed to create connection between stud {} and hole {}",
		    stud_path, hole_path));
	}
	const auto &[csid, conn_path] = *connected;
	return conn_path.GetAsString();
}

export bool deallocate_connection(const PathStr &connection_path) {
	return usd_graph().disconnect(pxr::SdfPath{connection_path});
}

export bool deallocate_all_managed(std::int64_t env_id) {
	return usd_graph().allocator().deallocate_managed_all(env_id);
}

export nlohmann::ordered_json export_lego(std::int64_t env_id) {
	return LegoRuntime::Serializer{}.export_usd_graph(usd_graph(), env_id);
}

export std::tuple<std::unordered_map<std::int64_t, PathStr>,
                  std::unordered_map<std::int64_t, PathStr>>
import_lego(const nlohmann::ordered_json &json, std::int64_t env_id,
            const std::optional<QuatArray> &ref_rot,
            const std::optional<PosArray> &ref_pos) {
	Transformd T_env_ref = transform_from_arrays(ref_rot, ref_pos);
	auto imported = LegoRuntime::Serializer{}.import_usd_graph(
	    json, usd_graph(), env_id, T_env_ref);
	// Convert id to path
	std::unordered_map<std::int64_t, PathStr> part_paths;
	for (const auto &[json_id, pid] : imported.parts) {
		part_paths.emplace(json_id, usd_part_path(pid));
	}
	std::unordered_map<std::int64_t, PathStr> conn_paths;
	for (const auto &[json_id, csid] : imported.conns) {
		conn_paths.emplace(json_id, usd_conn_path(csid));
	}
	return {std::move(part_paths), std::move(conn_paths)};
}

export std::unordered_map<std::int64_t, TransformTuple>
compute_structure_transforms(const nlohmann::ordered_json &json,
                             std::int64_t root) {
	using Graph = LegoGraph<World::PartTypeList>;
	Graph g;
	JsonTopology topology = json;
	auto imported = LegoRuntime::Serializer{}.import_graph(topology, g);
	auto &jid2pid = imported.parts;
	// Build inverse mapping for later lookup
	std::unordered_map<PartId, std::int64_t> pid2jid;
	for (const auto &[jid, pid] : jid2pid) {
		pid2jid.emplace(pid, jid);
	}
	// Compute transforms
	std::unordered_map<std::int64_t, TransformTuple> result;
	for (auto [pid_u, T_root_u] :
	     structure_transforms(g, root, topology.pose_hints, imported.parts)) {
		std::int64_t jid_u = pid2jid.at(pid_u);
		result.emplace(jid_u, transform_to_arrays(T_root_u));
	}
	return result;
}

export std::tuple<std::vector<PathStr>, std::vector<PathStr>>
compute_connected_component(const PathStr &part_path) {
	const PartId *part_id_ptr =
	    usd_topology().parts().template find_key<PartId>(
	        pxr::SdfPath{part_path});
	if (!part_id_ptr) {
		return {{}, {}};
	}
	PartId part_id = *part_id_ptr;
	std::vector<PathStr> part_paths;
	std::vector<PathStr> conn_paths;
	for (PartId pid : usd_topology().component_view(part_id).vertices()) {
		auto entry = usd_topology().parts().entry_of(pid);
		part_paths.emplace_back(entry.key<pxr::SdfPath>().GetAsString());
		entry.visit([&](const auto &pw) {
			for (ConnSegId csid : pw.incomings()) {
				conn_paths.emplace_back(usd_conn_path(csid));
			}
		});
	}
	return {std::move(part_paths), std::move(conn_paths)};
}

export bool are_parts_connected(const PathStr &part_a_path,
                                const PathStr &part_b_path) {
	return usd_topology().is_connected(usd_part_id(part_a_path),
	                                   usd_part_id(part_b_path));
}

export bool does_connection_exist(const PathStr &stud_path, InterfaceId stud_if,
                                  const PathStr &hole_path,
                                  InterfaceId hole_if) {
	ConnSegRef conn_ref{{usd_part_id(stud_path), stud_if},
	                    {usd_part_id(hole_path), hole_if}};
	return usd_topology().connection_segments().contains(conn_ref);
}

TableRect parse_table_rect(const BBox2dArray &table_xy, double table_z) {
	return {
	    .x_min = table_xy[0],
	    .y_min = table_xy[1],
	    .x_max = table_xy[2],
	    .y_max = table_xy[3],
	    .z = table_z,
	};
}

std::tuple<std::vector<PathStr>, std::vector<PathStr>>
arrange_results_to_str(ArrangeResult &&result) {
	return {
	    usd_part_paths(result.placed),
	    usd_part_paths(result.not_placed),
	};
}

export std::tuple<std::vector<PathStr>, std::vector<PathStr>>
arrange_parts_on_table(
    const std::vector<PathStr> &parts_to_arrange,
    const std::optional<std::vector<PathStr>> &parts_to_avoid,
    const std::optional<std::vector<BBox2dArray>> &obstacles,
    const BBox2dArray &table_xy, double table_z,
    std::optional<double> clearance_xy, std::optional<double> grid_resolution,
    std::optional<bool> allow_rotation,
    std::optional<bool> avoid_all_other_parts,
    const std::optional<std::vector<std::int32_t>> &structure_ids) {
	ArrangeConfig config;
	config.region = parse_table_rect(table_xy, table_z);
	if (clearance_xy) {
		config.clearance_xy = *clearance_xy;
	}
	if (grid_resolution) {
		config.grid_resolution = *grid_resolution;
	}
	if (allow_rotation) {
		config.allow_rotation = *allow_rotation;
	}
	if (avoid_all_other_parts) {
		config.avoid_all_other_parts = *avoid_all_other_parts;
	}
	std::vector<PartId> arrange_pids = usd_part_ids(parts_to_arrange);
	std::vector<PartId> avoid_pids;
	if (parts_to_avoid) {
		avoid_pids = usd_part_ids(*parts_to_avoid);
	}
	std::vector<BBox2d> regions_to_avoid;
	if (obstacles) {
		regions_to_avoid.reserve(obstacles->size());
		for (const auto &obs_arr : *obstacles) {
			regions_to_avoid.push_back({
			    .min = {obs_arr[0], obs_arr[1]},
			    .max = {obs_arr[2], obs_arr[3]},
			});
		}
	}
	std::span<const std::int32_t> structure_ids_view{};
	if (structure_ids) {
		structure_ids_view = *structure_ids;
	}

	ArrangeResult result = arrange_parts_on_table(
	    lego_world().usd_graph(), config, arrange_pids, avoid_pids,
	    regions_to_avoid, structure_ids_view);
	return arrange_results_to_str(std::move(result));
}

export std::vector<BBox2dArray>
compute_obstacle_regions(const std::vector<PathStr> &obstacle_paths,
                         const BBox2dArray &table_xy, double table_z,
                         double clearance_height) {
	TableRect table_region = parse_table_rect(table_xy, table_z);
	const pxr::UsdStageRefPtr &stage = usd_graph().stage();
	std::vector<pxr::SdfPath> obstacle_paths_pxr;
	obstacle_paths_pxr.reserve(obstacle_paths.size());
	for (auto &path_str : obstacle_paths) {
		obstacle_paths_pxr.emplace_back(path_str);
	}
	std::vector<BBox2dArray> result;
	std::vector<BBox2d> regions = compute_regions_to_avoid(
	    stage, table_region, clearance_height, obstacle_paths_pxr);
	result.reserve(regions.size());
	for (auto &region : regions) {
		result.push_back({
		    region.min.x(),
		    region.min.y(),
		    region.max.x(),
		    region.max.y(),
		});
	}
	return result;
}

export std::tuple<std::vector<PathStr>, std::vector<PathStr>>
arrange_parts_in_workspace(
    const PathStr &workspace_path, const std::vector<PathStr> &parts_to_arrange,
    const std::optional<std::vector<std::int32_t>> &structure_ids) {
	const pxr::UsdStageRefPtr &stage = usd_graph().stage();
	pxr::UsdPrim workspace_prim =
	    stage->GetPrimAtPath(pxr::SdfPath{workspace_path});
	std::optional<WorkspaceConfig> workspace_config =
	    WorkspaceConfig::from_prim(workspace_prim);
	if (!workspace_config) {
		throw std::runtime_error(std::format(
		    "arrange_parts_in_workspace: {} is not a valid workspace prim",
		    workspace_path));
	}
	std::vector<PartId> arrange_pids = usd_part_ids(parts_to_arrange);
	std::span<const std::int32_t> structure_ids_view{};
	if (structure_ids) {
		structure_ids_view = *structure_ids;
	}
	ArrangeResult result = arrange_parts_in_workspace(
	    usd_graph(), *workspace_config, arrange_pids, structure_ids_view);
	return arrange_results_to_str(std::move(result));
}

export using AssemblyThresholds = bricksim::AssemblyThresholds;

export std::string repr_assembly_thresholds(const AssemblyThresholds &t) {
	return std::format(
	    "AssemblyThresholds(enabled={}, distance_tolerance={}, "
	    "max_penetration={}, z_angle_tolerance={}, required_force={}, "
	    "yaw_tolerance={}, position_tolerance={})",
	    t.Enabled, t.DistanceTolerance, t.MaxPenetration, t.ZAngleTolerance,
	    t.RequiredForce, t.YawTolerance, t.PositionTolerance);
}

export void set_assembly_thresholds(const AssemblyThresholds &thr) {
	LegoRuntime::instance().set_assembly_thresholds(thr);
}

export AssemblyThresholds get_assembly_thresholds() {
	return LegoRuntime::instance().get_assembly_thresholds();
}

export using BreakageThresholds = bricksim::BreakageThresholds;

export std::string repr_breakage_thresholds(const BreakageThresholds &t) {
	return std::format(
	    "BreakageThresholds(enabled={}, contact_regularization={}, "
	    "clutch_axial_compliance={}, clutch_radial_compliance={}, "
	    "clutch_tangential_compliance={}, friction_coefficient={}, "
	    "preloaded_force={}, slack_fraction_warn={}, "
	    "slack_fraction_b_floor={}, debug_dump={}, breakage_cooldown_time={})",
	    t.Enabled, t.ContactRegularization, t.ClutchAxialCompliance,
	    t.ClutchRadialCompliance, t.ClutchTangentialCompliance,
	    t.FrictionCoefficient, t.PreloadedForce, t.SlackFractionWarn,
	    t.SlackFractionBFloor, t.DebugDump, t.BreakageCooldownTime);
}

export void set_breakage_thresholds(const BreakageThresholds &thr) {
	LegoRuntime::instance().set_breakage_thresholds(thr);
}

export BreakageThresholds get_breakage_thresholds() {
	return LegoRuntime::instance().get_breakage_thresholds();
}

std::optional<PathStr> lookup_path_by_physx_pid(PartId physx_pid) {
	auto *bridge = lego_world().bridge();
	if (!bridge) {
		return std::nullopt;
	}
	const auto &mapping = bridge->part_mapping();
	const UsdPartId *usd_pid_ptr =
	    mapping.template find_key<UsdPartId>(PhysicsPartId{physx_pid});
	if (!usd_pid_ptr) {
		return std::nullopt;
	}
	return usd_part_path(usd_pid_ptr->value());
}

export double get_connection_utilization(const PathStr &connection_path) {
	World &world = lego_world();
	auto *bridge = world.bridge();
	if (!bridge) {
		return -1.0;
	}
	auto *physics_graph = lego_world().physics_graph();
	if (!physics_graph) {
		// If bridge exists, physics graph should also exist
		throw std::runtime_error("Physics graph is not available");
	}
	const ConnSegId *usd_csid_ptr =
	    usd_topology().connection_segments().template find_key<ConnSegId>(
	        pxr::SdfPath{connection_path});
	if (!usd_csid_ptr) {
		// No such connection
		return -1.0;
	}
	const PhysicsConnSegId *physics_csid_ptr =
	    bridge->connection_mapping().find_key<PhysicsConnSegId>(
	        UsdConnSegId{*usd_csid_ptr});
	if (!physics_csid_ptr) {
		// This happens on graph divergence
		return -1.0;
	}
	auto *csw = physics_graph->topology().connection_segments().find_value(
	    physics_csid_ptr->value());
	if (!csw) {
		return -1.0;
	}
	return csw->utilization();
}

export struct ConnectionInfo {
	ConnSegId physics_csid;
	PartId physics_stud_pid;
	PartId physics_hole_pid;
	InterfaceId stud_ifid;
	InterfaceId hole_ifid;
	std::array<int, 2> offset;
	int yaw;
	std::int64_t env_id;
	PartId usd_stud_pid;
	PartId usd_hole_pid;
	PathStr stud_path;
	PathStr hole_path;
	std::optional<ConnSegId> usd_csid;
	std::optional<PathStr> conn_path;

	ConnectionInfo() = default;
	ConnectionInfo(const bricksim::ConnectionInfo &info)
	    : physics_csid(info.physics_csid),
	      physics_stud_pid(info.physics_stud_pid),
	      physics_hole_pid(info.physics_hole_pid), stud_ifid(info.stud_ifid),
	      hole_ifid(info.hole_ifid),
	      offset({info.conn_seg.offset.x(), info.conn_seg.offset.y()}),
	      yaw(static_cast<int>(info.conn_seg.yaw)), env_id(info.env_id),
	      usd_stud_pid(info.usd_stud_pid), usd_hole_pid(info.usd_hole_pid),
	      stud_path(info.stud_path.GetAsString()),
	      hole_path(info.hole_path.GetAsString()), usd_csid(info.usd_csid),
	      conn_path(info.conn_path.transform(
	          [](const pxr::SdfPath &p) { return p.GetAsString(); })) {}

	std::string repr() const {
		return std::format(
		    "ConnectionInfo(physics_csid={}, physics_stud_pid={}, "
		    "physics_hole_pid={}, stud_ifid={}, hole_ifid={}, offset=[{}, {}], "
		    "yaw={}, env_id={}, usd_stud_pid={}, usd_hole_pid={}, "
		    "stud_path='{}', hole_path='{}', usd_csid={}, conn_path={})",
		    physics_csid, physics_stud_pid, physics_hole_pid, stud_ifid,
		    hole_ifid, offset[0], offset[1], yaw, env_id, usd_stud_pid,
		    usd_hole_pid, stud_path, hole_path,
		    usd_csid ? std::to_string(*usd_csid) : "None",
		    conn_path ? std::format("'{}'", *conn_path) : "None");
	}
};

export std::vector<ConnectionInfo> get_assembled_connections(bool clear) {
	auto *bridge = lego_world().bridge();
	if (!bridge) {
		return {};
	}
	auto conns = bridge->get_assembled_connections();
	std::vector<ConnectionInfo> result;
	result.reserve(conns.size());
	for (const auto &conn : conns) {
		result.emplace_back(conn);
	}
	if (clear) {
		bridge->clear_assembled_connections();
	}
	return result;
}

export std::vector<ConnectionInfo> get_disassembled_connections(bool clear) {
	auto *bridge = lego_world().bridge();
	if (!bridge) {
		return {};
	}
	auto conns = bridge->get_disassembled_connections();
	std::vector<ConnectionInfo> result;
	result.reserve(conns.size());
	for (const auto &conn : conns) {
		result.emplace_back(conn);
	}
	if (clear) {
		bridge->clear_disassembled_connections();
	}
	return result;
}

export struct AssemblyDebugInfo {
	bool accepted;
	double relative_distance;
	double tilt;
	double projected_force;
	double yaw_error;
	double position_error;
	std::array<double, 2> grid_pos;
	std::array<int, 2> grid_pos_snapped;
	PathStr stud_path;
	InterfaceId stud_interface;
	PathStr hole_path;
	InterfaceId hole_interface;

	std::string repr() const {
		return std::format(
		    "AssemblyDebugInfo(accepted={}, relative_distance={}, tilt={}, "
		    "projected_force={}, yaw_error={}, position_error={}, "
		    "grid_pos=[{}, {}], grid_pos_snapped=[{}, {}], stud_path='{}', "
		    "stud_interface={}, hole_path='{}', hole_interface={})",
		    accepted, relative_distance, tilt, projected_force, yaw_error,
		    position_error, grid_pos[0], grid_pos[1], grid_pos_snapped[0],
		    grid_pos_snapped[1], stud_path, stud_interface, hole_path,
		    hole_interface);
	}

	static std::optional<AssemblyDebugInfo>
	from(const PhysicsAssemblyDebugInfo &info) {
		const auto &[stud_ref, hole_ref] = info.csref;
		const auto &[stud_pid, stud_if] = stud_ref;
		const auto &[hole_pid, hole_if] = hole_ref;
		auto stud_path_opt = lookup_path_by_physx_pid(stud_pid);
		auto hole_path_opt = lookup_path_by_physx_pid(hole_pid);
		if (!stud_path_opt || !hole_path_opt) {
			return std::nullopt;
		}
		return AssemblyDebugInfo{
		    .accepted = info.accepted,
		    .relative_distance = info.relative_distance,
		    .tilt = info.tilt,
		    .projected_force = info.projected_force,
		    .yaw_error = info.yaw_error,
		    .position_error = info.position_error,
		    .grid_pos =
		        {
		            info.grid_pos(0),
		            info.grid_pos(1),
		        },
		    .grid_pos_snapped =
		        {
		            info.grid_pos_snapped(0),
		            info.grid_pos_snapped(1),
		        },
		    .stud_path = *stud_path_opt,
		    .stud_interface = stud_if,
		    .hole_path = *hole_path_opt,
		    .hole_interface = hole_if,
		};
	}
};

export std::vector<AssemblyDebugInfo> get_assembly_debug_infos() {
	std::vector<AssemblyDebugInfo> result;
	auto *physics_graph = lego_world().physics_graph();
	if (!physics_graph) {
		return result;
	}
	for (auto &&info : physics_graph->get_assembly_debug_infos()) {
		auto py_info_opt = AssemblyDebugInfo::from(info);
		if (py_info_opt) {
			result.emplace_back(*py_info_opt);
		}
	}
	return result;
}

export std::tuple<std::unordered_map<PathStr, PartId>,
                  std::unordered_map<PathStr, ConnSegId>>
get_usd_id_mappings() {
	std::unordered_map<PathStr, PartId> part_mapping;
	std::unordered_map<PathStr, ConnSegId> conn_mapping;
	usd_topology().parts().for_each([&](const auto &keys) {
		part_mapping.emplace(std::get<pxr::SdfPath>(keys).GetAsString(),
		                     std::get<PartId>(keys));
	});
	for (const auto &entry : usd_topology().connection_segments().view()) {
		conn_mapping.emplace(entry.key<pxr::SdfPath>().GetAsString(),
		                     entry.key<ConnSegId>());
	}
	return {std::move(part_mapping), std::move(conn_mapping)};
}

export std::tuple<std::unordered_map<PathStr, PartId>,
                  std::unordered_map<PathStr, ConnSegId>>
get_physx_id_mappings() {
	auto *bridge = lego_world().bridge();
	if (!bridge) {
		throw std::runtime_error("Physics graph is not available");
	}
	std::unordered_map<PathStr, PartId> part_mapping;
	std::unordered_map<PathStr, ConnSegId> conn_mapping;
	for (const auto &[physx_id, usd_id] : bridge->part_mapping().view()) {
		part_mapping.emplace(usd_topology()
		                         .parts()
		                         .key_of<pxr::SdfPath>(usd_id.value())
		                         .GetAsString(),
		                     physx_id.value());
	}
	for (const auto &[physx_id, usd_id] : bridge->connection_mapping().view()) {
		conn_mapping.emplace(usd_topology()
		                         .connection_segments()
		                         .key_of<pxr::SdfPath>(usd_id.value())
		                         .GetAsString(),
		                     physx_id.value());
	}
	return {std::move(part_mapping), std::move(conn_mapping)};
}

export void set_sync_to_usd(bool sync) {
	LegoRuntime::instance().set_sync_to_usd(sync);
}

export bool get_sync_to_usd() {
	return LegoRuntime::instance().get_sync_to_usd();
}

export void update_part_prototypes() {
	usd_graph().update_part_prototypes();
}

export using PhysicsStepProfiling = bricksim::PhysicsStepProfiling;

export PhysicsStepProfiling get_last_step_profiling() {
	auto *physics_graph = lego_world().physics_graph();
	if (!physics_graph) {
		throw std::runtime_error("Physics graph is not available");
	}
	return physics_graph->last_step_profiling();
}

export std::string repr_physics_step_profiling(const PhysicsStepProfiling &p) {
	return std::format("PhysicsStepProfiling(sim_time={}, step_time={:.3f} ms)",
	                   p.sim_time, p.step_time * 1000.0);
}

} // namespace bricksim::api
