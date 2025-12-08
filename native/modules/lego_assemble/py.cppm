export module lego_assemble.py;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.core.assembly;
import lego_assemble.physx.physics_graph;
import lego_assemble.usd.arrange;
import lego_assemble.omni.usd_physics_bridge;
import lego_assemble.omni.lego_runtime;
import lego_assemble.utils.conversions;
import lego_assemble.utils.transforms;
import lego_assemble.utils.c4_rotation;
import lego_assemble.io.topology;
import lego_assemble.vendor;

namespace lego_assemble::py {

using World = LegoRuntime::World;

World &lego_world() {
	World *world = LegoRuntime::instance().world();
	if (!world) {
		throw std::runtime_error("No stage is currently attached");
	}
	return *world;
}

export std::string
allocate_brick_part(std::array<BrickUnit, 3> dimensions, BrickColor color,
                    std::int64_t env_id,
                    std::optional<std::array<double, 4>> rot,
                    std::optional<std::array<double, 3>> pos) {
	auto &usd_graph = lego_world().usd_graph();
	BrickPart part{dimensions[0], dimensions[1], dimensions[2], color};
	auto added =
	    usd_graph.template add_part<BrickPart>(env_id, std::move(part));
	if (!added) {
		throw std::runtime_error(std::format(
		    "Failed to allocate brick part of dimensions {}x{}x{} in env {}",
		    dimensions[0], dimensions[1], dimensions[2], env_id));
	}
	const auto &[pid, path] = *added;
	if (rot || pos) {
		auto rot_val = rot.value_or({1.0, 0.0, 0.0, 0.0});
		auto pos_val = pos.value_or({0.0, 0.0, 0.0});
		Transformd T_env_part{
		    as<Eigen::Quaterniond>(rot_val),
		    as<Eigen::Vector3d>(pos_val),
		};
		usd_graph.set_component_transform(pid, T_env_part);
	}
	return path.GetAsString();
}

export bool deallocate_part(const std::string &part_path) {
	auto &usd_graph = lego_world().usd_graph();
	pxr::SdfPath path{part_path};
	return usd_graph.remove_part(path);
}

export std::tuple<std::array<double, 4>, std::array<double, 3>>
compute_graph_transform(const std::string &a_path_str,
                        const std::string &b_path_str) {
	auto &usd_graph = lego_world().usd_graph();
	pxr::SdfPath a_path{a_path_str};
	pxr::SdfPath b_path{b_path_str};
	auto result = usd_graph.topology().lookup_transform(a_path, b_path);
	if (!result) {
		throw std::runtime_error(
		    std::format("No connection path found between parts {} and {}",
		                a_path_str, b_path_str));
	}
	const auto &[R, t] = *result;
	return {as_array<double>(R), as_array<double, 3>(t)};
}

export std::tuple<std::array<double, 4>, std::array<double, 3>>
compute_connection_transform(const std::string &stud_path_str,
                             InterfaceId stud_if,
                             const std::string &hole_path_str,
                             InterfaceId hole_if,
                             std::array<BrickUnit, 2> offset, int yaw) {
	auto &usd_graph = lego_world().usd_graph();
	pxr::SdfPath stud_path{stud_path_str};
	pxr::SdfPath hole_path{hole_path_str};
	const auto &topology = usd_graph.topology();
	const auto *stud_pid_ptr =
	    topology.parts().template project_key<pxr::SdfPath, PartId>(stud_path);
	if (!stud_pid_ptr) {
		throw std::runtime_error(std::format(
		    "Stud path {} does not correspond to a known part", stud_path_str));
	}
	const auto *hole_pid_ptr =
	    topology.parts().template project_key<pxr::SdfPath, PartId>(hole_path);
	if (!hole_pid_ptr) {
		throw std::runtime_error(std::format(
		    "Hole path {} does not correspond to a known part", hole_path_str));
	}
	InterfaceRef stud_ref{*stud_pid_ptr, stud_if};
	InterfaceRef hole_ref{*hole_pid_ptr, hole_if};
	ConnectionSegment seg{
	    .offset = as<Eigen::Vector2i>(offset),
	    .yaw = to_c4(yaw),
	};
	auto stud_spec = topology.get_interface_spec(stud_ref);
	if (!stud_spec) {
		throw std::runtime_error(
		    std::format("Stud interface {} on part {} is not found", stud_if,
		                stud_path_str));
	}
	auto hole_spec = topology.get_interface_spec(hole_ref);
	if (!hole_spec) {
		throw std::runtime_error(
		    std::format("Hole interface {} on part {} is not found", hole_if,
		                hole_path_str));
	}
	auto [R, t] = seg.compute_transform(*stud_spec, *hole_spec);
	return {as_array<double>(R), as_array<double, 3>(t)};
}

export std::string create_connection(const std::string &stud_path_str,
                                     InterfaceId stud_if,
                                     const std::string &hole_path_str,
                                     InterfaceId hole_if,
                                     std::array<BrickUnit, 2> offset, int yaw) {
	auto &usd_graph = lego_world().usd_graph();
	pxr::SdfPath stud_path{stud_path_str};
	pxr::SdfPath hole_path{hole_path_str};
	const auto &topology = usd_graph.topology();
	const auto *stud_pid_ptr =
	    topology.parts().template project_key<pxr::SdfPath, PartId>(stud_path);
	if (!stud_pid_ptr) {
		throw std::runtime_error(std::format(
		    "Stud path {} does not correspond to a known part", stud_path_str));
	}
	const auto *hole_pid_ptr =
	    topology.parts().template project_key<pxr::SdfPath, PartId>(hole_path);
	if (!hole_pid_ptr) {
		throw std::runtime_error(std::format(
		    "Hole path {} does not correspond to a known part", hole_path_str));
	}
	InterfaceRef stud_ref{*stud_pid_ptr, stud_if};
	InterfaceRef hole_ref{*hole_pid_ptr, hole_if};
	ConnectionSegment seg{
	    .offset = as<Eigen::Vector2i>(offset),
	    .yaw = to_c4(yaw),
	};
	auto connected = usd_graph.connect(stud_ref, hole_ref, seg);
	if (!connected) {
		throw std::runtime_error(std::format(
		    "Failed to create connection between stud {} and hole {}",
		    stud_path_str, hole_path_str));
	}
	const auto &[csid, conn_path] = *connected;
	return conn_path.GetAsString();
}

export bool deallocate_connection(const std::string &connection_path) {
	auto &usd_graph = lego_world().usd_graph();
	pxr::SdfPath path{connection_path};
	return usd_graph.disconnect(path);
}

export bool deallocate_all_managed(std::int64_t env_id) {
	auto &usd_graph = lego_world().usd_graph();
	return usd_graph.allocator().deallocate_managed_all(env_id);
}

export std::string export_lego(std::int64_t env_id) {
	auto &usd_graph = lego_world().usd_graph();
	nlohmann::ordered_json json =
	    LegoRuntime::Serializer{}.export_usd_graph(usd_graph, env_id);
	return json.dump();
}

export void import_lego(const std::string &json_str, std::int64_t env_id,
                        std::optional<std::array<double, 4>> ref_rot,
                        std::optional<std::array<double, 3>> ref_pos) {
	nlohmann::ordered_json json = nlohmann::ordered_json::parse(json_str);
	auto &usd_graph = lego_world().usd_graph();
	auto ref_rot_val = ref_rot.value_or({1.0, 0.0, 0.0, 0.0});
	auto ref_pos_val = ref_pos.value_or({0.0, 0.0, 0.0});
	Transformd T_env_ref{
	    as<Eigen::Quaterniond>(ref_rot_val),
	    as<Eigen::Vector3d>(ref_pos_val),
	};
	LegoRuntime::Serializer{}.import(json, usd_graph, env_id, T_env_ref);
}

export std::tuple<std::vector<std::string>, std::vector<std::string>>
compute_connected_component(const std::string &part_path_str) {
	auto &usd_graph = lego_world().usd_graph();
	pxr::SdfPath part_path{part_path_str};
	const auto &topology = usd_graph.topology();
	const PartId *part_id_ptr =
	    topology.parts().template project_key<pxr::SdfPath, PartId>(part_path);
	if (!part_id_ptr) {
		return {{}, {}};
	}
	PartId part_id = *part_id_ptr;
	std::vector<std::string> part_paths;
	std::vector<std::string> conn_paths;
	for (PartId pid : topology.component_view(part_id).vertices()) {
		const pxr::SdfPath *p_path_ptr =
		    topology.parts().template project_key<PartId, pxr::SdfPath>(pid);
		if (!p_path_ptr) {
			log_error("PartId {} has no corresponding path", pid);
			continue;
		}
		part_paths.push_back(p_path_ptr->GetAsString());

		auto add_conn = [&](ConnSegId csid) {
			const pxr::SdfPath *c_path_ptr =
			    topology.connection_segments()
			        .template project<ConnSegId, pxr::SdfPath>(csid);
			if (!c_path_ptr) {
				log_error("ConnSegId {} has no corresponding path", csid);
				return;
			}
			conn_paths.push_back(c_path_ptr->GetAsString());
		};
		bool visited = topology.parts().visit(pid, [&](const auto &pw) {
			for (ConnSegId csid : pw.incomings()) {
				add_conn(csid);
			}
			for (ConnSegId csid : pw.outgoings()) {
				add_conn(csid);
			}
		});
		if (!visited) {
			log_error("Failed to visit PartId {}", pid);
		}
	}
	return {part_paths, conn_paths};
}

export std::tuple<std::vector<std::string>, std::vector<std::string>>
arrange_bricks_on_table(
    std::vector<std::string> parts_to_arrange,
    std::vector<std::string> parts_to_avoid,
    std::optional<std::vector<std::array<double, 4>>> obstacles,
    std::array<double, 4> table_xy, double table_z,
    std::optional<double> clearance_xy, std::optional<double> grid_resolution,
    std::optional<bool> allow_rotation) {
	ArrangeConfig config;
	config.region = {
	    .x_min = table_xy[0],
	    .y_min = table_xy[1],
	    .x_max = table_xy[2],
	    .y_max = table_xy[3],
	    .z = table_z,
	};
	if (clearance_xy) {
		config.clearance_xy = *clearance_xy;
	}
	if (grid_resolution) {
		config.grid_resolution = *grid_resolution;
	}
	if (allow_rotation) {
		config.allow_rotation = *allow_rotation;
	}
	const auto &topology = lego_world().usd_graph().topology();
	std::vector<PartId> arrange_pids;
	arrange_pids.reserve(parts_to_arrange.size());
	for (const auto &part_path_str : parts_to_arrange) {
		pxr::SdfPath part_path{part_path_str};
		const auto *part_id_ptr =
		    topology.parts().template project_key<pxr::SdfPath, PartId>(
		        part_path);
		if (!part_id_ptr) {
			throw std::runtime_error(std::format(
			    "arrange_bricks_on_table: part path {} does not exist in graph",
			    part_path_str));
		}
		arrange_pids.push_back(*part_id_ptr);
	}
	std::vector<PartId> avoid_pids;
	avoid_pids.reserve(parts_to_avoid.size());
	for (const auto &part_path_str : parts_to_avoid) {
		pxr::SdfPath part_path{part_path_str};
		const auto *part_id_ptr =
		    topology.parts().template project_key<pxr::SdfPath, PartId>(
		        part_path);
		if (!part_id_ptr) {
			throw std::runtime_error(std::format(
			    "arrange_bricks_on_table: part path {} does not exist in graph",
			    part_path_str));
		}
		avoid_pids.push_back(*part_id_ptr);
	}
	std::vector<BBox2d> avoid_zones;
	if (obstacles) {
		avoid_zones.reserve(obstacles->size());
		for (const auto &obs_arr : *obstacles) {
			avoid_zones.push_back({
			    .min = {obs_arr[0], obs_arr[1]},
			    .max = {obs_arr[2], obs_arr[3]},
			});
		}
	}
	ArrangeResult result =
	    arrange_bricks_on_table(lego_world().usd_graph(), config, arrange_pids,
	                            avoid_pids, avoid_zones);
	std::vector<std::string> placed_paths;
	placed_paths.reserve(result.placed.size());
	for (PartId pid : result.placed) {
		const pxr::SdfPath *part_path_ptr =
		    topology.parts().template project_key<PartId, pxr::SdfPath>(pid);
		if (!part_path_ptr) {
			throw std::runtime_error(std::format(
			    "arrange_bricks_on_table: part id {} has no corresponding path",
			    pid));
		}
		placed_paths.push_back(part_path_ptr->GetAsString());
	}
	std::vector<std::string> not_placed_paths;
	not_placed_paths.reserve(result.not_placed.size());
	for (PartId pid : result.not_placed) {
		const pxr::SdfPath *part_path_ptr =
		    topology.parts().template project_key<PartId, pxr::SdfPath>(pid);
		if (!part_path_ptr) {
			throw std::runtime_error(std::format(
			    "arrange_bricks_on_table: part id {} has no corresponding path",
			    pid));
		}
		not_placed_paths.push_back(part_path_ptr->GetAsString());
	}
	return {placed_paths, not_placed_paths};
}

export using AssemblyThresholds = lego_assemble::AssemblyThresholds;

export std::string repr_assembly_thresholds(const AssemblyThresholds &t) {
	return std::format(
	    "AssemblyThresholds(distance_tolerance={}, max_penetration={}, "
	    "z_angle_tolerance={}, required_force={}, yaw_tolerance={}, "
	    "position_tolerance={})",
	    t.DistanceTolerance, t.MaxPenetration, t.ZAngleTolerance,
	    t.RequiredForce, t.YawTolerance, t.PositionTolerance);
}

export void set_assembly_thresholds(const AssemblyThresholds &thr) {
	LegoRuntime::instance().set_assembly_thresholds(thr);
}

export AssemblyThresholds get_assembly_thresholds() {
	return LegoRuntime::instance().get_assembly_thresholds();
}

std::optional<std::string> lookup_path_by_physx_pid(PartId physx_pid) {
	using PhysicsPartId = World::Bridge::PhysicsPartId;
	using UsdPartId = World::Bridge::UsdPartId;
	auto *bridge = lego_world().bridge();
	if (!bridge) {
		return std::nullopt;
	}
	const auto &mapping = bridge->part_mapping();
	const UsdPartId *usd_pid_ptr =
	    mapping.template project<PhysicsPartId, UsdPartId>(
	        PhysicsPartId{physx_pid});
	if (!usd_pid_ptr) {
		return std::nullopt;
	}
	const auto &usd_topology = lego_world().usd_graph().topology();
	PartId usd_pid = usd_pid_ptr->value();
	const pxr::SdfPath *part_path_ptr =
	    usd_topology.parts().template project_key<PartId, pxr::SdfPath>(
	        usd_pid);
	if (!part_path_ptr) {
		return std::nullopt;
	}
	return part_path_ptr->GetAsString();
}

export struct PyAssemblyDebugInfo {
	bool accepted;
	double relative_distance;
	double tilt;
	double projected_force;
	double yaw_error;
	double position_error;
	std::array<double, 2> grid_pos;
	std::array<int, 2> grid_pos_snapped;
	std::string stud_path;
	InterfaceId stud_interface;
	std::string hole_path;
	InterfaceId hole_interface;

	std::string repr() const {
		return std::format(
		    "AssemblyDebugInfo(accepted={}, relative_distance={}, tilt={}, "
		    "projected_force={}, yaw_error={}, position_error={}, "
		    "grid_pos=[{}, {}], "
		    "grid_pos_snapped=[{}, {}], stud_path='{}', stud_interface={}, "
		    "hole_path='{}', hole_interface={})",
		    accepted, relative_distance, tilt, projected_force, yaw_error,
		    position_error, grid_pos[0], grid_pos[1], grid_pos_snapped[0],
		    grid_pos_snapped[1], stud_path, stud_interface, hole_path,
		    hole_interface);
	}

	static std::optional<PyAssemblyDebugInfo>
	from(const PhysicsAssemblyDebugInfo &info) {
		const auto &[stud_ref, hole_ref] = info.csref;
		const auto &[stud_pid, stud_if] = stud_ref;
		const auto &[hole_pid, hole_if] = hole_ref;
		auto stud_path_opt = lookup_path_by_physx_pid(stud_pid);
		auto hole_path_opt = lookup_path_by_physx_pid(hole_pid);
		if (!stud_path_opt || !hole_path_opt) {
			return std::nullopt;
		}
		return PyAssemblyDebugInfo{
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

export std::vector<PyAssemblyDebugInfo> get_assembly_debug_infos() {
	std::vector<PyAssemblyDebugInfo> result;
	auto *physics_graph = lego_world().physics_graph();
	if (!physics_graph) {
		return result;
	}
	for (auto &&info : physics_graph->get_assembly_debug_infos()) {
		auto py_info_opt = PyAssemblyDebugInfo::from(info);
		if (py_info_opt) {
			result.push_back(*py_info_opt);
		}
	}
	return result;
}

} // namespace lego_assemble::py
