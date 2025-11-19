export module lego_assemble.py;

import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.core.assembly;
import lego_assemble.omni.lego_runtime;
import lego_assemble.utils.conversions;
import lego_assemble.utils.transforms;
import lego_assemble.utils.c4_rotation;
import lego_assemble.io.json;
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

export std::string allocate_brick_part(std::array<BrickUnit, 3> dimensions,
                                       BrickColor color, std::int64_t env_id,
                                       std::array<double, 4> rot,
                                       std::array<double, 3> pos) {
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
	usd_graph.set_component_transform(
	    pid, {as<Eigen::Quaterniond>(rot), as<Eigen::Vector3d>(pos)});
	return path.GetAsString();
}

export bool deallocate_part(const std::string &part_path) {
	auto &usd_graph = lego_world().usd_graph();
	pxr::SdfPath path{part_path};
	return usd_graph.remove_part(path);
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
	ConnectionSegment seg{.offset = as<Eigen::Vector2i>(offset),
	                      .yaw = to_c4(yaw)};
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
                        std::array<double, 4> ref_rot,
                        std::array<double, 3> ref_pos) {
	nlohmann::ordered_json json = nlohmann::ordered_json::parse(json_str);
	auto &usd_graph = lego_world().usd_graph();
	LegoRuntime::Serializer{}.import(json, usd_graph, env_id,
	                                 Transformd{as<Eigen::Quaterniond>(ref_rot),
	                                            as<Eigen::Vector3d>(ref_pos)});
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

} // namespace lego_assemble::py
