import lego_assemble.api;

#include <unistd.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pybind11_json.hpp"

using namespace lego_assemble::api;

// Remember to update _native.pyi when changing the API below.
PYBIND11_MODULE(_native, m) {
	m.doc() = "lego_assemble: native module";

	m.def(
	    "allocate_brick_part", &allocate_brick_part,
	    pybind11::arg("dimensions"), pybind11::arg("color"),
	    pybind11::arg("env_id"), pybind11::arg("rot") = std::nullopt,
	    pybind11::arg("pos") = std::nullopt,
	    "Allocate a new brick part in the specified environment with given "
	    "dimensions, color and pose (in meters, wxyz quaternion). Returns the "
	    "allocated brick path.");

	m.def("deallocate_part", &deallocate_part, pybind11::arg("part_path"),
	      "Deallocate the specified managed part. Returns true if successful.");

	m.def("compute_graph_transform", &compute_graph_transform,
	      pybind11::arg("a_path"), pybind11::arg("b_path"),
	      "Compute the gain-graph relative transform between two parts given "
	      "their prim paths. Returns (rot, pos) in meters, where rot is a wxyz "
	      "quaternion for {}^{a}T_b (a<-b). Throws if the parts are "
	      "disconnected or unknown.");

	m.def("compute_connection_transform", &compute_connection_transform,
	      pybind11::arg("stud_path"), pybind11::arg("stud_if"),
	      pybind11::arg("hole_path"), pybind11::arg("hole_if"),
	      pybind11::arg("offset"), pybind11::arg("yaw"),
	      "Compute the relative transform induced by a single connection "
	      "segment between the specified stud and hole interfaces with the "
	      "given grid offset and yaw index (0..3). Returns (rot, pos) for "
	      "{}^{stud}T_hole in meters, without modifying USD or the topology. "
	      "Throws if parts or interfaces are unknown.");

	m.def("create_connection", &create_connection, pybind11::arg("stud_path"),
	      pybind11::arg("stud_if"), pybind11::arg("hole_path"),
	      pybind11::arg("hole_if"), pybind11::arg("offset"),
	      pybind11::arg("yaw"),
	      "Create a managed connection between the specified stud and hole "
	      "interfaces with given grid offset and yaw index (0..3). Returns the "
	      "created connection path.");

	m.def("deallocate_connection", &deallocate_connection,
	      pybind11::arg("connection_path"),
	      "Deallocate the specified managed connection. Returns true if "
	      "successful.");

	m.def("deallocate_all_managed", &deallocate_all_managed,
	      pybind11::arg("env_id"),
	      "Deallocate all managed bricks and their managed connections in the "
	      "specified environment. Returns true if successful.");

	m.def("export_lego", &export_lego, pybind11::arg("env_id"),
	      "Export the lego topology of the specified environment as a JSON "
	      "object (schema 'lego_assemble/lego_topology@2').");

	m.def("import_lego", &import_lego, pybind11::arg("json"),
	      pybind11::arg("env_id"), pybind11::arg("ref_rot") = std::nullopt,
	      pybind11::arg("ref_pos") = std::nullopt,
	      "Import the lego topology from the given JSON object into the "
	      "specified environment, applying the given reference-to-environment "
	      "transform (pose in meters, wxyz quaternion). Returns "
	      "(part_paths, connection_paths) as dicts mapping JSON ids to USD "
	      "prim paths for the imported objects.");

	m.def("compute_structure_transforms", &compute_structure_transforms,
	      pybind11::arg("json"), pybind11::arg("root"),
	      "Compute transforms {}^{root}T_part for all parts in the structure "
	      "described by the given topology JSON. root is a JSON part id. "
	      "For the root connected component, transforms are derived purely "
	      "from graph connectivity. For other disconnected components, pose "
	      "hints are used to align them to the reference frame. Returns a "
	      "dict mapping JSON part ids to (rot, pos) tuples in meters with wxyz "
	      "quaternions.");

	m.def("compute_connected_component", &compute_connected_component,
	      pybind11::arg("part_path"),
	      "Return (part_paths, connection_paths) for the connected component "
	      "of the specified part path. Returns two empty lists if the part is "
	      "unknown.");

	m.def("are_parts_connected", &are_parts_connected,
	      pybind11::arg("part_a_path"), pybind11::arg("part_b_path"),
	      "Return true if the specified parts are connected in the current "
	      "USD topology. Throws if either part is unknown.");

	m.def("does_connection_exist", &does_connection_exist,
	      pybind11::arg("stud_path"), pybind11::arg("stud_if"),
	      pybind11::arg("hole_path"), pybind11::arg("hole_if"),
	      "Return true if a connection segment exists between the specified "
	      "stud and hole interfaces. Throws if either part is unknown.");

	m.def("compute_obstacle_regions", &compute_obstacle_regions,
	      pybind11::arg("obstacle_paths"), pybind11::arg("table_xy"),
	      pybind11::arg("table_z"), pybind11::arg("clearance_height"),
	      "Compute 2D obstacle bounding boxes on the table for the given "
	      "obstacle prim paths. obstacle_paths must all belong to the same "
	      "environment. table_xy is the table bounding box (x_min, y_min, "
	      "x_max, y_max) in meters and table_z is the table height in "
	      "meters. clearance_height specifies the vertical band around the "
	      "table in meters within which obstacles are considered. Returns a "
	      "list of rectangles (x_min, y_min, x_max, y_max) in meters in the "
	      "environment frame.");

	m.def("arrange_parts_on_table", &arrange_parts_on_table,
	      pybind11::arg("parts_to_arrange"),
	      pybind11::arg("parts_to_avoid") = std::nullopt,
	      pybind11::arg("obstacles") = std::nullopt, pybind11::arg("table_xy"),
	      pybind11::arg("table_z"),
	      pybind11::arg("clearance_xy") = std::nullopt,
	      pybind11::arg("grid_resolution") = std::nullopt,
	      pybind11::arg("allow_rotation") = std::nullopt,
	      pybind11::arg("avoid_all_other_parts") = std::nullopt,
	      pybind11::arg("structure_ids") = std::nullopt,
	      "Arrange the specified parts on a rectangular table region in the "
	      "environment frame. Each entry in parts_to_arrange is treated as a "
	      "seed. If structure_ids is not provided, each connected component "
	      "formed by the seeds is treated as an independent structure and "
	      "packed as a single rectangle. If structure_ids is provided, seeds "
	      "sharing the same structure id are grouped into a structure; all "
	      "connected components belonging to a structure are moved as a rigid "
	      "group, preserving relative poses between components in the same "
	      "structure. Parts to arrange and avoid are given as USD prim paths. "
	      "Obstacles, if provided, are a list of rectangles given as "
	      "(x_min, y_min, x_max, y_max) in meters. table_xy is the table "
	      "bounding box (x_min, y_min, x_max, y_max) in meters and table_z is "
	      "the table height in meters. Clearance, grid_resolution and "
	      "allow_rotation are optional tuning parameters. Returns "
	      "(placed_paths, not_placed_paths), where the paths are the subset of "
	      "seed paths that were placed or not placed.");

	m.def(
	    "arrange_parts_in_workspace", &arrange_parts_in_workspace,
	    pybind11::arg("workspace_path"), pybind11::arg("parts_to_arrange"),
	    pybind11::arg("structure_ids") = std::nullopt,
	    "Arrange the specified parts inside a workspace region defined by a "
	    "workspace prim. workspace_path is the USD prim path of the "
	    "workspace; parts_to_arrange are USD prim paths for the parts to "
	    "move. If structure_ids is provided, seeds sharing the same structure "
	    "id are grouped and moved as rigid structures, preserving their "
	    "internal relative poses. Returns (placed_paths, not_placed_paths) "
	    "for the subset of seed paths that were placed or not placed.");

	pybind11::class_<AssemblyThresholds>(m, "AssemblyThresholds",
	                                     "Assembly detection thresholds.")
	    .def(pybind11::init<>())
	    .def_readwrite("distance_tolerance",
	                   &AssemblyThresholds::DistanceTolerance)
	    .def_readwrite("max_penetration", &AssemblyThresholds::MaxPenetration)
	    .def_readwrite("z_angle_tolerance",
	                   &AssemblyThresholds::ZAngleTolerance)
	    .def_readwrite("required_force", &AssemblyThresholds::RequiredForce)
	    .def_readwrite("yaw_tolerance", &AssemblyThresholds::YawTolerance)
	    .def_readwrite("position_tolerance",
	                   &AssemblyThresholds::PositionTolerance)
	    .def("__repr__", &repr_assembly_thresholds);

	m.def("set_assembly_thresholds", &set_assembly_thresholds,
	      pybind11::arg("thresholds"),
	      "Set the assembly detection thresholds.");

	m.def("get_assembly_thresholds", &get_assembly_thresholds,
	      "Get the current assembly detection thresholds.");

	pybind11::class_<AssemblyDebugInfo>(m, "AssemblyDebugInfo",
	                                    "Debug information for assembly "
	                                    "detection of a connection segment.")
	    .def_readonly("accepted", &AssemblyDebugInfo::accepted)
	    .def_readonly("relative_distance",
	                  &AssemblyDebugInfo::relative_distance)
	    .def_readonly("tilt", &AssemblyDebugInfo::tilt)
	    .def_readonly("projected_force", &AssemblyDebugInfo::projected_force)
	    .def_readonly("yaw_error", &AssemblyDebugInfo::yaw_error)
	    .def_readonly("position_error", &AssemblyDebugInfo::position_error)
	    .def_readonly("grid_pos", &AssemblyDebugInfo::grid_pos)
	    .def_readonly("grid_pos_snapped", &AssemblyDebugInfo::grid_pos_snapped)
	    .def_readonly("stud_path", &AssemblyDebugInfo::stud_path)
	    .def_readonly("stud_interface", &AssemblyDebugInfo::stud_interface)
	    .def_readonly("hole_path", &AssemblyDebugInfo::hole_path)
	    .def_readonly("hole_interface", &AssemblyDebugInfo::hole_interface)
	    .def("__repr__", &AssemblyDebugInfo::repr);

	m.def("get_assembly_debug_infos", &get_assembly_debug_infos,
	      "Get the assembly detection debug information for all detected "
	      "connection segments since the last simulation step.");
}
