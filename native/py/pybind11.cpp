import lego_assemble.py;

#include <unistd.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace lego_assemble::py;

// Remember to update _native.pyi when changing the API below.
PYBIND11_MODULE(_native, m) {
	m.doc() = "lego_assemble: native module";

	m.def(
	    "allocate_brick_part", &allocate_brick_part,
	    pybind11::arg("dimensions"), pybind11::arg("color"),
	    pybind11::arg("env_id"), pybind11::arg("rot"), pybind11::arg("pos"),
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
	      "string (schema 'lego_assemble/lego_topology@1').");

	m.def("import_lego", &import_lego, pybind11::arg("json_str"),
	      pybind11::arg("env_id"), pybind11::arg("ref_rot"),
	      pybind11::arg("ref_pos"),
	      "Import the lego topology from the given JSON string into the "
	      "specified environment, applying the given reference-to-environment "
	      "transform (pose in meters, wxyz quaternion).");

	m.def("compute_connected_component", &compute_connected_component,
	      pybind11::arg("part_path"),
	      "Return (part_paths, connection_paths) for the connected component "
	      "of the specified part path. Returns two empty lists if the part is "
	      "unknown.");

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
}
