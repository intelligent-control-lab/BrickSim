#include "BrickAllocator.h"
#include "BrickSpawner.h"
#include "LegoGraph.h"
#include "LegoJointManager.h"
#include "LegoTopologySerialization.h" // IWYU pragma: keep

#include <omni/usd/UsdContextIncludes.h>

#include <carb/BindingsUtils.h>
#include <omni/usd/UsdContext.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// Remember to update _native.pyi when changing the API below.
PYBIND11_MODULE(_native, m) {
	m.doc() = "lego_assemble: native module";

	m.def(
	    "init_natives",
	    []() {
		    bool success = true;
		    success &= lego_assemble::initLegoJointManager();
		    return success;
	    },
	    "Initialize native C++ components");

	m.def(
	    "deinit_natives",
	    []() {
		    bool success = true;
		    success &= lego_assemble::deinitLegoJointManager();
		    return success;
	    },
	    "Destroy native C++ components");

	m.def(
	    "create_brick_prim",
	    [](std::string path, std::array<lego_assemble::BrickUnit, 3> dimensions,
	       lego_assemble::BrickColor color) {
		    auto stage = omni::usd::UsdContext::getContext()->getStage();
		    lego_assemble::createBrickPrim(stage, pxr::SdfPath(path),
		                                   dimensions, color);
	    },
	    pybind11::arg("path"), pybind11::arg("dimensions"),
	    pybind11::arg("color"),
	    "Create a brick prim at the specified path with given dimensions and "
	    "color");

	m.def(
	    "allocate_brick",
	    [](std::array<lego_assemble::BrickUnit, 3> dimensions,
	       lego_assemble::BrickColor color, lego_assemble::EnvId env_id) {
		    auto stage = omni::usd::UsdContext::getContext()->getStage();
		    auto [brick_id, path] =
		        lego_assemble::allocateBrick(stage, dimensions, color, env_id);
		    return std::make_tuple(brick_id, path.GetString());
	    },
	    pybind11::arg("dimensions"), pybind11::arg("color"),
	    pybind11::arg("env_id"),
	    "Allocate a new brick in the specified environment with given "
	    "dimensions and color. Returns the allocated brick ID and its path.");
	m.def(
	    "deallocate_brick",
	    [](lego_assemble::EnvId env_id, lego_assemble::BrickId brick_id) {
		    auto stage = omni::usd::UsdContext::getContext()->getStage();
		    return lego_assemble::deallocateBrick(stage, env_id, brick_id);
	    },
	    pybind11::arg("env_id"), pybind11::arg("brick_id"),
	    "Deallocate the specified brick from the given environment. Returns "
	    "true if successful.");
	m.def(
	    "deallocate_all_bricks_in_env",
	    [](lego_assemble::EnvId env_id) {
		    auto stage = omni::usd::UsdContext::getContext()->getStage();
		    lego_assemble::deallocateAllBricksInEnv(stage, env_id);
	    },
	    pybind11::arg("env_id"),
	    "Deallocate all bricks in the specified environment.");

	m.def(
	    "export_lego_topology",
	    [](std::string root_path) {
		    auto stage = omni::usd::UsdContext::getContext()->getStage();
		    auto topology = lego_assemble::exportLegoTopology(
		        stage, pxr::SdfPath(root_path));
		    nlohmann::ordered_json j = topology;
		    return j.dump();
	    },
	    pybind11::arg("root_path"),
	    "Export the lego topology under the specified root path");

	using Thresholds = lego_assemble::LegoGraph::Thresholds;
	pybind11::class_<Thresholds>(m, "LegoThresholds",
	                             "Assembly detection thresholds.")
	    .def(pybind11::init<>())
	    .def_readwrite("distance_tolerance", &Thresholds::DistanceTolerance)
	    .def_readwrite("max_penetration", &Thresholds::MaxPenetration)
	    .def_readwrite("z_angle_tolerance", &Thresholds::ZAngleTolerance)
	    .def_readwrite("required_force", &Thresholds::RequiredForce)
	    .def_readwrite("yaw_tolerance", &Thresholds::YawTolerance)
	    .def_readwrite("position_tolerance", &Thresholds::PositionTolerance)
	    .def("__repr__", [](const Thresholds &t) {
		    std::ostringstream os;
		    os << "LegoThresholds(distance_tolerance=" << t.DistanceTolerance
		       << ", max_penetration=" << t.MaxPenetration
		       << ", z_angle_tolerance=" << t.ZAngleTolerance
		       << ", required_force=" << t.RequiredForce
		       << ", yaw_tolerance=" << t.YawTolerance
		       << ", position_tolerance=" << t.PositionTolerance << ")";
		    return os.str();
	    });
	m.def("set_lego_thresholds", &lego_assemble::setLegoThresholds,
	      pybind11::arg("thresholds"),
	      "Set the assembly detection thresholds.");
	m.def(
	    "get_lego_thresholds",
	    []() {
		    Thresholds out;
		    lego_assemble::getLegoThresholds(out);
		    return out;
	    },
	    "Get the assembly detection thresholds.");
}
// Declare as a Carbonite bindings module for Python so logging and builtins
// are registered even when imported outside Kit, and to define CARB globals.
CARB_BINDINGS("lego_assemble", "python")
