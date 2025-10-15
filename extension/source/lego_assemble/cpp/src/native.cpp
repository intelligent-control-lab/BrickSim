#include "LegoBricks.h"
#include "LegoJointManager.h"
#include "LegoSerialization.h" // IWYU pragma: keep

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
	    "create_brick",
	    [](std::string path, std::array<lego_assemble::BrickUnit, 3> dimensions,
	       lego_assemble::BrickColor color) {
		    auto stage = omni::usd::UsdContext::getContext()->getStage();
		    lego_assemble::createBrick(stage, pxr::SdfPath(path), dimensions,
		                               color);
	    },
	    pybind11::arg("path"), pybind11::arg("dimensions"),
	    pybind11::arg("color"),
	    "Create a brick prim at the specified path with given dimensions and "
	    "color");

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
}
// Declare as a Carbonite bindings module for Python so logging and builtins
// are registered even when imported outside Kit, and to define CARB globals.
CARB_BINDINGS("lego_assemble._native", "python")
