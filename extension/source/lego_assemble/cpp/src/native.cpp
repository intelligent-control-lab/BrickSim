#include "LegoJointManager.h"

#include <carb/BindingsUtils.h>
#include <pybind11/pybind11.h>

// Remember to update _native.pyi when changing the API below.
PYBIND11_MODULE(_native, m) {
	m.doc() = "lego_assemble: native module";

	m.def(
	    "get_physx_joint_force_torque",
	    [](const std::string &sdfPath) -> pybind11::object {
		    auto res =
		        lego_assemble::getPhysxJointForceTorque(pxr::SdfPath(sdfPath));
		    if (!res)
			    return pybind11::none();
		    const auto &[force, torque] = *res;
		    return pybind11::make_tuple(
		        pybind11::make_tuple(force.x, force.y, force.z),
		        pybind11::make_tuple(torque.x, torque.y, torque.z));
	    },
	    pybind11::arg("sdf_path"), "Get force/torque applied on a PhysX joint");

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
}
// Declare as a Carbonite bindings module for Python so logging and builtins
// are registered even when imported outside Kit, and to define CARB globals.
CARB_BINDINGS("lego_assemble._native", "python")
