#include "StageUpdateListener.h"
#include <carb/BindingsUtils.h>
#include <pybind11/pybind11.h>

// Remember to update _native.pyi when changing the API below.
PYBIND11_MODULE(_native, m) {
	m.doc() = "lego_assemble: native module";

	m.def(
    "enqueue_joint_inv_mass_inertia",
    [](const std::string &sdfPath, float inv_mass0, float inv_inertia0,
       float inv_mass1, float inv_inertia1) {
            return lego_assemble::EnqueueSetJointInvMassInertia(
                pxr::SdfPath(sdfPath), inv_mass0, inv_inertia0, inv_mass1, inv_inertia1);
    },
	    pybind11::arg("sdf_path"), pybind11::arg("inv_mass0") = -1.0f,
	    pybind11::arg("inv_inertia0") = -1.0f,
	    pybind11::arg("inv_mass1") = -1.0f,
	    pybind11::arg("inv_inertia1") = -1.0f,
	    "Queue setting PxJoint inv mass/inertia scales for when PhysX joint exists.");

	// Stage update listener
	m.def(
	    "create_stage_update_listener",
	    []() { return lego_assemble::CreateStageUpdateListener(); },
	    "Create the stage update listener (returns True on success).");

	m.def(
	    "destroy_stage_update_listener",
	    []() { return lego_assemble::DestroyStageUpdateListener(); },
	    "Destroy the stage update listener if present (returns True if "
	    "destroyed).");
}
// Declare as a Carbonite bindings module for Python so logging and builtins
// are registered even when imported outside Kit, and to define CARB globals.
CARB_BINDINGS("lego_assemble._native", "python")
