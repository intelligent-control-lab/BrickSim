#include <PxPhysicsAPI.h>
#include <pybind11/pybind11.h>

static int get_physx_version() {
    return PX_PHYSICS_VERSION;
}

PYBIND11_MODULE(_native, m) {
    m.doc() = "lego_assemble: minimal C++ bindings (pybind11)";
    m.def("get_physx_version", &get_physx_version, "Return PX_PHYSICS_VERSION");
}
