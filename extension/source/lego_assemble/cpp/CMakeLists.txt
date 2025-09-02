cmake_minimum_required(VERSION 3.18)

project(lego_assemble_native LANGUAGES CXX)

# Always export compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Python and pybind11
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

include(FetchContent)
set(PYBIND11_VERSION 2.12.0)
FetchContent_Declare(
  pybind11
  URL https://github.com/pybind/pybind11/archive/refs/tags/v${PYBIND11_VERSION}.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(pybind11)

# Build the Python extension module
pybind11_add_module(_native src/native.cpp)

target_compile_features(_native PRIVATE cxx_std_17)

# IsaacSim target-deps configuration (variables only)
include(cmake/IsaacSimTargetDeps.cmake)

# Only include and library directories here (no search logic, no linking)
target_include_directories(_native PRIVATE
  "${TARGET_DEPS_DIR}/omni_client_library/include"
  "${TARGET_DEPS_DIR}/usd/${ISAAC_CFG_DIR}/include"
  "${TARGET_DEPS_DIR}/usd/${ISAAC_CFG_DIR}/include/boost"
  "${TARGET_DEPS_DIR}/usd_ext_physics/${ISAAC_CFG_DIR}/include"
  "${TARGET_DEPS_DIR}/omni_physics/${ISAAC_CFG_DIR}/include"
  "${TARGET_DEPS_DIR}/python/include"
  "${TARGET_DEPS_DIR}/physx/include"
  "${TARGET_DEPS_DIR}/cuda/include"
)

target_link_directories(_native PRIVATE
  "${TARGET_DEPS_DIR}/python/lib"
  "${TARGET_DEPS_DIR}/usd/${ISAAC_CFG_DIR}/lib"
  "${TARGET_DEPS_DIR}/usd_ext_physics/${ISAAC_CFG_DIR}/lib"
  "${TARGET_DEPS_DIR}/physx/bin/${ISAAC_PLATFORM_DIR}/${ISAAC_CFG_DIR}"
  "${TARGET_DEPS_DIR}/cuda/lib64"
  "${TARGET_DEPS_DIR}/nvtx/lib64"
)

# PhysX: link static libs explicitly; fail if missing
target_compile_definitions(_native PRIVATE PX_PHYSX_STATIC_LIB)
target_link_libraries(_native PRIVATE
  PhysXExtensions_static_64
  PhysX_static_64
  PhysXPvdSDK_static_64
  PhysXCooking_static_64
  PhysXCommon_static_64
  PhysXFoundation_static_64
)

# Allow the caller (setup.py) to override the output directory for the built module
set(EXT_OUTPUT_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "Directory to place the built python extension")
set_target_properties(_native PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${EXT_OUTPUT_DIR}")
