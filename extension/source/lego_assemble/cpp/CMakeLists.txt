cmake_minimum_required(VERSION 4.1)

set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "d0edc3af-4c50-42ea-a356-e2862fe7a444")

project(lego_assemble_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_MODULE_STD ON)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

include(FetchContent)

set(PYBIND11_VERSION 2.12.0)
FetchContent_Declare(
  pybind11
  URL https://github.com/pybind/pybind11/archive/refs/tags/v${PYBIND11_VERSION}.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(pybind11)

set(EIGEN_VERSION 5.0.0)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_Declare(eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        ${EIGEN_VERSION}
  GIT_SHALLOW    TRUE
  GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(eigen) # defines Eigen3::Eigen

# IsaacSim target-deps configuration
if(NOT DEFINED ISAACSIM_BUILD_DIR)
  if(DEFINED ENV{ISAACSIM_BUILD_DIR})
    set(ISAACSIM_BUILD_DIR "$ENV{ISAACSIM_BUILD_DIR}")
  endif()
endif()
if(NOT DEFINED ISAACSIM_BUILD_DIR)
  message(FATAL_ERROR "ISAACSIM_BUILD_DIR is not set. Point it to your IsaacSim _build directory.")
endif()
if(NOT IS_DIRECTORY "${ISAACSIM_BUILD_DIR}")
  message(FATAL_ERROR "ISAACSIM_BUILD_DIR does not exist: ${ISAACSIM_BUILD_DIR}")
endif()
message(STATUS "IsaacSim build dir: ${ISAACSIM_BUILD_DIR}")

# Patching SDK to workaround bugs in header files
find_program(PATCH_EXECUTABLE patch REQUIRED)
message(STATUS "Found patch command: ${PATCH_EXECUTABLE}")
add_custom_target(patch_sdk_usd ALL
    COMMAND ${PATCH_EXECUTABLE}
            --input="${CMAKE_CURRENT_SOURCE_DIR}/sdk_patches/usd_include.patch"
            --forward
            -p1
            --directory="${ISAACSIM_BUILD_DIR}/target-deps/usd/release/include"
            --backup || (exit 0)
    COMMENT "Applying USD include patch"
)
add_custom_target(patch_sdk_kit ALL
    COMMAND ${PATCH_EXECUTABLE}
            --input="${CMAKE_CURRENT_SOURCE_DIR}/sdk_patches/kit_include.patch"
            --forward
            -p1
            --directory="${ISAACSIM_BUILD_DIR}/linux-x86_64/release/kit/dev/include"
            --backup || (exit 0)
    COMMENT "Applying kit include patch"
)

file(GLOB_RECURSE CXX_MODULE_IFACES "${CMAKE_CURRENT_SOURCE_DIR}/modules/*.cppm")
add_library(lego_assemble_modules STATIC)
set_target_properties(lego_assemble_modules PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_sources(lego_assemble_modules PUBLIC
  FILE_SET cxx_modules TYPE CXX_MODULES
  BASE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/modules"
  FILES ${CXX_MODULE_IFACES}
)
target_include_directories(lego_assemble_modules SYSTEM PUBLIC
  "${ISAACSIM_BUILD_DIR}/linux-x86_64/release/kit/dev/include"
  "${ISAACSIM_BUILD_DIR}/target-deps/carb_sdk_plugins/include"
  "${ISAACSIM_BUILD_DIR}/target-deps/cuda/include"
  "${ISAACSIM_BUILD_DIR}/target-deps/omni_client_library/include"
  "${ISAACSIM_BUILD_DIR}/target-deps/omni_physics/release/include"
  "${ISAACSIM_BUILD_DIR}/target-deps/physx/include"
  "${ISAACSIM_BUILD_DIR}/target-deps/python/include/python3.11"
  "${ISAACSIM_BUILD_DIR}/target-deps/usd_ext_physics/release/include"
  "${ISAACSIM_BUILD_DIR}/target-deps/usd/release/include"
  "${ISAACSIM_BUILD_DIR}/target-deps/usd/release/include/boost"
)
target_link_directories(lego_assemble_modules PUBLIC
  "${ISAACSIM_BUILD_DIR}/linux-x86_64/release/extsbuild/omni.usd.core/bin"
  "${ISAACSIM_BUILD_DIR}/linux-x86_64/release/extscache/omni.usd.schema.audio-0.0.0+b0a86421.lx64.r.cp311/lib"
  "${ISAACSIM_BUILD_DIR}/target-deps/carb_sdk_plugins/_build/linux-x86_64/release"
  "${ISAACSIM_BUILD_DIR}/target-deps/cuda/lib64"
  "${ISAACSIM_BUILD_DIR}/target-deps/nvtx/lib64"
  "${ISAACSIM_BUILD_DIR}/target-deps/physx/bin/linux.x86_64/checked"
  "${ISAACSIM_BUILD_DIR}/target-deps/python/lib"
  "${ISAACSIM_BUILD_DIR}/target-deps/usd_ext_physics/release/lib"
  "${ISAACSIM_BUILD_DIR}/target-deps/usd/release/lib"
)
target_link_libraries(lego_assemble_modules PUBLIC
  Eigen3::Eigen
  python3.11
  boost_python311
  PhysX_static_64
  PhysXCommon_static_64
  PhysXFoundation_static_64
  PhysXCooking_static_64
  PhysXPvdSDK_static_64
  PhysXExtensions_static_64
  carb # Order matters: carb before usd
  usd_arch
  usd_gf
  usd_hd
  usd_kind
  usd_sdf
  usd_tf
  usd_usd
  usd_usdGeom
  usd_usdImaging
  usd_usdPhysics
  usd_usdUtils
  usd_vt
  omni.usd
  physxSchema
  omniAudioSchema
)

add_library(_native MODULE)
target_sources(_native PRIVATE
  FILE_SET cxx_modules TYPE CXX_MODULES
  BASE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/py"
  FILES "${CMAKE_CURRENT_SOURCE_DIR}/py/lego_assemble/native.cppm"
)
target_link_libraries(_native PRIVATE lego_assemble_modules pybind11::module)
pybind11_extension(_native)

# Allow the caller (setup.py) to override the output directory for the built module
set(EXT_OUTPUT_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "Directory to place the built python extension")
set_target_properties(_native PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${EXT_OUTPUT_DIR}")

# Tests
enable_testing()

add_executable(TestConversions tests/TestConversions.cpp)
target_link_libraries(TestConversions PRIVATE lego_assemble_modules)
add_test(NAME TestConversions COMMAND TestConversions)
