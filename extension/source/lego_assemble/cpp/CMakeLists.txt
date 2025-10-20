cmake_minimum_required(VERSION 3.5)

project(lego_assemble_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
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

file(GLOB_RECURSE CXX_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
pybind11_add_module(_native ${CXX_SRC})

# IsaacSim target-deps configuration
if(NOT DEFINED TARGET_DEPS_DIR)
  if(DEFINED ENV{ISAACSIM_TARGET_DEPS})
    set(TARGET_DEPS_DIR "$ENV{ISAACSIM_TARGET_DEPS}")
  endif()
endif()
if(NOT DEFINED TARGET_DEPS_DIR)
  message(FATAL_ERROR "TARGET_DEPS_DIR/ISAACSIM_TARGET_DEPS is not set. Point it to your IsaacSim _build/target-deps directory.")
endif()
if(NOT IS_DIRECTORY "${TARGET_DEPS_DIR}")
  message(FATAL_ERROR "TARGET_DEPS_DIR does not exist: ${TARGET_DEPS_DIR}")
endif()
message(STATUS "IsaacSim target-deps: ${TARGET_DEPS_DIR}")

# Patching SDK to workaround bugs in header files
find_program(PATCH_EXECUTABLE patch REQUIRED)
message(STATUS "Found patch command: ${PATCH_EXECUTABLE}")
add_custom_target(patch_sdk_usd ALL
    COMMAND ${PATCH_EXECUTABLE}
            --input="${CMAKE_CURRENT_SOURCE_DIR}/sdk_patches/usd_include.patch"
            --forward
            -p1
            --directory="${TARGET_DEPS_DIR}/usd/release/include"
            --backup || (exit 0)
    COMMENT "Applying USD include patch"
)
add_custom_target(patch_sdk_kit ALL
    COMMAND ${PATCH_EXECUTABLE}
            --input="${CMAKE_CURRENT_SOURCE_DIR}/sdk_patches/kit_include.patch"
            --forward
            -p1
            --directory="${TARGET_DEPS_DIR}/../linux-x86_64/release/kit/dev/include"
            --backup || (exit 0)
    COMMENT "Applying kit include patch"
)

target_include_directories(_native PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(_native SYSTEM PRIVATE
  "${TARGET_DEPS_DIR}/../linux-x86_64/release/kit/dev/include"
  "${TARGET_DEPS_DIR}/omni_client_library/include"
  "${TARGET_DEPS_DIR}/usd/release/include"
  "${TARGET_DEPS_DIR}/usd/release/include/boost"
  "${TARGET_DEPS_DIR}/usd_ext_physics/release/include"
  "${TARGET_DEPS_DIR}/omni_physics/release/include"
  "${TARGET_DEPS_DIR}/carb_sdk_plugins/include"
  "${TARGET_DEPS_DIR}/python/include"
  "${TARGET_DEPS_DIR}/physx/include"
  "${TARGET_DEPS_DIR}/cuda/include"
)

target_link_directories(_native PRIVATE
  "${TARGET_DEPS_DIR}/python/lib"
  "${TARGET_DEPS_DIR}/usd/release/lib"
  "${TARGET_DEPS_DIR}/usd_ext_physics/release/lib"
  "${TARGET_DEPS_DIR}/physx/bin/linux.x86_64/checked"
  "${TARGET_DEPS_DIR}/carb_sdk_plugins/_build/linux-x86_64/release"
  "${TARGET_DEPS_DIR}/cuda/lib64"
  "${TARGET_DEPS_DIR}/nvtx/lib64"
  "${TARGET_DEPS_DIR}/../linux-x86_64/release/extsbuild/omni.usd.core/bin"
)

target_compile_definitions(_native PRIVATE PX_PHYSX_STATIC_LIB)
target_link_libraries(_native PRIVATE
  Eigen3::Eigen
  PhysXExtensions_static_64
  PhysX_static_64
  PhysXPvdSDK_static_64
  PhysXCooking_static_64
  PhysXCommon_static_64
  PhysXFoundation_static_64
  carb
  omni.usd
  usd_usd
  usd_usdUtils
  usd_usdPhysics
  usd_sdf
  usd_tf
  usd_arch
  physxSchema
)

# Allow the caller (setup.py) to override the output directory for the built module
set(EXT_OUTPUT_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "Directory to place the built python extension")
set_target_properties(_native PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${EXT_OUTPUT_DIR}")
