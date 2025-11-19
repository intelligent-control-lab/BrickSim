cmake_minimum_required(VERSION 4.2)

set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "d0edc3af-4c50-42ea-a356-e2862fe7a444")

# Allow overriding from environment variables
if(DEFINED ENV{CMAKE_CXX_COMPILER_AR})
  set(CMAKE_CXX_COMPILER_AR "$ENV{CMAKE_CXX_COMPILER_AR}" CACHE FILEPATH "" FORCE)
endif()
if(DEFINED ENV{CMAKE_CXX_COMPILER_RANLIB})
  set(CMAKE_CXX_COMPILER_RANLIB "$ENV{CMAKE_CXX_COMPILER_RANLIB}" CACHE FILEPATH "" FORCE)
endif()
if(DEFINED ENV{CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS})
  set(CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS "$ENV{CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS}" CACHE FILEPATH "" FORCE)
endif()

project(lego_assemble_native LANGUAGES CXX)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_MODULE_STD ON)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
add_compile_options("-ftemplate-backtrace-limit=0" "-Wall" "-Wextra" "-Wpedantic" "-Werror")

set(SAN_CFLAGS
  "-fsanitize=address,undefined,leak"
  "-fno-omit-frame-pointer"
  "-fsanitize-address-use-after-scope"
  "-UNDEBUG"
)
set(SAN_LDFLAGS
  "-fsanitize=address,undefined,leak"
)
# LeakSanitizer suppressions for known third-party singletons (USD Tf registry)
set(LSAN_SUPPRESSIONS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/resources/lsan.supp")
set(SAN_ENV "LSAN_OPTIONS=suppressions=${LSAN_SUPPRESSIONS_FILE};ASAN_OPTIONS=fast_unwind_on_malloc=0")

include(FetchContent)
include(FetchIsaacDeps)
isaacsim_fetch_all_deps()

# Link against Isaac Sim's bundled Python
set(Python3_EXECUTABLE "${ISAACSIM_PYTHON}/bin/python3.11" CACHE FILEPATH "Python interpreter for pybind11" FORCE)
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

set(PYBIND11_VERSION 2.12.0)
set(PYBIND11_FINDPYTHON ON CACHE BOOL "Use CMake FindPython in pybind11")
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

FetchContent_Declare(nlohmann_json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(nlohmann_json)

include(SdkPatches)
set(SDK_PATCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdk_patches")
apply_sdk_patch_once(patch_usd "${ISAACSIM_USD_INCLUDE}" "${SDK_PATCH_DIR}/usd_include.patch" 1)
apply_sdk_patch_once(patch_kit "${ISAACSIM_KIT_INCLUDE}" "${SDK_PATCH_DIR}/kit_include.patch" 1)

file(GLOB_RECURSE CXX_MODULE_IFACES "${CMAKE_CURRENT_SOURCE_DIR}/modules/*.cppm")
add_library(lego_assemble_modules STATIC)
set_target_properties(lego_assemble_modules PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_sources(lego_assemble_modules PUBLIC
  FILE_SET cxx_modules TYPE CXX_MODULES
  BASE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/modules"
  FILES ${CXX_MODULE_IFACES}
)
add_dependencies(lego_assemble_modules patch_usd_patch patch_kit_patch)
target_include_directories(lego_assemble_modules SYSTEM PUBLIC
  "${ISAACSIM_KIT_INCLUDE}"
  "${ISAACSIM_CARB_INCLUDE}"
  "${ISAACSIM_OMNI_CLIENT_INCLUDE}"
  "${ISAACSIM_OMNI_PHYSICS_INCLUDE}"
  "${ISAACSIM_PHYSX_INCLUDE}"
  "${ISAACSIM_PYTHON_INCLUDE}"
  "${ISAACSIM_USD_EXT_PHYSICS_INCLUDE}"
  "${ISAACSIM_USD_INCLUDE}"
  "${ISAACSIM_USD_BOOST_INCLUDE}"
)
target_link_directories(lego_assemble_modules PUBLIC
  "${ISAACSIM_OMNI_USD_CORE_LIBDIR}"
  "${ISAACSIM_OMNI_USD_SCHEMA_AUDIO_LIBDIR}"
  "${ISAACSIM_CARB_LIBDIR}"
  "${ISAACSIM_PHYSX_LIBDIR}"
  "${ISAACSIM_PYTHON_LIBDIR}"
  "${ISAACSIM_USD_EXT_PHYSICS_LIBDIR}"
  "${ISAACSIM_USD_LIBDIR}"
)
target_link_libraries(lego_assemble_modules PUBLIC
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
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
target_include_directories(lego_assemble_modules BEFORE PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/shims")
# target_compile_options(lego_assemble_modules PRIVATE $<$<CONFIG:DEBUG>:${SAN_CFLAGS}>)
# target_link_options(lego_assemble_modules PRIVATE $<$<CONFIG:DEBUG>:${SAN_LDFLAGS}>)

add_library(_native MODULE)
target_sources(_native PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/py/pybind11.cpp")
target_include_directories(_native BEFORE PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/shims")
target_link_libraries(_native PRIVATE lego_assemble_modules pybind11::module)
pybind11_extension(_native)

# Tests
enable_testing()

include(AbiCheck)
lego_add_abi_check(_native)

function(add_lego_test TEST_NAME)
  add_executable(${TEST_NAME} ${ARGN})
  target_include_directories(${TEST_NAME} BEFORE PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/shims")
  target_link_libraries(${TEST_NAME} PRIVATE lego_assemble_modules)
  target_compile_options(${TEST_NAME} PRIVATE ${SAN_CFLAGS})
  target_link_options(${TEST_NAME} PRIVATE ${SAN_LDFLAGS})
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
  set_tests_properties(${TEST_NAME} PROPERTIES ENVIRONMENT
    "LSAN_OPTIONS=suppressions=${LSAN_SUPPRESSIONS_FILE};ASAN_OPTIONS=fast_unwind_on_malloc=0"
  )
  lego_add_abi_check(${TEST_NAME})
endfunction()

add_lego_test(TestConversions tests/TestConversions.cpp)
add_lego_test(TestPolyStore tests/TestPolyStore.cpp)
add_lego_test(TestPolyStore2 tests/TestPolyStore2.cpp)
add_lego_test(TestBrickSpecs tests/TestBrickSpecs.cpp)
add_lego_test(TestMultiKeySet tests/TestMultiKeySet.cpp)
add_lego_test(TestMultiKeyMap tests/TestMultiKeyMap.cpp)
add_lego_test(TestDynamicGraph tests/TestDynamicGraph.cpp)
add_lego_test(TestLegoGraph tests/TestLegoGraph.cpp)
add_lego_test(TestLegoGraphPMR tests/TestLegoGraphPMR.cpp)
add_lego_test(TestLegoGraphSelfConnect tests/TestLegoGraphSelfConnect.cpp)
add_lego_test(TestLegoGraphMultiSegEdge tests/TestLegoGraphMultiSegEdge.cpp)
add_lego_test(TestLegoGraphHooks tests/TestLegoGraphHooks.cpp)
add_lego_test(TestPhysicsGraph tests/TestPhysicsGraph.cpp)
add_lego_test(TestUsdLegoGraph tests/TestUsdLegoGraph.cpp)
add_lego_test(TestJsonIO tests/TestJsonIO.cpp)

# Ensure the CMake-provided std modules provider compiles <iostream>
# with the "no-symver" path by overriding bits/c++config.h just for
# the provider target (avoid bleeding into user targets).
foreach(_lvl IN ITEMS 26 23)
  if (TARGET "__CMAKE::CXX${_lvl}")
    get_target_property(_real "__CMAKE::CXX${_lvl}" ALIASED_TARGET)
    if (_real)
      target_include_directories(${_real} BEFORE PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/shims")
      # Ensure the provider itself is built as PIC.
      set_property(TARGET ${_real} PROPERTY POSITION_INDEPENDENT_CODE ON)
    endif()
  endif()
  if (TARGET "__cmake_cxx${_lvl}")
    target_include_directories("__cmake_cxx${_lvl}" BEFORE PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/shims")
    # Ensure libstdc++ support objects (std.cc, std.compat.cc, ...) are PIC.
    set_property(TARGET "__cmake_cxx${_lvl}" PROPERTY POSITION_INDEPENDENT_CODE ON)
  endif()
endforeach()
