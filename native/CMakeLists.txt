cmake_minimum_required(VERSION 4.3.1)

set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "451f2fe2-a8a2-47c3-bc32-94786d8fc91b")

project(bricksim_native LANGUAGES CXX)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_MODULE_STD ON)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG" CACHE STRING "" FORCE)
string(APPEND CMAKE_CXX_FLAGS " -I${CMAKE_CURRENT_SOURCE_DIR}/shims")
add_compile_options("-march=x86-64-v3" "-ftemplate-backtrace-limit=0" "-Wall" "-Wextra" "-Wpedantic" "-Werror" "-Wno-reserved-module-identifier")

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

include(PrepareArtifacts)
include(FetchIsaacDeps)
include(FetchSourceDeps)
isaacsim_fetch_all_deps()

# Link against Isaac Sim's bundled Python
set(Python3_EXECUTABLE "${ISAACSIM_PYTHON}/bin/python3.11" CACHE FILEPATH "Python interpreter for pybind11" FORCE)
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
bricksim_fetch_source_deps()

function(bricksim_use_shims TARGET_NAME)
  target_include_directories(${TARGET_NAME} BEFORE PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/shims")
endfunction()

file(GLOB_RECURSE CXX_MODULE_IFACES "${CMAKE_CURRENT_SOURCE_DIR}/modules/*.cppm")
add_library(bricksim_modules STATIC)
set_target_properties(bricksim_modules PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_sources(bricksim_modules PUBLIC
  FILE_SET cxx_modules TYPE CXX_MODULES
  BASE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/modules"
  FILES ${CXX_MODULE_IFACES}
)
target_link_libraries(bricksim_modules PUBLIC
  osqpstatic
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  isaacsim_sdk
)
bricksim_use_shims(bricksim_modules)

add_library(core MODULE)
target_sources(core PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/py/pybind11.cpp")
bricksim_use_shims(core)
target_link_libraries(core PRIVATE bricksim_modules pybind11::module)
pybind11_extension(core)
set_target_properties(core PROPERTIES
  SKIP_BUILD_RPATH ON
  BUILD_WITH_INSTALL_RPATH OFF
  INSTALL_RPATH ""
)

function(bricksim_add_tool TOOL_NAME)
  add_executable(${TOOL_NAME} tools/${TOOL_NAME}.cpp)
  bricksim_use_shims(${TOOL_NAME})
  target_link_libraries(${TOOL_NAME} PRIVATE bricksim_modules)
endfunction()

bricksim_add_tool(solve_breakage)
bricksim_add_tool(static_solve)

include(AbiCheck)
bricksim_add_abi_check(core)

enable_testing()
add_custom_target(bricksim_tests)

function(bricksim_add_test TEST_NAME)
  add_executable(${TEST_NAME} EXCLUDE_FROM_ALL ${ARGN})
  bricksim_use_shims(${TEST_NAME})
  target_link_libraries(${TEST_NAME} PRIVATE bricksim_modules)
  target_compile_options(${TEST_NAME} PRIVATE ${SAN_CFLAGS})
  target_link_options(${TEST_NAME} PRIVATE ${SAN_LDFLAGS})
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
  set_tests_properties(${TEST_NAME} PROPERTIES ENVIRONMENT "${SAN_ENV}")
  add_dependencies(bricksim_tests ${TEST_NAME})
  bricksim_add_abi_check(${TEST_NAME})
endfunction()

bricksim_add_test(TestConversions tests/TestConversions.cpp)
bricksim_add_test(TestPolyStore tests/TestPolyStore.cpp)
bricksim_add_test(TestBrickSpecs tests/TestBrickSpecs.cpp)
bricksim_add_test(TestMultiKeySet tests/TestMultiKeySet.cpp)
bricksim_add_test(TestMultiKeyMap tests/TestMultiKeyMap.cpp)
bricksim_add_test(TestDynamicGraph tests/TestDynamicGraph.cpp)
bricksim_add_test(TestLegoGraph tests/TestLegoGraph.cpp)
bricksim_add_test(TestLegoGraphSelfConnect tests/TestLegoGraphSelfConnect.cpp)
bricksim_add_test(TestLegoGraphMultiSegEdge tests/TestLegoGraphMultiSegEdge.cpp)
bricksim_add_test(TestLegoGraphHooks tests/TestLegoGraphHooks.cpp)
bricksim_add_test(TestPhysicsGraph tests/TestPhysicsGraph.cpp)
bricksim_add_test(TestUsdLegoGraph tests/TestUsdLegoGraph.cpp)
bricksim_add_test(TestTopologyIO tests/TestTopologyIO.cpp)
bricksim_add_test(TestConstraintScheduler tests/TestConstraintScheduler.cpp)
bricksim_add_test(TestRandomRegularGraphSchedulingPolicy tests/TestRandomRegularGraphSchedulingPolicy.cpp)
bricksim_add_test(TestPack2dMaxrect tests/TestPack2dMaxrect.cpp)
bricksim_add_test(TestPhysicsGraphAssemblyClosure tests/TestPhysicsGraphAssemblyClosure.cpp)
