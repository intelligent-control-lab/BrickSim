## FetchSourceDeps.cmake
##
## Prepare and add source dependencies used by native targets.

include(PrepareArtifacts)

function(bricksim_fetch_source_deps)
  set(SOURCE_DEPS_ROOT "${CMAKE_BINARY_DIR}/_deps/source")
  prepare_artifacts(
    CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/source_deps.toml"
    OUTPUT_ROOT "${SOURCE_DEPS_ROOT}"
    OUT_ROOT_VARIABLE SOURCE_DEPS_ROOT
  )

  set(pybind11_SOURCE_DIR "${SOURCE_DEPS_ROOT}/pybind11")
  set(eigen_SOURCE_DIR "${SOURCE_DEPS_ROOT}/eigen")
  set(osqp_SOURCE_DIR "${SOURCE_DEPS_ROOT}/osqp")
  set(qdldl_SOURCE_DIR "${SOURCE_DEPS_ROOT}/qdldl")
  set(nlohmann_json_SOURCE_DIR "${SOURCE_DEPS_ROOT}/nlohmann_json")

  set(PYBIND11_FINDPYTHON ON CACHE BOOL "Use CMake FindPython in pybind11")
  add_subdirectory("${pybind11_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/_deps/pybind11-build" SYSTEM)

  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
  add_subdirectory("${eigen_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/_deps/eigen-build" SYSTEM)

  add_subdirectory("${nlohmann_json_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/_deps/nlohmann_json-build" SYSTEM)

  set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
  set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
  set(OSQP_BUILD_DEMO_EXE   OFF CACHE BOOL "" FORCE)
  set(OSQP_ENABLE_PRINTING  ON CACHE BOOL "" FORCE)
  set(OSQP_ENABLE_PROFILING ON CACHE BOOL "" FORCE)
  set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "" FORCE)
  set(OSQP_USE_FLOAT        OFF CACHE BOOL "" FORCE)
  set(OSQP_USE_LONG         OFF CACHE BOOL "" FORCE)
  set(OSQP_DEBUG            OFF CACHE BOOL "" FORCE)
  set(OSQP_CODEGEN          OFF CACHE BOOL "" FORCE)
  set(OSQP_ALGEBRA_BACKEND  "builtin" CACHE STRING "" FORCE)
  set(FETCHCONTENT_SOURCE_DIR_QDLDL "${qdldl_SOURCE_DIR}" CACHE PATH "" FORCE)
  add_subdirectory("${osqp_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/_deps/osqp-build" SYSTEM)
  target_compile_options(OSQPLIB PRIVATE -Wno-everything)
  target_compile_options(osqpstatic PRIVATE -Wno-everything)
endfunction()
