cmake_minimum_required(VERSION 3.24)

project(breakage_cuda LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# We pin a sane default arch list; user can override with CMAKE_CUDA_ARCHITECTURES.
# CMake 3.18+ auto-initializes this to "52" when the variable was not user-set,
# so a `NOT DEFINED` guard is not enough to keep our default. We treat the CMake
# default of 52 as "no real choice was made" and replace it with our list. Users
# who want a specific arch can still pass -DCMAKE_CUDA_ARCHITECTURES=... to opt
# out of this override.
if(NOT CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "52")
  set(CMAKE_CUDA_ARCHITECTURES "70;75;80;86;89;90")
endif()

include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

# ---------------------------------------------------------------------------
# Eigen (header-only)
# ---------------------------------------------------------------------------
FetchContent_Declare(eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

# ---------------------------------------------------------------------------
# nlohmann_json
# ---------------------------------------------------------------------------
FetchContent_Declare(json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW TRUE
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(json)

# ---------------------------------------------------------------------------
# OSQP (CPU baseline). Pinned to v1.0.0 to match the API used by breakage.cppm.
# ---------------------------------------------------------------------------
FetchContent_Declare(osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG v1.0.0
  GIT_SHALLOW TRUE
)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_USE_LONG OFF CACHE BOOL "" FORCE)
set(OSQP_USE_FLOAT OFF CACHE BOOL "" FORCE)
set(OSQP_ALGEBRA_BACKEND "builtin" CACHE STRING "" FORCE)
FetchContent_MakeAvailable(osqp)

# ---------------------------------------------------------------------------
# CUDA toolkit components
# ---------------------------------------------------------------------------
find_package(CUDAToolkit REQUIRED)

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(breakage_cuda_core STATIC
  src/system.cpp
  src/ref_cpu.cpp
  src/kernels.cu
  src/linalg.cu
  src/qp_solver.cu
)
target_include_directories(breakage_cuda_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(breakage_cuda_core PUBLIC
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  osqpstatic
  CUDA::cudart
  CUDA::cublas
  CUDA::cusparse
  CUDA::cusolver
)
set_target_properties(breakage_cuda_core PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_RESOLVE_DEVICE_SYMBOLS ON
  CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}"
)

# Suppress noisy diagnostics on Eigen + nvcc and treat real issues as warnings.
target_compile_options(breakage_cuda_core PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic>
  $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr --expt-extended-lambda -Xcompiler=-Wall>
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
enable_testing()

# NOTE: On WSL2 the real CUDA driver libcuda.so.1 lives at /usr/lib/wsl/lib but
# is not on the default loader search path. Since libcudart resolves the driver
# via dlopen at runtime, RPATH/RUNPATH on the binary is NOT honored for that
# dlopen call. The portable workaround is to export
#   LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH
# before running the test/bench binaries. See README.md.
function(breakage_cuda_add_test TEST_NAME)
  add_executable(${TEST_NAME} ${ARGN})
  target_link_libraries(${TEST_NAME} PRIVATE breakage_cuda_core)
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
  if(UNIX AND NOT APPLE)
    set_tests_properties(${TEST_NAME} PROPERTIES
      ENVIRONMENT "LD_LIBRARY_PATH=/usr/lib/wsl/lib:$ENV{LD_LIBRARY_PATH}")
  endif()
endfunction()

breakage_cuda_add_test(test_dense_kernels tests/test_dense_kernels.cpp)
breakage_cuda_add_test(test_qp_random tests/test_qp_random.cpp)
breakage_cuda_add_test(test_full_pipeline tests/test_full_pipeline.cpp)

# Bench is just an executable, not a test.
add_executable(breakage_cuda_bench tests/bench.cpp)
target_link_libraries(breakage_cuda_bench PRIVATE breakage_cuda_core)
