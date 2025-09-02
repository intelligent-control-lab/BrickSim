# Minimal Isaac Sim target-deps setup.
#
# Ensure TARGET_DEPS_DIR is defined:
# - Prefer the CMake variable TARGET_DEPS_DIR when passed by the build.
# - Otherwise read from the environment ISAACSIM_TARGET_DEPS.
# - Error if missing.

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

# Map CMake build type to Isaac Sim folder names
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
string(TOLOWER "${CMAKE_BUILD_TYPE}" _cfg_l)
if(_cfg_l STREQUAL "debug")
  set(ISAAC_CFG_DIR "debug")
else()
  set(ISAAC_CFG_DIR "checked")
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(ISAAC_PLATFORM_DIR "linux.x86_64")
else()
  message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}")
endif()

message(STATUS "IsaacSim target-deps: ${TARGET_DEPS_DIR}")
message(STATUS "IsaacSim config     : ${ISAAC_CFG_DIR}")
message(STATUS "IsaacSim platform   : ${ISAAC_PLATFORM_DIR}")
