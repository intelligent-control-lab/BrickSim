## FetchIsaacDeps.cmake
##
## Prepare Isaac Sim SDK archives for native builds.

include(PrepareArtifacts)

set(_ISAACSIM_DEPS_DEFAULT_ROOT "${CMAKE_BINARY_DIR}/_deps/isaacsim")
set(_ISAACSIM_DEPS_ROOT_UNDER_SOURCE OFF)
if(DEFINED ISAACSIM_DEPS_ROOT)
  set(_isaacsim_current_deps_root "${ISAACSIM_DEPS_ROOT}")
  cmake_path(ABSOLUTE_PATH _isaacsim_current_deps_root BASE_DIRECTORY "${CMAKE_BINARY_DIR}" NORMALIZE OUTPUT_VARIABLE _isaacsim_current_deps_root_abs)
  cmake_path(IS_PREFIX CMAKE_CURRENT_SOURCE_DIR "${_isaacsim_current_deps_root_abs}" NORMALIZE _ISAACSIM_DEPS_ROOT_UNDER_SOURCE)
endif()
if(DEFINED ISAACSIM_DEPS_ROOT AND _ISAACSIM_DEPS_ROOT_UNDER_SOURCE)
  set(ISAACSIM_DEPS_ROOT "${_ISAACSIM_DEPS_DEFAULT_ROOT}" CACHE PATH "Prepared Isaac Sim SDK dependency root" FORCE)
else()
  set(ISAACSIM_DEPS_ROOT "${_ISAACSIM_DEPS_DEFAULT_ROOT}" CACHE PATH "Prepared Isaac Sim SDK dependency root")
endif()
set(ISAACSIM_DEPS_CACHE_ROOT "" CACHE PATH "Optional user cache root for Isaac Sim SDK archives and extracted trees")

function(isaacsim_fetch_all_deps)
  set(_prepare_artifacts_args
    CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/isaac_deps.toml"
    OUTPUT_ROOT "${ISAACSIM_DEPS_ROOT}"
    OUT_ROOT_VARIABLE ISAACSIM_DEPS_ROOT
  )
  if(ISAACSIM_DEPS_CACHE_ROOT)
    list(APPEND _prepare_artifacts_args CACHE_ROOT "${ISAACSIM_DEPS_CACHE_ROOT}")
  endif()
  prepare_artifacts(${_prepare_artifacts_args})

  set(ISAACSIM_KIT                        "${ISAACSIM_DEPS_ROOT}/kit")
  set(ISAACSIM_CARB                       "${ISAACSIM_DEPS_ROOT}/carb")
  set(ISAACSIM_OMNI_CLIENT                "${ISAACSIM_DEPS_ROOT}/omni_client")
  set(ISAACSIM_PHYSX                      "${ISAACSIM_DEPS_ROOT}/physx")
  set(ISAACSIM_USD                        "${ISAACSIM_DEPS_ROOT}/usd")
  set(ISAACSIM_USD_EXT_PHYSICS            "${ISAACSIM_DEPS_ROOT}/usd_ext_physics")
  set(ISAACSIM_OMNI_PHYSICS               "${ISAACSIM_DEPS_ROOT}/omni_physics")
  set(ISAACSIM_PYTHON                     "${ISAACSIM_DEPS_ROOT}/python")
  set(ISAACSIM_OMNI_USD_CORE              "${ISAACSIM_DEPS_ROOT}/omni_usd_core")
  set(ISAACSIM_OMNI_USD_SCHEMA_AUDIO      "${ISAACSIM_DEPS_ROOT}/omni_usd_schema_audio")

  set(ISAACSIM_KIT_INCLUDE                  "${ISAACSIM_KIT}/dev/include")
  set(ISAACSIM_CARB_INCLUDE                 "${ISAACSIM_CARB}/include")
  set(ISAACSIM_CARB_LIBDIR                  "${ISAACSIM_CARB}/_build/linux-x86_64/release")
  set(ISAACSIM_OMNI_CLIENT_INCLUDE          "${ISAACSIM_OMNI_CLIENT}/include")
  set(ISAACSIM_PHYSX_INCLUDE                "${ISAACSIM_PHYSX}/include")
  set(ISAACSIM_PHYSX_LIBDIR                 "${ISAACSIM_PHYSX}/bin/linux.x86_64/checked")
  set(ISAACSIM_PYTHON_INCLUDE               "${ISAACSIM_PYTHON}/include/python3.11")
  set(ISAACSIM_PYTHON_LIBDIR                "${ISAACSIM_PYTHON}/lib")
  set(ISAACSIM_USD_INCLUDE                  "${ISAACSIM_USD}/include")
  set(ISAACSIM_USD_BOOST_INCLUDE            "${ISAACSIM_USD}/include/boost")
  set(ISAACSIM_USD_LIBDIR                   "${ISAACSIM_USD}/lib")
  set(ISAACSIM_USD_EXT_PHYSICS_INCLUDE      "${ISAACSIM_USD_EXT_PHYSICS}/include")
  set(ISAACSIM_USD_EXT_PHYSICS_LIBDIR       "${ISAACSIM_USD_EXT_PHYSICS}/lib")
  set(ISAACSIM_OMNI_PHYSICS_INCLUDE         "${ISAACSIM_OMNI_PHYSICS}/include")
  set(ISAACSIM_OMNI_USD_CORE_LIBDIR         "${ISAACSIM_OMNI_USD_CORE}/bin")
  set(ISAACSIM_OMNI_USD_SCHEMA_AUDIO_LIBDIR "${ISAACSIM_OMNI_USD_SCHEMA_AUDIO}/lib")

  if(NOT TARGET isaacsim_sdk)
    add_library(isaacsim_sdk INTERFACE)
  endif()

  target_include_directories(isaacsim_sdk SYSTEM INTERFACE
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

  target_link_directories(isaacsim_sdk INTERFACE
    "${ISAACSIM_OMNI_USD_CORE_LIBDIR}"
    "${ISAACSIM_OMNI_USD_SCHEMA_AUDIO_LIBDIR}"
    "${ISAACSIM_CARB_LIBDIR}"
    "${ISAACSIM_PHYSX_LIBDIR}"
    "${ISAACSIM_PYTHON_LIBDIR}"
    "${ISAACSIM_USD_EXT_PHYSICS_LIBDIR}"
    "${ISAACSIM_USD_LIBDIR}"
  )

  target_link_libraries(isaacsim_sdk INTERFACE
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
    usd_usdShade
    usd_usdGeom
    usd_usdImaging
    usd_usdPhysics
    usd_usdUtils
    usd_vt
    omni.usd
    physxSchema
    omniAudioSchema
  )

  return(PROPAGATE
    ISAACSIM_KIT ISAACSIM_KIT_INCLUDE
    ISAACSIM_CARB ISAACSIM_CARB_INCLUDE ISAACSIM_CARB_LIBDIR
    ISAACSIM_OMNI_CLIENT ISAACSIM_OMNI_CLIENT_INCLUDE
    ISAACSIM_PHYSX ISAACSIM_PHYSX_INCLUDE ISAACSIM_PHYSX_LIBDIR
    ISAACSIM_PYTHON ISAACSIM_PYTHON_INCLUDE ISAACSIM_PYTHON_LIBDIR
    ISAACSIM_USD ISAACSIM_USD_INCLUDE ISAACSIM_USD_BOOST_INCLUDE ISAACSIM_USD_LIBDIR
    ISAACSIM_USD_EXT_PHYSICS ISAACSIM_USD_EXT_PHYSICS_INCLUDE ISAACSIM_USD_EXT_PHYSICS_LIBDIR
    ISAACSIM_OMNI_PHYSICS ISAACSIM_OMNI_PHYSICS_INCLUDE
    ISAACSIM_OMNI_USD_CORE ISAACSIM_OMNI_USD_CORE_LIBDIR
    ISAACSIM_OMNI_USD_SCHEMA_AUDIO ISAACSIM_OMNI_USD_SCHEMA_AUDIO_LIBDIR
  )
endfunction()
