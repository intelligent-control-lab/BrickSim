## FetchIsaacDeps.cmake
##
## Download and extract Isaac Sim SDK archives.
##
## Directive:
##   isaacsim_dep(<url> <sha256> <EXTRACT_PATH>)
##     - Downloads the dependency to <ISAACSIM_DEPS_ROOT>/<name>
##     - Extracts it into <build>/_deps/<name>
##     - Sets <EXTRACT_PATH> to the extracted root path.
##
## After isaacsim_fetch_all_deps(), the following variables are available:
##  - ISAACSIM_KIT_INCLUDE
##  - ISAACSIM_CARB_INCLUDE, ISAACSIM_CARB_LIBDIR
##  - ISAACSIM_OMNI_CLIENT_INCLUDE
##  - ISAACSIM_PHYSX_INCLUDE, ISAACSIM_PHYSX_LIBDIR
##  - ISAACSIM_PYTHON_INCLUDE, ISAACSIM_PYTHON_LIBDIR
##  - ISAACSIM_USD_INCLUDE, ISAACSIM_USD_BOOST_INCLUDE, ISAACSIM_USD_LIBDIR
##  - ISAACSIM_USD_EXT_PHYSICS_INCLUDE, ISAACSIM_USD_EXT_PHYSICS_LIBDIR
##  - ISAACSIM_OMNI_PHYSICS_INCLUDE
##  - ISAACSIM_NVTX_LIBDIR
##  - ISAACSIM_OMNI_USD_CORE_LIBDIR
##  - ISAACSIM_OMNI_USD_SCHEMA_AUDIO_LIBDIR

set(ISAACSIM_DEPS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/_deps" CACHE PATH "IsaacSim vendor folder")

function(_isaacsim_ensure_dir DIR)
  if(NOT IS_DIRECTORY "${DIR}")
    file(MAKE_DIRECTORY "${DIR}")
  endif()
endfunction()

function(_isaacsim_download URL OUT_PATH SHA256)
  if(NOT EXISTS "${OUT_PATH}")
    message(STATUS "Downloading ${URL}")
    file(DOWNLOAD "${URL}" "${OUT_PATH}" SHOW_PROGRESS EXPECTED_HASH "SHA256=${SHA256}")
  endif()
endfunction()

function(_isaacsim_extract ARCHIVE OUT_DIR)
  _isaacsim_ensure_dir("${OUT_DIR}")
  find_program(SEVENZIP NAMES 7z 7zz REQUIRED)
  # Use a stamp file to avoid repeated extraction
  set(stamp "${OUT_DIR}/.extracted")
  if(EXISTS "${stamp}")
    return()
  endif()
  execute_process(COMMAND "${SEVENZIP}" x -snld20 -y "-o${OUT_DIR}" "${ARCHIVE}" RESULT_VARIABLE rc)
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR "7z extraction failed: ${ARCHIVE}")
  endif()
  # Write current timestamp to stamp file
  string(TIMESTAMP now "%Y-%m-%dT%H:%M:%SZ" UTC)
  file(WRITE "${stamp}" "${now}\n")
endfunction()

function(isaacsim_dep URL SHA256 VAR_PREFIX)
  # Keep archives in source tree; extract into build tree so it is cleaned with the build dir
  set(_archives_dir "${ISAACSIM_DEPS_ROOT}")
  set(_extract_dir  "${CMAKE_BINARY_DIR}/_deps")
  _isaacsim_ensure_dir("${_archives_dir}")
  _isaacsim_ensure_dir("${_extract_dir}")

  get_filename_component(_fname "${URL}" NAME)
  set(_archive "${_archives_dir}/${_fname}")
  _isaacsim_download("${URL}" "${_archive}" "${SHA256}")

  set(_base "${_fname}")
  string(REGEX REPLACE "\\.(zip|7z)$" "" _base "${_base}")
  set(_dest "${_extract_dir}/${_base}")
  _isaacsim_extract("${_archive}" "${_dest}")

  # Export root path
  set(${VAR_PREFIX} "${_dest}" PARENT_SCOPE)
endfunction()

function(isaacsim_fetch_all_deps)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/kit-kernel%40107.3.1%2Bisaac.206797.8131b85d.gl.manylinux_2_35_x86_64.release.zip" "c92f4a33359847111807f45081a0ea4e28f8d865700261814bba94a6235d1005" ISAACSIM_KIT)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/carb_sdk%2Bplugins.manylinux_2_35_x86_64%40206.4%2Brelease.9396.3c6f8191.gl.7z" "9eb7f193f2190078824967d404a98cc758de11bcb4ab519331009a0821bdf9f6" ISAACSIM_CARB)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/omni_client_library.linux-x86_64%402.65.0.7z" "7edacdad8cd5f276c0d76fc6ca2b1aa7874cf3b9009eddfd458e0716bee7b181" ISAACSIM_OMNI_CLIENT)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/physxsdk%405.6.1.b07ddd12-release-107.3-linux-x86_64.zip" "143bbf205a18ebd13dee178be7237f8b43f1fbe588f7d5cd2c01f745ab477032" ISAACSIM_PHYSX)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/usd.py311.manylinux_2_35_x86_64.stock.release%400.24.05.kit.6-gl.14415%2Bd9efdd65.7z" "50ecb86d03d19c98d89d7bed425c1b33edbece6098047e3b4e6443ed93215946" ISAACSIM_USD)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/usd_ext_physics%4024.05%2Brelease.40469.09c54277.gl.manylinux_2_35_x86_64.release.7z" "de5bafc6a132c1a180b12953ee581f062ddf4d4d5364d35459ada4f2aa0b414c" ISAACSIM_USD_EXT_PHYSICS)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/omni_physics%40107.3.18-32193518-release_107.3-214216c8-manylinux_2_35_x86_64.7z" "7e46bcc5505506859282fd203febea58d762e1487ecf19bd7a37cd4af3309006" ISAACSIM_OMNI_PHYSICS)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/nvtx%403-20180925-lic3.7z" "27bc550f838cdc323fe2f25e6e762e539f5fee3c83da4fe446ef17f63c44022d" ISAACSIM_NVTX)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/cuda%4011.8.0-3-linux-x86_64-release.7z" "7172eaa52a079abc7070b57378d080740aa6a96bf2f917a155e505670585c526" ISAACSIM_CUDA)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/python%403.11.13%2Bnv1-linux-x86_64.7z" "f8db1b23d8b4bf0c39fa60aec179e25e61582ea96727ca67a044bc9af3074650" ISAACSIM_PYTHON)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/omni.usd.core-0c0397b2080ca5ac.zip" "68410754b785e506ce2e6928d3157128faaf19f2b097e84b5957b20e8e50f578" ISAACSIM_OMNI_USD_CORE)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/omni.usd.schema.audio-e6f3b0dc8dc66d41.zip" "3f26cb31b604507148a5113427b4b143c9c36eef4a9fa0561d1e1341d1e4f519" ISAACSIM_OMNI_USD_SCHEMA_AUDIO)

  set(ISAACSIM_KIT_INCLUDE                  "${ISAACSIM_KIT}/dev/include"                   PARENT_SCOPE)
  set(ISAACSIM_CARB_INCLUDE                 "${ISAACSIM_CARB}/include"                      PARENT_SCOPE)
  set(ISAACSIM_CARB_LIBDIR                  "${ISAACSIM_CARB}/_build/linux-x86_64/release"  PARENT_SCOPE)
  set(ISAACSIM_OMNI_CLIENT_INCLUDE          "${ISAACSIM_OMNI_CLIENT}/include"               PARENT_SCOPE)
  set(ISAACSIM_PHYSX_INCLUDE                "${ISAACSIM_PHYSX}/include"                     PARENT_SCOPE)
  set(ISAACSIM_PHYSX_LIBDIR                 "${ISAACSIM_PHYSX}/bin/linux.x86_64/checked"    PARENT_SCOPE)
  set(ISAACSIM_PYTHON_INCLUDE               "${ISAACSIM_PYTHON}/include/python3.11"         PARENT_SCOPE)
  set(ISAACSIM_PYTHON_LIBDIR                "${ISAACSIM_PYTHON}/lib"                        PARENT_SCOPE)
  set(ISAACSIM_USD_INCLUDE                  "${ISAACSIM_USD}/include"                       PARENT_SCOPE)
  set(ISAACSIM_USD_BOOST_INCLUDE            "${ISAACSIM_USD}/include/boost"                 PARENT_SCOPE)
  set(ISAACSIM_USD_LIBDIR                   "${ISAACSIM_USD}/lib"                           PARENT_SCOPE)
  set(ISAACSIM_USD_EXT_PHYSICS_INCLUDE      "${ISAACSIM_USD_EXT_PHYSICS}/include"           PARENT_SCOPE)
  set(ISAACSIM_USD_EXT_PHYSICS_LIBDIR       "${ISAACSIM_USD_EXT_PHYSICS}/lib"               PARENT_SCOPE)
  set(ISAACSIM_OMNI_PHYSICS_INCLUDE         "${ISAACSIM_OMNI_PHYSICS}/include"              PARENT_SCOPE)
  set(ISAACSIM_NVTX_LIBDIR                  "${ISAACSIM_NVTX}/lib64"                        PARENT_SCOPE)
  set(ISAACSIM_CUDA_INCLUDE                 "${ISAACSIM_CUDA}/targets/x86_64-linux/include" PARENT_SCOPE)
  set(ISAACSIM_CUDA_LIBDIR                  "${ISAACSIM_CUDA}/targets/x86_64-linux/lib"     PARENT_SCOPE)
  set(ISAACSIM_OMNI_USD_CORE_LIBDIR         "${ISAACSIM_OMNI_USD_CORE}/bin"                 PARENT_SCOPE)
  set(ISAACSIM_OMNI_USD_SCHEMA_AUDIO_LIBDIR "${ISAACSIM_OMNI_USD_SCHEMA_AUDIO}/lib"         PARENT_SCOPE)
endfunction()
