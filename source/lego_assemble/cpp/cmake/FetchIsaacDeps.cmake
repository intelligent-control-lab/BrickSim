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
##   isaacsim_fetch_all_deps()
##     - Fetches all required Isaac Sim dependencies and sets variables.

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
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/kit-kernel%40107.3.3%2Bisaac.229672.69cbf6ad.gl.manylinux_2_35_x86_64.release.zip" "d25c458933838dee75eabb941f9193865444653265eed5ddd60c6ca7c212d661" ISAACSIM_KIT)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/carb_sdk%2Bplugins.manylinux_2_35_x86_64%40206.6%2Brelease.9587.07f17b1b.gl.7z" "88399f41325b0569e57d4cf38999a613ea47baf744caeac1039bc82d684270fd" ISAACSIM_CARB)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/omni_client_library.linux-x86_64%402.67.0.7z" "41967c3e9904ac9f85d56c60a422cabf43425e29e27ac50f37a502f81170096b" ISAACSIM_OMNI_CLIENT)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/physxsdk%405.6.1.f9c67de2-release-107.3-linux-x86_64.zip" "d5abba06e8c2b09f51776de8697661f340cdf3e78182ec5b0d237ac4a6c23906" ISAACSIM_PHYSX)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/usd.py311.manylinux_2_35_x86_64.stock.release%400.24.05.kit.7-gl.16400%2B05f48f24.7z" "66723886655415aa634bba6b9dd225d37b295cfc3a522aa4abc6c37de6d53809" ISAACSIM_USD)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/usd_ext_physics%4024.05%2Brelease.40469.09c54277.gl.manylinux_2_35_x86_64.release.7z" "de5bafc6a132c1a180b12953ee581f062ddf4d4d5364d35459ada4f2aa0b414c" ISAACSIM_USD_EXT_PHYSICS)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/omni_physics%40107.3.26-36011981-release_107.3-3a61992c-manylinux_2_35_x86_64.7z" "720062a96cf03bcd36cfb439ed9aac2640cb8c37da8de0e7b4e8b28f52972397" ISAACSIM_OMNI_PHYSICS)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/python%403.11.13%2Bnv1-linux-x86_64.7z" "f8db1b23d8b4bf0c39fa60aec179e25e61582ea96727ca67a044bc9af3074650" ISAACSIM_PYTHON)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/omni.usd.core-9ce8448d1db810d8.zip" "9606a5faa889114bd7c39677e7b1d25971964767b08b21983bfa673b513e2d42" ISAACSIM_OMNI_USD_CORE)
  isaacsim_dep("https://d4i3qtqj3r0z5.cloudfront.net/omni.usd.schema.audio-e6f3b0dc8dc66d41.zip" "3f26cb31b604507148a5113427b4b143c9c36eef4a9fa0561d1e1341d1e4f519" ISAACSIM_OMNI_USD_SCHEMA_AUDIO)

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
