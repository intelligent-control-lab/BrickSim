## PrepareArtifacts.cmake
##
## CMake wrapper for prepare_artifacts.py.
##
## Include this module from a CMake project that wants to prepare archive
## artifacts from a TOML manifest at configure time. The TOML manifest is read by
## prepare_artifacts.py, which downloads archives into a user-level cache,
## extracts pristine cache trees, prepares output directories with hardlinks or
## reflinked copies for patch-touched files, and applies configured patches.
##
## Basic use:
##   include(PrepareArtifacts)
##   prepare_artifacts(
##     CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/deps.toml"
##     OUTPUT_ROOT "${CMAKE_BINARY_DIR}/_deps"
##   )
##
## Optional arguments:
##   CACHE_ROOT <path>
##     Override the user-level cache root used by prepare_artifacts.py.
##
##   SCRIPT <path>
##     Override the path to prepare_artifacts.py. By default this module expects
##     the script next to PrepareArtifacts.cmake.
##
##   OUT_ROOT_VARIABLE <variable>
##     Return the absolute prepared output root to the caller.
##
## The module automatically tracks the TOML file, patch files reported by
## prepare_artifacts.py --print-inputs, and the Python script itself as CMake
## configure dependencies.

include_guard(GLOBAL)

function(_prepare_artifacts_find_host_python OUT_VAR)
  unset(PREPARE_ARTIFACTS_PYTHON CACHE)
  find_program(PREPARE_ARTIFACTS_PYTHON NAMES python3.11 python3 REQUIRED NO_CACHE)
  execute_process(
    COMMAND "${PREPARE_ARTIFACTS_PYTHON}" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 11) else 1)"
    RESULT_VARIABLE _python_version_rc
    OUTPUT_QUIET
    ERROR_QUIET
  )
  if(NOT _python_version_rc EQUAL 0)
    message(FATAL_ERROR "prepare_artifacts.py requires python3 >= 3.11: ${PREPARE_ARTIFACTS_PYTHON}")
  endif()
  set(${OUT_VAR} "${PREPARE_ARTIFACTS_PYTHON}" PARENT_SCOPE)
endfunction()

function(_prepare_artifacts_track_inputs PYTHON SCRIPT CONFIG)
  execute_process(
    COMMAND "${PYTHON}" "${SCRIPT}" --config "${CONFIG}" --print-inputs
    RESULT_VARIABLE _inputs_rc
    OUTPUT_VARIABLE _inputs
    ERROR_VARIABLE _inputs_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _inputs_rc EQUAL 0)
    message(FATAL_ERROR "Failed to read artifact preparation inputs:\n${_inputs_error}")
  endif()

  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${SCRIPT}")
  string(REPLACE "\n" ";" _input_list "${_inputs}")
  foreach(_input IN LISTS _input_list)
    if(NOT _input STREQUAL "")
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_input}")
    endif()
  endforeach()
endfunction()

function(prepare_artifacts)
  cmake_parse_arguments(
    ARG
    ""
    "CONFIG;OUTPUT_ROOT;CACHE_ROOT;SCRIPT;OUT_ROOT_VARIABLE"
    ""
    ${ARGN}
  )

  if(NOT ARG_CONFIG)
    message(FATAL_ERROR "prepare_artifacts requires CONFIG")
  endif()
  if(NOT ARG_OUTPUT_ROOT)
    message(FATAL_ERROR "prepare_artifacts requires OUTPUT_ROOT")
  endif()

  if(ARG_SCRIPT)
    get_filename_component(_script "${ARG_SCRIPT}" ABSOLUTE)
  else()
    set(_script "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/prepare_artifacts.py")
  endif()

  _prepare_artifacts_find_host_python(_python)
  find_program(PREPARE_ARTIFACTS_7ZZ NAMES 7zz REQUIRED)
  find_program(PREPARE_ARTIFACTS_PATCH NAMES patch REQUIRED)
  find_program(PREPARE_ARTIFACTS_CP NAMES cp REQUIRED)
  find_program(PREPARE_ARTIFACTS_WGET NAMES wget)

  get_filename_component(_config "${ARG_CONFIG}" ABSOLUTE)
  get_filename_component(_output_root "${ARG_OUTPUT_ROOT}" ABSOLUTE BASE_DIR "${CMAKE_BINARY_DIR}")

  _prepare_artifacts_track_inputs("${_python}" "${_script}" "${_config}")

  set(_command
    "${_python}" "${_script}"
    --config "${_config}"
    --output-root "${_output_root}"
    --sevenzip "${PREPARE_ARTIFACTS_7ZZ}"
    --patch "${PREPARE_ARTIFACTS_PATCH}"
    --cp "${PREPARE_ARTIFACTS_CP}"
  )
  if(PREPARE_ARTIFACTS_WGET)
    list(APPEND _command --wget "${PREPARE_ARTIFACTS_WGET}")
  endif()
  if(ARG_CACHE_ROOT)
    get_filename_component(_cache_root "${ARG_CACHE_ROOT}" ABSOLUTE)
    list(APPEND _command --cache-root "${_cache_root}")
  endif()

  execute_process(
    COMMAND ${_command}
    RESULT_VARIABLE _prepare_artifacts_rc
  )
  if(NOT _prepare_artifacts_rc EQUAL 0)
    message(FATAL_ERROR "Artifact preparation failed")
  endif()

  if(ARG_OUT_ROOT_VARIABLE)
    set(${ARG_OUT_ROOT_VARIABLE} "${_output_root}" PARENT_SCOPE)
  endif()
endfunction()
