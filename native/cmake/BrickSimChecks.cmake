include_guard(GLOBAL)

get_filename_component(BRICKSIM_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

function(_bricksim_normalize_files OUT_VAR)
  set(NORMALIZED_FILES)
  foreach(FILE_PATH IN LISTS ARGN)
    get_filename_component(ABS_FILE_PATH "${FILE_PATH}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    file(TO_CMAKE_PATH "${ABS_FILE_PATH}" ABS_FILE_PATH)
    list(APPEND NORMALIZED_FILES "${ABS_FILE_PATH}")
  endforeach()
  set(${OUT_VAR} "${NORMALIZED_FILES}" PARENT_SCOPE)
endfunction()

function(bricksim_add_format_target)
  cmake_parse_arguments(PARSE_ARGV 0 ARG "" "" "FILES")
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "bricksim_add_format_target: unexpected arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_FILES)
    message(FATAL_ERROR "bricksim_add_format_target requires FILES")
  endif()

  find_program(CLANG_FORMAT_BIN NAMES clang-format REQUIRED)
  _bricksim_normalize_files(BRICKSIM_FORMAT_FILES ${ARG_FILES})

  add_custom_target(format
    COMMAND "${CLANG_FORMAT_BIN}" -i --style=file ${BRICKSIM_FORMAT_FILES}
    WORKING_DIRECTORY "${BRICKSIM_REPO_ROOT}"
    VERBATIM
    COMMAND_EXPAND_LISTS
    USES_TERMINAL
  )
endfunction()

function(bricksim_add_lint_target)
  cmake_parse_arguments(PARSE_ARGV 0 ARG "" "" "FORMAT_FILES;TIDY_FILES")
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "bricksim_add_lint_target: unexpected arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_FORMAT_FILES)
    message(FATAL_ERROR "bricksim_add_lint_target requires FORMAT_FILES")
  endif()
  if(NOT ARG_TIDY_FILES)
    message(FATAL_ERROR "bricksim_add_lint_target requires TIDY_FILES")
  endif()

  find_program(CLANG_FORMAT_BIN NAMES clang-format REQUIRED)
  find_program(CLANG_TIDY_BIN NAMES clang-tidy REQUIRED)
  _bricksim_normalize_files(BRICKSIM_LINT_FORMAT_FILES ${ARG_FORMAT_FILES})
  _bricksim_normalize_files(BRICKSIM_LINT_TIDY_FILES ${ARG_TIDY_FILES})

  set(BRICKSIM_LINT_TARGETS)
  set(BRICKSIM_FORMAT_INDEX 0)
  foreach(BRICKSIM_FORMAT_FILE IN LISTS BRICKSIM_LINT_FORMAT_FILES)
    set(FORMAT_TARGET "${PROJECT_NAME}_bricksim_lint_format_${BRICKSIM_FORMAT_INDEX}")

    add_custom_target("${FORMAT_TARGET}"
      COMMAND "${CLANG_FORMAT_BIN}" --dry-run --Werror --style=file "${BRICKSIM_FORMAT_FILE}"
      WORKING_DIRECTORY "${BRICKSIM_REPO_ROOT}"
      VERBATIM
    )

    list(APPEND BRICKSIM_LINT_TARGETS "${FORMAT_TARGET}")
    math(EXPR BRICKSIM_FORMAT_INDEX "${BRICKSIM_FORMAT_INDEX} + 1")
  endforeach()

  set(BRICKSIM_TIDY_INDEX 0)
  foreach(BRICKSIM_TIDY_FILE IN LISTS BRICKSIM_LINT_TIDY_FILES)
    set(TIDY_TARGET "${PROJECT_NAME}_bricksim_lint_tidy_${BRICKSIM_TIDY_INDEX}")

    add_custom_target("${TIDY_TARGET}"
      COMMAND "${CLANG_TIDY_BIN}"
              -p "${CMAKE_BINARY_DIR}"
              -quiet
              --config-file "${BRICKSIM_REPO_ROOT}/.clang-tidy"
              "--warnings-as-errors=*"
              "${BRICKSIM_TIDY_FILE}"
      WORKING_DIRECTORY "${BRICKSIM_REPO_ROOT}"
      VERBATIM
    )

    list(APPEND BRICKSIM_LINT_TARGETS "${TIDY_TARGET}")
    math(EXPR BRICKSIM_TIDY_INDEX "${BRICKSIM_TIDY_INDEX} + 1")
  endforeach()

  add_custom_target(lint
    DEPENDS ${BRICKSIM_LINT_TARGETS}
  )
endfunction()

function(bricksim_add_checks)
  cmake_parse_arguments(PARSE_ARGV 0 ARG "" "" "DIRS")
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "bricksim_add_checks: unexpected arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_DIRS)
    message(FATAL_ERROR "bricksim_add_checks requires DIRS")
  endif()

  set(BRICKSIM_FORMAT_FILES)
  set(BRICKSIM_LINT_FILES)
  foreach(DIR_PATH IN LISTS ARG_DIRS)
    get_filename_component(ABS_DIR_PATH "${DIR_PATH}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    if(NOT IS_DIRECTORY "${ABS_DIR_PATH}")
      message(FATAL_ERROR "bricksim_add_checks: directory '${DIR_PATH}' does not exist")
    endif()
    file(GLOB_RECURSE DIR_FORMAT_FILES CONFIGURE_DEPENDS
      "${ABS_DIR_PATH}/*.cpp"
      "${ABS_DIR_PATH}/*.cppm"
      "${ABS_DIR_PATH}/*.hpp"
    )
    file(GLOB_RECURSE DIR_LINT_FILES CONFIGURE_DEPENDS
      "${ABS_DIR_PATH}/*.cpp"
      "${ABS_DIR_PATH}/*.cppm"
    )
    list(APPEND BRICKSIM_FORMAT_FILES ${DIR_FORMAT_FILES})
    list(APPEND BRICKSIM_LINT_FILES ${DIR_LINT_FILES})
  endforeach()

  list(REMOVE_DUPLICATES BRICKSIM_FORMAT_FILES)
  list(REMOVE_DUPLICATES BRICKSIM_LINT_FILES)
  bricksim_add_format_target(FILES ${BRICKSIM_FORMAT_FILES})
  bricksim_add_lint_target(
    FORMAT_FILES ${BRICKSIM_FORMAT_FILES}
    TIDY_FILES ${BRICKSIM_LINT_FILES}
  )
endfunction()
