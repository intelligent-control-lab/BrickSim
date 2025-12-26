## SdkPatches.cmake
##
## Provides a simple helper to apply SDK header patches exactly once per
## build tree and include root, with automatic restore from .orig backups
## prior to reapplying. Fails fast on errors.

find_program(PATCH_EXECUTABLE patch REQUIRED)

function(apply_sdk_patch_once name include_dir patch_file strip)
  # Unique stamp per include root; re-run if the patch file changes
  string(SHA1 DIR_SHA "${include_dir}")
  set(stamp "${CMAKE_BINARY_DIR}/_patches/${name}-${DIR_SHA}.stamp")

  add_custom_command(
    OUTPUT "${stamp}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_BINARY_DIR}/_patches"
    COMMAND /usr/bin/env bash "${CMAKE_CURRENT_SOURCE_DIR}/cmake/apply_patch_once.sh"
            --patch-exe "${PATCH_EXECUTABLE}"
            --include-dir "${include_dir}"
            --patch-file "${patch_file}"
            --strip "${strip}"
            --stamp "${stamp}"
    DEPENDS "${patch_file}"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/apply_patch_once.sh"
    VERBATIM
    COMMENT "Applying ${name} patch in ${include_dir}"
  )

  add_custom_target("${name}" DEPENDS "${stamp}")
endfunction()
