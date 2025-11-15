function(lego_add_abi_check TARGET_NAME)
  if(NOT TARGET ${TARGET_NAME})
    message(FATAL_ERROR "lego_add_abi_check: target '${TARGET_NAME}' does not exist")
  endif()

  # Add a post-build step that fails the build if violating symbols are found.
  # Uses exactly the same readelf + grep + sort + uniq pipeline as provided by the user,
  # wrapped to cause a non-zero exit when matches are present.'
  #
  # ABI requirements:
  #   GLIBC >= 2.35
  #   GLIBCXX >= 3.4.30
  #   CXXABI >= 1.3.13
  add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND bash -lc [=[
RESULT=$(llvm-readelf --dyn-syms -W "$0" | grep -Po 'UND\s+\K\S+@(?:GLIBC_(?:2\.35\.\d+|2\.(?:3[6-9]|[4-9]\d)(?:\.\d+)?|[3-9]\.\d+(?:\.\d+)?)|GLIBCXX_(?:3\.4\.(?:3[1-9]|[4-9]\d)|3\.[5-9]\.\d+|[4-9]\.\d+\.\d+)|CXXABI_(?:1\.3\.(?:1[4-9]|[2-9]\d)|1\.[4-9]\.\d+|[2-9]\.\d+\.\d+))' | sort | uniq); test -z "$RESULT" || { echo "ABI check failed for: $0"; echo "$RESULT"; exit 1; }
]=] "$<TARGET_FILE:${TARGET_NAME}>"
    VERBATIM
  )
endfunction()
