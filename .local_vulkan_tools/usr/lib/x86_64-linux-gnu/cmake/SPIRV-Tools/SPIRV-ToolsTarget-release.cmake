#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SPIRV-Tools-static" for configuration "Release"
set_property(TARGET SPIRV-Tools-static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(SPIRV-Tools-static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libSPIRV-Tools.a"
  )

list(APPEND _cmake_import_check_targets SPIRV-Tools-static )
list(APPEND _cmake_import_check_files_for_SPIRV-Tools-static "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libSPIRV-Tools.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
