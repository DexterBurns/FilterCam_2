#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "velodyne_driver::velodyne_input" for configuration "Debug"
set_property(TARGET velodyne_driver::velodyne_input APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(velodyne_driver::velodyne_input PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libvelodyne_input.so"
  IMPORTED_SONAME_DEBUG "libvelodyne_input.so"
  )

list(APPEND _cmake_import_check_targets velodyne_driver::velodyne_input )
list(APPEND _cmake_import_check_files_for_velodyne_driver::velodyne_input "${_IMPORT_PREFIX}/lib/libvelodyne_input.so" )

# Import target "velodyne_driver::velodyne_driver" for configuration "Debug"
set_property(TARGET velodyne_driver::velodyne_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(velodyne_driver::velodyne_driver PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_DEBUG "tf2_ros::tf2_ros"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libvelodyne_driver.so"
  IMPORTED_SONAME_DEBUG "libvelodyne_driver.so"
  )

list(APPEND _cmake_import_check_targets velodyne_driver::velodyne_driver )
list(APPEND _cmake_import_check_files_for_velodyne_driver::velodyne_driver "${_IMPORT_PREFIX}/lib/libvelodyne_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
