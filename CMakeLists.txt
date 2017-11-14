cmake_minimum_required(VERSION 3.5)

# Set default build to Release
if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type specified, setting to Release")
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_subdirectory("cpp")
add_subdirectory("python")

add_custom_command(
    OUTPUT export_module_command
    COMMAND cp "${CMAKE_CURRENT_BINARY_DIR}/cpp/uav_planning.cpython*.so" ${CMAKE_CURRENT_SOURCE_DIR}/python/fire_rs/
    DEPENDS uav_planning
    )

add_custom_target(
     export_module ALL DEPENDS export_module_command
    )
