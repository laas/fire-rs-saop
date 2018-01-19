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
        OUTPUT export_firemapping_command
        COMMAND cp "${CMAKE_CURRENT_BINARY_DIR}/cpp/firemapping.cpython*.so" ${CMAKE_CURRENT_SOURCE_DIR}/python/fire_rs/
        DEPENDS firemapping
)

add_custom_target(
        export_firemapping ALL DEPENDS export_firemapping_command
)

add_custom_command(
    OUTPUT export_uav_planning_command
    COMMAND cp "${CMAKE_CURRENT_BINARY_DIR}/cpp/uav_planning.cpython*.so" ${CMAKE_CURRENT_SOURCE_DIR}/python/fire_rs/
    DEPENDS uav_planning
    )

add_custom_target(
     export_uav_planning ALL DEPENDS export_uav_planning_command
)

if(WITH_IMC_INTERFACE)
add_custom_command(
    OUTPUT export_neptus_interface_command
    COMMAND cp "${CMAKE_CURRENT_BINARY_DIR}/cpp/neptus_interface.cpython*.so" ${CMAKE_CURRENT_SOURCE_DIR}/python/fire_rs/
    DEPENDS neptus_interface
)

add_custom_target(
     export_neptus_interface ALL DEPENDS export_neptus_interface_command
)
endif(WITH_IMC_INTERFACE)
