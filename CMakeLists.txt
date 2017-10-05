cmake_minimum_required(VERSION 3.5)

add_subdirectory("cpp")
add_subdirectory("python")

add_custom_command(
    OUTPUT export_module_command
    COMMAND cp ${PLANNING_MODULE} ${CMAKE_CURRENT_SOURCE_DIR}/python/fire_rs/
    DEPENDS uav_planning
    )

add_custom_target(
     export_module ALL DEPENDS export_module_command
    )
