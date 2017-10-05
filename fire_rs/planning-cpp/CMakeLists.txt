cmake_minimum_required(VERSION 3.5)
project(planning_cpp)


if(EXISTS "${CMAKE_SOURCE_DIR}/pybind11")
    add_subdirectory("pybind11")
else()
    find_package(pybind11 2.2.1 REQUIRED)
endif()

set(CMAKE_CXX_STANDARD 11)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated")
endif()

set(SOURCE_FILES src/ext/dubins.cpp src/ext/dubins.h src/trajectory.h src/uav.h
        src/waypoint.h src/raster.h src/visibility.h src/planning.h src/ext/optional.h src/utils.h
        src/neighborhoods/dubins_optimization.h src/neighborhoods/insertions.h src/vns_interface.h
        src/plan.h src/fire_data.h src/neighborhoods/moves.h src/neighborhoods/shuffling.h src/neighborhoods/smoothing.h
        src/dubins3d.h)

add_executable(planning_cpp ${SOURCE_FILES} src/test/main.cpp)
add_executable(dubins_test ${SOURCE_FILES} src/test/main_dubins.cpp)

pybind11_add_module(uav_planning ${SOURCE_FILES} src/python_interface.cpp)