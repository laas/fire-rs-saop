cmake_minimum_required(VERSION 3.5)
project(planning_cpp)


if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/pybind11")
    add_subdirectory("pybind11")
else()
    find_package(pybind11 2.2.1 REQUIRED)
endif()

set(CMAKE_CXX_STANDARD 11)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated")
endif()

set(SOURCE_FILES src/ext/dubins.cpp src/ext/dubins.h src/ext/json.hpp src/core/structures/trajectory.h src/core/structures/uav.h
        src/core/structures/waypoint.h src/raster.h src/vns/visibility.h src/ext/optional.h src/utils.h
        src/vns/neighborhoods/dubins_optimization.h src/vns/neighborhoods/insertions.h src/vns/vns_interface.h
        src/vns/plan.h src/vns/fire_data.h src/vns/neighborhoods/moves.h src/vns/neighborhoods/shuffling.h src/vns/neighborhoods/smoothing.h
        src/dubins3d.h  src/core/structures/trajectories.h src/core/updates/updates.cpp src/core/updates/updates.h src/utils.cpp src/vns/factory.cpp src/vns/factory.h)

add_executable(tests ${SOURCE_FILES} src/test/core/test_reversible_updates.h src/test/test_dubins.h
        src/test/test_position_manipulation.h src/test/main_tests.cpp)

pybind11_add_module(uav_planning ${SOURCE_FILES} src/python_interface.cpp)

set(PLANNING_MODULE_NAME "uav_planning.cpython-35m-x86_64-linux-gnu.so")
set(PLANNING_MODULE "${CMAKE_CURRENT_BINARY_DIR}/${PLANNING_MODULE_NAME}" PARENT_SCOPE)
