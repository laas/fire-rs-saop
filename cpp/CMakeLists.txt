cmake_minimum_required(VERSION 3.5)
project(planning_cpp)


if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/pybind11")
    add_subdirectory("pybind11")
else()
    find_package(pybind11 2.2.1 REQUIRED)
endif()

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(SOURCE_FILES src/ext/dubins.cpp src/ext/dubins.h src/ext/json.hpp src/core/structures/trajectory.h 
    src/core/structures/uav.h src/core/structures/waypoint.h src/raster.h src/vns/visibility.h src/ext/optional.h 
    src/utils.h src/vns/neighborhoods/dubins_optimization.h src/vns/neighborhoods/insertions.h src/vns/vns_interface.h
    src/vns/plan.h src/vns/fire_data.h src/vns/neighborhoods/moves.h src/vns/neighborhoods/shuffling.h
    src/vns/neighborhoods/smoothing.h src/dubins3d.h  src/core/structures/trajectories.h src/core/updates/updates.cpp
    src/core/updates/updates.h src/utils.cpp src/vns/factory.cpp src/vns/factory.h)

pybind11_add_module(uav_planning ${SOURCE_FILES} src/python_interface.cpp)

IF (BUILD_TESTING)
    find_package(Boost COMPONENTS unit_test_framework REQUIRED)
    add_executable(tests ${SOURCE_FILES} src/test/core/test_reversible_updates.h src/test/test_dubins.h 
    src/test/test_position_manipulation.h src/test/main_tests.cpp)
    target_link_libraries(tests ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
    add_test(tests tests)
ENDIF(BUILD_TESTING)
