cmake_minimum_required(VERSION 3.5)
project(planning_cpp)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/pybind11")
    add_subdirectory("pybind11")
else ()
    find_package(pybind11 2.2.1 REQUIRED)
endif ()

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused-parameter -Wdelete-non-virtual-dtor -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

#set(CMAKE_CXX_VISIBILITY_PRESET hidden)

set(CORE_SOURCE_FILES
        src/ext/json.hpp
        src/ext/optional.hpp
        src/ext/ThreadPool.hpp
        src/cpp_py_utils.hpp
        src/utils.cpp
        src/utils.hpp
        src/ext/dubins.cpp
        src/ext/dubins.h
        src/core/dubins3d.cpp
        src/core/dubins3d.hpp
        src/core/dubinswind.cpp
        src/core/dubinswind.hpp
        src/core/fire_data.cpp
        src/core/fire_data.hpp
        src/core/raster.hpp
        src/core/trajectories.hpp
        src/core/trajectory.cpp
        src/core/trajectory.hpp
        src/core/uav.cpp
        src/core/uav.hpp
        src/core/updates/updates.cpp
        src/core/updates/updates.hpp
        src/core/waypoint.hpp
        src/core/SharedQueue.hpp)

set(PLANNING_SOURCE_FILES
        src/vns/factory.cpp
        src/vns/factory.hpp
        src/vns/plan.hpp
        src/vns/plan.cpp
        src/vns/neighborhoods/dubins_optimization.hpp
        src/vns/neighborhoods/insertions.hpp
        src/vns/neighborhoods/moves.hpp
        src/vns/neighborhoods/shuffling.hpp
        src/vns/neighborhoods/smoothing.hpp
        src/vns/visibility.hpp
        src/vns/vns_interface.hpp
        src/vns/utility.hpp
        src/vns/utility.cpp)

set(NEPTUSINTERFACE_SOURCE_FILES
        src/ext/coordinates.hpp
        src/neptus/imc_comm.cpp
        src/neptus/imc_comm.hpp
        src/neptus/imc_message_factories.hpp
        src/neptus/geography.hpp
        src/neptus/saop_to_dune.hpp
        src/neptus/saop_server.cpp
        src/neptus/saop_server.hpp
        )

set(MAPPING_SOURCE_FILES
        src/firemapping/ghostmapper.hpp
        )

add_subdirectory("IMC")

add_library(saop SHARED
        ${CORE_SOURCE_FILES}
        ${PLANNING_SOURCE_FILES}
        ${MAPPING_SOURCE_FILES}
        ${NEPTUSINTERFACE_SOURCE_FILES}
        )

find_package(GDAL REQUIRED)

set(Boost_USE_MULTITHREADED ON)
add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(Boost COMPONENTS log log_setup thread date_time filesystem system REQUIRED)

find_package(ZLIB REQUIRED)

target_link_libraries(saop
        imc
        ${CMAKE_THREAD_LIBS_INIT}
        ${GDAL_LIBRARY}
        ${ZLIB_LIBRARIES}
        ${Boost_LIBRARIES}
        )

# Pybind modules
pybind11_add_module(uav_planning
        src/planning_py.cpp
        src/saop_logging.cpp
        src/saop_logging.hpp)
target_link_libraries(uav_planning PUBLIC saop)

pybind11_add_module(firemapping
        src/firemapping_py.cpp
        src/saop_logging.cpp
        src/saop_logging.hpp)
target_link_libraries(firemapping PUBLIC saop)

pybind11_add_module(neptus_interface
        src/neptus_py.cpp
        src/saop_logging.cpp
        src/saop_logging.hpp)
target_link_libraries(neptus_interface PUBLIC saop)

IF (BUILD_TESTING)
    find_package(Boost COMPONENTS unit_test_framework REQUIRED)
    add_executable(tests
            src/test/core/test_reversible_updates.hpp
            src/test/test_dubins.hpp
            src/test/test_dubinswind.hpp
            src/test/test_position_manipulation.hpp
            src/test/main_tests.cpp
            )
    target_link_libraries(tests
            saopcore
            ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
            )
    add_test(tests tests)

    add_executable(imc_server
            src/test/main_imc_server.cpp
            )
    target_link_libraries(imc_server
            libimc
            ${Boost_LIBRARIES}
            )

    add_executable(saop_to_dune
            src/test/main_saop_to_dune.cpp
            src/neptus/imc_message_factories.hpp
            )
    target_link_libraries(saop_to_dune
            libimc
            ${Boost_LIBRARIES}
            )

    add_executable(imc_client
            src/test/main_imc_client.cpp
            )
    target_link_libraries(imc_client
            libimc
            ${Boost_LIBRARIES}
            )
ENDIF (BUILD_TESTING)
