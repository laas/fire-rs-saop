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

set(SOURCE_FILES src/ext/dubins.cpp src/ext/dubins.h src/ext/json.hpp src/core/structures/trajectory.hpp
        src/core/structures/uav.hpp src/core/structures/waypoint.hpp src/raster.hpp src/vns/visibility.hpp src/ext/optional.hpp
        src/utils.hpp src/vns/neighborhoods/dubins_optimization.hpp src/vns/neighborhoods/insertions.hpp src/vns/vns_interface.hpp
        src/vns/plan.hpp src/vns/fire_data.hpp src/vns/neighborhoods/moves.hpp src/vns/neighborhoods/shuffling.hpp
        src/vns/neighborhoods/smoothing.hpp src/dubins3d.hpp src/core/structures/trajectories.hpp src/core/updates/updates.cpp
        src/core/updates/updates.hpp src/utils.cpp src/vns/factory.cpp src/vns/factory.hpp)

pybind11_add_module(uav_planning ${SOURCE_FILES} src/python_interface.cpp)

IF (BUILD_TESTING)
    find_package(Boost COMPONENTS unit_test_framework REQUIRED)
    add_executable(tests ${SOURCE_FILES} src/test/core/test_reversible_updates.hpp src/test/test_dubins.hpp
            src/test/test_position_manipulation.hpp src/test/main_tests.cpp)
    target_link_libraries(tests ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
    add_test(tests tests)
ENDIF(BUILD_TESTING)

IF (BUILD_IMC)
   add_subdirectory("IMC")
   find_package(Boost COMPONENTS system thread REQUIRED)
   find_package(Threads)

   add_executable(imc_server src/test/main_imc_server.cpp IMC/Base/Factory.cpp)
   target_link_libraries(imc_server IMC_INTERFACE ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

   add_executable(imc_client src/test/main_imc_client.cpp IMC/Base/Factory.cpp)
   target_link_libraries(imc_client IMC_INTERFACE ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
ENDIF (BUILD_IMC)
