cmake_minimum_required(VERSION 3.5)
project(planning_cpp)

find_package(pybind11 REQUIRED)

set(CMAKE_CXX_STANDARD 11)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated")
endif()

set(SOURCE_FILES src/main.cpp src/dubins.cpp src/dubins.h src/trajectory.h)
add_executable(planning_cpp ${SOURCE_FILES})