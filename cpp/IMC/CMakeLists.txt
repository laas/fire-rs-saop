cmake_minimum_required(VERSION 3.0)
project (IMC_INTERFACE)

set (PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


add_library(IMC_INTERFACE INTERFACE)
target_include_directories(IMC_INTERFACE INTERFACE IMC/Base IMC/Spec)