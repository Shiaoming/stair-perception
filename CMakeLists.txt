cmake_minimum_required(VERSION 3.1.0)

project(stair)

set(CMAKE_CXX_STANDARD 14)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

include_directories("${PROJECT_INCLUDE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}")

add_subdirectory(modules/gui)
add_subdirectory(modules/imu)
add_subdirectory(modules/k2g)
add_subdirectory(modules/sender)
add_subdirectory(modules/stairperception)
add_subdirectory(test)