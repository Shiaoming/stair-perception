cmake_minimum_required(VERSION 2.8.11)

project(stairperception)

# Check for c++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

option(WITH_SERIALIZATION "adds support for boost serialization" ON)
if(${WITH_SERIALIZATION})
    find_package(Boost REQUIRED)
    include_directories(${INCLUDE})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_SERIALIZATION")
    set(BOOST_LIBS "boost_serialization")
endif()

find_package(PCL REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
aux_source_directory(. DIR_SRCS)

include_directories(../../include/ ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
