cmake_minimum_required(VERSION 2.8.11)

project(stair_perception_gui)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
#set(CMAKE_AUTORCC ON) # Resource files
#set(CMAKE_AUTOUIC ON) # UI files

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

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

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
aux_source_directory(. DIR_SRCS)

get_target_property(QtCore_location Qt5::Core LOCATION)

include_directories(../include/ ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 
                    ../modules/gui/
                    ../modules/imu/
                    ../modules/k2g/
                    ../modules/sender/
        ../modules/stairperception/)
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gui imu k2g sender stairperception)

qt5_use_modules(${PROJECT_NAME} Widgets Core)
