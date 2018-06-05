cmake_minimum_required(VERSION 2.8.11)

project(gui_test)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
#set(CMAKE_AUTORCC ON) # Resource files
#set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets Core)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


get_target_property(QtCore_location Qt5::Core LOCATION)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ../../../include/ ../)
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} gui_test.cpp)
target_link_libraries(${PROJECT_NAME} gui ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
qt5_use_modules(${PROJECT_NAME} Widgets Core)
