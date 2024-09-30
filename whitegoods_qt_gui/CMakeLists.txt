cmake_minimum_required(VERSION 3.0.2)

project(whitegoods_qt_gui)

## Find required ROS and Qt packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui UiTools)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)

## Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
  )

add_executable(${PROJECT_NAME}_node
    src/main.cpp
    resource/main_window.ui
    src/whitegoods_qt_gui/main_window.cpp include/whitegoods_qt_gui/main_window.h
)

target_link_libraries(${PROJECT_NAME}_node
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::UiTools
  yaml-cpp
  ${catkin_LIBRARIES}
)


