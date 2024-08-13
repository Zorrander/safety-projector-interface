cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_projector_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)


find_package(OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  LIBRARIES model
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(button
  src/tuni_whitegoods_projector_interface/button.cpp
)

add_library(static_border
  src/tuni_whitegoods_projector_interface/staticBorder.cpp
)

add_library(instruction
  src/tuni_whitegoods_projector_interface/instruction.cpp
)

#add_library(ui
#  src/tuni_whitegoods_projector_interface/ui.cpp
#)

add_library(model
  src/tuni_whitegoods_projector_interface/model.cpp
)


install(TARGETS model
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

