cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tuni_whitegoods_msgs
  tuni_whitegoods_projector_interface
  tuni_whitegoods_view
  tuni_whitegoods_perception
)
find_package(yaml-cpp REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
                 tuni_whitegoods_msgs
                 tuni_whitegoods_projector_interface
                 tuni_whitegoods_view
                 tuni_whitegoods_perception
  LIBRARIES projector_interface_controller
)


include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(projector_interface_controller
  src/tuni_whitegoods_controller/projector_interface_controller.cpp
)

## Specify libraries to link a library or executable target against
 target_link_libraries(projector_interface_controller
   ${catkin_LIBRARIES}
   yaml-cpp
 )


install(TARGETS projector_interface_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
