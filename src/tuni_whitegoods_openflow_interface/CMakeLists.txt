cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_openflow_interface)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  integration
  tuni_whitegoods_projector
)

catkin_package(
  CATKIN_DEPENDS roscpp 
                 std_msgs
                 sensor_msgs 
                 actionlib 
                 actionlib_msgs
                 integration
                 tuni_whitegoods_projector
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  integration
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

file(GLOB PROJECTOR_INTERFACE_SOURCES "src/projector_interface/*.cpp")


add_library(projector_interface ${PROJECTOR_INTERFACE_SOURCES})


add_executable(static_border_booking_node src/static_border_booking_node.cpp)
target_link_libraries(static_border_booking_node 
                        projector_interface
                        ${catkin_LIBRARIES}
                      )
add_dependencies(static_border_booking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(smart_interface_node src/smart_interface_node.cpp)
target_link_libraries(smart_interface_node 
                        projector_interface
                        ${catkin_LIBRARIES}
                      )
add_dependencies(smart_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

