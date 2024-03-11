cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_projector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  unity_msgs
  integration
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 sensor_msgs
                 std_msgs
                 cv_bridge
                 image_transport
                 unity_msgs
                 integration
  LIBRARIES border
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(border
  src/border/Border.cpp
  src/border/DynamicBorder.cpp
  src/border/StaticBorder.cpp
  src/border/StaticBorderManager.cpp
)

add_executable(border_node src/safety_depthmap.cpp)
target_link_libraries(border_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(projector_opt_node src/projector_opt.cpp)
target_link_libraries(projector_opt_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS border
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)