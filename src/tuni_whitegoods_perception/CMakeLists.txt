cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_perception)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_filters
  unity_msgs
  scene_calibration
  pcl_conversions
  pcl_ros
  integration
  tf2
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
  tuni_whitegoods_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
                   roscpp
                   sensor_msgs
                   std_msgs
                   cv_bridge
                   image_transport
                   unity_msgs
                   message_filters
                   integration
                   pcl_conversions
                   pcl_ros
                   tf2
                   tf2_msgs
                   tf2_ros
                   tf2_sensor_msgs
                   tuni_whitegoods_msgs
    LIBRARIES depth_interface
  )


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(depth_interface src/tuni_whitegoods_perception/depth_interface.cpp)
target_link_libraries(depth_interface
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
)

add_executable(depth_interface_node src/depth_interface_node.cpp)
target_link_libraries(depth_interface_node
  depth_interface
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
)

install(TARGETS depth_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
