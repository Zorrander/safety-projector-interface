cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_perception)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


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
)

find_package(OpenCV 4 REQUIRED)
find_package(k4a REQUIRED)
find_package(k4arecord REQUIRED)
find_package(k4abt REQUIRED )
set(K4A_LIBS k4a::k4a k4a::k4arecord)


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
  ${K4A_LIBS}
)

#add_library(border_tracking src/tuni_whitegoods_perception/border_tracking.cpp)
#target_link_libraries(border_tracking
#  ${catkin_LIBRARIES} 
#  ${OpenCV_LIBS}
#  ${K4A_LIBS}
#)

add_executable(depth_interface_node src/depth_interface_node.cpp)
target_link_libraries(depth_interface_node
  depth_interface
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${K4A_LIBS}
)

add_executable(interaction_detection_node src/detect_interaction.cpp)
target_link_libraries(interaction_detection_node
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
  ${K4A_LIBS}
)

#add_executable(border_violation_tracking_node src/border_violation_tracking_node.cpp)
#target_link_libraries(border_violation_tracking_node
#  border_tracking
#  ${catkin_LIBRARIES} 
#  ${OpenCV_LIBS}
#  ${K4A_LIBS}
#)

#add_executable(pcl_fusion_node src/pcl_fusion_node.cpp)
#target_link_libraries(pcl_fusion_node
#   ${catkin_LIBRARIES} 
#   ${OpenCV_LIBS}
#   ${K4A_LIBS}
#)

install(TARGETS depth_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
