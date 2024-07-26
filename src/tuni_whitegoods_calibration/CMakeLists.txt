cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_calibration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
  tuni_whitegoods_perception
  tuni_whitegoods_msgs
  pcl_conversions
  pcl_ros
  integration
  tf2
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS message_runtime
                   roscpp
                   sensor_msgs
                   std_msgs
                   cv_bridge
                   image_transport
                   unity_msgs
                   message_filters
                   tuni_whitegoods_perception
                   tuni_whitegoods_msgs
                   integration
                   pcl_conversions
                   pcl_ros
                   tf2
                   tf2_msgs
                   tf2_ros
                   tf2_sensor_msgs
  )


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



