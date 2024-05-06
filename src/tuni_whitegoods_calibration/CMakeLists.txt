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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package( k4a REQUIRED)
find_package( k4arecord REQUIRED)
find_package( k4abt REQUIRED )
set(K4A_LIBS k4a::k4a k4a::k4arecord)


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

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${K4A_LIBS}
)


#add_executable(points_depth src/server_points.cpp)
#target_link_libraries(points_depth 
#                        ${catkin_LIBRARIES} 
#                        ${K4A_LIBS})


#add_executable(points_wrapper src/wrapper.cpp)
#target_link_libraries(points_wrapper 
#                        ${catkin_LIBRARIES})
