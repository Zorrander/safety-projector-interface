cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_view)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  integration
  tf2_ros
  tf2_geometry_msgs
  tuni_whitegoods_msgs

)


find_package(OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 sensor_msgs
                 std_msgs
                 cv_bridge
                 image_transport
                 integration
                 tf2_ros
                 tf2_geometry_msgs
                 tuni_whitegoods_msgs
  LIBRARIES project_view
            robot_view
            camera_view
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(project_view
  src/tuni_whitegoods_view/project_view.cpp
)

add_library(robot_view
  src/tuni_whitegoods_view/robot_view.cpp
)

add_library(camera_view
  src/tuni_whitegoods_view/camera_view.cpp
)



