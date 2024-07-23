cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_projector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  unity_msgs
  integration
  tf2_ros
  tf2_geometry_msgs
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
                 tf2_ros
                 tf2_geometry_msgs
  LIBRARIES border
)

catkin_python_setup()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(border
  src/border/Border.cpp
  src/border/StaticBorder.cpp
  src/border/StaticBorderManager.cpp
)


add_executable(projector_opt_node src/projector_opt.cpp)
target_link_libraries(projector_opt_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

