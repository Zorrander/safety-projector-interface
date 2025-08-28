cmake_minimum_required(VERSION 3.16)
project(tuni_whitegoods_transformations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 and build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Messages
find_package(tuni_whitegoods_msgs REQUIRED)

# Include headers
include_directories(
  include
)

# Camera Transform Component
add_library(camera_transform_component SHARED
  src/camera_transform_node.cpp
)
ament_target_dependencies(camera_transform_component
  rclcpp
  rclcpp_components
  tuni_whitegoods_msgs
  yaml-cpp
)
target_link_libraries(camera_transform_component yaml-cpp)

rclcpp_components_register_nodes(camera_transform_component
  "tuni_whitegoods_transformations::CameraTransformNode"
)

# Projector Transform Component
add_library(projector_transform_component SHARED
  src/projector_transform_node.cpp
)
ament_target_dependencies(projector_transform_component
  rclcpp
  rclcpp_components
  OpenCV
  tuni_whitegoods_msgs
)
rclcpp_components_register_nodes(projector_transform_component
  "tuni_whitegoods_transformations::ProjectorTransformNode"
)

# TF Transform Component
add_library(tf_transform_component SHARED
  src/tf_transform_node.cpp
)
ament_target_dependencies(tf_transform_component
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  tuni_whitegoods_msgs
)
rclcpp_components_register_nodes(tf_transform_component
  "tuni_whitegoods_transformations::TFTransformNode"
)

# Install targets
install(TARGETS
  camera_transform_component
  projector_transform_component
  tf_transform_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Package export
ament_export_dependencies(
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  yaml-cpp
  tuni_whitegoods_msgs
)

ament_package()
