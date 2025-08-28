cmake_minimum_required(VERSION 3.16)
project(tuni_whitegoods_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)


# Add service files
set(srv_files
  "srv/Transform3DToPixel.srv"
  "srv/TransformPixelTo3D.srv"
  "srv/TransformRobotCameraCoordinates.srv"
  "srv/TransformPixelToProjection.srv"

)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()