cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_msgs)


find_package(catkin REQUIRED
    message_generation
    geometry_msgs
)


add_service_files(
  FILES
  Transform3DToPixel.srv
  TransformPixelTo3D.srv
  TransformPixelToProjection.srv
  TransformRobotCameraCoordinates.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)
