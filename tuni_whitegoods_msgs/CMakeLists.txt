cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED
    message_generation
    geometry_msgs
    sensor_msgs
    std_msgs
)

add_message_files(
  FILES
  Projection.msg
  DynamicArea.msg
  HandsState.msg
  RobotViewElement.msg
  ZoneState.msg
  Button.msg
  StaticBorder.msg
)


add_service_files(
  FILES
  Transform3DToPixel.srv
  TransformPixelTo3D.srv
  TransformMovingTable.srv
  TransformPixelToProjection.srv
  TransformRobotCameraCoordinates.srv
  DisplayAreaStates.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)
