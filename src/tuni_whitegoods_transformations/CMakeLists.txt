cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_transformations)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED
  roscpp
  tf2_ros
  tuni_whitegoods_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES pinhole_camera
   CATKIN_DEPENDS roscpp tuni_whitegoods_msgs tf2_ros
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(pinhole_camera src/tuni_whitegoods_transformations/pinhole_camera.cpp)


add_executable(transform_camera_point_server src/transform_camera_point_server.cpp)
target_link_libraries(transform_camera_point_server 
      pinhole_camera
      ${catkin_LIBRARIES}
)


add_executable(transform_projector_point_server src/transform_projector_point_server.cpp)
target_link_libraries(transform_projector_point_server 
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
)


add_executable(transform_robot_point_server src/transform_robot_point_server.cpp)
target_link_libraries(transform_robot_point_server 
      ${catkin_LIBRARIES}
)


install(TARGETS pinhole_camera
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)



