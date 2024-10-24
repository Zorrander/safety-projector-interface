cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_transformations)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


find_package(catkin REQUIRED
  roscpp
  tf2_ros
  tuni_whitegoods_msgs
)

find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
   CATKIN_DEPENDS roscpp 
                  tuni_whitegoods_msgs 
                  tf2_ros
   LIBRARIES pinhole_camera
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(transform_camera_point_server src/transform_camera_point_server.cpp)
target_link_libraries(transform_camera_point_server 
      ${catkin_LIBRARIES}
      yaml-cpp
)
add_dependencies(transform_camera_point_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(transform_projector_point_server src/transform_projector_point_server.cpp)
target_link_libraries(transform_projector_point_server 
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
)
add_dependencies(transform_projector_point_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(transform_robot_point_server src/transform_robot_point_server.cpp)
target_link_libraries(transform_robot_point_server 
      ${catkin_LIBRARIES}
)
add_dependencies(transform_robot_point_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(transform_table_point_server src/transform_table_point_server.cpp)
target_link_libraries(transform_table_point_server 
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
)
add_dependencies(transform_table_point_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})