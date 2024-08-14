cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_view)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  LIBRARIES view
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(view
  src/tuni_whitegoods_view/project_view.cpp
  src/tuni_whitegoods_view/robot_view.cpp
  src/tuni_whitegoods_view/camera_view.cpp
  src/tuni_whitegoods_view/view.cpp
)
target_link_libraries(view
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
)
add_dependencies(view ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



install(TARGETS view
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/tuni_whitegoods_view/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)