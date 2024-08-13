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
target_link_libraries(project_view
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
)
add_dependencies(project_view ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(robot_view
  src/tuni_whitegoods_view/robot_view.cpp
)
target_link_libraries(robot_view
  ${catkin_LIBRARIES} 
)
add_dependencies(robot_view ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(camera_view
  src/tuni_whitegoods_view/camera_view.cpp
)
target_link_libraries(camera_view
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
)
add_dependencies(camera_view ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



install(TARGETS project_view robot_view camera_view
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/tuni_whitegoods_view/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)