cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_perception)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
                   roscpp
                   sensor_msgs
                   cv_bridge
                   tuni_whitegoods_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
                   sensor_msgs
                   cv_bridge
                   tuni_whitegoods_msgs
    LIBRARIES object_detection
  )


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(object_detection
  src/tuni_whitegoods_perception/object_detection_node.cpp
)
target_link_libraries(object_detection
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
)
add_dependencies(object_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})






