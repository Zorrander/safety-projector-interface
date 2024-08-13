cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_perception)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
                   roscpp
                   sensor_msgs
                   cv_bridge
                   tuni_whitegoods_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp
                   sensor_msgs
                   cv_bridge
                   tuni_whitegoods_msgs
  )


include_directories(${catkin_INCLUDE_DIRS})


#add_executable(object_detection_node src/object_detection_node.cpp)
#target_link_libraries(object_detection_node
#  ${catkin_LIBRARIES} 
#  ${OpenCV_LIBS} 
#)




