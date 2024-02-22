cmake_minimum_required(VERSION 3.0.2)
project(projector_interface)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  integration
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  unity_msgs
)

find_package(OpenCV 4 REQUIRED)


catkin_package(
  CATKIN_DEPENDS roscpp 
                 std_msgs
                 sensor_msgs 
                 actionlib 
                 actionlib_msgs
                 geometry_msgs
                 cv_bridge
                 image_transport
                 unity_msgs
                 message_filters
                 integration
)

###########
## Build ##
###########

include_directories(
  include
  integration
  ${OpenCV_LIBS}
  ${catkin_INCLUDE_DIRS}
  ${actionlib_INCLUDE_DIRS}
)

add_library(border
  src/border/Border.cpp
  src/border/DynamicBorder.cpp
  src/border/StaticBorder.cpp
  src/border/StaticBorderManager.cpp
)

## Declare a C++ library
add_library(static_border_booking
    src/projector_interface/book_operator_static_border_server.cpp
    src/projector_interface/book_robot_static_border_server.cpp
    src/projector_interface/release_operator_static_border_server.cpp
    src/projector_interface/release_robot_static_border_server.cpp
 )

 add_library(dynamic_border
    src/projector_interface/safety_border_server.cpp
 )


 add_library(smart_interface
    src/projector_interface/button_projection_server.cpp
    src/projector_interface/instruction_projection_server.cpp
    src/projector_interface/button_color_server.cpp
    src/projector_interface/unset_projection_server.cpp
    src/projector_interface/user_interface_server.cpp
 )

 add_library(robot_motion
  src/projector_interface/move_robot_server.cpp
 )


add_executable(border_node src/safety_depthmap.cpp)
target_link_libraries(border_node
  ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS} 
)


add_executable(dynamic_border_node src/dynamic_border_node.cpp)
target_link_libraries(dynamic_border_node 
                        dynamic_border
                        border
                        ${catkin_LIBRARIES}
                      )

add_executable(static_border_booking_node src/static_border_booking_node.cpp)
target_link_libraries(static_border_booking_node 
                        static_border_booking
                        border
                        ${catkin_LIBRARIES}
                      )
add_dependencies(static_border_booking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_dependencies(dynamic_border_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(smart_interface_node src/smart_interface_node.cpp)
target_link_libraries(smart_interface_node 
                        smart_interface
                        ${catkin_LIBRARIES}
                      )
add_dependencies(smart_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robot_motion_node src/robot_motion_node.cpp)
target_link_libraries(robot_motion_node 
                        robot_motion
                        ${catkin_LIBRARIES}
                      )
add_dependencies(robot_motion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_projector_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
