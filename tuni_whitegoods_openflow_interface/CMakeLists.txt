cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_openflow_interface)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  integration
  tuni_whitegoods_controller
)


#find_package(OpenGL REQUIRED)
#find_package(glfw3 3.3 REQUIRED)
#find_package(GLEW REQUIRED)


#include(FetchContent)
#FetchContent_Declare(imgui
#        GIT_REPOSITORY https://github.com/ocornut/imgui.git
#        GIT_TAG v1.89.8-docking
#        CONFIGURE_COMMAND ""
#        BUILD_COMMAND ""
#        )
#FetchContent_MakeAvailable(imgui)

catkin_package(
  CATKIN_DEPENDS roscpp 
                 std_msgs
                 sensor_msgs 
                 actionlib 
                 actionlib_msgs
                 integration
                 tuni_whitegoods_controller
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  #${OPENGL_INCLUDE_DIRS}
)

#file(GLOB USER_INTERFACE_SOURCES "src/user_interface/*.cpp")
file(GLOB PROJECTOR_INTERFACE_SOURCES "src/projector_interface/*.cpp")
file(GLOB ROBOT_INTERFACE_SOURCES "src/robot_interface/*.cpp")

#add_library(imgui 
#    ${imgui_SOURCE_DIR}/imgui.cpp
#    ${imgui_SOURCE_DIR}/imgui_draw.cpp
#    ${imgui_SOURCE_DIR}/imgui_tables.cpp
#    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
#    ${imgui_SOURCE_DIR}/imgui_demo.cpp
#    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
#    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
#)
#target_include_directories(imgui PUBLIC
#  ${imgui_SOURCE_DIR} 
#  ${imgui_SOURCE_DIR}/backends
#)

#target_link_libraries(imgui 
#  glfw 
#  OpenGL::GL
#)

add_library(projector_interface ${PROJECTOR_INTERFACE_SOURCES})
add_dependencies(projector_interface
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(projector_interface
  ${catkin_LIBRARIES}
)

#add_library(user_interface ${USER_INTERFACE_SOURCES})
#add_dependencies(user_interface
#  ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(user_interface
#  ${catkin_LIBRARIES}
#  glfw 
#  OpenGL::GL 
#  GLEW::GLEW
#  imgui
#)

add_executable(smart_interface_node src/smart_interface_node.cpp)
target_link_libraries(smart_interface_node 
                        projector_interface
                        #user_interface
                        ${catkin_LIBRARIES}
                      )
add_dependencies(smart_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_library(robot_interface ${ROBOT_INTERFACE_SOURCES})
# add_executable(robot_motion_node src/robot_motion_node.cpp)
# target_link_libraries(robot_motion_node
#                        robot_interface 
#                        ${catkin_LIBRARIES}
#                      )
# add_dependencies(robot_motion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
