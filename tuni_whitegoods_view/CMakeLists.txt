cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_view)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  integration
  tf2
  tf2_ros
  tf2_geometry_msgs
  tuni_whitegoods_msgs
  tuni_whitegoods_projector_interface

)

find_package(OpenCV 4 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(GLEW REQUIRED)

include(FetchContent)
FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.89.8-docking
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        )
FetchContent_MakeAvailable(imgui)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 sensor_msgs
                 std_msgs
                 cv_bridge
                 image_transport
                 integration
                 tf2
                 tf2_ros
                 tf2_geometry_msgs
                 tuni_whitegoods_msgs
                 tuni_whitegoods_projector_interface
  LIBRARIES view
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
)


add_library(imgui 
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
  ${imgui_SOURCE_DIR} 
  ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui 
  glfw 
  OpenGL::GL
)

add_library(view
  src/tuni_whitegoods_view/view.cpp
  src/tuni_whitegoods_view/project_view.cpp
  src/tuni_whitegoods_view/robot_view.cpp
  src/tuni_whitegoods_view/camera_view.cpp
  
)

target_link_libraries(view
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  glfw 
  OpenGL::GL 
  GLEW::GLEW
  imgui
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