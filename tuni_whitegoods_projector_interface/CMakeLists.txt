cmake_minimum_required(VERSION 3.0.2)
project(tuni_whitegoods_projector_interface)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tuni_whitegoods_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tuni_whitegoods_msgs
  LIBRARIES hmi_model
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(hmi_model
  src/tuni_whitegoods_projector_interface/button.cpp
  src/tuni_whitegoods_projector_interface/static_border.cpp
  src/tuni_whitegoods_projector_interface/instruction.cpp
  #src/tuni_whitegoods_projector_interface/ui.cpp
  src/tuni_whitegoods_projector_interface/model.cpp
  src/tuni_whitegoods_projector_interface/display_area.cpp
  src/tuni_whitegoods_projector_interface/hand.cpp
)
target_link_libraries(hmi_model
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
)
add_dependencies(hmi_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



install(TARGETS hmi_model
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

