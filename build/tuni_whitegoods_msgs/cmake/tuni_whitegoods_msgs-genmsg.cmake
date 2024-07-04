# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tuni_whitegoods_msgs: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tuni_whitegoods_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv" NAME_WE)
add_custom_target(_tuni_whitegoods_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tuni_whitegoods_msgs" "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(tuni_whitegoods_msgs
  "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tuni_whitegoods_msgs
)

### Generating Module File
_generate_module_cpp(tuni_whitegoods_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tuni_whitegoods_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tuni_whitegoods_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tuni_whitegoods_msgs_generate_messages tuni_whitegoods_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv" NAME_WE)
add_dependencies(tuni_whitegoods_msgs_generate_messages_cpp _tuni_whitegoods_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tuni_whitegoods_msgs_gencpp)
add_dependencies(tuni_whitegoods_msgs_gencpp tuni_whitegoods_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tuni_whitegoods_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(tuni_whitegoods_msgs
  "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tuni_whitegoods_msgs
)

### Generating Module File
_generate_module_eus(tuni_whitegoods_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tuni_whitegoods_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tuni_whitegoods_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tuni_whitegoods_msgs_generate_messages tuni_whitegoods_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv" NAME_WE)
add_dependencies(tuni_whitegoods_msgs_generate_messages_eus _tuni_whitegoods_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tuni_whitegoods_msgs_geneus)
add_dependencies(tuni_whitegoods_msgs_geneus tuni_whitegoods_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tuni_whitegoods_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(tuni_whitegoods_msgs
  "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tuni_whitegoods_msgs
)

### Generating Module File
_generate_module_lisp(tuni_whitegoods_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tuni_whitegoods_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tuni_whitegoods_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tuni_whitegoods_msgs_generate_messages tuni_whitegoods_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv" NAME_WE)
add_dependencies(tuni_whitegoods_msgs_generate_messages_lisp _tuni_whitegoods_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tuni_whitegoods_msgs_genlisp)
add_dependencies(tuni_whitegoods_msgs_genlisp tuni_whitegoods_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tuni_whitegoods_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(tuni_whitegoods_msgs
  "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tuni_whitegoods_msgs
)

### Generating Module File
_generate_module_nodejs(tuni_whitegoods_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tuni_whitegoods_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tuni_whitegoods_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tuni_whitegoods_msgs_generate_messages tuni_whitegoods_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv" NAME_WE)
add_dependencies(tuni_whitegoods_msgs_generate_messages_nodejs _tuni_whitegoods_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tuni_whitegoods_msgs_gennodejs)
add_dependencies(tuni_whitegoods_msgs_gennodejs tuni_whitegoods_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tuni_whitegoods_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(tuni_whitegoods_msgs
  "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tuni_whitegoods_msgs
)

### Generating Module File
_generate_module_py(tuni_whitegoods_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tuni_whitegoods_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tuni_whitegoods_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tuni_whitegoods_msgs_generate_messages tuni_whitegoods_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odin3/safety-projector-interface/src/tuni_whitegoods_msgs/srv/PointsDepthMap.srv" NAME_WE)
add_dependencies(tuni_whitegoods_msgs_generate_messages_py _tuni_whitegoods_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tuni_whitegoods_msgs_genpy)
add_dependencies(tuni_whitegoods_msgs_genpy tuni_whitegoods_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tuni_whitegoods_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tuni_whitegoods_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tuni_whitegoods_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(tuni_whitegoods_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tuni_whitegoods_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tuni_whitegoods_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(tuni_whitegoods_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tuni_whitegoods_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tuni_whitegoods_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(tuni_whitegoods_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tuni_whitegoods_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tuni_whitegoods_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(tuni_whitegoods_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tuni_whitegoods_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tuni_whitegoods_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tuni_whitegoods_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(tuni_whitegoods_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
