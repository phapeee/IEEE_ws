# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "april_tag_detection: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(april_tag_detection_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv" NAME_WE)
add_custom_target(_april_tag_detection_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "april_tag_detection" "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv" "geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(april_tag_detection
  "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/april_tag_detection
)

### Generating Module File
_generate_module_cpp(april_tag_detection
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/april_tag_detection
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(april_tag_detection_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(april_tag_detection_generate_messages april_tag_detection_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv" NAME_WE)
add_dependencies(april_tag_detection_generate_messages_cpp _april_tag_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(april_tag_detection_gencpp)
add_dependencies(april_tag_detection_gencpp april_tag_detection_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS april_tag_detection_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(april_tag_detection
  "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/april_tag_detection
)

### Generating Module File
_generate_module_eus(april_tag_detection
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/april_tag_detection
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(april_tag_detection_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(april_tag_detection_generate_messages april_tag_detection_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv" NAME_WE)
add_dependencies(april_tag_detection_generate_messages_eus _april_tag_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(april_tag_detection_geneus)
add_dependencies(april_tag_detection_geneus april_tag_detection_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS april_tag_detection_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(april_tag_detection
  "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/april_tag_detection
)

### Generating Module File
_generate_module_lisp(april_tag_detection
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/april_tag_detection
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(april_tag_detection_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(april_tag_detection_generate_messages april_tag_detection_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv" NAME_WE)
add_dependencies(april_tag_detection_generate_messages_lisp _april_tag_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(april_tag_detection_genlisp)
add_dependencies(april_tag_detection_genlisp april_tag_detection_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS april_tag_detection_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(april_tag_detection
  "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/april_tag_detection
)

### Generating Module File
_generate_module_nodejs(april_tag_detection
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/april_tag_detection
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(april_tag_detection_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(april_tag_detection_generate_messages april_tag_detection_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv" NAME_WE)
add_dependencies(april_tag_detection_generate_messages_nodejs _april_tag_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(april_tag_detection_gennodejs)
add_dependencies(april_tag_detection_gennodejs april_tag_detection_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS april_tag_detection_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(april_tag_detection
  "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/april_tag_detection
)

### Generating Module File
_generate_module_py(april_tag_detection
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/april_tag_detection
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(april_tag_detection_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(april_tag_detection_generate_messages april_tag_detection_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/april_tag_detection/srv/GetAprilTag.srv" NAME_WE)
add_dependencies(april_tag_detection_generate_messages_py _april_tag_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(april_tag_detection_genpy)
add_dependencies(april_tag_detection_genpy april_tag_detection_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS april_tag_detection_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/april_tag_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/april_tag_detection
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(april_tag_detection_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/april_tag_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/april_tag_detection
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(april_tag_detection_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/april_tag_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/april_tag_detection
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(april_tag_detection_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/april_tag_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/april_tag_detection
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(april_tag_detection_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/april_tag_detection)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/april_tag_detection\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/april_tag_detection
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(april_tag_detection_generate_messages_py geometry_msgs_generate_messages_py)
endif()
