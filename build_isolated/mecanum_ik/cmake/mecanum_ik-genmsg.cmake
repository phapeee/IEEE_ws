# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mecanum_ik: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imecanum_ik:/home/ubuntu/IEEE_ws/src/mecanum_ik/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mecanum_ik_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg" NAME_WE)
add_custom_target(_mecanum_ik_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mecanum_ik" "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mecanum_ik
  "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mecanum_ik
)

### Generating Services

### Generating Module File
_generate_module_cpp(mecanum_ik
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mecanum_ik
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mecanum_ik_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mecanum_ik_generate_messages mecanum_ik_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg" NAME_WE)
add_dependencies(mecanum_ik_generate_messages_cpp _mecanum_ik_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mecanum_ik_gencpp)
add_dependencies(mecanum_ik_gencpp mecanum_ik_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mecanum_ik_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mecanum_ik
  "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mecanum_ik
)

### Generating Services

### Generating Module File
_generate_module_eus(mecanum_ik
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mecanum_ik
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mecanum_ik_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mecanum_ik_generate_messages mecanum_ik_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg" NAME_WE)
add_dependencies(mecanum_ik_generate_messages_eus _mecanum_ik_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mecanum_ik_geneus)
add_dependencies(mecanum_ik_geneus mecanum_ik_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mecanum_ik_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mecanum_ik
  "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mecanum_ik
)

### Generating Services

### Generating Module File
_generate_module_lisp(mecanum_ik
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mecanum_ik
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mecanum_ik_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mecanum_ik_generate_messages mecanum_ik_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg" NAME_WE)
add_dependencies(mecanum_ik_generate_messages_lisp _mecanum_ik_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mecanum_ik_genlisp)
add_dependencies(mecanum_ik_genlisp mecanum_ik_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mecanum_ik_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mecanum_ik
  "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mecanum_ik
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mecanum_ik
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mecanum_ik
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mecanum_ik_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mecanum_ik_generate_messages mecanum_ik_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg" NAME_WE)
add_dependencies(mecanum_ik_generate_messages_nodejs _mecanum_ik_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mecanum_ik_gennodejs)
add_dependencies(mecanum_ik_gennodejs mecanum_ik_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mecanum_ik_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mecanum_ik
  "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mecanum_ik
)

### Generating Services

### Generating Module File
_generate_module_py(mecanum_ik
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mecanum_ik
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mecanum_ik_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mecanum_ik_generate_messages mecanum_ik_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/IEEE_ws/src/mecanum_ik/msg/vector4_msg.msg" NAME_WE)
add_dependencies(mecanum_ik_generate_messages_py _mecanum_ik_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mecanum_ik_genpy)
add_dependencies(mecanum_ik_genpy mecanum_ik_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mecanum_ik_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mecanum_ik)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mecanum_ik
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(mecanum_ik_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mecanum_ik)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mecanum_ik
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(mecanum_ik_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mecanum_ik)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mecanum_ik
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(mecanum_ik_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mecanum_ik)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mecanum_ik
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(mecanum_ik_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mecanum_ik)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mecanum_ik\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mecanum_ik
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(mecanum_ik_generate_messages_py geometry_msgs_generate_messages_py)
endif()
