# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ds4_msgs: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ids4_msgs:/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ds4_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg" NAME_WE)
add_custom_target(_ds4_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ds4_msgs" "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ds4_msgs
  "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(ds4_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ds4_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ds4_msgs_generate_messages ds4_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg" NAME_WE)
add_dependencies(ds4_msgs_generate_messages_cpp _ds4_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ds4_msgs_gencpp)
add_dependencies(ds4_msgs_gencpp ds4_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ds4_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ds4_msgs
  "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(ds4_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ds4_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ds4_msgs_generate_messages ds4_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg" NAME_WE)
add_dependencies(ds4_msgs_generate_messages_eus _ds4_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ds4_msgs_geneus)
add_dependencies(ds4_msgs_geneus ds4_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ds4_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ds4_msgs
  "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(ds4_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ds4_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ds4_msgs_generate_messages ds4_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg" NAME_WE)
add_dependencies(ds4_msgs_generate_messages_lisp _ds4_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ds4_msgs_genlisp)
add_dependencies(ds4_msgs_genlisp ds4_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ds4_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ds4_msgs
  "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ds4_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ds4_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ds4_msgs_generate_messages ds4_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg" NAME_WE)
add_dependencies(ds4_msgs_generate_messages_nodejs _ds4_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ds4_msgs_gennodejs)
add_dependencies(ds4_msgs_gennodejs ds4_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ds4_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ds4_msgs
  "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(ds4_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ds4_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ds4_msgs_generate_messages ds4_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/autoware/Autoware/src/drivers/ds4/ds4_msgs/msg/DS4.msg" NAME_WE)
add_dependencies(ds4_msgs_generate_messages_py _ds4_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ds4_msgs_genpy)
add_dependencies(ds4_msgs_genpy ds4_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ds4_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ds4_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ds4_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ds4_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ds4_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ds4_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
