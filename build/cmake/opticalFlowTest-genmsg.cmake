# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "opticalFlowTest: 1 messages, 0 services")

set(MSG_I_FLAGS "-IopticalFlowTest:/home/liuxiaobu/catkin_drone/src/test_optical_flow-wrz/msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(opticalFlowTest_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/liuxiaobu/catkin_drone/src/test_optical_flow-wrz/msg/optical_flow_rad.msg" NAME_WE)
add_custom_target(_opticalFlowTest_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opticalFlowTest" "/home/liuxiaobu/catkin_drone/src/test_optical_flow-wrz/msg/optical_flow_rad.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(opticalFlowTest
  "/home/liuxiaobu/catkin_drone/src/test_optical_flow-wrz/msg/optical_flow_rad.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opticalFlowTest
)

### Generating Services

### Generating Module File
_generate_module_cpp(opticalFlowTest
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opticalFlowTest
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(opticalFlowTest_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(opticalFlowTest_generate_messages opticalFlowTest_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liuxiaobu/catkin_drone/src/test_optical_flow-wrz/msg/optical_flow_rad.msg" NAME_WE)
add_dependencies(opticalFlowTest_generate_messages_cpp _opticalFlowTest_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(opticalFlowTest_gencpp)
add_dependencies(opticalFlowTest_gencpp opticalFlowTest_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS opticalFlowTest_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(opticalFlowTest
  "/home/liuxiaobu/catkin_drone/src/test_optical_flow-wrz/msg/optical_flow_rad.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opticalFlowTest
)

### Generating Services

### Generating Module File
_generate_module_lisp(opticalFlowTest
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opticalFlowTest
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(opticalFlowTest_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(opticalFlowTest_generate_messages opticalFlowTest_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liuxiaobu/catkin_drone/src/test_optical_flow-wrz/msg/optical_flow_rad.msg" NAME_WE)
add_dependencies(opticalFlowTest_generate_messages_lisp _opticalFlowTest_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(opticalFlowTest_genlisp)
add_dependencies(opticalFlowTest_genlisp opticalFlowTest_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS opticalFlowTest_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(opticalFlowTest
  "/home/liuxiaobu/catkin_drone/src/test_optical_flow-wrz/msg/optical_flow_rad.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opticalFlowTest
)

### Generating Services

### Generating Module File
_generate_module_py(opticalFlowTest
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opticalFlowTest
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(opticalFlowTest_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(opticalFlowTest_generate_messages opticalFlowTest_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liuxiaobu/catkin_drone/src/test_optical_flow-wrz/msg/optical_flow_rad.msg" NAME_WE)
add_dependencies(opticalFlowTest_generate_messages_py _opticalFlowTest_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(opticalFlowTest_genpy)
add_dependencies(opticalFlowTest_genpy opticalFlowTest_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS opticalFlowTest_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opticalFlowTest)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opticalFlowTest
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(opticalFlowTest_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(opticalFlowTest_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opticalFlowTest)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opticalFlowTest
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(opticalFlowTest_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(opticalFlowTest_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opticalFlowTest)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opticalFlowTest\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opticalFlowTest
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(opticalFlowTest_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(opticalFlowTest_generate_messages_py std_msgs_generate_messages_py)
endif()
