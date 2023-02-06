# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lidar_msgs: 6 messages, 0 services")

set(MSG_I_FLAGS "-Ilidar_msgs:/home/capstone/catkin_ws/src/lidar_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lidar_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg" NAME_WE)
add_custom_target(_lidar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_msgs" "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg" "lidar_msgs/box"
)

get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg" NAME_WE)
add_custom_target(_lidar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_msgs" "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg" "lidar_msgs/PointStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg" NAME_WE)
add_custom_target(_lidar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_msgs" "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg" "lidar_msgs/PointStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg" NAME_WE)
add_custom_target(_lidar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_msgs" "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg" ""
)

get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg" NAME_WE)
add_custom_target(_lidar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_msgs" "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg" "lidar_msgs/BoxInfo:lidar_msgs/box"
)

get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg" NAME_WE)
add_custom_target(_lidar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_msgs" "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_msgs
)
_generate_msg_cpp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_msgs
)
_generate_msg_cpp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_msgs
)
_generate_msg_cpp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_msgs
)
_generate_msg_cpp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg;/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_msgs
)
_generate_msg_cpp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(lidar_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lidar_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lidar_msgs_generate_messages lidar_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_cpp _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_cpp _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_cpp _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_cpp _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_cpp _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_cpp _lidar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_msgs_gencpp)
add_dependencies(lidar_msgs_gencpp lidar_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_msgs
)
_generate_msg_eus(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_msgs
)
_generate_msg_eus(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_msgs
)
_generate_msg_eus(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_msgs
)
_generate_msg_eus(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg;/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_msgs
)
_generate_msg_eus(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(lidar_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lidar_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lidar_msgs_generate_messages lidar_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_eus _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_eus _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_eus _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_eus _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_eus _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_eus _lidar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_msgs_geneus)
add_dependencies(lidar_msgs_geneus lidar_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_msgs
)
_generate_msg_lisp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_msgs
)
_generate_msg_lisp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_msgs
)
_generate_msg_lisp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_msgs
)
_generate_msg_lisp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg;/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_msgs
)
_generate_msg_lisp(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(lidar_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lidar_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lidar_msgs_generate_messages lidar_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_lisp _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_lisp _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_lisp _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_lisp _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_lisp _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_lisp _lidar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_msgs_genlisp)
add_dependencies(lidar_msgs_genlisp lidar_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_msgs
)
_generate_msg_nodejs(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_msgs
)
_generate_msg_nodejs(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_msgs
)
_generate_msg_nodejs(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_msgs
)
_generate_msg_nodejs(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg;/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_msgs
)
_generate_msg_nodejs(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(lidar_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lidar_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lidar_msgs_generate_messages lidar_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_nodejs _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_nodejs _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_nodejs _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_nodejs _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_nodejs _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_nodejs _lidar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_msgs_gennodejs)
add_dependencies(lidar_msgs_gennodejs lidar_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_msgs
)
_generate_msg_py(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_msgs
)
_generate_msg_py(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_msgs
)
_generate_msg_py(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_msgs
)
_generate_msg_py(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg"
  "${MSG_I_FLAGS}"
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg;/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_msgs
)
_generate_msg_py(lidar_msgs
  "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(lidar_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lidar_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lidar_msgs_generate_messages lidar_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/BoxInfo.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_py _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_py _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/Curb.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_py _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/box.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_py _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/SocketBox.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_py _lidar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/capstone/catkin_ws/src/lidar_msgs/msg/PointStamped.msg" NAME_WE)
add_dependencies(lidar_msgs_generate_messages_py _lidar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_msgs_genpy)
add_dependencies(lidar_msgs_genpy lidar_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(lidar_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(lidar_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(lidar_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(lidar_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(lidar_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(lidar_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(lidar_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(lidar_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(lidar_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(lidar_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
