cmake_minimum_required(VERSION 3.0.2)
project(audio_feedback)

set(CMAKE_CXX_COMPILER g++-11)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  AudioWarning.msg
  LidarCurb.msg
  LidarObject.msg
  UltrasonicCurb.msg
  UltrasonicObject.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
