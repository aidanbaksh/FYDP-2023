cmake_minimum_required(VERSION 3.0.2)
project(audio_feedback)

set(CMAKE_CXX_COMPILER g++-11)
add_compile_options(-std=c++17)

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
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_node

# )



# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
