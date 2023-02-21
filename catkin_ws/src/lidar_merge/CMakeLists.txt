cmake_minimum_required(VERSION 3.0.2)
project(lidar_merge)

# use gcc 11 and compile with c++17 features enabled
# c++20 is not used due to an issue with boost signals2: https://github.com/boostorg/signals2/issues/50
# set(CMAKE_CXX_COMPILER g++-11)
add_compile_options(-std=c++11)

message("PCL ROS dir is ${PCL_ROS_DIR}")
message("pcl ros dir is ${pcl_ros_DIR}")
message("pcl dir is ${PCL_DIR}")
message("home is ${HOME}")
message("module path ${CMAKE_MODULE_PATH}")

# set(PCL_ROS_DIR ${HOME}/noetic_install/install_isolated/share/pcl_ros/cmake)
# find_package(pcl_ros REQUIRED)
# find_package(PCL_ROS REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  tf_conversions
  pcl_ros
)

add_definitions(${pcl_ros_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "-O0")

message("pcl include dirs ${pcl_ros_INCLUDE_DIRS}")
message("pcl ros defns ${pcl_ros_DEFINITIONS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lidar_merge
  CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs tf_conversions pcl_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${pcl_ros_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  src/lidar_merge_node.cpp
  src/PointCloudMerger.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${pcl_ros_LIBRARY_DIR}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
