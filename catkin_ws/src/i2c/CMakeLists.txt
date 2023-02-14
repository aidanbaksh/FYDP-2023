cmake_minimum_required(VERSION 3.0.2)
project(i2c)

# use gcc 11 and compile with c++20 features enabled
set(CMAKE_CXX_COMPILER g++-11)
add_compile_options(-std=c++2a)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# build libi2c
execute_process (
   COMMAND make libi2c.a
   WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libi2c"
)
add_library(libi2c STATIC IMPORTED)
set_target_properties(libi2c PROPERTIES
                      IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libi2c/libi2c.a
                      INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/libi2c/include)


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
#  LIBRARIES i2c
   CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  src/i2c_node.cpp
  src/I2C_Manager.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node libi2c)
    
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  libi2c
)
