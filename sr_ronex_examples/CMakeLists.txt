cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_examples)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs sr_ronex_msgs sr_ronex_utilities roscpp)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES std_msgs)

###########
## Build ##
###########

add_definitions(-Wall)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(sr_ronex_examples
#   src/${PROJECT_NAME}/sr_ronex
# )

## Declare a cpp executable
add_executable(sr_ronex_parse_parameter_server src/sr_ronex_parse_parameter_server.cpp)
add_executable(sr_ronex_read_analog_data src/sr_ronex_read_analog_data.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(sr_ronex_read_analog_data sr_ronex_msgs_gencpp ${PROJECT_NAME}_gencfg)

# catkin_package parameters
# http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
  CATKIN_DEPENDS std_msgs sr_ronex_msgs sr_ronex_utilities roscpp
  INCLUDE_DIRS include
)

## Specify libraries to link a library or executable target against
target_link_libraries(sr_ronex_parse_parameter_server ${catkin_LIBRARIES})
target_link_libraries(sr_ronex_read_analog_data ${catkin_LIBRARIES})
