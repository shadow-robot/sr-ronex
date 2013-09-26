cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_examples)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs sr_ronex_msgs sr_ronex_hardware_interface sr_ronex_utilities pluginlib pr2_controller_interface realtime_tools dynamic_reconfigure )

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

# catkin_package parameters
# http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# You must invoke catkin_package() before adding any targets (libraries and executables).
# The reason is because catkin_package() will change the location where the targets are built.
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs sr_ronex_msgs sr_ronex_utilities sr_ronex_hardware_interface pr2_mechanism_controllers pr2_controller_interface pr2_mechanism_model pluginlib
  INCLUDE_DIRS include
)

## Declare a cpp executable
add_executable(sr_ronex_parse_parameter_server src/sr_ronex_parse_parameter_server.cpp)
add_executable(sr_ronex_read_analog_data src/sr_ronex_read_analog_data.cpp)
add_executable(sr_ronex_flash_LED_with_PWM src/sr_ronex_flash_LED_with_PWM.cpp)
add_executable(change_ronex_configuration src/change_ronex_configuration.cpp)
add_library(sr_ronex_simple_controller src/sr_ronex_simple_controller.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(sr_ronex_read_analog_data sr_ronex_msgs_gencpp ${PROJECT_NAME}_gencfg)
add_dependencies(sr_ronex_flash_LED_with_PWM sr_ronex_msgs_gencpp ${PROJECT_NAME}_gencfg)
add_dependencies(sr_ronex_simple_controller sr_ronex_msgs_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sr_ronex_parse_parameter_server ${catkin_LIBRARIES})
target_link_libraries(sr_ronex_read_analog_data ${catkin_LIBRARIES})
target_link_libraries(sr_ronex_flash_LED_with_PWM ${catkin_LIBRARIES})
target_link_libraries(change_ronex_configuration ${catkin_LIBRARIES})
target_link_libraries(sr_ronex_simple_controller ${catkin_LIBRARIES})

