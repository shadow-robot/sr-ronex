cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_examples)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs sr_ronex_msgs sr_ronex_hardware_interface sr_ronex_utilities pluginlib ros_ethercat_model realtime_tools dynamic_reconfigure controller_interface xacro)

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
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs sr_ronex_msgs sr_ronex_utilities dynamic_reconfigure sr_ronex_hardware_interface realtime_tools ros_ethercat_model pluginlib controller_interface xacro
  INCLUDE_DIRS include
  LIBRARIES sr_ronex_simple_controller
)

## Declare a cpp executable
add_executable(sr_ronex_parse_parameter_server src/sr_ronex_parse_parameter_server.cpp)
add_executable(sr_ronex_read_analog_data src/sr_ronex_read_analog_data.cpp)
add_executable(sr_ronex_flash_LED_with_PWM src/sr_ronex_flash_LED_with_PWM.cpp)
add_executable(change_ronex_configuration src/change_ronex_configuration.cpp)
add_library(sr_ronex_simple_controller src/sr_ronex_simple_controller.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(sr_ronex_read_analog_data ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(sr_ronex_flash_LED_with_PWM ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(sr_ronex_simple_controller ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(sr_ronex_parse_parameter_server ${catkin_LIBRARIES})
target_link_libraries(sr_ronex_read_analog_data ${catkin_LIBRARIES})
target_link_libraries(sr_ronex_flash_LED_with_PWM ${catkin_LIBRARIES})
target_link_libraries(change_ronex_configuration ${catkin_LIBRARIES})
target_link_libraries(sr_ronex_simple_controller ${catkin_LIBRARIES})


install(TARGETS sr_ronex_parse_parameter_server sr_ronex_read_analog_data sr_ronex_flash_LED_with_PWM change_ronex_configuration
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/src/*.py")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY src/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
   FILES_MATCHING PATTERN "*.cpp")

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY model/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/model)

install(FILES controller_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

