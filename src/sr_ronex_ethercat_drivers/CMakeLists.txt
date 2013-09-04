# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_ethercat_drivers)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS pr2_msgs ethercat_hardware std_msgs sr_common_msgs pr2_hardware_interface eml roscpp realtime_tools diagnostic_msgs diagnostic_updater pluginlib sr_ronex_external_protocol sr_ronex_hardware_interface dynamic_reconfigure tinyxml)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#add_message_files(
  FILES
  # TODO: List your msg files here
)

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

add_definitions(-Wall)
add_library(
  sr_ronex_ethercat_drivers src/standard_ethercat_device.cpp src/sr_board_0x.cpp
  src/cod_decod/cod_decod_manager.cpp src/cod_decod/cod_decod.cpp src/cod_decod/cod_decod_std_io.cpp
  src/sr_board_mk2_gio.cpp)

catkin_add_gtest(test/test_utils test/test_utils.cpp)## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS pr2_msgs ethercat_hardware std_msgs sr_common_msgs pr2_hardware_interface eml roscpp realtime_tools diagnostic_msgs diagnostic_updater pluginlib sr_ronex_external_protocol sr_ronex_hardware_interface dynamic_reconfigure tinyxml
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)