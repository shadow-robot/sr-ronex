cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()
#rosbuild_genmsg()

add_definitions(-Wall)
rosbuild_add_library(
  sr_ronex_ethercat_drivers src/standard_ethercat_device.cpp src/sr_board_0x.cpp
  src/cod_decod/cod_decod_manager.cpp src/cod_decod/cod_decod.cpp src/cod_decod/cod_decod_std_io.cpp )
