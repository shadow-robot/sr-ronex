cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(ronex_mechanism_model src/ronex_transmission.cpp src/mapping/ronex_mapping.cpp src/mapping/general_io/analogue_to_position.cpp src/mapping/general_io/analogue_to_effort.cpp)

rosbuild_add_executable(test/test_ronex_transmission test/test_ronex_transmission.cpp)
rosbuild_add_gtest_build_flags(test/test_ronex_transmission)
target_link_libraries(test/test_ronex_transmission ronex_mechanism_model)
rosbuild_add_rostest(test/test_ronex_transmission.test)