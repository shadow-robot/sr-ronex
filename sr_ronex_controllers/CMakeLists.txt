# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs sr_ronex_msgs sr_ronex_hardware_interface sr_ronex_utilities pluginlib pr2_controller_interface realtime_tools)

include_directories( include ${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
add_library(sr_ronex_controllers src/general_io_passthrough_controller.cpp)
target_link_libraries(sr_ronex_controllers ${catkin_LIBRARIES})

add_dependencies(sr_ronex_controllers sr_ronex_msgs_gencpp)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS roscpp std_msgs std_srvs sr_ronex_msgs sr_ronex_hardware_interface pr2_mechanism_controllers pr2_controller_interface pr2_mechanism_model pluginlib
    INCLUDE_DIRS include
)
