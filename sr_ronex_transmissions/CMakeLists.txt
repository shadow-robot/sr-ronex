# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_transmissions)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rostest sr_ronex_hardware_interface pr2_mechanism_model)

include_directories(include ${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
add_library(ronex_mechanism_model src/ronex_transmission.cpp src/mapping/general_io/analogue_to_position.cpp src/mapping/general_io/analogue_to_effort.cpp src/mapping/general_io/command_to_pwm.cpp)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS sr_ronex_hardware_interface pr2_mechanism_model roscpp rostest
    INCLUDE_DIRS include
)


include(CheckFunctionExists)
CHECK_FUNCTION_EXISTS(add_rostest_gtest HAVE_GTEST)
if(HAVE_GTEST)
  add_rostest_gtest(test_ronex_transmission test/test_ronex_transmission.test test/test_ronex_transmission.cpp)
  target_link_libraries(test_ronex_transmission tinyxml ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
  add_dependencies(test_ronex_transmission pr2_mechanism_model tinyxml)
endif(HAVE_GTEST)

