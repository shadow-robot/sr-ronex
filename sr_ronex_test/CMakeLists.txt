# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_test)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy rostest )

set(COVERAGE "OFF" CACHE STRING "Enable coverage generation.")
message(STATUS "Using COVERAGE: ${COVERAGE}")
if("${COVERAGE}" STREQUAL "ON")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
endif()

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package( CATKIN_DEPENDS )
IF ("$ENV{RONEX_TESTS_WITH_HARDWARE}" STREQUAL "yes")
  add_rostest(test/sr_ronex.test)
ENDIF ("$ENV{RONEX_TESTS_WITH_HARDWARE}" STREQUAL "yes")
