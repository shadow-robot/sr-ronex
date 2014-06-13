# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_transmissions)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rostest sr_ronex_hardware_interface ros_ethercat_model)

include_directories(include ${catkin_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
add_library(sr_ronex_transmissions
    src/ronex_transmission.cpp
    src/mapping/general_io/analogue_to_position.cpp
    src/mapping/general_io/analogue_to_effort.cpp
    src/mapping/general_io/command_to_pwm.cpp)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS sr_ronex_hardware_interface ros_ethercat_model roscpp rostest
    INCLUDE_DIRS include
    LIBRARIES sr_ronex_transmissions
)

#if(COMMAND add_rostest_gtest)
#  add_rostest_gtest(test_ronex_transmission test/test_ronex_transmission.test test/test_ronex_transmission.cpp)
#  target_link_libraries(test_ronex_transmission ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#  add_dependencies(test_ronex_transmission sr_ronex_drivers)
#endif(COMMAND add_rostest_gtest)

install(TARGETS sr_ronex_transmissions
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES transmission_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
