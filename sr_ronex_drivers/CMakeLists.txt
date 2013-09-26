# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")

cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_drivers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs sr_ronex_msgs sr_ronex_utilities roscpp rostest diagnostic_msgs diagnostic_updater pluginlib sr_ronex_external_protocol sr_ronex_hardware_interface dynamic_reconfigure realtime_tools ethercat_hardware)

include_directories(include ${catkin_INCLUDE_DIRS} )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/GeneralIO.cfg)

add_definitions(-Wall)
add_library(
  sr_ronex_drivers src/standard_ethercat_device.cpp src/sr_board_0x.cpp
  src/cod_decod/cod_decod_manager.cpp src/cod_decod/cod_decod.cpp src/cod_decod/cod_decod_std_io.cpp
  src/sr_board_mk2_gio.cpp)
target_link_libraries(sr_ronex_drivers gcov ${catkin_LIBRARIES})

add_dependencies(sr_ronex_drivers sr_ronex_msgs_gencpp ${PROJECT_NAME}_gencfg)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS pr2_msgs ethercat_hardware std_msgs sr_ronex_msgs pr2_hardware_interface eml roscpp rostest realtime_tools diagnostic_msgs diagnostic_updater pluginlib sr_ronex_external_protocol sr_ronex_hardware_interface dynamic_reconfigure tinyxml
    INCLUDE_DIRS include
)


if(COMMAND add_rostest_gtest)
  link_directories(/opt/ros/hydro/lib)
  add_rostest_gtest( test_ethercat_drivers test/test_ethercat_drivers.test test/test_ethercat_drivers.cpp )
  target_link_libraries( test_ethercat_drivers sr_ronex_drivers ethercat_hardware eml rt gcov ${catkin_LIBRARIES} ${GTEST_LIBRARIES} )
endif(COMMAND add_rostest_gtest)


install(TARGETS sr_ronex_drivers
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES ethercat_device_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY cfg/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)


