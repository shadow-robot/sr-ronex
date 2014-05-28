# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_drivers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    sr_ronex_msgs
    sr_ronex_utilities
    roscpp
    rostest
    diagnostic_msgs
    diagnostic_updater
    pluginlib
    sr_ronex_external_protocol
    sr_ronex_hardware_interface
    dynamic_reconfigure
    realtime_tools
    ros_ethercat_model
    ros_ethercat_hardware
)

include_directories(include ${catkin_INCLUDE_DIRS} )

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/GeneralIO.cfg cfg/SPI.cfg)

add_definitions(-Wall)
add_library(sr_ronex_drivers
    src/standard_ethercat_device.cpp
    src/sr_board_0x.cpp
    src/sr_board_mk2_gio.cpp
    src/sr_spi.cpp
    src/sr_tcat.cpp
    src/cod_decod/cod_decod_manager.cpp
    src/cod_decod/cod_decod.cpp
    src/cod_decod/cod_decod_std_io.cpp
)
target_link_libraries(sr_ronex_drivers ${catkin_LIBRARIES})

add_dependencies(sr_ronex_drivers ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS
        ros_ethercat_hardware
        std_msgs
        sr_ronex_msgs
        ros_ethercat_model
        roscpp
        rostest
        realtime_tools
        diagnostic_msgs
        diagnostic_updater
        pluginlib
        sr_ronex_external_protocol
        sr_ronex_hardware_interface
        dynamic_reconfigure
    INCLUDE_DIRS include
    LIBRARIES sr_ronex_drivers
)


if(COMMAND add_rostest_gtest)
  add_rostest_gtest( test_ethercat_drivers test/test_ethercat_drivers.test test/test_ethercat_drivers.cpp )
  target_link_libraries( test_ethercat_drivers sr_ronex_drivers ${catkin_LIBRARIES} ${GTEST_LIBRARIES} )
  add_dependencies(test_ethercat_drivers ${catkin_EXPORTED_TARGETS})
endif(COMMAND add_rostest_gtest)


install(TARGETS sr_ronex_drivers
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES ethercat_device_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY cfg/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
