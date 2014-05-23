# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_hardware_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp ros_ethercat_hardware sr_ronex_external_protocol ros_ethercat_model)

include_directories(include ${catkin_INCLUDE_DIRS})

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS roscpp ros_ethercat_hardware sr_ronex_external_protocol ros_ethercat_model
    INCLUDE_DIRS include
)


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
