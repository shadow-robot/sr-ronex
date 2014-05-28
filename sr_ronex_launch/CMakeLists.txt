# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_launch)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY loaders/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/loaders)

