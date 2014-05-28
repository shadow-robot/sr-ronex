cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_utilities)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(COMMAND add_rostest_gtest)
  add_rostest_gtest(test_ronex_utilities test/test_ronex_utilities.test test/test_sr_ronex_utilities.cpp)
  target_link_libraries(test_ronex_utilities ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
endif(COMMAND add_rostest_gtest)
