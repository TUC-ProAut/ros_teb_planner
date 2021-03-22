cmake_minimum_required(VERSION 3.0.2)
project(teb_planner_pa_matlab)

set(CMAKE_BUILD_TYPE Release)


## find catkin
find_package(catkin REQUIRED COMPONENTS
  teb_planner_pa_msgs
)


## config catkin
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    teb_planner_pa_msgs
  DEPENDS
)


## installation
install(DIRECTORY
  scripts matlab_gen
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
