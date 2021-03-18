cmake_minimum_required(VERSION 3.0.2)
project(teb_planner_pa_msgs)

set(CMAKE_BUILD_TYPE Release)


## find catkin
find_package(catkin REQUIRED COMPONENTS
  teb_local_planner

  std_msgs
  geometry_msgs
  nav_msgs
  costmap_converter
  visualization_msgs

  message_generation
)


## msgs and services
add_message_files(
  FILES
    Request.msg
    Respond.msg
)
add_service_files(
  FILES
    Plan.srv
)
generate_messages(
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    costmap_converter
    visualization_msgs
    teb_local_planner
)


## config catkin
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    teb_local_planner

    std_msgs
    geometry_msgs
    nav_msgs
    costmap_converter
    visualization_msgs

    message_runtime
  DEPENDS
)


## installation
install(DIRECTORY
  srv msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
