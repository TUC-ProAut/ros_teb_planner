cmake_minimum_required(VERSION 2.8.3)
project(teb_planner_pa)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure

  teb_local_planner
  nav_core

  std_msgs
  geometry_msgs
  nav_msgs
  costmap_converter
  visualization_msgs
  teb_planner_pa_msgs

  message_generation

  tf
  tf_conversions
)

## C++11 support
## Unfortunately, the 3d-party dependency libg2o requires c++11 starting from ROS Jade.
## Even if the ROS Jade specifications do not want c++11-only packages,
## we cannot compile without c++11 enabled. Another option would be to downgrade
## libg2o third-party package.
## By now, if you do not want c++11, please refer to the ros indigo version.
if(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support which is required
    by linked third party packages starting from ROS Jade. Ignore this message for ROS Indigo.")
  endif()
endif()

#add dynamic reconfigure api
#generate_dynamic_reconfigure_options(
  #cfg/TebLocalPlannerReconfigure.cfg
#)

# catkin
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    teb_planner_pa
  CATKIN_DEPENDS
    roscpp
    dynamic_reconfigure

    teb_local_planner
    nav_core

    std_msgs
    geometry_msgs
    nav_msgs
    costmap_converter
    visualization_msgs
    teb_planner_pa_msgs

    tf
    tf_conversions
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Build the teb_planner_pa library
add_library(teb_planner_pa
   src/visualization_pa.cpp
)

target_link_libraries(teb_planner_pa
   ${catkin_LIBRARIES}
)


## Build executables (planner node)
add_executable(teb_planner_node_pa src/teb_planner_node_pa.cpp)

add_dependencies(teb_planner_node_pa
  teb_planner_pa
  #${PROJECT_NAME}_gencfg
)

target_link_libraries(teb_planner_node_pa
  teb_planner_pa
  ${catkin_LIBRARIES}
)

## Build executables (test node)
add_executable(service_test_node_pa src/service_test_node_pa.cpp)

#add_dependencies(service_test_node_pa
  #${PROJECT_NAME}_gencfg
#)

target_link_libraries(service_test_node_pa
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS teb_planner_node_pa service_test_node_pa
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
