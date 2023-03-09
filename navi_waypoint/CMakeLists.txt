cmake_minimum_required(VERSION 3.0.2)
project(navi_waypoint)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  geometry_msgs
  roscpp
  roslib
  zetabank_msgs
  std_msgs
  tf
)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES navi_Waypoint
  CATKIN_DEPENDS actionlib move_base_msgs geometry_msgs roscpp roslib std_msgs zetabank_msgs tf
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/navi_waypoint_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

add_executable(navigation_waypoints_node src/navigation_waypoints_node.cpp)

target_link_libraries(navigation_waypoints_node   ${catkin_LIBRARIES} yaml-cpp)


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#catkin_install_python(PROGRAMS scripts/ncWPGUI.py
#                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})