cmake_minimum_required(VERSION 3.0.2)
project(make_waypoint)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES make_Waypoint
  CATKIN_DEPENDS geometry_msgs roscpp roslib std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/make_waypoint_node.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
