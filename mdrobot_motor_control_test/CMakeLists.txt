cmake_minimum_required(VERSION 3.0.2)
project(mdrobot_motor_control_test)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  serial
  std_msgs
  tf
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mdrobot_motor_control_test
  CATKIN_DEPENDS 
    geometry_msgs 
    roscpp 
    rospy
    serial 
    std_msgs
    tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

 add_library(${PROJECT_NAME}_library
    src/mdrobot_motor_control.cpp
 )
target_link_libraries(${PROJECT_NAME}_library ${catkin_LIBRARIES})


 add_executable(${PROJECT_NAME}_node src/mdrobot_motor_control.cpp src/mdrobot_motor_control_test_node.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )


