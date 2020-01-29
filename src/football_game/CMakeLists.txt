cmake_minimum_required(VERSION 2.8.3)
project(football_game)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  urdf
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  rospy
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ReachGoal.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot1_description
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
 add_executable(compute_goal src/compute_goal.cpp)
 add_executable(reach_goal src/reach_goal.cpp)
 add_executable(tf_broadcaster src/tf_broadcaster.cpp)
 
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robot1_description_node robot1_description_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(compute_goal ${catkin_LIBRARIES})
 target_link_libraries(reach_goal ${catkin_LIBRARIES})
 target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
