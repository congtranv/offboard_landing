cmake_minimum_required(VERSION 3.0.2)
project(offboard_landing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES offboard_landing
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs
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

# add_executable(${PROJECT_NAME}_node src/offboard_landing_node.cpp)

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(hovering src/hovering_node.cpp)
add_executable(setpoint src/setpoint_node.cpp)
add_executable(gps src/gps_node.cpp)
add_executable(setmode src/setmode.cpp)

target_link_libraries(hovering
  ${catkin_LIBRARIES}
)
target_link_libraries(setpoint
  ${catkin_LIBRARIES}
)
target_link_libraries(gps
  ${catkin_LIBRARIES}
)
target_link_libraries(setmode
  ${catkin_LIBRARIES}
)
