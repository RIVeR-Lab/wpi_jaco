cmake_minimum_required(VERSION 2.8.3)
project(jaco_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  interactive_markers
  jaco_ros
  roscpp
  sensor_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES jaco_arm_hw
 CATKIN_DEPENDS actionlib geometry_msgs interactive_markers jaco_ros roscpp sensor_msgs visualization_msgs
#  DEPENDS system_lib
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

## Declare a cpp executable
add_executable(jaco_joy_teleop src/jaco_joy_teleop.cpp)
target_link_libraries(jaco_joy_teleop
  ${catkin_LIBRARIES}
)

add_executable(jaco_key_teleop src/jaco_key_teleop.cpp)
target_link_libraries(jaco_key_teleop
  ${catkin_LIBRARIES}
)

add_executable(jaco_interactive_manipulation src/jaco_interactive_manipulation.cpp)
target_link_libraries(jaco_interactive_manipulation
  ${catkin_LIBRARIES}
)

