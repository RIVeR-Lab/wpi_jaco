cmake_minimum_required(VERSION 2.8.3)
project(jaco_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs wpi_jaco_msgs)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(jaco_joy_teleop src/jaco_joy_teleop.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(jaco_joy_teleop
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable/library
add_dependencies(jaco_joy_teleop wpi_jaco_msgs_gencpp)

add_executable(jaco_key_teleop src/jaco_key_teleop.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(jaco_key_teleop
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable/library
add_dependencies(jaco_key_teleop wpi_jaco_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS jaco_joy_teleop jaco_key_teleop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
