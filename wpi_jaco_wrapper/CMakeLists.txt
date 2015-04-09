cmake_minimum_required(VERSION 2.8.3)
project(wpi_jaco_wrapper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib controller_manager control_msgs ecl_geometry hardware_interface wpi_jaco_msgs jaco_sdk rail_manipulation_msgs roscpp sensor_msgs std_msgs std_srvs)

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
add_executable(jaco_arm_trajectory_node src/jaco_arm_trajectory_node.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(jaco_arm_trajectory_node
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable/library
add_dependencies(jaco_arm_trajectory_node wpi_jaco_msgs_gencpp)

## Declare a cpp executable
add_executable(jaco_manipulation src/jaco_manipulation.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(jaco_manipulation
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable/library
add_dependencies(jaco_manipulation wpi_jaco_msgs_gencpp rail_manipulation_msgs_gencpp)

## Declare a cpp executable
add_executable(jaco_kinematics src/jaco_kinematics.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(jaco_kinematics
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable/library
add_dependencies(jaco_kinematics wpi_jaco_msgs_gencpp)

## Declare a cpp executable
add_executable(jaco_conversions src/jaco_conversions.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(jaco_conversions
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable/library
add_dependencies(jaco_conversions wpi_jaco_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS jaco_conversions jaco_kinematics jaco_manipulation jaco_arm_trajectory_node
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

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
