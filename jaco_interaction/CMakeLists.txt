cmake_minimum_required(VERSION 2.8.3)
project(jaco_interaction)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib interactive_markers rail_manipulation_msgs roscpp sensor_msgs std_srvs wpi_jaco_msgs)

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

add_executable(jaco_interactive_manipulation src/jaco_interactive_manipulation.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(jaco_interactive_manipulation
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable/library
add_dependencies(jaco_interactive_manipulation wpi_jaco_msgs_gencpp rail_manipulation_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS jaco_interactive_manipulation
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

## Install RVIZ configurations
install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
