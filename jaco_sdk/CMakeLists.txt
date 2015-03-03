cmake_minimum_required(VERSION 2.8.3)
project(jaco_sdk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

# find the 32 or 64 bit libraries
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "64bit")
else(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "32bit")
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

find_library(JACO_COMM_LIB NAMES Kinova.API.CommLayerUbuntu PATHS "lib/${ARCH}")
find_library(JACO_USB_LIB NAMES Kinova.API.USBCommandLayerUbuntu PATHS "lib/${ARCH}")

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Kinova.API.CommLayerUbuntu Kinova.API.USBCommandLayerUbuntu
  DEPENDS libusb-1.0
)

add_library(Kinova.API.CommLayerUbuntu ${JACO_COMM_LIB})
add_custom_command(TARGET Kinova.API.CommLayerUbuntu POST_BUILD COMMAND cp ${JACO_COMM_LIB} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/)
set_target_properties(Kinova.API.CommLayerUbuntu PROPERTIES LINKER_LANGUAGE CXX )

add_library(Kinova.API.USBCommandLayerUbuntu ${JACO_USB_LIB})
add_custom_command(TARGET Kinova.API.USBCommandLayerUbuntu POST_BUILD COMMAND cp ${JACO_USB_LIB} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/)
set_target_properties(Kinova.API.USBCommandLayerUbuntu PROPERTIES LINKER_LANGUAGE CXX )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(FILES ${JACO_COMM_LIB} ${JACO_USB_LIB}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
