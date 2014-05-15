cmake_minimum_required(VERSION 2.8.3)
project(jaco_sdk)

find_package(catkin REQUIRED)

set(ARCH "64bit")

message(STATUS "${ARCH}")

find_library(JACO_COMM_LIB NAMES Kinova.API.CommLayerUbuntu PATHS "lib/${ARCH}")
find_library(JACO_USB_LIB NAMES Kinova.API.USBCommandLayerUbuntu PATHS "lib/${ARCH}")

set(JACO_LIBS ${JACO_COMM_LIB} ${JACO_USB_LIB})

message(STATUS "${JACO_LIBS}")

catkin_package(
  INCLUDE_DIRS include/${ARCH}
  LIBRARIES ${JACO_LIBS}
)


install( DIRECTORY include/${ARCH}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install( FILES ${JACO_LIBS}
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})