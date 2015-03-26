cmake_minimum_required(VERSION 2.8.3)
project(jaco_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)