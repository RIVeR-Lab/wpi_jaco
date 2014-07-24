cmake_minimum_required(VERSION 2.8.3)
project(jaco_msgs)

find_package(catkin REQUIRED COMPONENTS
	actionlib_msgs
	geometry_msgs
)

add_message_files(
  FILES
  AngularCommand.msg
  CartesianCommand.msg
  JacoFingerVel.msg
)

add_service_files(
  FILES
  EulerToQuaternion.srv
  JacoFK.srv
  QuaternionToEuler.srv
)

add_action_files(
  FILES
  ExecuteGrasp.action
  ExecutePickup.action
)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs)

