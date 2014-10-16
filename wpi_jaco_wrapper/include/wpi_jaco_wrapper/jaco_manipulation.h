/*!
 * \jaco_manipulation.h
 * \brief Provides manipulation actions for the JACO arm.
 *
 * jaco_manipulation creates a ROS node that provides action servers for executing
 * manipulation actions including grasping, releasing, and object pickup.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 */

#ifndef JACO_MANIPULATION_H_
#define JACO_MANIPULATION_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <wpi_jaco_msgs/AngularCommand.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/ExecuteGraspAction.h>
#include <wpi_jaco_msgs/ExecutePickupAction.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <sensor_msgs/JointState.h>

#define NUM_JACO_JOINTS 6
#define NUM_JACO_FINGER_JOINTS 3
#define NUM_JOINTS (NUM_JACO_JOINTS+NUM_JACO_FINGER_JOINTS)

#define MAX_FINGER_VEL 30 //maximum finger actuator velocity
#define DEFAULT_LIFT_VEL .1 //the default velocity for lifting objects during pickup (m/s)
#define LIFT_HEIGHT .15 //height for object pickup (m)
#define LIFT_TIMEOUT 5 //timeout for pickup action (s)
#define GRIPPER_OPEN_THRESHOLD .02 //gripper position where the fingers are considered "open"

/*!
 * \class JacoManipulation
 * \brief Provides manipulation actions for the JACO arm.
 *
 * JacoManipulation creates a ROS node that provides action servers for executing
 * manipulation actions including grasping, releasing, and object pickup.
 */
class JacoManipulation
{
private:
  ros::NodeHandle n;

  // Messages
  ros::Publisher cartesianCmdPublisher;
  ros::Publisher angularCmdPublisher;
  ros::Subscriber jointStateSubscriber;

  // Services
  ros::ServiceClient cartesianPositionClient;

  // Actionlib
  actionlib::SimpleActionServer<wpi_jaco_msgs::ExecuteGraspAction> executeGraspServer;
  actionlib::SimpleActionServer<wpi_jaco_msgs::ExecutePickupAction> executePickupServer;

  double jointPos[NUM_JOINTS];

public:
  /**
   * \brief Constructor
   */
  JacoManipulation();

  /**
   * \brief Callback for the executeGraspServer, closes the gripper until an object is grasped, alternatively opens the gripper fully
   * @param goal action goal
   */
  void execute_grasp(const wpi_jaco_msgs::ExecuteGraspGoalConstPtr &goal);

  /**
   * \brief Callback for the executePickupServer, lifts the arm while applying input to keep the gripper closed
   * @param goal action goal
   */
  void execute_pickup(const wpi_jaco_msgs::ExecutePickupGoalConstPtr &goal);

  /**
   * \brief Callback for joint state updates
   * @param msg joint state message
   */
  void jointStateCallback(const sensor_msgs::JointState msg);

};

#endif
