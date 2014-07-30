#ifndef JACO_MANIPULATION_H_
#define JACO_MANIPULATION_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <jaco_msgs/AngularCommand.h>
#include <jaco_msgs/CartesianCommand.h>
#include <jaco_msgs/ExecuteGraspAction.h>
#include <jaco_msgs/ExecutePickupAction.h>
#include <jaco_msgs/GetCartesianPosition.h>
#include <sensor_msgs/JointState.h>

#define NUM_JACO_JOINTS 6
#define NUM_JACO_FINGER_JOINTS 3
#define NUM_JOINTS (NUM_JACO_JOINTS+NUM_JACO_FINGER_JOINTS)

#define MAX_FINGER_VEL 30 //maximum finger actuator velocity
#define DEFAULT_LIFT_VEL .1 //the default velocity for lifting objects during pickup (m/s)
#define LIFT_HEIGHT .15 //height for object pickup (m)
#define LIFT_TIMEOUT 5 //timeout for pickup action (s)

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
  actionlib::SimpleActionServer<jaco_msgs::ExecuteGraspAction> executeGraspServer;
  actionlib::SimpleActionServer<jaco_msgs::ExecutePickupAction> executePickupServer;

  double jointPos[NUM_JOINTS];

public:
  /**
   * Constructor
   */
  JacoManipulation();

  /**
   * Callback for the executeGraspServer, closes the gripper until an object is grasped,
   * alternatively opens the gripper fully
   * @param goal action goal
   */
  void execute_grasp(const jaco_msgs::ExecuteGraspGoalConstPtr &goal);

  /**
   * Callback for the executePickupServer, lifts the arm while applying input to
   * keep the gripper closed
   * @param goal action goal
   */
  void execute_pickup(const jaco_msgs::ExecutePickupGoalConstPtr &goal);

  void jointStateCallback(const sensor_msgs::JointState msg);

};

#endif
