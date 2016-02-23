/*!
 * \jaco_manipulation.h
 * \brief Provides manipulation actions for the JACO arm.
 *
 * jaco_manipulation creates a ROS node that provides action servers for executing
 * manipulation actions including grasping, releasing, and object pickup.
 *
 * \author David Kent, GT - dekent@gatech.edu
 */

#ifndef JACO_MANIPULATION_H_
#define JACO_MANIPULATION_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <std_srvs/Empty.h>
#include <wpi_jaco_msgs/AngularCommand.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <sensor_msgs/JointState.h>

#define NUM_JACO_JOINTS 6

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
public:
  typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction>     GripperClient;
  typedef actionlib::SimpleActionServer<rail_manipulation_msgs::GripperAction>  GripperServer;
  typedef actionlib::SimpleActionServer<rail_manipulation_msgs::LiftAction>     LiftServer;
  /**
   * \brief Constructor
   */
  JacoManipulation();

  /**
   * \brief Callback for the executeGraspServer, closes the gripper until an object is grasped, alternatively opens the gripper fully
   * @param goal action goal
   */
  void execute_gripper(const rail_manipulation_msgs::GripperGoalConstPtr &goal);

  /**
   * \brief Callback for the executePickupServer, lifts the arm while applying input to keep the gripper closed
   * @param goal action goal
   */
  void execute_lift(const rail_manipulation_msgs::LiftGoalConstPtr &goal);

  /**
   * \brief Callback for joint state updates
   * @param msg joint state message
   */
  void jointStateCallback(const sensor_msgs::JointState msg);

private:
  bool loadParameters(const ros::NodeHandle n);

  ros::NodeHandle n, pnh;

  // Parameters
  std::string arm_name_;
  std::string topic_prefix_;
  double  gripper_closed_;
  double  gripper_open_;
  int     num_fingers_;
  int     num_joints_;
  bool    kinova_gripper_;

  // Messages
  ros::Publisher cartesianCmdPublisher;
  ros::Publisher angularCmdPublisher;
  ros::Subscriber jointStateSubscriber;

  // Services
  ros::ServiceClient cartesianPositionClient;
  ros::ServiceClient eraseTrajectoriesClient;

  // Actionlib
  GripperClient*  acGripper;
  GripperServer*  asGripper;
  LiftServer*     asLift;

  std::vector<double> joint_pos_;


};

#endif
