#ifndef JACO_ARM_TRAJECTORY_NODE_H_
#define JACO_ARM_TRAJECTORY_NODE_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/foreach.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <ecl/geometry.hpp>
#include <wpi_jaco_msgs/AngularCommand.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <wpi_jaco_msgs/JacoFK.h>
#include <wpi_jaco_msgs/QuaternionToEuler.h>
#include <sensor_msgs/JointState.h>

#include <jaco_sdk/Kinova.API.UsbCommandLayerUbuntu.h>

#define NUM_JACO_JOINTS 6
#define NUM_JACO_FINGER_JOINTS 3
#define NUM_JOINTS (NUM_JACO_JOINTS+NUM_JACO_FINGER_JOINTS)

#define LARGE_ACTUATOR_VELOCITY 0.8378 //maximum velocity of large actuator (joints 1-3) (rad/s)
#define SMALL_ACTUATOR_VELOCITY 1.0472 //maximum velocity of small actuator (joints 4-6) (rad/s)
#define TIME_SCALING_FACTOR 1.5 //keep the trajectory at a followable speed

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

//gains for trajectory follower
#define KP 300.0
#define KV 20.0
#define ERROR_THRESHOLD .03 //threshold in radians for combined joint error to consider motion a success

//control types
#define ANGULAR_CONTROL 1
#define CARTESIAN_CONTROL 2

namespace jaco_arm
{

class JacoArmTrajectoryController
{
private:
  // Messages
  ros::Publisher joint_state_pub_; //publisher for joint states
  ros::Publisher cartesianCmdPublisher; //publisher for Cartesian arm commands
  ros::Publisher angularCmdPublisher; //publisher for angular arm commands
  ros::Subscriber cartesianCmdSubscriber; //subscriber for Cartesian arm commands
  ros::Subscriber angularCmdSubscriber; //subscriber for angular arm commands

  // Services
  ros::ServiceClient jaco_fk_client; //forward kinematics client
  ros::ServiceClient qe_client; //quaternion to euler (XYZ) conversion client
  ros::ServiceServer cartesianPositionServer; //service server to get end effector pose

  ros::Timer joint_state_timer_; //timer for joint state publisher

  // Actionlib
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajectory_server_; //point-to-point trajectory follower
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smooth_trajectory_server_; //smooth point-to-point trajectory follower based on Cartesian end effector positions
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smooth_joint_trajectory_server; //smooth point-to-point trajectory follower based on joint velocity control
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_server_; //gripper command action server

  boost::recursive_mutex api_mutex;

public:
  /**
   * Constructor
   * @param nh ROS node handle
   * @param pnh ROS private node handle
   */
  JacoArmTrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh);

  /**
   * Destructor
   */
  virtual ~JacoArmTrajectoryController();

  /**
   * Reads joint states from the arm and publishes them as a JointState message
   */
  void update_joint_states();

  /**
   * Callback for the trajectory_server_, executes a joint angle trajectory
   * @param goal action goal
   */
  void execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

  /**
   * Callback for the smooth_trajectory_server_, executes a smoother trajectory
   * by converting joint angle trajectories to end effector cartesian trajectories
   * NOTE: the trajectories must not fall within constraints defined internally
   * on the JACO for singularity avoidance
   * @param goal action goal
   */
  void execute_smooth_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

  /**
   * Callback for the smooth_joint_trajectory_server, executes a smoothed trajectory
   * by interpolating a set of joint trajectory points, smoothing the corners, and
   * and following the trajectory with a velocity controller
   * @param goal action goal
   */
  void execute_joint_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

  /**
   * Callback for the gripper_server_, executes a gripper command
   * @param goal action goal
   */
  void execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal);

private:
  std::vector<std::string> joint_names;
  double joint_pos[NUM_JOINTS];
  double joint_vel[NUM_JOINTS];
  double joint_eff[NUM_JOINTS];

  unsigned int controlType; //current state of control

  /**
   * Callback for sending an angular command to the arm
   * @param msg angular command and info
   */
  void angularCmdCallback(const wpi_jaco_msgs::AngularCommand& msg);

  /**
   * Callback for sending a Cartesian command to the arm
   * @param msg Cartesian command and info
   */
  void cartesianCmdCallback(const wpi_jaco_msgs::CartesianCommand& msg);

  /**
   * Stripped-down angular trajectory point sending to the arm, use this for
   * trajectory followers that need very quick response
   * @param point angular trajectory point to send to the arm
   * @param erase if true, clear the trajectory point stack before sending point
   */
  void executeAngularTrajectoryPoint(TrajectoryPoint point, bool erase);

  /**
   * Stripped-down Cartesian trajectory point sending to the arm, use this for
   * trajectory followers that need very quick response
   * @param point Cartesian trajectory point to send to the arm
   * @param erase if true, clear the trajectory point stack before sending point
   */
  void executeCartesianTrajectoryPoint(TrajectoryPoint point, bool erase);

  /**
   * Service callback for getting the current Cartesian pose of the end effector,
   * this allows other nodes to get the pose which is normally only accessible
   * through the Kinova API
   * @param req empty service request
   * @param res service response including the end effector pose
   * @return true on success
   */
  bool getCartesianPosition(wpi_jaco_msgs::GetCartesianPosition::Request &req,
                            wpi_jaco_msgs::GetCartesianPosition::Response &res);
};

}

#endif
