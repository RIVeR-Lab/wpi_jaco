#ifndef JACO_ARM_TRAJECTORY_NODE_H_
#define JACO_ARM_TRAJECTORY_NODE_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/thread/recursive_mutex.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <ecl/geometry.hpp>
#include <geometry_msgs/Twist.h>
#include <jaco_msgs/AngularCommand.h>
#include <jaco_msgs/CartesianCommand.h>
#include <jaco_msgs/ExecuteGraspAction.h>
#include <jaco_msgs/ExecutePickupAction.h>
#include <jaco_msgs/EulerToQuaternion.h>
#include <jaco_msgs/JacoFingerVel.h>
#include <jaco_msgs/JacoFK.h>
#include <jaco_msgs/QuaternionToEuler.h>

#include <Kinova.API.UsbCommandLayerUbuntu.h>

#define NUM_JACO_JOINTS 6
#define NUM_JACO_FINGER_JOINTS 3
#define NUM_JOINTS (NUM_JACO_JOINTS+NUM_JACO_FINGER_JOINTS)

#define LARGE_ACTUATOR_VELOCITY 0.8378 //maximum velocity of large actuator (joints 1-3) (rad/s)
#define SMALL_ACTUATOR_VELOCITY 1.0472 //maximum velocity of small actuator (joints 4-6) (rad/s)
#define TIME_SCALING_FACTOR 1.5 //keep the trajectory at a followable speed

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

#define MAX_FINGER_VEL 30 //maximum finger actuator velocity
#define DEFAULT_LIFT_VEL .1 //the default velocity for lifting objects during pickup (m/s)
#define LIFT_HEIGHT .15 //height for object pickup (m)
#define LIFT_TIMEOUT 5 //timeout for pickup action (s)

//gains for trajectory follower
#define KP 300.0
#define KV 20.0
#define ERROR_THRESHOLD .03 //threshold in radians for combined joint error to consider motion a success

//control types
#define ANGULAR_CONTROL 1
#define CARTESIAN_CONTROL 2

namespace jaco_arm{

class JacoArmTrajectoryController
{
private:
  // Messages
  ros::Publisher joint_state_pub_;
  ros::Publisher cartesianCmdPublisher;
  ros::Publisher angularCmdPublisher;
  ros::Subscriber cartesianCmdVelSubscriber; //subscriber to cartesian velocity commands
  ros::Subscriber fingerCmdVelSubscriber; //subscriber for finger velocity commands
  ros::Subscriber positionCmdSubscriber; //subscriber for single cartesian position commands
  ros::Subscriber cartesianCmdSubscriber;
  ros::Subscriber angularCmdSubscriber;

  // Services
  ros::ServiceClient jaco_fk_client;
  ros::ServiceClient qe_client;
  ros::ServiceClient eq_client;
  
  ros::Timer joint_state_timer_;

  // Actionlib
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajectory_server_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smooth_trajectory_server_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smooth_joint_trajectory_server;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_server_;
  actionlib::SimpleActionServer<jaco_msgs::ExecuteGraspAction> executeGraspServer;
  actionlib::SimpleActionServer<jaco_msgs::ExecutePickupAction> executePickupServer;

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
  
private:
  std::vector<std::string> joint_names;
  double joint_pos[NUM_JOINTS];
  double joint_vel[NUM_JOINTS];
  double joint_eff[NUM_JOINTS];
  
  unsigned int controlType;
  
  /**
   * Callback for single cartesian position commands
   * \param msg cartesian position command for the end effector
   */
  void positionCmdCallback(const geometry_msgs::Pose::ConstPtr& msg);
  
  void cartesianCmdCallback(const jaco_msgs::CartesianCommand& msg);
  
  void angularCmdCallback(const jaco_msgs::AngularCommand& msg);
  
  void executeTrajectoryPoint(TrajectoryPoint point, bool erase);
};

}

#endif
