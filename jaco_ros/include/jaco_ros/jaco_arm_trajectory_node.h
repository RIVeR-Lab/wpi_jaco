#ifndef JACO_ARM_TRAJECTORY_NODE_H_
#define JACO_ARM_TRAJECTORY_NODE_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/thread/recursive_mutex.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/Twist.h>
#include <jaco_ros/ExecuteGraspAction.h>
#include <jaco_ros/EulerToQuaternion.h>
#include <jaco_ros/JacoFingerVel.h>
#include <jaco_ros/JacoFK.h>
#include <jaco_ros/QuaternionToEuler.h>

#define NUM_JACO_JOINTS 6
#define NUM_JACO_FINGER_JOINTS 3
#define NUM_JOINTS (NUM_JACO_JOINTS+NUM_JACO_FINGER_JOINTS)

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

#define MAX_FINGER_VEL 30

namespace jaco_arm{

class JacoArmTrajectoryController
{
private:
	// Messages
  ros::Publisher joint_state_pub_;
  ros::Subscriber cartesianCmdVelSubscriber; //subscriber to cartesian velocity commands
  ros::Subscriber fingerCmdVelSubscriber; //subscriber for finger velocity commands
  ros::Subscriber positionCmdSubscriber; //subscriber for single cartesian position commands

	// Services
  ros::ServiceClient jaco_fk_client;
  ros::ServiceClient qe_client;
  ros::ServiceClient eq_client;
  
  ros::Timer joint_state_timer_;

	// Actionlib
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajectory_server_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smooth_trajectory_server_;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_server_;
  actionlib::SimpleActionServer<jaco_ros::ExecuteGraspAction> executeGraspServer;

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
   * Callback for the gripper_server_, executes a gripper command
   * @param goal action goal
   */
  void execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal);
  
  /**
  * Callback for the executeGraspServer, closes the gripper until an object is grasped,
  * alternatively opens the gripper fully
  * @param goal action goal
  */
  void execute_grasp(const jaco_ros::ExecuteGraspGoalConstPtr &goal);
  
private:
  std::vector<std::string> joint_names;
  double joint_pos[NUM_JOINTS];
  double joint_vel[NUM_JOINTS];
  double joint_eff[NUM_JOINTS];
  
  /**
   * Callback for cartesian velocity commands
   * \param msg twist message containing velocity inputs
   */
  void cartesianCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  
  /**
   * Callback for finger velocity commands
   * \param msg finger velocity message containing gripper velocity inputs
   */
  void fingerCmdVelCallback(const jaco_ros::JacoFingerVel::ConstPtr& msg);
  
  /**
   * Helper function for the fingerCmdVel subscriber, defined separately here so
   * that other functions within this class can call it more easily
   * @param msg finger velocities at which to move each of the fingers
   */
  void moveFingers(jaco_ros::JacoFingerVel msg);
  
  /**
	 * Callback for single cartesian position commands
	 * \param msg cartesian position command for the end effector
	 */
	void positionCmdCallback(const geometry_msgs::Pose::ConstPtr& msg);
};

}

#endif
