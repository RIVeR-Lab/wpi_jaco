#ifndef JACO_ARM_TRAJECTORY_NODE_H_
#define JACO_ARM_TRAJECTORY_NODE_H_

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <boost/thread/recursive_mutex.hpp>
#include <actionlib/server/simple_action_server.h>

#define NUM_JACO_JOINTS 6
#define NUM_JACO_FINGER_JOINTS 3
#define NUM_JOINTS (NUM_JACO_JOINTS+NUM_JACO_FINGER_JOINTS)

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

namespace jaco_arm{

class JacoArmTrajectoryController
{
private:
  ros::Publisher joint_state_pub_;
  ros::Timer joint_state_timer_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajectory_server_;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_server_;
  boost::recursive_mutex api_mutex;
public:
  JacoArmTrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh);
  virtual ~JacoArmTrajectoryController();
  void update_joint_states();
  void send_trajectory_point(const std::vector<std::string>& joint_names, trajectory_msgs::JointTrajectoryPoint point, bool clear_trajectory = false);
  void execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
  void execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal);


private:
  std::vector<std::string> joint_names;
  double joint_pos[NUM_JOINTS];
  double joint_vel[NUM_JOINTS];
  double joint_eff[NUM_JOINTS];
};

}

#endif
