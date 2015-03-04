/*!
 * \jaco_interactive_manipulation.h
 * \brief Allows for control of the JACO with a interactive markers.
 *
 * jaco_interactive_manipulation creates a ROS node that displays interactive markers
 * for the JACO arm and allows control of the end effector position and gripper through
 * an interactive marker client.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date June 26, 2014
 */

#ifndef JACO_INTERACTIVE_MANIPULATION_H_
#define JACO_INTERACTIVE_MANIPULATION_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/HomeArmAction.h>
#include <wpi_jaco_msgs/JacoFK.h>
#include <wpi_jaco_msgs/QuaternionToEuler.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>

/*!
 * \class jacoInteractiveManipulation
 * \brief Allows for control of the JACO arm with interactive markers.
 *
 * jaco_joy_teleop creates a ROS node that displays interactive markers for
 * the JACO arm, and allows control of the arm's end effector pose and gripper
 * commands through an interactive marker client such as rviz.
 */
class JacoInteractiveManipulation
{

public:

  /**
   * \brief Constructor
   */
  JacoInteractiveManipulation();

  /**
   * \brief Process feedback for the interactive marker on the JACO's end effector
   * @param feedback interactive marker feedback
   */
  void processHandMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * \brief Callback for the joint state listener
   * @param msg new joint state message
   */
  void updateJoints(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * \brief Update the interactive marker on the JACO's end effector to move based on the the current joint state of the arm
   */
  void updateMarkerPosition();

private:

  /**
   * \brief Create the interactive marker on the JACO's end effector, including pose controls and menus
   */
  void makeHandMarker();

  /**
   * \brief Send a 0 velocity command to the robot's arm
   */
  void sendStopCommand();

  ros::NodeHandle n;

  //messages
  ros::Publisher cartesianCmd;
  ros::Subscriber jointStateSubscriber;

  //services
  ros::ServiceClient eraseTrajectoriesClient;
  ros::ServiceClient jacoFkClient;	//!< forward kinematics
  ros::ServiceClient qeClient;	//!< rotation representation conversion client

  //actionlib
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> acGripper;
  actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction> acLift;
  actionlib::SimpleActionClient<wpi_jaco_msgs::HomeArmAction> acHome;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer;	//!< interactive marker server
  interactive_markers::MenuHandler menuHandler;	//!< interactive marker menu handler
  std::vector<float> joints;	//!< current joint state
  bool lockPose;//!< flag to stop the arm from updating on pose changes, this is used to prevent the slight movement when left clicking on the center of the marker
};

#endif
