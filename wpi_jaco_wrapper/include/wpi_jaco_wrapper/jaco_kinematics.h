/*!
 * \jaco_kinematics.h
 * \brief Provides services for JACO kinematics.
 *
 * jaco_kinematics creates a ROS node that provides services for converting calculating
 * kinematics for the JACO arm.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 */

#ifndef JACO_ARM_KINEMATICS_H_
#define JACO_ARM_KINEMATICS_H_

#include <ros/ros.h>
#include <wpi_jaco_msgs/JacoFK.h>
#include <tf/tf.h>

//Link lengths and offsets
#define D1 .2755
#define D2 .4100
#define D3 .2073
#define D4 .0743
#define D5 .0743
#define D6 .1687
#define E2 .0098

#define PI 3.14159

/*!
 * \class JacoKinematics
 * \brief Provides services for JACO kinematics.
 *
 * JacoKinematics creates a ROS node that provides services for converting calculating
 * kinematics for the JACO arm.
 */
class JacoKinematics
{

public:

  JacoKinematics(void);

  /**
   * \brief Callback for the forward kinematics service
   * @param req service request
   * @param res service response
   * @return true on success
   */
  bool callFK(wpi_jaco_msgs::JacoFK::Request &req, wpi_jaco_msgs::JacoFK::Response &res);

  /**
   * \brief Calculates the forward kinematics for the JACO arm
   * @param joints vector of joint angles from the arm
   * @return pose of the end effector relative to the arm's base
   */
  geometry_msgs::PoseStamped calculateFK(std::vector<float> joints);

  /**
   * \brief Generates a transform given D-H parameters
   * @param theta joint angle
   * @param d link length
   * @param a offset
   * @param alpha angle offset
   * @return the transform for one link
   */
  tf::Transform generateTransform(float theta, float d, float a, float alpha);

private:
  bool loadParameters(const ros::NodeHandle n);

  ros::NodeHandle n;
  ros::ServiceServer fkServer;

  std::string arm_name_;

  //robot parameters
  std::vector<float> ds; //!< d parameters in the D-H convention
  std::vector<float> as; //!< a parameters in the D-H convention
  std::vector<float> alphas; //!< alpha parameters in the D-H convention
};

#endif
