/*!
 * \jaco_conversions.h
 * \brief Provides services for conversions between 3D rotation representations.
 *
 * jaco_conversions creates a ROS node that provides services for converting
 * between the JACO's internal representation of 3D rotations (Euler xyz convention)
 * and commonly used representations in ROS (quaternions)
 *
 * \author David Kent, GT - dekent@gatech.edu
 */

#ifndef JACO_CONVERSIONS_H_
#define JACO_CONVERSIONS_H_

#include <ros/ros.h>
#include <wpi_jaco_msgs/EulerToQuaternion.h>
#include <wpi_jaco_msgs/QuaternionToEuler.h>

/*!
 * \class JacoConversions
 * \brief Provides services for conversions between 3D rotation representations.
 *
 * JacoConversions creates a ROS node that provides services for converting
 * between the JACO's internal representation of 3D rotations (Euler xyz convention)
 * and commonly used representations in ROS (quaternions)
 */
class JacoConversions
{

public:

  JacoConversions(void);

  /**
   * \brief Callback for the Euler to Quaternion service
   * @param req service request
   * @param res service response
   * @return true on success
   */
  bool callEQ(wpi_jaco_msgs::EulerToQuaternion::Request &req, wpi_jaco_msgs::EulerToQuaternion::Response &res);

  /**
   * \brief Callback for the Quaternion to Euler service
   * @param req service request
   * @param res service response
   * @return true on success
   */
  bool callQE(wpi_jaco_msgs::QuaternionToEuler::Request &req, wpi_jaco_msgs::QuaternionToEuler::Response &res);

private:
  bool loadParameters(const ros::NodeHandle n);

  ros::NodeHandle n;
  ros::ServiceServer eqServer;
  ros::ServiceServer qeServer;

  std::string        arm_name_;
  std::string        topic_prefix_;
};

#endif
