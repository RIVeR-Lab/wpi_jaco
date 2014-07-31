#ifndef JACO_CONVERSIONS_H_
#define JACO_CONVERSIONS_H_

#include <ros/ros.h>
#include <wpi_jaco_msgs/EulerToQuaternion.h>
#include <wpi_jaco_msgs/QuaternionToEuler.h>

class JacoConversions
{

public:

  JacoConversions(void);

  /**
   * Callback for the Euler to Quaternion service
   * @param req service request
   * @param res service response
   * @return true on success
   */
  bool callEQ(wpi_jaco_msgs::EulerToQuaternion::Request &req, wpi_jaco_msgs::EulerToQuaternion::Response &res);

  /**
   * Callback for the Quaternion to Euler service
   * @param req service request
   * @param res service response
   * @return true on success
   */
  bool callQE(wpi_jaco_msgs::QuaternionToEuler::Request &req, wpi_jaco_msgs::QuaternionToEuler::Response &res);

private:
  ros::NodeHandle n;
  ros::ServiceServer eqServer;
  ros::ServiceServer qeServer;
};

#endif
