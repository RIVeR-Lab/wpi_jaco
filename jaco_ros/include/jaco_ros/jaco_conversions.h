#ifndef JACO_CONVERSIONS_H_
#define JACO_CONVERSIONS_H_

#include <ros/ros.h>
#include <jaco_msgs/EulerToQuaternion.h>
#include <jaco_msgs/QuaternionToEuler.h>

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
  bool callEQ(jaco_msgs::EulerToQuaternion::Request &req, jaco_msgs::EulerToQuaternion::Response &res);

  /**
   * Callback for the Quaternion to Euler service
   * @param req service request
   * @param res service response
   * @return true on success
   */
  bool callQE(jaco_msgs::QuaternionToEuler::Request &req, jaco_msgs::QuaternionToEuler::Response &res);

private:
  ros::NodeHandle n;
  ros::ServiceServer eqServer;
  ros::ServiceServer qeServer;
};

#endif
