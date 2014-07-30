#ifndef JACO_ARM_KINEMATICS_H_
#define JACO_ARM_KINEMATICS_H_

#include <ros/ros.h>
#include <jaco_msgs/JacoFK.h>
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

class JacoKinematics
{

public:

  JacoKinematics(void);

  /**
   * Callback for the forward kinematics service
   * @param req service request
   * @param res service response
   * @return true on success
   */
  bool callFK(jaco_msgs::JacoFK::Request &req, jaco_msgs::JacoFK::Response &res);

  /**
   * Calculates the forward kinematics for the JACO arm
   * @param joints vector of joint angles from the arm
   * @return pose of the end effector relative to the arm's base
   */
  geometry_msgs::PoseStamped calculateFK(std::vector<float> joints);

  /**
   * Generates a transform given D-H parameters
   * @param theta joint angle
   * @param d link length
   * @param a offset
   * @param alpha angle offset
   * @return the transform for one link
   */
  tf::Transform generateTransform(float theta, float d, float a, float alpha);

private:
  ros::NodeHandle n;
  ros::ServiceServer fkServer;
  ros::Publisher visPublisher;	//pose publisher for debugging and visualization

  //robot parameters
  std::vector<float> ds;
  std::vector<float> as;
  std::vector<float> alphas;
};

#endif
