#include <wpi_jaco_wrapper/jaco_conversions.h>

using namespace std;

JacoConversions::JacoConversions(void)
{
  loadParameters(n);

  //advertise service
  eqServer = n.advertiseService(topic_prefix_ + "_conversions/euler_to_quaternion", &JacoConversions::callEQ, this);
  qeServer = n.advertiseService(topic_prefix_ + "_conversions/quaternion_to_euler", &JacoConversions::callQE, this);
}

bool JacoConversions::callEQ(wpi_jaco_msgs::EulerToQuaternion::Request &req,
                             wpi_jaco_msgs::EulerToQuaternion::Response &res)
{
  float t1 = req.roll;
  float t2 = req.pitch;
  float t3 = req.yaw;

  // Calculate the quaternion given roll, pitch, and yaw (rotation order XYZ -- 1:X, 2:Y, 3:Z)
  res.orientation.w = -sin(t1 / 2.0) * sin(t2 / 2.0) * sin(t3 / 2.0) + cos(t1 / 2.0) * cos(t2 / 2.0) * cos(t3 / 2.0);
  res.orientation.x = sin(t1 / 2.0) * cos(t2 / 2.0) * cos(t3 / 2.0) + sin(t2 / 2.0) * sin(t3 / 2.0) * cos(t1 / 2.0);
  res.orientation.y = -sin(t1 / 2.0) * sin(t3 / 2.0) * cos(t2 / 2.0) + sin(t2 / 2.0) * cos(t1 / 2.0) * cos(t3 / 2.0);
  res.orientation.z = sin(t1 / 2.0) * sin(t2 / 2.0) * cos(t3 / 2.0) + sin(t3 / 2.0) * cos(t2 / 2.0) * cos(t2 / 2.0);

  return true;
}

bool JacoConversions::callQE(wpi_jaco_msgs::QuaternionToEuler::Request &req,
                             wpi_jaco_msgs::QuaternionToEuler::Response &res)
{
  float q1 = req.orientation.w;
  float q2 = req.orientation.x;
  float q3 = req.orientation.y;
  float q4 = req.orientation.z;

  // Calculate necessary elements of the rotation matrix
  float m11 = pow(q1, 2) + pow(q2, 2) - pow(q3, 2) - pow(q4, 2);
  float m12 = 2 * (q2 * q3 - q1 * q4);
  float m13 = 2 * (q2 * q4 + q1 * q3);
  float m23 = 2 * (q3 * q4 - q1 * q2);
  float m33 = pow(q1, 2) - pow(q2, 2) - pow(q3, 2) + pow(q4, 2);

  // Calculate the roll, pitch, and yaw (rotation order XYZ -- 1:X, 2:Y, 3:Z)
  res.roll = atan2(-m23, m33);
  res.pitch = atan2(m13, sqrt(1 - pow(m13, 2)));
  res.yaw = atan2(-m12, m11);

  return true;
}

bool JacoConversions::loadParameters(const ros::NodeHandle n)
{
    n.param("wpi_jaco/arm_name", arm_name_, std::string("jaco"));

    // Update topic prefix
    if (arm_name_ == "jaco2")
        topic_prefix_ = "jaco";
    else
        topic_prefix_ = arm_name_;

    //! @todo MdL [IMPR]: Return is values are all correctly loaded.
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jaco_conversions");

  JacoConversions jc;

  ros::spin();
}

