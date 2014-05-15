#include <ros/ros.h>
#include <jaco_ros/jaco_arm.h>
#include <controller_manager/controller_manager.h>




int main( int argc, char** argv ){
  ros::init(argc, argv, "jaco_arm_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double rate;
  pnh.param<double>("controller_rate", rate, 10);

  jaco_arm::JacoArm robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate controller_rate(rate);
  ros::Time last = ros::Time::now();
  ROS_INFO_STREAM("Running jaco controller at rate: " << rate);
  while (ros::ok())
  {
    robot.read();
    ros::Time now = ros::Time::now();
    cm.update(now, now-last);
    robot.write();
    last = now;
    ROS_DEBUG_STREAM_THROTTLE(5, "Jaco Controller Cycle Time: " << controller_rate.cycleTime().toSec());
    controller_rate.sleep();
  }

}
