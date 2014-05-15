#ifndef JACO_ARM_H_
#define JACO_ARM_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#define NUM_JACO_JOINTS 6
#define NUM_JACO_FINGER_JOINTS 3

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

namespace jaco_arm{

class JacoArm : public hardware_interface::RobotHW
{
public:
  JacoArm();
  virtual ~JacoArm();
  void read();
  void write();

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double joint_cmd[NUM_JACO_JOINTS];
  double joint_pos[NUM_JACO_JOINTS];
  double joint_vel[NUM_JACO_JOINTS];
  double joint_eff[NUM_JACO_JOINTS];

  double finger_cmd[NUM_JACO_FINGER_JOINTS];
  double finger_pos[NUM_JACO_FINGER_JOINTS];
  double finger_vel[NUM_JACO_FINGER_JOINTS];
  double finger_eff[NUM_JACO_FINGER_JOINTS];
};

}

#endif
