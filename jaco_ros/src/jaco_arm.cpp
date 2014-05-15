#include <jaco_ros/jaco_arm.h>
#include <sstream>
#include <Kinova.API.UsbCommandLayerUbuntu.h>

namespace jaco_arm{

  JacoArm::JacoArm() 
  {
    InitAPI();
    ros::Duration(1.0).sleep();
    StartControlAPI();
    ros::Duration(3.0).sleep();
    StopControlAPI();

    MoveHome();
    InitFingers();

    read();
    for(int joint_id = 0; joint_id < NUM_JACO_JOINTS; ++joint_id){
      joint_cmd[joint_id] = joint_pos[joint_id];
    }
    for(int finger_id = 0; finger_id < NUM_JACO_FINGER_JOINTS; ++finger_id){
      finger_cmd[finger_id] = finger_pos[finger_id];
    }
    

    for(int joint_id = 0; joint_id < NUM_JACO_JOINTS; ++joint_id){
      std::stringstream joint_name_stream;
      joint_name_stream << "jaco_joint_" << (joint_id+1);
      std::string joint_name = joint_name_stream.str();
      hardware_interface::JointStateHandle state_handle(joint_name, &joint_pos[joint_id], &joint_vel[joint_id], &joint_eff[joint_id]);
      jnt_state_interface.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_name), &joint_cmd[joint_id]);
      jnt_pos_interface.registerHandle(pos_handle);
    }
    for(int finger_id = 0; finger_id < NUM_JACO_FINGER_JOINTS; ++finger_id){
      std::stringstream finger_name_stream;
      finger_name_stream << "jaco_joint_finger_" << (finger_id+1);
      std::string finger_name = finger_name_stream.str();
      hardware_interface::JointStateHandle state_handle(finger_name, &finger_pos[finger_id], &finger_vel[finger_id], &finger_eff[finger_id]);
      jnt_state_interface.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(finger_name), &finger_cmd[finger_id]);
      jnt_pos_interface.registerHandle(pos_handle);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
  }
  JacoArm::~JacoArm() 
  {
    StopControlAPI();
    CloseAPI();
  }

  void JacoArm::read(){
    AngularPosition force_data;
    GetAngularForce(force_data);
    joint_eff[0] = force_data.Actuators.Actuator1;
    joint_eff[1] = force_data.Actuators.Actuator2;
    joint_eff[2] = force_data.Actuators.Actuator3;
    joint_eff[3] = force_data.Actuators.Actuator4;
    joint_eff[4] = force_data.Actuators.Actuator5;
    joint_eff[5] = force_data.Actuators.Actuator6;
    finger_eff[0] = force_data.Fingers.Finger1;
    finger_eff[1] = force_data.Fingers.Finger2;
    finger_eff[2] = force_data.Fingers.Finger3;

    AngularPosition velocity_data;
    GetAngularVelocity(velocity_data);
    joint_vel[0] = velocity_data.Actuators.Actuator1 * DEG_TO_RAD;
    joint_vel[1] = velocity_data.Actuators.Actuator2 * DEG_TO_RAD;
    joint_vel[2] = velocity_data.Actuators.Actuator3 * DEG_TO_RAD;
    joint_vel[3] = velocity_data.Actuators.Actuator4 * DEG_TO_RAD;
    joint_vel[4] = velocity_data.Actuators.Actuator5 * DEG_TO_RAD;
    joint_vel[5] = velocity_data.Actuators.Actuator6 * DEG_TO_RAD;
    finger_vel[0] = velocity_data.Fingers.Finger1 * DEG_TO_RAD;
    finger_vel[1] = velocity_data.Fingers.Finger2 * DEG_TO_RAD;
    finger_vel[2] = velocity_data.Fingers.Finger3 * DEG_TO_RAD;

    AngularPosition position_data;
    GetAngularPosition(position_data);
    joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
    joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
    joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
    joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
    joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
    joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;
    finger_pos[0] = position_data.Fingers.Finger1 * DEG_TO_RAD;
    finger_pos[1] = position_data.Fingers.Finger2 * DEG_TO_RAD;
    finger_pos[2] = position_data.Fingers.Finger3 * DEG_TO_RAD;
  }

  void JacoArm::write(){
    AngularInfo angles;
    FingersPosition fingers;
    angles.Actuator1 = joint_cmd[0]*RAD_TO_DEG;
    angles.Actuator2 = joint_cmd[1]*RAD_TO_DEG;
    angles.Actuator3 = joint_cmd[2]*RAD_TO_DEG;
    angles.Actuator4 = joint_cmd[3]*RAD_TO_DEG;
    angles.Actuator5 = joint_cmd[4]*RAD_TO_DEG;
    angles.Actuator6 = joint_cmd[5]*RAD_TO_DEG;
    //TODO Gripper controller only supports one joint
    ROS_INFO("%f", finger_cmd[0]);
    fingers.Finger1 = finger_cmd[0]*RAD_TO_DEG;
    fingers.Finger2 = finger_cmd[0]*RAD_TO_DEG;
    fingers.Finger3 = finger_cmd[0]*RAD_TO_DEG;

    TrajectoryPoint point;
    memset(&point, 0, sizeof(point));
    EraseAllTrajectories();
    StopControlAPI();
    StartControlAPI();
    SetAngularControl();
    
    point.LimitationsActive = false;
    point.Position.Delay = 0.0;
    point.Position.Type = ANGULAR_POSITION;
    point.Position.Actuators = angles; 
    point.Position.HandMode = POSITION_MODE;
    point.Position.Fingers = fingers;

    SendBasicTrajectory(point);
  }

}
