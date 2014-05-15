#include <jaco_ros/jaco_arm_trajectory_node.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <boost/foreach.hpp>

namespace jaco_arm{

  JacoArmTrajectoryController::JacoArmTrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh) : trajectory_server_(nh, "arm_controller", boost::bind(&JacoArmTrajectoryController::execute_trajectory, this, _1), false), gripper_server_(nh, "fingers_controller", boost::bind(&JacoArmTrajectoryController::execute_gripper, this, _1), false)
  {
    InitAPI();
    ros::Duration(1.0).sleep();
    StartControlAPI();
    ros::Duration(3.0).sleep();
    StopControlAPI();

    MoveHome();
    InitFingers();

    for(int joint_id = 0; joint_id < NUM_JACO_JOINTS; ++joint_id){
      std::stringstream joint_name_stream;
      joint_name_stream << "jaco_joint_" << (joint_id+1);
      std::string joint_name = joint_name_stream.str();
      joint_names.push_back(joint_name);
    }
    for(int finger_id = 0; finger_id < NUM_JACO_FINGER_JOINTS; ++finger_id){
      std::stringstream finger_name_stream;
      finger_name_stream << "jaco_joint_finger_" << (finger_id+1);
      std::string finger_name = finger_name_stream.str();
      joint_names.push_back(finger_name);
    }

    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    update_joint_states();
    trajectory_server_.start();
    gripper_server_.start();
    joint_state_timer_ = nh.createTimer(ros::Duration(0.05), boost::bind(&JacoArmTrajectoryController::update_joint_states, this));
  }
  JacoArmTrajectoryController::~JacoArmTrajectoryController() 
  {
    StopControlAPI();
    CloseAPI();
  }

  void JacoArmTrajectoryController::update_joint_states(){
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    AngularPosition force_data;
    GetAngularForce(force_data);
    joint_eff[0] = force_data.Actuators.Actuator1;
    joint_eff[1] = force_data.Actuators.Actuator2;
    joint_eff[2] = force_data.Actuators.Actuator3;
    joint_eff[3] = force_data.Actuators.Actuator4;
    joint_eff[4] = force_data.Actuators.Actuator5;
    joint_eff[5] = force_data.Actuators.Actuator6;
    joint_eff[6] = force_data.Fingers.Finger1;
    joint_eff[7] = force_data.Fingers.Finger2;
    joint_eff[8] = force_data.Fingers.Finger3;

    AngularPosition velocity_data;
    GetAngularVelocity(velocity_data);
    joint_vel[0] = velocity_data.Actuators.Actuator1 * DEG_TO_RAD;
    joint_vel[1] = velocity_data.Actuators.Actuator2 * DEG_TO_RAD;
    joint_vel[2] = velocity_data.Actuators.Actuator3 * DEG_TO_RAD;
    joint_vel[3] = velocity_data.Actuators.Actuator4 * DEG_TO_RAD;
    joint_vel[4] = velocity_data.Actuators.Actuator5 * DEG_TO_RAD;
    joint_vel[5] = velocity_data.Actuators.Actuator6 * DEG_TO_RAD;
    joint_vel[6] = velocity_data.Fingers.Finger1 * DEG_TO_RAD;
    joint_vel[7] = velocity_data.Fingers.Finger2 * DEG_TO_RAD;
    joint_vel[8] = velocity_data.Fingers.Finger3 * DEG_TO_RAD;

    AngularPosition position_data;
    GetAngularPosition(position_data);
    joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
    joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
    joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
    joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
    joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
    joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;
    joint_pos[6] = position_data.Fingers.Finger1 * DEG_TO_RAD;
    joint_pos[7] = position_data.Fingers.Finger2 * DEG_TO_RAD;
    joint_pos[8] = position_data.Fingers.Finger3 * DEG_TO_RAD;

    sensor_msgs::JointState state;
    state.header.stamp = ros::Time::now();
    state.name = joint_names;
    state.position.assign(joint_pos, joint_pos+NUM_JOINTS);
    state.velocity.assign(joint_pos, joint_vel+NUM_JOINTS);
    state.effort.assign(joint_pos, joint_eff+NUM_JOINTS);
    joint_state_pub_.publish(state);
  }

  void JacoArmTrajectoryController::send_trajectory_point(const std::vector<std::string>& trajectory_joint_names, trajectory_msgs::JointTrajectoryPoint point, bool clear_trajectory){
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    if(clear_trajectory){
      EraseAllTrajectories();
      StopControlAPI();
      StartControlAPI();
      SetAngularControl();
    }

    ROS_INFO("Trajectory Point");
    double joint_cmd[NUM_JACO_JOINTS];
    for(int trajectory_index = 0; trajectory_index<trajectory_joint_names.size(); ++trajectory_index){
      std::string joint_name = trajectory_joint_names[trajectory_index];
      int joint_index = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), joint_name));
      if(joint_index >=0 && joint_index < NUM_JACO_JOINTS){
        ROS_INFO("%s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, point.positions[trajectory_index]);
	joint_cmd[joint_index] = point.positions[trajectory_index];
      }
    }


    AngularInfo angles;
    FingersPosition fingers;
    angles.Actuator1 = joint_cmd[0]*RAD_TO_DEG;
    angles.Actuator2 = joint_cmd[1]*RAD_TO_DEG;
    angles.Actuator3 = joint_cmd[2]*RAD_TO_DEG;
    angles.Actuator4 = joint_cmd[3]*RAD_TO_DEG;
    angles.Actuator5 = joint_cmd[4]*RAD_TO_DEG;
    angles.Actuator6 = joint_cmd[5]*RAD_TO_DEG;

    TrajectoryPoint jaco_point;
    memset(&jaco_point, 0, sizeof(jaco_point));
    
    jaco_point.LimitationsActive = false;
    jaco_point.Position.Delay = 0.0;
    jaco_point.Position.Type = ANGULAR_POSITION;
    jaco_point.Position.Actuators = angles; 
    jaco_point.Position.HandMode = HAND_NOMOVEMENT;

    //SendBasicTrajectory(jaco_point);
  }


  void JacoArmTrajectoryController::execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal){
    bool first = true;
    BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point, goal->trajectory.points){
      send_trajectory_point(goal->trajectory.joint_names, point, first);
      first = false;
    }

    ros::Rate rate(10);
    int trajectory_size;
    while(trajectory_size > 0){
      {
	boost::recursive_mutex::scoped_lock lock(api_mutex);

	TrajectoryFIFO Trajectory_Info;
	memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
	GetGlobalTrajectoryInfo(Trajectory_Info);
	trajectory_size = Trajectory_Info.TrajectoryCount;
      }
      ROS_INFO("%f, %f, %f, %f, %f, %f", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);
      rate.sleep();
    }
    ROS_INFO("Trajectory Control Complete.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    trajectory_server_.setSucceeded(result);
  }
  void JacoArmTrajectoryController::execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal){
    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      EraseAllTrajectories();
      StopControlAPI();
      StartControlAPI();
      SetAngularControl();

      update_joint_states();

      AngularInfo angles;
      FingersPosition fingers;
      angles.Actuator1 = joint_pos[0];
      angles.Actuator2 = joint_pos[1];
      angles.Actuator3 = joint_pos[2];
      angles.Actuator4 = joint_pos[3];
      angles.Actuator5 = joint_pos[4];
      angles.Actuator6 = joint_pos[5];
      fingers.Finger1 = goal->command.position;
      fingers.Finger2 = goal->command.position;
      fingers.Finger3 = goal->command.position;

      TrajectoryPoint jaco_point;
      memset(&jaco_point, 0, sizeof(jaco_point));
      
      jaco_point.LimitationsActive = false;
      jaco_point.Position.Delay = 0.0;
      jaco_point.Position.Type = ANGULAR_POSITION;
      jaco_point.Position.Actuators = angles; 
      jaco_point.Position.HandMode = POSITION_MODE;
      jaco_point.Position.Fingers = fingers;
      SendBasicTrajectory(jaco_point);
    }

    ros::Rate rate(10);
    int trajectory_size;
    while(trajectory_size > 0){
      {
	boost::recursive_mutex::scoped_lock lock(api_mutex);

	TrajectoryFIFO Trajectory_Info;
	memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
	GetGlobalTrajectoryInfo(Trajectory_Info);
	trajectory_size = Trajectory_Info.TrajectoryCount;
      }
      rate.sleep();
    }

    update_joint_states();
    control_msgs::GripperCommandResult result;
    result.position = joint_pos[6];
    result.position = joint_eff[6];
    result.stalled = false;
    result.reached_goal = true;
    gripper_server_.setSucceeded(result);
  }

}

int main( int argc, char** argv ){
  ros::init(argc, argv, "jaco_arm_trajectory_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  jaco_arm::JacoArmTrajectoryController robot(nh, pnh);
  ros::spin();

}
