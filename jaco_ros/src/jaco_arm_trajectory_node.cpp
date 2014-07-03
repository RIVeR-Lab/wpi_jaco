#include <jaco_ros/jaco_arm_trajectory_node.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <boost/foreach.hpp>

namespace jaco_arm{

  JacoArmTrajectoryController::JacoArmTrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh) : 
  	trajectory_server_(nh, "arm_controller", boost::bind(&JacoArmTrajectoryController::execute_trajectory, this, _1), false), 
  	smooth_trajectory_server_(nh, "smooth_arm_controller", boost::bind(&JacoArmTrajectoryController::execute_smooth_trajectory, this, _1), false), 
  	gripper_server_(nh, "fingers_controller", boost::bind(&JacoArmTrajectoryController::execute_gripper, this, _1), false),
  	executeGraspServer(nh, "jaco_arm/execute_grasp", boost::bind(&JacoArmTrajectoryController::execute_grasp, this, _1), false)
  {
    InitAPI();
    ros::Duration(1.0).sleep();
    StartControlAPI();
    ros::Duration(3.0).sleep();
    StopControlAPI();

		// Initialize arm
    MoveHome();
    InitFingers();
		SetFrameType(0); //set end effector to move with respect to the fixed frame

		// Initialize joint names
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

		// Messages
    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    update_joint_states();
		cartesianCmdVelSubscriber = nh.subscribe("jaco_arm/cmd_vel", 1, &JacoArmTrajectoryController::cartesianCmdVelCallback, this);
		fingerCmdVelSubscriber = nh.subscribe("jaco_arm/finger_cmd_vel", 1, &JacoArmTrajectoryController::fingerCmdVelCallback, this);
    positionCmdSubscriber = nh.subscribe("jaco_arm/position_cmd", 1, &JacoArmTrajectoryController::positionCmdCallback, this);
    
    // Services
    jaco_fk_client = nh.serviceClient<jaco_ros::JacoFK>("jaco_fk");
    eq_client = nh.serviceClient<jaco_ros::EulerToQuaternion>("jaco_conversions/euler_to_quaternion");
    qe_client = nh.serviceClient<jaco_ros::QuaternionToEuler>("jaco_conversions/quaternion_to_euler");

		// Action servers
    trajectory_server_.start();    
    smooth_trajectory_server_.start();
    gripper_server_.start();
    executeGraspServer.start();
    
    joint_state_timer_ = nh.createTimer(ros::Duration(0.05), boost::bind(&JacoArmTrajectoryController::update_joint_states, this));
  }
  
  JacoArmTrajectoryController::~JacoArmTrajectoryController() 
  {
    StopControlAPI();
    CloseAPI();
  }

	/** Adjust angle to equivalent angle on [-pi, pi]
   *  @param angle the angle to be simplified (-inf, inf)
   *  @return the simplified angle on [-pi, pi]
   */
  static inline double simplify_angle(double angle)
  {
    double previous_rev = floor(angle/(2.0*M_PI))*2.0*M_PI;
    double next_rev = ceil(angle/(2.0*M_PI))*2.0*M_PI;
    double current_rev;
    if(fabs(angle - previous_rev) < fabs(angle - next_rev))
      return angle - previous_rev;
    return angle - next_rev;
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
    joint_pos[0] = simplify_angle(position_data.Actuators.Actuator1 * DEG_TO_RAD);
    joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
    joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
    joint_pos[3] = simplify_angle(position_data.Actuators.Actuator4 * DEG_TO_RAD);
    joint_pos[4] = simplify_angle(position_data.Actuators.Actuator5 * DEG_TO_RAD);
    joint_pos[5] = simplify_angle(position_data.Actuators.Actuator6 * DEG_TO_RAD);
    joint_pos[6] = position_data.Fingers.Finger1 * DEG_TO_RAD;
    joint_pos[7] = position_data.Fingers.Finger2 * DEG_TO_RAD;
    joint_pos[8] = position_data.Fingers.Finger3 * DEG_TO_RAD;

    sensor_msgs::JointState state;
    state.header.stamp = ros::Time::now();
    state.name = joint_names;
    state.position.assign(joint_pos, joint_pos+NUM_JOINTS);
    state.velocity.assign(joint_vel, joint_vel+NUM_JOINTS);
    state.effort.assign(joint_eff, joint_eff+NUM_JOINTS);
    joint_state_pub_.publish(state);
  }

	/** Calculates nearest desired angle to the current angle
   *  @param desired desired joint angle [-pi, pi]
   *  @param current current angle (-inf, inf)
   *  @return the closest equivalent angle (-inf, inf)
   */
  static inline double nearest_equivalent(double desired, double current){
    //calculate current number of revolutions
    double previous_rev = floor(current/(2*M_PI));
    double next_rev = ceil(current/(2*M_PI));
    double current_rev;
    if(fabs(current - previous_rev*2*M_PI) < fabs(current - next_rev*2*M_PI))
      current_rev = previous_rev;
    else
      current_rev = next_rev;
    
    //determine closest angle
    double lowVal = (current_rev - 1)*2*M_PI + desired;
    double medVal = current_rev*2*M_PI + desired;
    double highVal = (current_rev + 1)*2*M_PI + desired;
    if(fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
      return lowVal;
    if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
      return medVal;
    return highVal;
  }

  void JacoArmTrajectoryController::execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      EraseAllTrajectories();
      StopControlAPI();
      StartControlAPI();
      SetAngularControl();

      update_joint_states();
      double current_joint_pos[NUM_JACO_JOINTS];

      AngularPosition position_data;
      GetAngularPosition(position_data);
      current_joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;

      BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point, goal->trajectory.points)
      {
				ROS_INFO("Trajectory Point");
				double joint_cmd[NUM_JACO_JOINTS];
				for(int trajectory_index = 0; trajectory_index<goal->trajectory.joint_names.size(); ++trajectory_index)
				{
					std::string joint_name = goal->trajectory.joint_names[trajectory_index];
					int joint_index = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), joint_name));
					if(joint_index >=0 && joint_index < NUM_JACO_JOINTS)
					{
						//ROS_INFO("Before: %s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, point.positions[trajectory_index]);
						if (joint_index != 1 && joint_index != 2)
	            joint_cmd[joint_index] = nearest_equivalent(simplify_angle(point.positions[trajectory_index]), current_joint_pos[joint_index]);
            else
              joint_cmd[joint_index] = point.positions[trajectory_index];
            //ROS_INFO("After:  %s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, joint_cmd[joint_index]);
					}
				}
        for (int i = 0; i < NUM_JACO_JOINTS; i ++)
          current_joint_pos[i] = joint_cmd[i];

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

				SendBasicTrajectory(jaco_point);
      }
    }

    ros::Rate rate(10);
    int trajectory_size;
    while(trajectory_size > 0)
    {
      {
				boost::recursive_mutex::scoped_lock lock(api_mutex);

				TrajectoryFIFO Trajectory_Info;
				memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
				GetGlobalTrajectoryInfo(Trajectory_Info);
				trajectory_size = Trajectory_Info.TrajectoryCount;
      }
      //ROS_INFO("%f, %f, %f, %f, %f, %f", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);
      rate.sleep();
    }
    ROS_INFO("Trajectory Control Complete.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    trajectory_server_.setSucceeded(result);
  }
  
  void JacoArmTrajectoryController::execute_smooth_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
  	ROS_INFO("\n\n**********Begin trajectory execution**********\n");
  	{
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      EraseAllTrajectories();
      StopControlAPI();
      StartControlAPI();
      SetCartesianControl();

      update_joint_states();
      double current_joint_pos[NUM_JACO_JOINTS];

      AngularPosition position_data;
      GetAngularPosition(position_data);
      current_joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;

      BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point, goal->trajectory.points)
      {
				ROS_INFO("Trajectory Point");
				double joint_cmd[NUM_JACO_JOINTS];
				for(int trajectory_index = 0; trajectory_index < goal->trajectory.joint_names.size(); ++trajectory_index)
				{
					std::string joint_name = goal->trajectory.joint_names[trajectory_index];
					int joint_index = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), joint_name));
					if(joint_index >= 0 && joint_index < NUM_JACO_JOINTS)
					{
						//ROS_INFO("Before: %s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, point.positions[trajectory_index]);
			      if (joint_index != 1 && joint_index != 2)
	            joint_cmd[joint_index] = nearest_equivalent(simplify_angle(point.positions[trajectory_index]), current_joint_pos[joint_index]);
            else
              joint_cmd[joint_index] = point.positions[trajectory_index];
            //ROS_INFO("After:  %s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, joint_cmd[joint_index]);
					}
				}
				
		    for (int i = 0; i < NUM_JACO_JOINTS; i ++)
          current_joint_pos[i] = joint_cmd[i];

				//convert joint angles to end effector pose
				jaco_ros::JacoFK fkSrv;
				for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
				{
					fkSrv.request.joints.push_back(joint_cmd[i]);
				}
				if (jaco_fk_client.call(fkSrv))
				{
					//conversion to rpy					
					jaco_ros::QuaternionToEuler qeSrv;
					qeSrv.request.orientation = fkSrv.response.handPose.pose.orientation;
					if (qe_client.call(qeSrv))
					{
						TrajectoryPoint jacoPoint;
						memset(&jacoPoint, 0, sizeof(jacoPoint));
						jacoPoint.Position.CartesianPosition.X = fkSrv.response.handPose.pose.position.x;
						jacoPoint.Position.CartesianPosition.Y = fkSrv.response.handPose.pose.position.y;
						jacoPoint.Position.CartesianPosition.Z = fkSrv.response.handPose.pose.position.z;
						jacoPoint.Position.CartesianPosition.ThetaX = qeSrv.response.roll;
						jacoPoint.Position.CartesianPosition.ThetaY = qeSrv.response.pitch;
						jacoPoint.Position.CartesianPosition.ThetaZ = qeSrv.response.yaw;
					
						ROS_INFO("Trajectory point: (%f, %f, %f); (%f, %f, %f)", jacoPoint.Position.CartesianPosition.X, jacoPoint.Position.CartesianPosition.Y, jacoPoint.Position.CartesianPosition.Z, jacoPoint.Position.CartesianPosition.ThetaX, jacoPoint.Position.CartesianPosition.ThetaY, jacoPoint.Position.CartesianPosition.ThetaZ);
					
						//send point to arm trajectory
						jacoPoint.LimitationsActive = false;
						jacoPoint.Position.Delay = 0.0;
						jacoPoint.Position.HandMode = HAND_NOMOVEMENT;
						jacoPoint.Position.Type = CARTESIAN_POSITION;
						SendBasicTrajectory(jacoPoint);
					}
					else
					{
						ROS_INFO("Quaternion to Euler Angle conversion service failed");
					}
				}
				else
				{
					ROS_INFO("Failed to call forward kinematics service");
				}
		  }
		}

		ros::Rate rate(10);
		int trajectory_size;
		int initialTrajectorySize;
		TrajectoryFIFO Trajectory_Info;
		memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
		GetGlobalTrajectoryInfo(Trajectory_Info);
		trajectory_size = Trajectory_Info.TrajectoryCount;
		initialTrajectorySize = trajectory_size;
		while(trajectory_size > 0)
		{
		  {
				boost::recursive_mutex::scoped_lock lock(api_mutex);

				GetGlobalTrajectoryInfo(Trajectory_Info);
				trajectory_size = Trajectory_Info.TrajectoryCount;
		  }
		  //ROS_INFO("%f, %f, %f, %f, %f, %f", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);
		  ROS_INFO("Trajectory points complete: %d; remaining: %d", initialTrajectorySize - trajectory_size, trajectory_size);
		  rate.sleep();
		}
		ROS_INFO("Trajectory Control Complete.");
		control_msgs::FollowJointTrajectoryResult result;
		result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
		smooth_trajectory_server_.setSucceeded(result);
	}
  
  void JacoArmTrajectoryController::execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal)
  {
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
    bool executionSucceeded = true;
    while(trajectory_size > 0)
    {
      {
				boost::recursive_mutex::scoped_lock lock(api_mutex);

				//check for preempt requests from clients
				if (gripper_server_.isPreemptRequested() || !ros::ok())
      	{
      		//preempt action server
      		ROS_INFO("Gripper action server preempted by client");
      		gripper_server_.setPreempted();
      		executionSucceeded = false;
      
      		//stop gripper control
      		EraseAllTrajectories();
     			StopControlAPI();
      	}

				TrajectoryFIFO Trajectory_Info;
				memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
				GetGlobalTrajectoryInfo(Trajectory_Info);
				trajectory_size = Trajectory_Info.TrajectoryCount;
      }
      rate.sleep();
    }

		if (executionSucceeded)
		{
		  update_joint_states();
		  control_msgs::GripperCommandResult result;
		  result.position = joint_pos[6];
		  result.position = joint_eff[6];
		  result.stalled = false;
		  result.reached_goal = true;
		  gripper_server_.setSucceeded(result);
	  }
  }
  
  void JacoArmTrajectoryController::execute_grasp(const jaco_ros::ExecuteGraspGoalConstPtr &goal)
  {
  	ROS_INFO("Executing grasp");
  
  	jaco_ros::JacoFingerVel fingerCmd;
  	
  	int direction = 1;
  	if (!goal->closeGripper)
  		direction = -1;
  	
  	if (goal->limitFingerVelocity)
  	{
  		fingerCmd.finger1Vel = direction*fabs(goal->fingerVelocities.finger1Vel);
  		fingerCmd.finger2Vel = direction*fabs(goal->fingerVelocities.finger2Vel);
  		fingerCmd.finger3Vel = direction*fabs(goal->fingerVelocities.finger3Vel);
  	}
  	else
  	{
  		fingerCmd.finger1Vel = direction*MAX_FINGER_VEL;
			fingerCmd.finger2Vel = direction*MAX_FINGER_VEL;
			fingerCmd.finger3Vel = direction*MAX_FINGER_VEL;
  	}
  	
  	ROS_INFO("Finger commands: %f, %f, %f", fingerCmd.finger1Vel, fingerCmd.finger2Vel, fingerCmd.finger3Vel);
  	
  	std::vector<float> prevFingerPos;
  	prevFingerPos.resize(3);
  	prevFingerPos.at(0) = joint_pos[6];
  	prevFingerPos.at(1) = joint_pos[7];
  	prevFingerPos.at(2) = joint_pos[8];
  	std::vector<float> currentFingerPos;
  	currentFingerPos.resize(3);
  	currentFingerPos.at(0) = joint_pos[6];
  	currentFingerPos.at(1) = joint_pos[7];
  	currentFingerPos.at(2) = joint_pos[8];
  	
  	moveFingers(fingerCmd);
  	
  	bool finishedGrasp = false;
  	int counter = 0;
  	while (!finishedGrasp)
  	{
  		moveFingers(fingerCmd);
  		counter ++;
  		if (counter == 5)
  		{
  			counter = 0;
  			AngularPosition position_data;
  			{
  				boost::recursive_mutex::scoped_lock lock(api_mutex);
					GetAngularPosition(position_data);
				}
				currentFingerPos.at(0) = position_data.Fingers.Finger1 * DEG_TO_RAD;
				currentFingerPos.at(1) = position_data.Fingers.Finger2 * DEG_TO_RAD;
				currentFingerPos.at(2) = position_data.Fingers.Finger3 * DEG_TO_RAD;
				//ROS_INFO("Fingers are at: %f, %f, %f", currentFingerPos.at(0), currentFingerPos.at(1), currentFingerPos.at(2));
				//ROS_INFO("Fingers were at: %f, %f, %f", prevFingerPos.at(0), prevFingerPos.at(1), prevFingerPos.at(2));
  			//ROS_INFO("---");
  			if (fabs(prevFingerPos.at(0) - currentFingerPos.at(0)) + fabs(prevFingerPos.at(1) - currentFingerPos.at(1)) + fabs(prevFingerPos.at(2) - currentFingerPos.at(2)) == 0.0)
  			{
  				finishedGrasp = true;
  			}
  			else
  			{
  				prevFingerPos.at(0) = currentFingerPos.at(0);
  				prevFingerPos.at(1) = currentFingerPos.at(1);
  				prevFingerPos.at(2) = currentFingerPos.at(2);
				}
  		}
  	}
  	
		jaco_ros::ExecuteGraspResult result;
		result.fingerJoints.resize(3);
		result.fingerJoints.at(0) = joint_pos[6];
		result.fingerJoints.at(1) = joint_pos[7];
		result.fingerJoints.at(2) = joint_pos[8];
	  executeGraspServer.setSucceeded(result);
	  ROS_INFO("Grasp execution succeeded.");
  }
  
  void JacoArmTrajectoryController::cartesianCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
  	{
  		boost::recursive_mutex::scoped_lock lock(api_mutex);
  	
			//take control of the arm
			EraseAllTrajectories();
		  StopControlAPI();
		  StartControlAPI();
		  SetCartesianControl();
		  
		  //populate the velocity command
		  TrajectoryPoint vel;
		  vel.InitStruct();
		  vel.Position.Type = CARTESIAN_VELOCITY;
		  vel.Position.CartesianPosition.X = msg->linear.x;
		  vel.Position.CartesianPosition.Y = msg->linear.y;
		  vel.Position.CartesianPosition.Z = msg->linear.z;
		  vel.Position.CartesianPosition.ThetaX = msg->angular.x;
			vel.Position.CartesianPosition.ThetaY = msg->angular.y;
			vel.Position.CartesianPosition.ThetaZ = msg->angular.z;
		
			//send the velocity command repeatedly for ~1/60th of a second (teleop publishes
			//at 60 Hz)
			EraseAllTrajectories();
			for (int i = 0; i < 8; i ++)
			{
				SendBasicTrajectory(vel);
				usleep(2000);
			}
		}
  }
  
  void JacoArmTrajectoryController::fingerCmdVelCallback(const jaco_ros::JacoFingerVel::ConstPtr& msg)
  {
  	moveFingers(*msg);
  }
  
  void JacoArmTrajectoryController::moveFingers(jaco_ros::JacoFingerVel msg)
  {
  	{
  		boost::recursive_mutex::scoped_lock lock(api_mutex);
  	
			//take control of the arm
			EraseAllTrajectories();
		  StopControlAPI();
		  StartControlAPI();
		  SetCartesianControl();
		  
		  //populate the velocity command
		  TrajectoryPoint vel;
		  vel.InitStruct();
		  vel.Position.Type = CARTESIAN_VELOCITY;
		  vel.Position.CartesianPosition.X = 0.0;
		  vel.Position.CartesianPosition.Y = 0.0;
		  vel.Position.CartesianPosition.Z = 0.0;
		  vel.Position.CartesianPosition.ThetaX = 0.0;
			vel.Position.CartesianPosition.ThetaY = 0.0;
			vel.Position.CartesianPosition.ThetaZ = 0.0;
		  vel.Position.HandMode = VELOCITY_MODE;
		  vel.Position.Fingers.Finger1 = msg.finger1Vel;
		  vel.Position.Fingers.Finger2 = msg.finger2Vel;
		  vel.Position.Fingers.Finger3 = msg.finger3Vel;
		  
			//send the velocity command repeatedly for ~1/60th of a second (teleop publishes
			//at 60 Hz)
			EraseAllTrajectories();
			for (int i = 0; i < 8; i ++)
			{
				SendBasicTrajectory(vel);
				usleep(2000);
			}
		}
  }
  
  void JacoArmTrajectoryController::positionCmdCallback(const geometry_msgs::Pose::ConstPtr& msg)
	{
		{
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      
      //take control of the arm
      EraseAllTrajectories();
      StopControlAPI();
      StartControlAPI();
      SetCartesianControl();

			//convert pose orientation to rpy					
			jaco_ros::QuaternionToEuler qeSrv;
			qeSrv.request.orientation = msg->orientation;
			if (qe_client.call(qeSrv))
			{
				TrajectoryPoint jacoPoint;
				memset(&jacoPoint, 0, sizeof(jacoPoint));
				jacoPoint.Position.CartesianPosition.X = msg->position.x;
				jacoPoint.Position.CartesianPosition.Y = msg->position.y;
				jacoPoint.Position.CartesianPosition.Z = msg->position.z;
				jacoPoint.Position.CartesianPosition.ThetaX = qeSrv.response.roll;
				jacoPoint.Position.CartesianPosition.ThetaY = qeSrv.response.pitch;
				jacoPoint.Position.CartesianPosition.ThetaZ = qeSrv.response.yaw;
			
				//send point to arm trajectory
				jacoPoint.LimitationsActive = false;
				jacoPoint.Position.Delay = 0.0;
				jacoPoint.Position.HandMode = HAND_NOMOVEMENT;
				jacoPoint.Position.Type = CARTESIAN_POSITION;
				SendBasicTrajectory(jacoPoint);
			}
			else
			{
				ROS_INFO("Quaternion to Euler Angle conversion service failed");
			}
		}
	}
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "jaco_arm_trajectory_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  jaco_arm::JacoArmTrajectoryController robot(nh, pnh);
  ros::spin();
  
  /*
  //temporary, for debugging
  ros::Rate loop_rate(10);
  while (ros::ok())
	{
		robot.checkPosition();
		ros::spinOnce();
		loop_rate.sleep();
	}
	*/
}
