#include <jaco_ros/jaco_arm_trajectory_node.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
//#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <boost/foreach.hpp>

using namespace std;

namespace jaco_arm{

  JacoArmTrajectoryController::JacoArmTrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh) : 
    trajectory_server_(nh, "arm_controller", boost::bind(&JacoArmTrajectoryController::execute_trajectory, this, _1), false), 
    smooth_trajectory_server_(nh, "smooth_arm_controller", boost::bind(&JacoArmTrajectoryController::execute_smooth_trajectory, this, _1), false),
    smooth_joint_trajectory_server(nh, "joint_velocity_controller", boost::bind(&JacoArmTrajectoryController::execute_joint_trajectory, this, _1), false),
    gripper_server_(nh, "fingers_controller", boost::bind(&JacoArmTrajectoryController::execute_gripper, this, _1), false),
    executeGraspServer(nh, "jaco_arm/execute_grasp", boost::bind(&JacoArmTrajectoryController::execute_grasp, this, _1), false),
    executePickupServer(nh, "jaco_arm/execute_pickup", boost::bind(&JacoArmTrajectoryController::execute_pickup, this, _1), false)
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
      stringstream joint_name_stream;
      joint_name_stream << "jaco_joint_" << (joint_id+1);
      string joint_name = joint_name_stream.str();
      joint_names.push_back(joint_name);
    }
    for(int finger_id = 0; finger_id < NUM_JACO_FINGER_JOINTS; ++finger_id){
      stringstream finger_name_stream;
      finger_name_stream << "jaco_joint_finger_" << (finger_id+1);
      string finger_name = finger_name_stream.str();
      joint_names.push_back(finger_name);
    }

    StartControlAPI();
    SetAngularControl();
    controlType = ANGULAR_CONTROL;

    // Messages
    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    cartesianCmdPublisher = nh.advertise<jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 1);
    angularCmdPublisher = nh.advertise<jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 1);
    update_joint_states();
    
    cartesianCmdSubscriber = nh.subscribe("jaco_arm/cartesian_cmd", 1, &JacoArmTrajectoryController::cartesianCmdCallback, this);
    angularCmdSubscriber = nh.subscribe("jaco_arm/angular_cmd", 1, &JacoArmTrajectoryController::angularCmdCallback, this);
    
    // Services
    jaco_fk_client = nh.serviceClient<jaco_msgs::JacoFK>("jaco_fk");
    eq_client = nh.serviceClient<jaco_msgs::EulerToQuaternion>("jaco_conversions/euler_to_quaternion");
    qe_client = nh.serviceClient<jaco_msgs::QuaternionToEuler>("jaco_conversions/quaternion_to_euler");

    // Action servers
    trajectory_server_.start();    
    smooth_trajectory_server_.start();
    smooth_joint_trajectory_server.start();
    gripper_server_.start();
    executeGraspServer.start();
    executePickupServer.start();
    
    joint_state_timer_ = nh.createTimer(ros::Duration(0.0333), boost::bind(&JacoArmTrajectoryController::update_joint_states, this));
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
    if (fabs(angle - previous_rev) < fabs(angle - next_rev))
      return angle - previous_rev;
    return angle - next_rev;
  }

  void JacoArmTrajectoryController::update_joint_states()
  {
    {
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
    }

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


  /*****************************************/
  /**********  Trajectory Control **********/
  /*****************************************/
  
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
          string joint_name = goal->trajectory.joint_names[trajectory_index];
          int joint_index = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), joint_name));
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
      //check for preempt requests from clients
      if (trajectory_server_.isPreemptRequested() || !ros::ok())
      {
        //stop gripper control
        {
          boost::recursive_mutex::scoped_lock lock(api_mutex);
          EraseAllTrajectories();
          StopControlAPI();
        }
        
        //preempt action server
        trajectory_server_.setPreempted();
        ROS_INFO("Joint trajectory server preempted by client");
        
        return;
      }
    
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
    
    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      EraseAllTrajectories();
      StopControlAPI();
    }
    
    ROS_INFO("Trajectory Control Complete.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    trajectory_server_.setSucceeded(result);
  }
  
  
  /*****************************************/
  /*****  Smoothed Trajectory Control ******/
  /*****************************************/
  
  void JacoArmTrajectoryController::execute_smooth_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
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
        double joint_cmd[NUM_JACO_JOINTS];
        for(int trajectory_index = 0; trajectory_index < goal->trajectory.joint_names.size(); ++trajectory_index)
        {
          string joint_name = goal->trajectory.joint_names[trajectory_index];
          int joint_index = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), joint_name));
          if(joint_index >= 0 && joint_index < NUM_JACO_JOINTS)
          {
            //convert angles from continuous joints to be correct for the arm API
            if (joint_index != 1 && joint_index != 2)
              joint_cmd[joint_index] = nearest_equivalent(simplify_angle(point.positions[trajectory_index]), current_joint_pos[joint_index]);
            else
              joint_cmd[joint_index] = point.positions[trajectory_index];
          }
        }
        
        for (int i = 0; i < NUM_JACO_JOINTS; i ++)
          current_joint_pos[i] = joint_cmd[i];

        //convert joint angles to end effector pose
        jaco_msgs::JacoFK fkSrv;
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
        {
          fkSrv.request.joints.push_back(joint_cmd[i]);
        }
        if (jaco_fk_client.call(fkSrv))
        {
          //conversion to rpy          
          jaco_msgs::QuaternionToEuler qeSrv;
          qeSrv.request.orientation = fkSrv.response.handPose.pose.orientation;
          if (qe_client.call(qeSrv))
          {
            TrajectoryPoint trajPoint;
            memset(&trajPoint, 0, sizeof(trajPoint));
            trajPoint.Position.Type = CARTESIAN_POSITION;
            trajPoint.Position.CartesianPosition.X = fkSrv.response.handPose.pose.position.x;
            trajPoint.Position.CartesianPosition.Y = fkSrv.response.handPose.pose.position.y;
            trajPoint.Position.CartesianPosition.Z = fkSrv.response.handPose.pose.position.z;
            trajPoint.Position.CartesianPosition.ThetaX = qeSrv.response.roll;
            trajPoint.Position.CartesianPosition.ThetaY = qeSrv.response.pitch;
            trajPoint.Position.CartesianPosition.ThetaZ = qeSrv.response.yaw;
            trajPoint.Position.HandMode = HAND_NOMOVEMENT;
          
            //for debugging:
            //ROS_INFO("Trajectory point: (%f, %f, %f); (%f, %f, %f)", trajPoint.Position.CartesianPosition.X, trajPoint.Position.CartesianPosition.Y, trajPoint.Position.CartesianPosition.Z, trajPoint.Position.CartesianPosition.ThetaX, trajPoint.Position.CartesianPosition.ThetaY, trajPoint.Position.CartesianPosition.ThetaZ);
          
            //send point to arm trajectory
            SendBasicTrajectory(trajPoint);
          }
          else
          {
            ROS_INFO("Quaternion to Euler Angle conversion service failed");
            {
              boost::recursive_mutex::scoped_lock lock(api_mutex);
              EraseAllTrajectories();
              StopControlAPI();
            }
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
            smooth_trajectory_server_.setSucceeded(result);
            return;
          }
        }
        else
        {
          ROS_INFO("Failed to call forward kinematics service");
          {
            boost::recursive_mutex::scoped_lock lock(api_mutex);
            EraseAllTrajectories();
            StopControlAPI();
          }
          control_msgs::FollowJointTrajectoryResult result;
          result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
          smooth_trajectory_server_.setSucceeded(result);
          return;
        }
      }
    }

    //wait for trajectory to complete execution
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
      //check for preempt requests from clients
      if (smooth_trajectory_server_.isPreemptRequested() || !ros::ok())
      {
        //stop gripper control
        {
          boost::recursive_mutex::scoped_lock lock(api_mutex);
          EraseAllTrajectories();
          StopControlAPI();
        }
        
        //preempt action server
        smooth_trajectory_server_.setPreempted();
        ROS_INFO("Smooth trajectory server preempted by client");
        
        return;
      }
      
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);

        GetGlobalTrajectoryInfo(Trajectory_Info);
        trajectory_size = Trajectory_Info.TrajectoryCount;
      }
      //ROS_INFO("%f, %f, %f, %f, %f, %f", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);
      //ROS_INFO("Trajectory points complete: %d; remaining: %d", initialTrajectorySize - trajectory_size, trajectory_size);
      rate.sleep();
    }
    ROS_INFO("Trajectory Control Complete.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smooth_trajectory_server_.setSucceeded(result);
  }
  
  void JacoArmTrajectoryController::execute_joint_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    float trajectoryPoints[NUM_JACO_JOINTS][goal->trajectory.points.size()];
    int numPoints = goal->trajectory.points.size();
    
    //get trajectory data
    for (unsigned int i = 0; i < numPoints; i ++)
    {
      for (unsigned int j = 0; j < NUM_JACO_JOINTS; j ++)
      {
        trajectoryPoints[j][i] = goal->trajectory.points.at(i).positions.at(j);
      }
    }
    
    //initialize arrays needed to fit a smooth trajectory to the given points
    ecl::Array<double> timePoints(numPoints);
    timePoints[0] = 0.0;
    vector< ecl::Array<double> > jointPoints;
    jointPoints.resize(NUM_JACO_JOINTS);
    float prevPoint[NUM_JACO_JOINTS];
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
    {
      jointPoints[i].resize(numPoints);
      jointPoints[i][0] = trajectoryPoints[i][0];
      prevPoint[i] = trajectoryPoints[i][0];
    }
    
    //determine time component of trajectories for each joint
    for (unsigned int i = 1; i < numPoints; i ++)
    {
      float maxTime = 0.0;
      for (unsigned int j = 0; j < NUM_JACO_JOINTS; j ++)
      {
        //calculate approximate time required to move to the next position
        float time = fabs(trajectoryPoints[j][i] - prevPoint[j]);
        if (j <= 2)
          time /= LARGE_ACTUATOR_VELOCITY;
        else
          time /= SMALL_ACTUATOR_VELOCITY;
        
        if (time > maxTime)
          maxTime = time;
        
        jointPoints[j][i] = trajectoryPoints[j][i];
        prevPoint[j] = trajectoryPoints[j][i];
      }
      
      timePoints[i] = timePoints[i-1] + maxTime*TIME_SCALING_FACTOR;
    }
    
    double max_curvature = 6.0;  //NOTE: this may need adjustment depending on the source of the trajectory points; this value seems to work fine for trajectories generated by MoveIt!.
    vector<ecl::SmoothLinearSpline> splines;
    splines.resize(6);
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
    {
      ecl::SmoothLinearSpline tempSpline(timePoints, jointPoints[i], max_curvature);
      splines.at(i) = tempSpline;
    }
    
    //control loop
    bool trajectoryComplete = false;
    double startTime = ros::Time::now().toSec();
    double t = 0;
    float error[NUM_JACO_JOINTS];
    float totalError;
    float prevError[NUM_JACO_JOINTS] = {0};
    float currentPoint;
    double current_joint_pos[NUM_JACO_JOINTS];
    AngularPosition position_data;
    TrajectoryPoint trajPoint;
    trajPoint.InitStruct();
    trajPoint.Position.Type = ANGULAR_VELOCITY;
    trajPoint.Position.HandMode = HAND_NOMOVEMENT;
    
    ros::Rate rate(600);
    
    while (!trajectoryComplete)
    {
      //update_joint_states();
      
      //check for preempt requests from clients
      if (smooth_joint_trajectory_server.isPreemptRequested())
      {
        //stop gripper control
        trajPoint.Position.Actuators.Actuator1 = 0.0;
        trajPoint.Position.Actuators.Actuator2 = 0.0;
        trajPoint.Position.Actuators.Actuator3 = 0.0;
        trajPoint.Position.Actuators.Actuator4 = 0.0;
        trajPoint.Position.Actuators.Actuator5 = 0.0;
        trajPoint.Position.Actuators.Actuator6 = 0.0;
        executeTrajectoryPoint(trajPoint, true);
        
        //preempt action server
        smooth_joint_trajectory_server.setPreempted();
        ROS_INFO("Joint trajectory server preempted by client");
        
        return;
      }
    
      //get time for trajectory
      t = ros::Time::now().toSec() - startTime;
      if (t > timePoints.at(timePoints.size() - 1))
      {
        //use final trajectory point as the goal to calculate error until the error
        //is small enough to be considered successful
        {
          boost::recursive_mutex::scoped_lock lock(api_mutex);
          GetAngularPosition(position_data);
        }
        current_joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
        current_joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
        current_joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
        current_joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
        current_joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
        current_joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;
        
        totalError = 0;
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
        {
          currentPoint = simplify_angle(current_joint_pos[i]);
          error[i] = nearest_equivalent(simplify_angle((splines.at(i))(timePoints.at(timePoints.size() - 1))), currentPoint) - currentPoint;
          totalError += fabs(error[i]);
        }
        
        if (totalError < .03)
        {
          trajPoint.Position.Actuators.Actuator1 = 0.0;
          trajPoint.Position.Actuators.Actuator2 = 0.0;
          trajPoint.Position.Actuators.Actuator3 = 0.0;
          trajPoint.Position.Actuators.Actuator4 = 0.0;
          trajPoint.Position.Actuators.Actuator5 = 0.0;
          trajPoint.Position.Actuators.Actuator6 = 0.0;
          executeTrajectoryPoint(trajPoint, true);
          trajectoryComplete = true;
          ROS_INFO("Trajectory complete!");
          break;
        }
      }
      else
      {
        //calculate error
        {
          boost::recursive_mutex::scoped_lock lock(api_mutex);
          GetAngularPosition(position_data);
        }
        current_joint_pos[0] = position_data.Actuators.Actuator1 * DEG_TO_RAD;
        current_joint_pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
        current_joint_pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
        current_joint_pos[3] = position_data.Actuators.Actuator4 * DEG_TO_RAD;
        current_joint_pos[4] = position_data.Actuators.Actuator5 * DEG_TO_RAD;
        current_joint_pos[5] = position_data.Actuators.Actuator6 * DEG_TO_RAD;
        for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
        {
          currentPoint = simplify_angle(current_joint_pos[i]);
          error[i] = nearest_equivalent(simplify_angle((splines.at(i))(t)), currentPoint) - currentPoint;
        }
      }
      
      //calculate control input
      //populate the velocity command
      trajPoint.Position.Actuators.Actuator1 = (KP*error[0] + KV*(error[0] - prevError[0]) * RAD_TO_DEG);
      trajPoint.Position.Actuators.Actuator2 = (KP*error[1] + KV*(error[1] - prevError[1]) * RAD_TO_DEG);
      trajPoint.Position.Actuators.Actuator3  = (KP*error[2] + KV*(error[2] - prevError[2]) * RAD_TO_DEG);
      trajPoint.Position.Actuators.Actuator4 = (KP*error[3] + KV*(error[3] - prevError[3]) * RAD_TO_DEG);
      trajPoint.Position.Actuators.Actuator5 = (KP*error[4] + KV*(error[4] - prevError[4]) * RAD_TO_DEG);
      trajPoint.Position.Actuators.Actuator6 = (KP*error[5] + KV*(error[5] - prevError[5]) * RAD_TO_DEG);
      
      //for debugging:
      //cout << "Errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << endl;
    
      //send the velocity command
      executeTrajectoryPoint(trajPoint, true);
      
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
      {
        prevError[i] = error[i];
      }
      
      rate.sleep();
    }
    
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smooth_joint_trajectory_server.setSucceeded(result);
  }
  
  
  /*****************************************/
  /***********  Gripper Control ************/
  /*****************************************/
  
  void JacoArmTrajectoryController::execute_gripper(const control_msgs::GripperCommandGoalConstPtr &goal)
  {
    jaco_msgs::AngularCommand cmd;
    cmd.position = true;
    cmd.armCommand = false;
    cmd.fingerCommand = true;
    cmd.repeat = false;
    cmd.fingers.resize(3);
    cmd.fingers[0] = goal->command.position;
    cmd.fingers[1] = goal->command.position;
    cmd.fingers[2] = goal->command.position;

    cartesianCmdPublisher.publish(cmd);

    ros::Rate rate(10);
    int trajectory_size = 1;
    while(trajectory_size > 0)
    {
      //check for preempt requests from clients
      if (gripper_server_.isPreemptRequested() || !ros::ok())
      {
        //stop gripper control
        cmd.position = false;
        cmd.fingers[0] = 0.0;
        cmd.fingers[1] = 0.0;
        cmd.fingers[2] = 0.0;
        cartesianCmdPublisher.publish(cmd);
      
        //preempt action server
        ROS_INFO("Gripper action server preempted by client");
        gripper_server_.setPreempted();
    
        return;
      }

      TrajectoryFIFO Trajectory_Info;
      memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        GetGlobalTrajectoryInfo(Trajectory_Info);
      }
      trajectory_size = Trajectory_Info.TrajectoryCount;
      rate.sleep();
    }
    
    //stop gripper control
    cmd.position = false;
    cmd.fingers[0] = 0.0;
    cmd.fingers[1] = 0.0;
    cmd.fingers[2] = 0.0;
    cartesianCmdPublisher.publish(cmd);
    
    control_msgs::GripperCommandResult result;
    result.position = joint_pos[6];
    result.effort = joint_eff[6];
    result.stalled = false;
    result.reached_goal = true;
    gripper_server_.setSucceeded(result);
  }
  
  void JacoArmTrajectoryController::execute_grasp(const jaco_msgs::ExecuteGraspGoalConstPtr &goal)
  {   
    jaco_msgs::CartesianCommand cmd;
    cmd.position = false;
    cmd.armCommand = false;
    cmd.fingerCommand = true;
    cmd.repeat = true;
    cmd.fingers.resize(3);
    
    //set direction for opening and closing
    int direction = 1;
    if (!goal->closeGripper)
      direction = -1;
    
    //set finger velocities if they were specified
    if (goal->limitFingerVelocity)
    {
      cmd.fingers[0] = direction*fabs(goal->fingerVelocities.finger1Vel);
      cmd.fingers[1] = direction*fabs(goal->fingerVelocities.finger2Vel);
      cmd.fingers[2] = direction*fabs(goal->fingerVelocities.finger3Vel);
    }
    else
    {
      cmd.fingers[0] = direction*MAX_FINGER_VEL;
      cmd.fingers[1] = direction*MAX_FINGER_VEL;
      cmd.fingers[2] = direction*MAX_FINGER_VEL;
    }
    
    //get initial finger position
    float prevFingerPos[3];
    prevFingerPos[0] = joint_pos[6];
    prevFingerPos[1] = joint_pos[7];
    prevFingerPos[2] = joint_pos[8];
    
    float currentFingerPos[3];
    currentFingerPos[0] = prevFingerPos[0];
    currentFingerPos[1] = prevFingerPos[1];
    currentFingerPos[2] = prevFingerPos[2];
    
    double prevCheckTime = ros::Time::now().toSec();
    bool finishedGrasp = false;
    while (!finishedGrasp)
    {
      //check for preempt requests from clients
      if (executeGraspServer.isPreemptRequested() || !ros::ok())
      {
        //stop gripper control
        cmd.fingers[0] = 0.0;
        cmd.fingers[1] = 0.0;
        cmd.fingers[2] = 0.0;
        cartesianCmdPublisher.publish(cmd);
        
        //preempt action server
        executeGraspServer.setPreempted();
        ROS_INFO("Grasp action server preempted by client");
        
        return;
      }
      
      cartesianCmdPublisher.publish(cmd);
      
      if (ros::Time::now().toSec() - prevCheckTime > .25)  //occaisionally check to see if the fingers have stopped moving
      {
        prevCheckTime = ros::Time::now().toSec();
        currentFingerPos[0] = joint_pos[6];
        currentFingerPos[1] = joint_pos[7];
        currentFingerPos[2] = joint_pos[8];
        
        //grasp is finished if fingers haven't moved since the last check
        if ((fabs(prevFingerPos[0] - currentFingerPos[0]) + fabs(prevFingerPos[1] - currentFingerPos[1]) + fabs(prevFingerPos[2] - currentFingerPos[2])) == 0.0)
        {
          finishedGrasp = true;
        }
        else
        {
          prevFingerPos[0] = currentFingerPos[0];
          prevFingerPos[1] = currentFingerPos[1];
          prevFingerPos[2] = currentFingerPos[2];
        }
      }
    }
    
    //stop arm
    cmd.fingers[0] = 0.0;
    cmd.fingers[1] = 0.0;
    cmd.fingers[2] = 0.0;
    cartesianCmdPublisher.publish(cmd);
    
    jaco_msgs::ExecuteGraspResult result;
    result.fingerJoints.resize(3);
    result.fingerJoints.at(0) = joint_pos[6];
    result.fingerJoints.at(1) = joint_pos[7];
    result.fingerJoints.at(2) = joint_pos[8];
    executeGraspServer.setSucceeded(result);
    ROS_INFO("Grasp execution complete");
  }
  
  void JacoArmTrajectoryController::execute_pickup(const jaco_msgs::ExecutePickupGoalConstPtr &goal)
  {  
    //get initial end effector height
    CartesianPosition pos;
    {
    	boost::recursive_mutex::scoped_lock lock(api_mutex);
    	GetCartesianPosition(pos);
  	}
  	float initialZ = pos.Coordinates.Z;
    
    //populate the velocity command
    jaco_msgs::CartesianCommand cmd;
    cmd.position = false;
    cmd.armCommand = true;
    cmd.fingerCommand = true;
    cmd.repeat = true;
    cmd.fingers.resize(3);
    cmd.arm.linear.x = 0.0;
    cmd.arm.linear.y = 0.0;
    if (goal->setLiftVelocity)
	    cmd.arm.linear.z = goal->liftVelocity;
	  else
	  	cmd.arm.linear.z = DEFAULT_LIFT_VEL;
    cmd.arm.angular.x = 0.0;
    cmd.arm.angular.y = 0.0;
    cmd.arm.angular.z = 0.0;
    if (goal->limitFingerVelocity)
    {
    	cmd.fingers[0] = fabs(goal->fingerVelocities.finger1Vel);
    	cmd.fingers[1] = fabs(goal->fingerVelocities.finger2Vel);
    	cmd.fingers[2] = fabs(goal->fingerVelocities.finger3Vel);
  	}
  	else
  	{
  		cmd.fingers[0] = MAX_FINGER_VEL;
  		cmd.fingers[1] = MAX_FINGER_VEL;
  		cmd.fingers[2] = MAX_FINGER_VEL;
		}
    
    bool finished = false;
    double startTime = ros::Time::now().toSec();
		jaco_msgs::ExecutePickupResult result;
		
		//send the lift and close command until a certain height has been reached, or the
		//action times out
    while (!finished)
    {
    	//update_joint_states();
    
    	//check for preempt requests from clients
      if (executePickupServer.isPreemptRequested() || !ros::ok())
      {
        //stop pickup action
        {
          boost::recursive_mutex::scoped_lock lock(api_mutex);
          EraseAllTrajectories();
        }
        
        //preempt action server
        executePickupServer.setPreempted();
        ROS_INFO("Pickup action server preempted by client");
        
        return;
      }
    
    	cartesianCmdPublisher.publish(cmd);
    	{
    	  boost::recursive_mutex::scoped_lock lock(api_mutex);
      	//check position
		  	GetCartesianPosition(pos);
	  	}
    	
			if (pos.Coordinates.Z - initialZ >= LIFT_HEIGHT)
			{
				finished = true;
				result.success = true;
			}
			else if (ros::Time::now().toSec() - startTime >= LIFT_TIMEOUT)
			{
				finished = true;
				result.success = false;
			}
	  }
    
    //stop arm
    cmd.fingers.resize(3);
    cmd.arm.linear.x = 0.0;
    cmd.arm.linear.y = 0.0;
  	cmd.arm.linear.z = 0.0;
    cmd.arm.angular.x = 0.0;
    cmd.arm.angular.y = 0.0;
    cmd.arm.angular.z = 0.0;
  	cmd.fingers[0] = 0.0;
  	cmd.fingers[1] = 0.0;
  	cmd.fingers[2] = 0.0;
  	cartesianCmdPublisher.publish(cmd);
    
    executePickupServer.setSucceeded(result);
    ROS_INFO("Pickup execution complete");
  }
  
  /*****************************************/
  /**********  Basic Arm Commands **********/
  /*****************************************/
  
  void JacoArmTrajectoryController::angularCmdCallback(const jaco_msgs::AngularCommand& msg)
  {
  	boost::recursive_mutex::scoped_lock lock(api_mutex);
  	
  	//take control of the arm
    EraseAllTrajectories();
    
    if (controlType != ANGULAR_CONTROL)
    {
      SetAngularControl();
      controlType = ANGULAR_CONTROL;
    }
    
    TrajectoryPoint jacoPoint;
    jacoPoint.InitStruct();
    
    //populate arm command
    if (msg.armCommand)
    {
      if (msg.position)
        jacoPoint.Position.Type = ANGULAR_POSITION;
      else
        jacoPoint.Position.Type = ANGULAR_VELOCITY;
      jacoPoint.Position.Actuators.Actuator1 = msg.joints[0];
      jacoPoint.Position.Actuators.Actuator2 = msg.joints[1];
      jacoPoint.Position.Actuators.Actuator3 = msg.joints[2];
      jacoPoint.Position.Actuators.Actuator4 = msg.joints[3];
      jacoPoint.Position.Actuators.Actuator5 = msg.joints[4];
      jacoPoint.Position.Actuators.Actuator6 = msg.joints[5];
    }
    else
    {
      jacoPoint.Position.Type = ANGULAR_VELOCITY;
      jacoPoint.Position.Actuators.Actuator1 = 0.0;
      jacoPoint.Position.Actuators.Actuator2 = 0.0;
      jacoPoint.Position.Actuators.Actuator3 = 0.0;
      jacoPoint.Position.Actuators.Actuator4 = 0.0;
      jacoPoint.Position.Actuators.Actuator5 = 0.0;
      jacoPoint.Position.Actuators.Actuator6 = 0.0;
    }
    
    //populate finger command
    if (msg.fingerCommand)
    {
      if (msg.position)
        jacoPoint.Position.HandMode = POSITION_MODE;
      else
        jacoPoint.Position.HandMode = VELOCITY_MODE;
      jacoPoint.Position.Fingers.Finger1 = msg.fingers[0];
      jacoPoint.Position.Fingers.Finger2 = msg.fingers[1];
      jacoPoint.Position.Fingers.Finger3 = msg.fingers[2];
    }
    else
      jacoPoint.Position.HandMode = HAND_NOMOVEMENT;
    
    //send command
    if (msg.position)
    {
      SendBasicTrajectory(jacoPoint);
    }
    else
    {
      if (msg.repeat)
      {
        //send the command repeatedly for ~1/60th of a second
        //(this is sometimes necessary for velocity commands to work correctly)
        ros::Rate rate(600);
        for (int i = 0; i < 10; i ++)
        {
          SendBasicTrajectory(jacoPoint);
          rate.sleep();
        }
      }
      else
        SendBasicTrajectory(jacoPoint);
    }
  }
  
  void JacoArmTrajectoryController::cartesianCmdCallback(const jaco_msgs::CartesianCommand& msg)
  {
  	boost::recursive_mutex::scoped_lock lock(api_mutex);
  	
  	//take control of the arm
    EraseAllTrajectories();
    
    if (controlType != CARTESIAN_CONTROL)
    {
      SetCartesianControl();
      controlType = CARTESIAN_CONTROL;
    }
    
    TrajectoryPoint jacoPoint;
    jacoPoint.InitStruct();
    
    //populate arm command
    if (msg.armCommand)
    {
      if (msg.position)
        jacoPoint.Position.Type = CARTESIAN_POSITION;
      else
        jacoPoint.Position.Type = CARTESIAN_VELOCITY;
      jacoPoint.Position.CartesianPosition.X = msg.arm.linear.x;
      jacoPoint.Position.CartesianPosition.Y = msg.arm.linear.y;
      jacoPoint.Position.CartesianPosition.Z = msg.arm.linear.z;
      jacoPoint.Position.CartesianPosition.ThetaX = msg.arm.angular.x;
      jacoPoint.Position.CartesianPosition.ThetaY = msg.arm.angular.y;
      jacoPoint.Position.CartesianPosition.ThetaZ = msg.arm.angular.z;
    }
    else
    {
      jacoPoint.Position.Type = CARTESIAN_VELOCITY;
      jacoPoint.Position.CartesianPosition.X = 0.0;
      jacoPoint.Position.CartesianPosition.Y = 0.0;
      jacoPoint.Position.CartesianPosition.Z = 0.0;
      jacoPoint.Position.CartesianPosition.ThetaX = 0.0;
      jacoPoint.Position.CartesianPosition.ThetaY = 0.0;
      jacoPoint.Position.CartesianPosition.ThetaZ = 0.0;
    }
    EraseAllTrajectories();
    //populate finger command
    if (msg.fingerCommand)
    {
      if (msg.position)
        jacoPoint.Position.HandMode = POSITION_MODE;
      else
        jacoPoint.Position.HandMode = VELOCITY_MODE;
      jacoPoint.Position.Fingers.Finger1 = msg.fingers[0];
      jacoPoint.Position.Fingers.Finger2 = msg.fingers[1];
      jacoPoint.Position.Fingers.Finger3 = msg.fingers[2];
    }
    else
      jacoPoint.Position.HandMode = HAND_NOMOVEMENT;
    
    //send command
    if (msg.position)
    {
      SendBasicTrajectory(jacoPoint);
    }
    else
    {
      SendBasicTrajectory(jacoPoint);
      if (msg.repeat)
      {
        //send the command repeatedly for ~1/60th of a second
        //(this is sometimes necessary for velocity commands to work correctly)
        ros::Rate rate(600);
        for (int i = 0; i < 10; i ++)
        {
          SendBasicTrajectory(jacoPoint);
          rate.sleep();
        }
      }
    }
  }
  
  void JacoArmTrajectoryController::executeTrajectoryPoint(TrajectoryPoint point, bool erase)
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    
    if (controlType != ANGULAR_CONTROL)
    {
      SetAngularControl();
      controlType = ANGULAR_CONTROL;
    }
    
    if (erase)
      EraseAllTrajectories();
      
    SendBasicTrajectory(point);
  }
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "jaco_arm_trajectory_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  jaco_arm::JacoArmTrajectoryController robot(nh, pnh);
  ros::spin();
}
