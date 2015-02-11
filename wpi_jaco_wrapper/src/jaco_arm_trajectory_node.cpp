#include <wpi_jaco_wrapper/jaco_arm_trajectory_node.h>

using namespace std;

namespace jaco
{
JacoArmTrajectoryController::JacoArmTrajectoryController(ros::NodeHandle nh, ros::NodeHandle pnh) :
    trajectory_server_(nh, "jaco_arm/arm_controller",
                       boost::bind(&JacoArmTrajectoryController::execute_trajectory, this, _1), false), smooth_trajectory_server_(
        nh, "jaco_arm/smooth_arm_controller",
        boost::bind(&JacoArmTrajectoryController::execute_smooth_trajectory, this, _1), false), smooth_joint_trajectory_server(
        nh, "jaco_arm/joint_velocity_controller",
        boost::bind(&JacoArmTrajectoryController::execute_joint_trajectory, this, _1), false), gripper_server_(
        nh, "jaco_arm/fingers_controller", boost::bind(&JacoArmTrajectoryController::execute_gripper, this, _1), false), home_arm_server(
        nh, "jaco_arm/home_arm", boost::bind(&JacoArmTrajectoryController::home_arm, this, _1), false)
{
  ros::NodeHandle private_nh("~");
  private_nh.param<double>("max_curvature", max_curvature, 10.0);

  ROS_INFO("max_curvature: %f", max_curvature);

  boost::recursive_mutex::scoped_lock lock(api_mutex);

  //ROS_INFO("Trying to initialize JACO API...");
  InitAPI();
  //ROS_INFO("Api initialized.");
  ros::Duration(1.0).sleep();
  //ROS_INFO("Starting control API...");
  StartControlAPI();
  //ROS_INFO("Control API started...");
  ros::Duration(3.0).sleep();
  //ROS_INFO("Stopping control API...");
  StopControlAPI();
  //ROS_INFO("Control API stopped.");

  // Initialize arm
  //ROS_INFO("Homing arm...");
  MoveHome();
  //ROS_INFO("Done.");
  //ROS_INFO("Initializing fingers...");
  InitFingers();
  //ROS_INFO("Done.");
  SetFrameType(0); //set end effector to move with respect to the fixed frame

  // Initialize joint names
  for (int joint_id = 0; joint_id < NUM_JACO_JOINTS; ++joint_id)
  {
    stringstream joint_name_stream;
    joint_name_stream << "jaco_joint_" << (joint_id + 1);
    string joint_name = joint_name_stream.str();
    joint_names.push_back(joint_name);
  }
  for (int finger_id = 0; finger_id < NUM_JACO_FINGER_JOINTS; ++finger_id)
  {
    stringstream finger_name_stream;
    finger_name_stream << "jaco_joint_finger_" << (finger_id + 1);
    string finger_name = finger_name_stream.str();
    joint_names.push_back(finger_name);
  }

  StartControlAPI();
  SetAngularControl();
  controlType = ANGULAR_CONTROL;
  eStopEnabled = false;

  // Messages
  joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("jaco_arm/joint_states", 1);
  cartesianCmdPublisher = nh.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 1);
  angularCmdPublisher = nh.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 1);
  update_joint_states();

  cartesianCmdSubscriber = nh.subscribe("jaco_arm/cartesian_cmd", 1, &JacoArmTrajectoryController::cartesianCmdCallback,
                                        this);
  angularCmdSubscriber = nh.subscribe("jaco_arm/angular_cmd", 1, &JacoArmTrajectoryController::angularCmdCallback,
                                      this);

  // Services
  jaco_fk_client = nh.serviceClient<wpi_jaco_msgs::JacoFK>("jaco_arm/kinematics/fk");
  qe_client = nh.serviceClient<wpi_jaco_msgs::QuaternionToEuler>("jaco_conversions/quaternion_to_euler");
  angularPositionServer = nh.advertiseService("jaco_arm/get_angular_position", &JacoArmTrajectoryController::getAngularPosition, this);
  cartesianPositionServer = nh.advertiseService("jaco_arm/get_cartesian_position",
                                                &JacoArmTrajectoryController::getCartesianPosition, this);
  eStopServer = nh.advertiseService("jaco_arm/software_estop", &JacoArmTrajectoryController::eStopCallback, this);
  eraseTrajectoriesServer = nh.advertiseService("jaco_arm/erase_trajectories", &JacoArmTrajectoryController::eraseTrajectoriesCallback, this);

  // Action servers
  trajectory_server_.start();
  smooth_trajectory_server_.start();
  smooth_joint_trajectory_server.start();
  gripper_server_.start();
  home_arm_server.start();

  joint_state_timer_ = nh.createTimer(ros::Duration(0.0333),
                                      boost::bind(&JacoArmTrajectoryController::update_joint_states, this));
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
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
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
    //NOTE: the finger units are arbitrary, but converting them as if they were in degrees provides an approximately correct visualization
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
  state.position.assign(joint_pos, joint_pos + NUM_JOINTS);
  state.velocity.assign(joint_vel, joint_vel + NUM_JOINTS);
  state.effort.assign(joint_eff, joint_eff + NUM_JOINTS);
  joint_state_pub_.publish(state);
}

/** Calculates nearest desired angle to the current angle
 *  @param desired desired joint angle [-pi, pi]
 *  @param current current angle (-inf, inf)
 *  @return the closest equivalent angle (-inf, inf)
 */
static inline double nearest_equivalent(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
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
  //cancel check
  if (eStopEnabled)
  {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
    trajectory_server_.setSucceeded(result);
    return;
  }

  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    EraseAllTrajectories();
  }

  update_joint_states();
  double current_joint_pos[NUM_JACO_JOINTS];

  AngularPosition position_data;
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

  //initialize trajectory point
  TrajectoryPoint trajPoint;
  trajPoint.InitStruct();
  trajPoint.Position.Type = ANGULAR_POSITION;
  trajPoint.Position.HandMode = HAND_NOMOVEMENT;
  BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point, goal->trajectory.points)
  {
    ROS_INFO("Trajectory Point");
    double joint_cmd[NUM_JACO_JOINTS];
    for (int trajectory_index = 0; trajectory_index < goal->trajectory.joint_names.size(); ++trajectory_index)
    {
      string joint_name = goal->trajectory.joint_names[trajectory_index];
      int joint_index = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), joint_name));
      if (joint_index >= 0 && joint_index < NUM_JACO_JOINTS)
      {
        //ROS_INFO("Before: %s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, point.positions[trajectory_index]);
        if (joint_index != 1 && joint_index != 2)
          joint_cmd[joint_index] = nearest_equivalent(simplify_angle(point.positions[trajectory_index]),
                                                      current_joint_pos[joint_index]);
        else
          joint_cmd[joint_index] = point.positions[trajectory_index];
        //ROS_INFO("After:  %s: (%d -> %d) = %f", joint_name.c_str(), trajectory_index, joint_index, joint_cmd[joint_index]);
      }
    }
    for (int i = 0; i < NUM_JACO_JOINTS; i++)
      current_joint_pos[i] = joint_cmd[i];

    AngularInfo angles;
    angles.Actuator1 = joint_cmd[0] * RAD_TO_DEG;
    angles.Actuator2 = joint_cmd[1] * RAD_TO_DEG;
    angles.Actuator3 = joint_cmd[2] * RAD_TO_DEG;
    angles.Actuator4 = joint_cmd[3] * RAD_TO_DEG;
    angles.Actuator5 = joint_cmd[4] * RAD_TO_DEG;
    angles.Actuator6 = joint_cmd[5] * RAD_TO_DEG;

    trajPoint.Position.Actuators = angles;

    executeAngularTrajectoryPoint(trajPoint, false);
  }

  ros::Rate rate(10);
  int trajectory_size;
  while (trajectory_size > 0)
  {
    //cancel check
    if (eStopEnabled)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      trajectory_server_.setSucceeded(result);
      return;
    }

    //check for preempt requests from clients
    if (trajectory_server_.isPreemptRequested() || !ros::ok())
    {
      //stop gripper control
      trajPoint.Position.Type = ANGULAR_VELOCITY;
      trajPoint.Position.Actuators.Actuator1 = 0.0;
      trajPoint.Position.Actuators.Actuator2 = 0.0;
      trajPoint.Position.Actuators.Actuator3 = 0.0;
      trajPoint.Position.Actuators.Actuator4 = 0.0;
      trajPoint.Position.Actuators.Actuator5 = 0.0;
      trajPoint.Position.Actuators.Actuator6 = 0.0;
      executeAngularTrajectoryPoint(trajPoint, true);

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

  //stop gripper control
  trajPoint.Position.Type = ANGULAR_VELOCITY;
  trajPoint.Position.Actuators.Actuator1 = 0.0;
  trajPoint.Position.Actuators.Actuator2 = 0.0;
  trajPoint.Position.Actuators.Actuator3 = 0.0;
  trajPoint.Position.Actuators.Actuator4 = 0.0;
  trajPoint.Position.Actuators.Actuator5 = 0.0;
  trajPoint.Position.Actuators.Actuator6 = 0.0;
  executeAngularTrajectoryPoint(trajPoint, true);

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
  //cancel check
  if (eStopEnabled)
  {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
    smooth_trajectory_server_.setSucceeded(result);
    return;
  }

  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    EraseAllTrajectories();
  }

  update_joint_states();
  double current_joint_pos[NUM_JACO_JOINTS];

  AngularPosition position_data;
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

  TrajectoryPoint trajPoint;
  trajPoint.InitStruct();
  trajPoint.Position.Type = CARTESIAN_POSITION;
  trajPoint.Position.HandMode = HAND_NOMOVEMENT;

  BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point, goal->trajectory.points)
  {
    double joint_cmd[NUM_JACO_JOINTS];
    for (int trajectory_index = 0; trajectory_index < goal->trajectory.joint_names.size(); ++trajectory_index)
    {
      string joint_name = goal->trajectory.joint_names[trajectory_index];
      int joint_index = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), joint_name));
      if (joint_index >= 0 && joint_index < NUM_JACO_JOINTS)
      {
        //convert angles from continuous joints to be correct for the arm API
        if (joint_index != 1 && joint_index != 2)
          joint_cmd[joint_index] = nearest_equivalent(simplify_angle(point.positions[trajectory_index]),
                                                      current_joint_pos[joint_index]);
        else
          joint_cmd[joint_index] = point.positions[trajectory_index];
      }
    }

    for (int i = 0; i < NUM_JACO_JOINTS; i++)
      current_joint_pos[i] = joint_cmd[i];

    //convert joint angles to end effector pose
    wpi_jaco_msgs::JacoFK fkSrv;
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
    {
      fkSrv.request.joints.push_back(joint_cmd[i]);
    }
    if (jaco_fk_client.call(fkSrv))
    {
      //conversion to rpy
      wpi_jaco_msgs::QuaternionToEuler qeSrv;
      qeSrv.request.orientation = fkSrv.response.handPose.pose.orientation;
      if (qe_client.call(qeSrv))
      {
        trajPoint.Position.CartesianPosition.X = fkSrv.response.handPose.pose.position.x;
        trajPoint.Position.CartesianPosition.Y = fkSrv.response.handPose.pose.position.y;
        trajPoint.Position.CartesianPosition.Z = fkSrv.response.handPose.pose.position.z;
        trajPoint.Position.CartesianPosition.ThetaX = qeSrv.response.roll;
        trajPoint.Position.CartesianPosition.ThetaY = qeSrv.response.pitch;
        trajPoint.Position.CartesianPosition.ThetaZ = qeSrv.response.yaw;

        //for debugging:
        //ROS_INFO("Trajectory point: (%f, %f, %f); (%f, %f, %f)", trajPoint.Position.CartesianPosition.X, trajPoint.Position.CartesianPosition.Y, trajPoint.Position.CartesianPosition.Z, trajPoint.Position.CartesianPosition.ThetaX, trajPoint.Position.CartesianPosition.ThetaY, trajPoint.Position.CartesianPosition.ThetaZ);

        //send point to arm trajectory
        executeCartesianTrajectoryPoint(trajPoint, false);
      }
      else
      {
        ROS_INFO("Quaternion to Euler Angle conversion service failed");

        trajPoint.Position.Type = ANGULAR_VELOCITY;
        trajPoint.Position.Actuators.Actuator1 = 0.0;
        trajPoint.Position.Actuators.Actuator2 = 0.0;
        trajPoint.Position.Actuators.Actuator3 = 0.0;
        trajPoint.Position.Actuators.Actuator4 = 0.0;
        trajPoint.Position.Actuators.Actuator5 = 0.0;
        trajPoint.Position.Actuators.Actuator6 = 0.0;
        executeAngularTrajectoryPoint(trajPoint, true);

        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        smooth_trajectory_server_.setSucceeded(result);
        return;
      }
    }
    else
    {
      ROS_INFO("Failed to call forward kinematics service");

      trajPoint.Position.Type = ANGULAR_VELOCITY;
      trajPoint.Position.Actuators.Actuator1 = 0.0;
      trajPoint.Position.Actuators.Actuator2 = 0.0;
      trajPoint.Position.Actuators.Actuator3 = 0.0;
      trajPoint.Position.Actuators.Actuator4 = 0.0;
      trajPoint.Position.Actuators.Actuator5 = 0.0;
      trajPoint.Position.Actuators.Actuator6 = 0.0;
      executeAngularTrajectoryPoint(trajPoint, true);

      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      smooth_trajectory_server_.setSucceeded(result);
      return;
    }
  }

  //wait for trajectory to complete execution
  ros::Rate rate(10);
  int trajectory_size;
  int initialTrajectorySize;
  TrajectoryFIFO Trajectory_Info;
  memset(&Trajectory_Info, 0, sizeof(Trajectory_Info));
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    GetGlobalTrajectoryInfo(Trajectory_Info);
  }
  trajectory_size = Trajectory_Info.TrajectoryCount;
  initialTrajectorySize = trajectory_size;
  while (trajectory_size > 0)
  {
    //cancel check
    if (eStopEnabled)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      smooth_trajectory_server_.setSucceeded(result);
      return;
    }

    //check for preempt requests from clients
    if (smooth_trajectory_server_.isPreemptRequested() || !ros::ok())
    {
      //stop gripper control
      trajPoint.Position.Type = ANGULAR_VELOCITY;
      trajPoint.Position.Actuators.Actuator1 = 0.0;
      trajPoint.Position.Actuators.Actuator2 = 0.0;
      trajPoint.Position.Actuators.Actuator3 = 0.0;
      trajPoint.Position.Actuators.Actuator4 = 0.0;
      trajPoint.Position.Actuators.Actuator5 = 0.0;
      trajPoint.Position.Actuators.Actuator6 = 0.0;
      executeAngularTrajectoryPoint(trajPoint, true);

      //preempt action server
      smooth_trajectory_server_.setPreempted();
      ROS_INFO("Smooth trajectory server preempted by client");

      return;
    }

    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      GetGlobalTrajectoryInfo(Trajectory_Info);
    }
    trajectory_size = Trajectory_Info.TrajectoryCount;

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
  //check for cancel
  if (eStopEnabled)
  {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
    smooth_joint_trajectory_server.setSucceeded(result);
    return;
  }

  float trajectoryPoints[NUM_JACO_JOINTS][goal->trajectory.points.size()];
  int numPoints = goal->trajectory.points.size();

  //get trajectory data
  for (unsigned int i = 0; i < numPoints; i++)
  {
    for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
    {
      trajectoryPoints[j][i] = goal->trajectory.points.at(i).positions.at(j);
    }
  }

  //initialize arrays needed to fit a smooth trajectory to the given points
  ecl::Array<double> timePoints(numPoints);
  timePoints[0] = 0.0;
  vector<ecl::Array<double> > jointPoints;
  jointPoints.resize(NUM_JACO_JOINTS);
  float prevPoint[NUM_JACO_JOINTS];
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
  {
    jointPoints[i].resize(numPoints);
    jointPoints[i][0] = trajectoryPoints[i][0];
    prevPoint[i] = trajectoryPoints[i][0];
  }

  //determine time component of trajectories for each joint
  for (unsigned int i = 1; i < numPoints; i++)
  {
    float maxTime = 0.0;
    for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
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

    timePoints[i] = timePoints[i - 1] + maxTime * TIME_SCALING_FACTOR;
  }

  vector<ecl::SmoothLinearSpline> splines;
  splines.resize(6);
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
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
    if (eStopEnabled)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      smooth_joint_trajectory_server.setSucceeded(result);
      return;
    }

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
      executeAngularTrajectoryPoint(trajPoint, true);

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
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        currentPoint = simplify_angle(current_joint_pos[i]);
        error[i] = nearest_equivalent(simplify_angle((splines.at(i))(timePoints.at(timePoints.size() - 1))),
                                      currentPoint) - currentPoint;
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
        executeAngularTrajectoryPoint(trajPoint, true);
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
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        currentPoint = simplify_angle(current_joint_pos[i]);
        error[i] = nearest_equivalent(simplify_angle((splines.at(i))(t)), currentPoint) - currentPoint;
      }
    }

    //calculate control input
    //populate the velocity command
    trajPoint.Position.Actuators.Actuator1 = (KP * error[0] + KV * (error[0] - prevError[0]) * RAD_TO_DEG);
    trajPoint.Position.Actuators.Actuator2 = (KP * error[1] + KV * (error[1] - prevError[1]) * RAD_TO_DEG);
    trajPoint.Position.Actuators.Actuator3 = (KP * error[2] + KV * (error[2] - prevError[2]) * RAD_TO_DEG);
    trajPoint.Position.Actuators.Actuator4 = (KP * error[3] + KV * (error[3] - prevError[3]) * RAD_TO_DEG);
    trajPoint.Position.Actuators.Actuator5 = (KP * error[4] + KV * (error[4] - prevError[4]) * RAD_TO_DEG);
    trajPoint.Position.Actuators.Actuator6 = (KP * error[5] + KV * (error[5] - prevError[5]) * RAD_TO_DEG);

    //for debugging:
    //cout << "Errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << endl;

    //send the velocity command
    executeAngularTrajectoryPoint(trajPoint, true);

    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
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
  //check for cancel
  if (eStopEnabled)
  {
    control_msgs::GripperCommandResult result;
    result.reached_goal = false;
    gripper_server_.setSucceeded(result);
    return;
  }

  wpi_jaco_msgs::AngularCommand cmd;
  cmd.position = true;
  cmd.armCommand = false;
  cmd.fingerCommand = true;
  cmd.repeat = false;
  cmd.fingers.resize(3);
  cmd.fingers[0] = goal->command.position;
  cmd.fingers[1] = goal->command.position;
  cmd.fingers[2] = goal->command.position;

  angularCmdPublisher.publish(cmd);

  ros::Rate rate(10);
  bool gripperMoving = true;
  while (gripperMoving)
  {
    //check for cancel
    if (eStopEnabled)
    {
      control_msgs::GripperCommandResult result;
      result.reached_goal = false;
      gripper_server_.setSucceeded(result);
      return;
    }

    rate.sleep();
    //check for preempt requests from clients
    if (gripper_server_.isPreemptRequested() || !ros::ok())
    {
      //stop gripper control
      cmd.position = false;
      cmd.fingers[0] = 0.0;
      cmd.fingers[1] = 0.0;
      cmd.fingers[2] = 0.0;
      angularCmdPublisher.publish(cmd);

      //preempt action server
      ROS_INFO("Gripper action server preempted by client");
      gripper_server_.setPreempted();

      return;
    }

    //see if fingers are still moving
    AngularPosition velocity_data;
    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      GetAngularVelocity(velocity_data);
    }
    float totalSpeed = fabs(velocity_data.Fingers.Finger1) + fabs(velocity_data.Fingers.Finger2) + fabs(velocity_data.Fingers.Finger3);
    if (totalSpeed <= 0.01)
    {
      gripperMoving = false;
    }
  }
  //stop gripper control
  cmd.position = false;
  cmd.fingers[0] = 0.0;
  cmd.fingers[1] = 0.0;
  cmd.fingers[2] = 0.0;
  angularCmdPublisher.publish(cmd);

  control_msgs::GripperCommandResult result;
  AngularPosition force_data;

  AngularPosition position_data;
  {
    GetAngularPosition(position_data);
    GetAngularForce(force_data);
  }
  float finalError = fabs(goal->command.position - position_data.Fingers.Finger1) + fabs(goal->command.position - position_data.Fingers.Finger2) + fabs(goal->command.position - position_data.Fingers.Finger3);
  if (finalError <= FINGER_ERROR_THRESHOLD)
    result.reached_goal = true;
  else
    result.reached_goal = false;
  result.position = position_data.Fingers.Finger1;
  result.effort =force_data.Fingers.Finger1;
  result.stalled = false;
  gripper_server_.setSucceeded(result);
}

/*****************************************/
/**********  Other Arm Actions  **********/
/*****************************************/

void JacoArmTrajectoryController::home_arm(const wpi_jaco_msgs::HomeArmGoalConstPtr &goal)
{
  //check for cancel
  if (eStopEnabled)
  {
    wpi_jaco_msgs::HomeArmResult result;
    result.success = false;
    home_arm_server.setSucceeded(result);
    return;
  }

  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    StopControlAPI();
    MoveHome();
    StartControlAPI();
  }

  if (goal->retract)
  {
    //check for cancel
    if (eStopEnabled)
    {
      wpi_jaco_msgs::HomeArmResult result;
      result.success = false;
      home_arm_server.setSucceeded(result);
      return;
    }

    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      //retract to given position
      controlType = ANGULAR_CONTROL;
      SetAngularControl();
    }

    angularCmdPublisher.publish(goal->retractPosition);

    ros::Rate rate(10);
    int trajectory_size = 1;
    while (trajectory_size > 0)
    {
      //check for cancel
      if (eStopEnabled)
      {
        wpi_jaco_msgs::HomeArmResult result;
        result.success = false;
        home_arm_server.setSucceeded(result);
        return;
      }

      //check for preempt requests from clients
      if (home_arm_server.isPreemptRequested() || !ros::ok())
      {
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
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    //set control type to previous value
    if (controlType == ANGULAR_CONTROL)
      SetAngularControl();
    else
      SetCartesianControl();
  }

  wpi_jaco_msgs::HomeArmResult result;
  result.success = true;
  home_arm_server.setSucceeded(result);
}

/*****************************************/
/**********  Basic Arm Commands **********/
/*****************************************/

void JacoArmTrajectoryController::angularCmdCallback(const wpi_jaco_msgs::AngularCommand& msg)
{
  if (eStopEnabled)
    return;

  //take control of the arm
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    EraseAllTrajectories();

    if (controlType != ANGULAR_CONTROL)
    {
      SetAngularControl();
      controlType = ANGULAR_CONTROL;
    }
  }

  TrajectoryPoint jacoPoint;
  jacoPoint.InitStruct();

  //populate arm command
  if (msg.armCommand)
  {
    if (msg.position)
    {
      jacoPoint.Position.Type = ANGULAR_POSITION;
      AngularPosition position_data;

      float current_joint_pos[6];
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

      jacoPoint.Position.Actuators.Actuator1 = nearest_equivalent(simplify_angle(msg.joints[0]),
                                                                  current_joint_pos[0]) * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator2 = nearest_equivalent(simplify_angle(msg.joints[1]),
                                                                  current_joint_pos[1]) * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator3 = nearest_equivalent(simplify_angle(msg.joints[2]),
                                                                  current_joint_pos[2]) * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator4 = nearest_equivalent(simplify_angle(msg.joints[3]),
                                                                  current_joint_pos[3]) * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator5 = nearest_equivalent(simplify_angle(msg.joints[4]),
                                                                  current_joint_pos[4]) * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator6 = nearest_equivalent(simplify_angle(msg.joints[5]),
                                                                  current_joint_pos[5]) * RAD_TO_DEG;
    }
    else
    {
      jacoPoint.Position.Type = ANGULAR_VELOCITY;
      jacoPoint.Position.Actuators.Actuator1 = msg.joints[0] * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator2 = msg.joints[1] * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator3 = msg.joints[2] * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator4 = msg.joints[3] * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator5 = msg.joints[4] * RAD_TO_DEG;
      jacoPoint.Position.Actuators.Actuator6 = msg.joints[5] * RAD_TO_DEG;
    }

  }
  else
  {
    if (msg.position)
    {
      fingerPositionControl(msg.fingers[0], msg.fingers[1], msg.fingers[2]);
      return;
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
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    SendBasicTrajectory(jacoPoint);
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    if (msg.repeat)
    {
      //send the command repeatedly for ~1/60th of a second
      //(this is sometimes necessary for velocity commands to work correctly)
      ros::Rate rate(600);
      for (int i = 0; i < 10; i++)
      {
        SendBasicTrajectory(jacoPoint);
        rate.sleep();
      }
    }
    else
      SendBasicTrajectory(jacoPoint);
  }
}

void JacoArmTrajectoryController::cartesianCmdCallback(const wpi_jaco_msgs::CartesianCommand& msg)
{
  if (eStopEnabled)
    return;

  //take control of the arm
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    EraseAllTrajectories();

    if (controlType != CARTESIAN_CONTROL)
    {
      SetCartesianControl();
      controlType = CARTESIAN_CONTROL;
    }
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
    if (msg.position)
    {
      fingerPositionControl(msg.fingers[0], msg.fingers[1], msg.fingers[2]);
      return;
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
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    SendBasicTrajectory(jacoPoint);
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    SendBasicTrajectory(jacoPoint);
    if (msg.repeat)
    {
      //send the command repeatedly for ~1/60th of a second
      //(this is sometimes necessary for velocity commands to work correctly)
      ros::Rate rate(600);
      for (int i = 0; i < 10; i++)
      {
        SendBasicTrajectory(jacoPoint);
        rate.sleep();
      }
    }
  }
}

void JacoArmTrajectoryController::fingerPositionControl(float f1, float f2, float f3)
{
  if (eStopEnabled)
    return;

  f1 = max(f1, .02f);
  f2 = max(f2, .02f);
  f3 = max(f3, .02f);

  TrajectoryPoint jacoPoint;
  jacoPoint.InitStruct();
  jacoPoint.Position.Type = ANGULAR_VELOCITY;
  jacoPoint.Position.Actuators.Actuator1 = 0.0;
  jacoPoint.Position.Actuators.Actuator2 = 0.0;
  jacoPoint.Position.Actuators.Actuator3 = 0.0;
  jacoPoint.Position.Actuators.Actuator4 = 0.0;
  jacoPoint.Position.Actuators.Actuator5 = 0.0;
  jacoPoint.Position.Actuators.Actuator6 = 0.0;
  jacoPoint.Position.HandMode = VELOCITY_MODE;

  bool goalReached = false;
  AngularPosition position_data;
  float error[3];
  float prevTotalError;
  float counter = 0; //check if error is unchanging, this likely means a finger is blocked by something so the controller should terminate
  vector<float> errorFinger1;
  vector<float> errorFinger2;
  vector<float> errorFinger3;
  errorFinger1.resize(10);
  errorFinger2.resize(10);
  errorFinger3.resize(10);
  for (unsigned int i = 0; i < errorFinger1.size(); i ++)
  {
    errorFinger1[i] = 0.0;
    errorFinger2[i] = 0.0;
    errorFinger3[i] = 0.0;
  }
  ros::Rate rate(600);
  while (!goalReached)
  {
    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);

      //get current finger position
      GetAngularPosition(position_data);
      error[0] = f1 - position_data.Fingers.Finger1;
      error[1] = f2 - position_data.Fingers.Finger2;
      error[2] = f3 - position_data.Fingers.Finger3;

      float totalError = fabs(error[0]) + fabs(error[1]) + fabs(error[2]);
      if (totalError == prevTotalError)
      {
        counter++;
      }
      else
      {
        counter = 0;
        prevTotalError = totalError;
      }

      if (totalError < FINGER_ERROR_THRESHOLD || counter > 40)
      {
        goalReached = true;
        jacoPoint.Position.Fingers.Finger1 = 0.0;
        jacoPoint.Position.Fingers.Finger2 = 0.0;
        jacoPoint.Position.Fingers.Finger3 = 0.0;
      }
      else
      {
        float errorSum[3] = {0};
        for (unsigned int i = 0; i < errorFinger1.size(); i++)
        {
          errorSum[0] += errorFinger1[i];
          errorSum[1] += errorFinger2[i];
          errorSum[2] += errorFinger3[i];
        }
        jacoPoint.Position.Fingers.Finger1 = max(min(KP_F * error[0] + KV_F * (error[0] - errorFinger1.front()) + KI_F * errorSum[0], 30.0), -30.0);
        jacoPoint.Position.Fingers.Finger2 = max(min(KP_F * error[1] + KV_F * (error[1] - errorFinger2.front()) + KI_F * errorSum[1], 30.0), -30.0);
        jacoPoint.Position.Fingers.Finger3 = max(min(KP_F * error[2] + KV_F * (error[2] - errorFinger3.front()) + KI_F * errorSum[2], 30.0), -30.0);
        errorFinger1.insert(errorFinger1.begin(), error[0]);
        errorFinger2.insert(errorFinger2.begin(), error[1]);
        errorFinger3.insert(errorFinger3.begin(), error[2]);
        errorFinger1.resize(10);
        errorFinger2.resize(10);
        errorFinger3.resize(10);
      }
      EraseAllTrajectories();
      SendBasicTrajectory(jacoPoint);
    }

    //check for cancel requests
    if (eStopEnabled)
      return;

    rate.sleep();
  }
}

void JacoArmTrajectoryController::executeAngularTrajectoryPoint(TrajectoryPoint point, bool erase)
{
  if (eStopEnabled)
    return;

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

void JacoArmTrajectoryController::executeCartesianTrajectoryPoint(TrajectoryPoint point, bool erase)
{
  if (eStopEnabled)
    return;

  boost::recursive_mutex::scoped_lock lock(api_mutex);

  if (controlType != CARTESIAN_CONTROL)
  {
    SetCartesianControl();
    controlType = CARTESIAN_CONTROL;
  }

  if (erase)
    EraseAllTrajectories();

  SendBasicTrajectory(point);
}

bool JacoArmTrajectoryController::getAngularPosition(wpi_jaco_msgs::GetAngularPosition::Request &req, wpi_jaco_msgs::GetAngularPosition::Response &res)
{
  res.pos.resize(NUM_JOINTS);
  for (unsigned int i = 0; i < NUM_JOINTS; i ++)
  {
    res.pos[i] = joint_pos[i];
  }

  return true;
}

bool JacoArmTrajectoryController::getCartesianPosition(wpi_jaco_msgs::GetCartesianPosition::Request &req,
                                                       wpi_jaco_msgs::GetCartesianPosition::Response &res)
{
  CartesianPosition pos;
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    GetCartesianPosition(pos);
  }
  res.pos.linear.x = pos.Coordinates.X;
  res.pos.linear.y = pos.Coordinates.Y;
  res.pos.linear.z = pos.Coordinates.Z;
  res.pos.angular.x = pos.Coordinates.ThetaX;
  res.pos.angular.y = pos.Coordinates.ThetaY;
  res.pos.angular.z = pos.Coordinates.ThetaZ;

  return true;
}

bool JacoArmTrajectoryController::eStopCallback(wpi_jaco_msgs::EStop::Request &req, wpi_jaco_msgs::EStop::Response &res)
{
  res.success = true;
  if (req.enableEStop)
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    ROS_INFO("Software emergency stop request received.");
    if (!eStopEnabled)
    {
      int stopResult = StopControlAPI();
      if (stopResult != NO_ERROR)
      {
        ROS_INFO("Error stopping arm control.");
        res.success = false;
      }
      stopResult = EraseAllTrajectories();
      if (stopResult != NO_ERROR)
      {
        ROS_INFO("Error stopping arm trajectories.");
        res.success = false;
      }
      eStopEnabled = true;
    }
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    ROS_INFO("Turning off software emergency stop.");
    if (eStopEnabled)
    {
      StopControlAPI();
      ros::Duration(0.05).sleep();
      StartControlAPI();
      if (controlType == ANGULAR_CONTROL)
        SetAngularControl();
      else
        SetCartesianControl();
      eStopEnabled = false;
    }
  }

  return true;
}

bool JacoArmTrajectoryController::eraseTrajectoriesCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if (EraseAllTrajectories() != NO_ERROR)
  {
    ROS_INFO("Error stopping arm trajectories.");
    return false;
  }
  return true;
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_arm_trajectory_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  jaco::JacoArmTrajectoryController robot(nh, pnh);
  ros::spin();
}
