#include <wpi_jaco_wrapper/jaco_manipulation.h>

using namespace std;

JacoManipulation::JacoManipulation() :
    acGripper("jaco_arm/fingers_controller/gripper", true),
    asGripper(n, "jaco_arm/manipulation/gripper", boost::bind(&JacoManipulation::execute_gripper, this, _1), false),
    asLift(n, "jaco_arm/manipulation/lift", boost::bind(&JacoManipulation::execute_lift, this, _1), false)
{
  // Messages
  cartesianCmdPublisher = n.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 1);
  angularCmdPublisher = n.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 1);

  jointStateSubscriber = n.subscribe("jaco_arm/joint_states", 1, &JacoManipulation::jointStateCallback, this);

  // Services
  cartesianPositionClient = n.serviceClient<wpi_jaco_msgs::GetCartesianPosition>("jaco_arm/get_cartesian_position");
  eraseTrajectoriesClient = n.serviceClient<std_srvs::Empty>("/jaco_arm/erase_trajectories");

  ROS_INFO("Waiting for gripper action server...");
  acGripper.waitForServer();
  ROS_INFO("Finished waiting for action server.");

  // Action servers
  asGripper.start();
  asLift.start();
}

void JacoManipulation::jointStateCallback(const sensor_msgs::JointState msg)
{
  for (unsigned int i = 0; i < NUM_JOINTS; i++)
  {
    jointPos[i] = msg.position[i];
  }
}

void JacoManipulation::execute_gripper(const rail_manipulation_msgs::GripperGoalConstPtr &goal)
{
  rail_manipulation_msgs::GripperResult result;

  if (asLift.isActive())
  {
    asGripper.setPreempted();
    ROS_INFO("Lift server already running, grasp action preempted");
    return;
  }

  float currentFingerPos[3];
  currentFingerPos[0] = jointPos[6];
  currentFingerPos[1] = jointPos[7];
  currentFingerPos[2] = jointPos[8];

  //check if grasp is already finished (for opening case only)
  if (!goal->close)
  {
    if (currentFingerPos[0] <= GRIPPER_OPEN_THRESHOLD && currentFingerPos[1] <= GRIPPER_OPEN_THRESHOLD && currentFingerPos[2] <= GRIPPER_OPEN_THRESHOLD)
    {
      ROS_INFO("Gripper is open.");
      result.success = true;
      asGripper.setSucceeded(result, "Open gripper action succeeded, as the gripper is already open.");
      return;
    }
  }

  control_msgs::GripperCommandGoal gripperGoal;
  if (goal->close)
    gripperGoal.command.position = GRIPPER_CLOSED;
  else
    gripperGoal.command.position = GRIPPER_OPEN;
  acGripper.sendGoal(gripperGoal);

  ros::Rate loopRate(30);
  while (!acGripper.getState().isDone())
  {
    //check for preempt requests from clients
    if (asGripper.isPreemptRequested() || !ros::ok())
    {
      acGripper.cancelAllGoals();
      //preempt action server
      asGripper.setPreempted();
      ROS_INFO("Gripper action server preempted by client");
      return;
    }
    loopRate.sleep();
  }

  rail_manipulation_msgs::GripperResult serverResult;
  serverResult.success = acGripper.getResult()->reached_goal;
  asGripper.setSucceeded(serverResult);
  ROS_INFO("Gripper action finished.");
}

void JacoManipulation::execute_lift(rail_manipulation_msgs::LiftGoalConstPtr const &goal)
{
  if (asGripper.isActive())
  {
    asLift.setPreempted();
    ROS_INFO("Gripper server already running, lift action preempted");
    return;
  }

  //get initial end effector height
  wpi_jaco_msgs::GetCartesianPosition srv;
  rail_manipulation_msgs::LiftResult result;
  if (cartesianPositionClient.call(srv))
  {
    float initialZ = srv.response.pos.linear.z;

    //populate the velocity command
    wpi_jaco_msgs::CartesianCommand cmd;
    cmd.position = false;
    cmd.armCommand = true;
    cmd.fingerCommand = true;
    cmd.repeat = true;
    cmd.fingers.resize(3);
    cmd.arm.linear.x = 0.0;
    cmd.arm.linear.y = 0.0;
    cmd.arm.linear.z = DEFAULT_LIFT_VEL;
    cmd.arm.angular.x = 0.0;
    cmd.arm.angular.y = 0.0;
    cmd.arm.angular.z = 0.0;
    cmd.fingers[0] = MAX_FINGER_VEL;
    cmd.fingers[1] = MAX_FINGER_VEL;
    cmd.fingers[2] = MAX_FINGER_VEL;

    bool finished = false;
    float currentZ;
    double startTime = ros::Time::now().toSec();

    //send the lift and close command until a certain height has been reached, or the
    //action times out
    while (!finished)
    {
      //check for preempt requests from clients
      if (asLift.isPreemptRequested() || !ros::ok())
      {
        //stop pickup action
        std_srvs::Empty emptySrv;
        if(!eraseTrajectoriesClient.call(emptySrv))
        {
          ROS_INFO("Could not call erase trajectories service");
        }

        //preempt action server
        asLift.setPreempted();
        ROS_INFO("Lift action server preempted by client");

        return;
      }

      cartesianCmdPublisher.publish(cmd);

      if (cartesianPositionClient.call(srv))
      {
        currentZ = srv.response.pos.linear.z;
      }
      else
      {
        ROS_INFO("Couldn't call Cartesian position server");
        result.success = false;
        break;
      }

      if (currentZ - initialZ >= LIFT_HEIGHT)
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
    std_srvs::Empty emptySrv;
    if(!eraseTrajectoriesClient.call(emptySrv))
    {
      ROS_INFO("Could not call erase trajectories service");
    }
  }
  else
  {
    ROS_INFO("Couldn't call Cartesian position server");
    result.success = false;
  }

  asLift.setSucceeded(result);
  ROS_INFO("Pickup execution complete");

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_manipulation");

  JacoManipulation jm;

  ros::spin();
}
