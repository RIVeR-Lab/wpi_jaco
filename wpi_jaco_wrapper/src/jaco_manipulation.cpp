#include <wpi_jaco_wrapper/jaco_manipulation.h>

using namespace std;

JacoManipulation::JacoManipulation()
{
  loadParameters(n);

  acGripper = new GripperClient(n, arm_name_ + "_arm/fingers_controller/gripper", true);
  asGripper = new GripperServer(n, arm_name_ + "_arm/manipulation/gripper", boost::bind(&JacoManipulation::execute_gripper, this, _1), false);
  asLift    = new LiftServer(n, arm_name_ + "_arm/manipulation/lift", boost::bind(&JacoManipulation::execute_lift, this, _1), false);

  // Messages
  cartesianCmdPublisher = n.advertise<wpi_jaco_msgs::CartesianCommand>(arm_name_ + "_arm/cartesian_cmd", 1);
  angularCmdPublisher = n.advertise<wpi_jaco_msgs::AngularCommand>(arm_name_ + "_arm/angular_cmd", 1);

  jointStateSubscriber = n.subscribe(arm_name_ + "_arm/joint_states", 1, &JacoManipulation::jointStateCallback, this);

  // Services
  cartesianPositionClient = n.serviceClient<wpi_jaco_msgs::GetCartesianPosition>(arm_name_ + "_arm/get_cartesian_position");
  eraseTrajectoriesClient = n.serviceClient<std_srvs::Empty>(arm_name_ + "_arm/erase_trajectories");

  ROS_INFO("Waiting for gripper action server...");
  acGripper->waitForServer();
  ROS_INFO("Finished waiting for action server.");

  // Action servers
  asGripper->start();
  asLift->start();
}

bool JacoManipulation::loadParameters(const ros::NodeHandle n)
{
    ROS_DEBUG("Loading parameters");

    n.param("wpi_jaco/arm_name", arm_name_, std::string("jaco"));
    n.param("wpi_jaco/gripper_closed", gripper_closed_, 0.0);
    n.param("wpi_jaco/gripper_open", gripper_open_, 65.0);
    n.param("wpi_jaco/num_fingers", num_fingers_, 3);

    num_joints_ = num_fingers_ + NUM_JACO_JOINTS;

    joint_pos_.resize(num_joints_);

    ROS_INFO("arm_name: %s", arm_name_.c_str());

    ROS_INFO("Parameters loaded.");

    //! @todo MdL [IMPR]: Return is values are all correctly loaded.
    return true;
}

void JacoManipulation::jointStateCallback(const sensor_msgs::JointState msg)
{
  for (unsigned int i = 0; i < num_joints_; i++)
    joint_pos_[i] = msg.position[i];
}

void JacoManipulation::execute_gripper(const rail_manipulation_msgs::GripperGoalConstPtr &goal)
{
  rail_manipulation_msgs::GripperResult result;

  if (asLift->isActive())
  {
    asGripper->setPreempted();
    ROS_INFO("Lift server already running, grasp action preempted");
    return;
  }

  float startingFingerPos[3];
  for (int i = 0 ; i < num_fingers_ ; i++)
    startingFingerPos[i] = joint_pos_[NUM_JACO_JOINTS+i];

  //check if grasp is already finished (for opening case only)
  if (!goal->close)
  {
    bool gripper_open = true;
    for (int i = 0 ; i < num_fingers_ ; i++)
      gripper_open = gripper_open && startingFingerPos[i] <= GRIPPER_OPEN_THRESHOLD;

    if (gripper_open)
    {
      ROS_INFO("Gripper is open.");
      result.success = true;
      asGripper->setSucceeded(result, "Open gripper action succeeded, as the gripper is already open.");
      return;
    }
  }

  control_msgs::GripperCommandGoal gripperGoal;
  if (goal->close)
    gripperGoal.command.position = gripper_closed_;
  else
    gripperGoal.command.position = gripper_open_;
  acGripper->sendGoal(gripperGoal);

  ros::Rate loopRate(30);
  while (!acGripper->getState().isDone())
  {
    //check for preempt requests from clients
    if (asGripper->isPreemptRequested() || !ros::ok())
    {
      acGripper->cancelAllGoals();
      //preempt action server
      asGripper->setPreempted();
      ROS_INFO("Gripper action server preempted by client");
      return;
    }
    loopRate.sleep();
  }

  rail_manipulation_msgs::GripperResult serverResult;
  //success occurs if the gripper has moved, as it is unlikely to reach the final "closed" position when grasping an object
  //serverResult.success = acGripper->getResult()->reached_goal;
  if (goal->close)
  {
    bool gripper_closing = true;
    for (int i = 0 ; i < num_fingers_ ; i++)
      gripper_closing = gripper_closing && joint_pos_[NUM_JACO_JOINTS+i] > startingFingerPos[i];

    serverResult.success = gripper_closing;
  }
  else
  {
    bool gripper_opening = true;
    for (int i = 0 ; i < num_fingers_ ; i++)
      gripper_opening = gripper_opening && joint_pos_[NUM_JACO_JOINTS+i] > startingFingerPos[i];

    serverResult.success = gripper_opening;
  }
  asGripper->setSucceeded(serverResult);
  ROS_INFO("Gripper action finished.");
}

void JacoManipulation::execute_lift(rail_manipulation_msgs::LiftGoalConstPtr const &goal)
{
  if (asGripper->isActive())
  {
    asLift->setPreempted();
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
      if (asLift->isPreemptRequested() || !ros::ok())
      {
        //stop pickup action
        std_srvs::Empty emptySrv;
        if(!eraseTrajectoriesClient.call(emptySrv))
        {
          ROS_INFO("Could not call erase trajectories service");
        }

        //preempt action server
        asLift->setPreempted();
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

  asLift->setSucceeded(result);
  ROS_INFO("Pickup execution complete");

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_manipulation");

  JacoManipulation jm;

  ros::spin();
}
