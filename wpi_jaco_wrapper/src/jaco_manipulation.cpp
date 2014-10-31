#include <wpi_jaco_wrapper/jaco_manipulation.h>

using namespace std;

JacoManipulation::JacoManipulation() :
    executeGraspServer(n, "jaco_arm/manipulation/grasp", boost::bind(&JacoManipulation::execute_grasp, this, _1),
                       false), executePickupServer(n, "jaco_arm/manipulation/pickup",
                                                   boost::bind(&JacoManipulation::execute_pickup, this, _1), false)
{
  // Messages
  cartesianCmdPublisher = n.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 1);
  angularCmdPublisher = n.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 1);

  jointStateSubscriber = n.subscribe("jaco_arm/joint_states", 1, &JacoManipulation::jointStateCallback, this);

  // Services
  cartesianPositionClient = n.serviceClient<wpi_jaco_msgs::GetCartesianPosition>("jaco_arm/get_cartesian_position");

  // Action servers
  executeGraspServer.start();
  executePickupServer.start();
}

void JacoManipulation::jointStateCallback(const sensor_msgs::JointState msg)
{
  for (unsigned int i = 0; i < NUM_JOINTS; i++)
  {
    jointPos[i] = msg.position[i];
  }
}

void JacoManipulation::execute_grasp(const wpi_jaco_msgs::ExecuteGraspGoalConstPtr &goal)
{
  if (executePickupServer.isActive())
  {
    executeGraspServer.setPreempted();
    ROS_INFO("Pickup server already running, grasp action preempted");
    return;
  }

  wpi_jaco_msgs::AngularCommand cmd;
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
    cmd.fingers[0] = direction * fabs(goal->fingerVelocities.finger1Vel);
    cmd.fingers[1] = direction * fabs(goal->fingerVelocities.finger2Vel);
    cmd.fingers[2] = direction * fabs(goal->fingerVelocities.finger3Vel);
  }
  else
  {
    cmd.fingers[0] = direction * MAX_FINGER_VEL;
    cmd.fingers[1] = direction * MAX_FINGER_VEL;
    cmd.fingers[2] = direction * MAX_FINGER_VEL;
  }

  //get initial finger position
  float prevFingerPos[3];
  prevFingerPos[0] = jointPos[6];
  prevFingerPos[1] = jointPos[7];
  prevFingerPos[2] = jointPos[8];

  float currentFingerPos[3];
  currentFingerPos[0] = prevFingerPos[0];
  currentFingerPos[1] = prevFingerPos[1];
  currentFingerPos[2] = prevFingerPos[2];

  double prevCheckTime = ros::Time::now().toSec();
  bool finishedGrasp = false;

  //check if grasp is already finished (for opening case only)
  if (!goal->closeGripper)
  {
    if (currentFingerPos[0] <= GRIPPER_OPEN_THRESHOLD && currentFingerPos[1] <= GRIPPER_OPEN_THRESHOLD && currentFingerPos[2] <= GRIPPER_OPEN_THRESHOLD)
    {
      ROS_INFO("Gripper already open.");
      finishedGrasp = true;
    }
  }
  
  while (!finishedGrasp)
  {
    //check for preempt requests from clients
    if (executeGraspServer.isPreemptRequested() || !ros::ok())
    {
      //stop gripper control
      cmd.fingers[0] = 0.0;
      cmd.fingers[1] = 0.0;
      cmd.fingers[2] = 0.0;
      angularCmdPublisher.publish(cmd);

      //preempt action server
      executeGraspServer.setPreempted();
      ROS_INFO("Grasp action server preempted by client");

      return;
    }

    angularCmdPublisher.publish(cmd);

    if (ros::Time::now().toSec() - prevCheckTime > .25)	//occaisionally check to see if the fingers have stopped moving
    {
      prevCheckTime = ros::Time::now().toSec();
      currentFingerPos[0] = jointPos[6];
      currentFingerPos[1] = jointPos[7];
      currentFingerPos[2] = jointPos[8];

      if (!goal->closeGripper)
      {
        if (currentFingerPos[0] <= GRIPPER_OPEN_THRESHOLD && currentFingerPos[1] <= GRIPPER_OPEN_THRESHOLD && currentFingerPos[2] <= GRIPPER_OPEN_THRESHOLD)
        {
          finishedGrasp = true;
        }
      }
      //grasp is finished if fingers haven't moved since the last check
      if ((fabs(prevFingerPos[0] - currentFingerPos[0]) + fabs(prevFingerPos[1] - currentFingerPos[1])
          + fabs(prevFingerPos[2] - currentFingerPos[2])) == 0.0)
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
  angularCmdPublisher.publish(cmd);

  wpi_jaco_msgs::ExecuteGraspResult result;
  result.fingerJoints.resize(3);
  result.fingerJoints.at(0) = jointPos[6];
  result.fingerJoints.at(1) = jointPos[7];
  result.fingerJoints.at(2) = jointPos[8];
  executeGraspServer.setSucceeded(result);
  ROS_INFO("Grasp execution complete");
}

void JacoManipulation::execute_pickup(const wpi_jaco_msgs::ExecutePickupGoalConstPtr &goal)
{
  if (executeGraspServer.isActive())
  {
    executePickupServer.setPreempted();
    ROS_INFO("Grasp server already running, pickup action preempted");
    return;
  }

  //get initial end effector height
  wpi_jaco_msgs::GetCartesianPosition srv;
  wpi_jaco_msgs::ExecutePickupResult result;
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
    float currentZ;
    double startTime = ros::Time::now().toSec();

    //send the lift and close command until a certain height has been reached, or the
    //action times out
    while (!finished)
    {
      //check for preempt requests from clients
      if (executePickupServer.isPreemptRequested() || !ros::ok())
      {
        //stop pickup action
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

        //preempt action server
        executePickupServer.setPreempted();
        ROS_INFO("Pickup action server preempted by client");

        return;
      }

      cartesianCmdPublisher.publish(cmd);

      if (cartesianPositionClient.call(srv))
      {
        currentZ = srv.response.pos.linear.z;
      }
      else
      {
        ROS_INFO("Couldn't call cartesian position server");
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
  }
  else
  {
    ROS_INFO("Couldn't call cartesian position server");
    result.success = false;
  }

  executePickupServer.setSucceeded(result);
  ROS_INFO("Pickup execution complete");

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_manipulation");

  JacoManipulation jm;

  ros::spin();
}
