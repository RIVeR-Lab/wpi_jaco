/*!
 * \jaco_joy_teleop.h
 * \brief Allows for control of the jaco arm with a joystick.
 *
 * jaco_joy_teleop creates a ROS node that allows for the control of the
 * JACO arm with a joystick. This node listens to a /joy topic.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date June 25, 2014
 */

#include <jaco_teleop/jaco_joy_teleop.h>

using namespace std;

jaco_joy_teleop::jaco_joy_teleop()
{
  // a private handle for this ROS node (allows retrieval of relative parameters)
  ros::NodeHandle private_nh("~");

  // create the ROS topics
  angular_cmd = node.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 10);
  cartesian_cmd = node.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 10);
  joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 10, &jaco_joy_teleop::joy_cback, this);

  eStopClient = node.serviceClient<wpi_jaco_msgs::EStop>("jaco_arm/software_estop");

  // read in throttle values
  private_nh.param<double>("linear_throttle_factor", linear_throttle_factor, 1.0);
  private_nh.param<double>("angular_throttle_factor", angular_throttle_factor, 1.0);
  private_nh.param<double>("finger_throttle_factor", finger_throttle_factor, 1.0);
  string str;
  private_nh.param<string>("controller_type", str, "digital");
  if (str.compare("digital") == 0)
    controllerType = DIGITAL;
  else
    controllerType = ANALOG;

  //initialize everything
  stopMessageSentArm = true;
  stopMessageSentFinger = true;
  EStopEnabled = false;
  helpDisplayed = false;
  mode = ARM_CONTROL;
  fingerCmd.position = false;
  fingerCmd.armCommand = false;
  fingerCmd.fingerCommand = true;
  fingerCmd.repeat = true;
  fingerCmd.fingers.resize(3);
  cartesianCmd.position = false;
  cartesianCmd.armCommand = true;
  cartesianCmd.fingerCommand = false;
  cartesianCmd.repeat = true;

  ROS_INFO("JACO joystick teleop started");

  puts(" ----------------------------------------");
  puts("| Jaco Joystick Teleop Help              |");
  puts("|----------------------------------------|*");
  puts("| Current Mode: Arm Control              |*");
  puts("|----------------------------------------|*");
  puts("| For help and controls, press:          |*");
  puts("|                          _             |*");
  puts("|                        _| |_           |*");
  puts("|  show finger controls |_   _|          |*");
  puts("|                         |_|            |*");
  puts("|                  show arm controls     |*");
  puts("|                                        |*");
  puts(" ----------------------------------------**");
  puts("  *****************************************");

  if (controllerType == ANALOG)
  {
    initLeftTrigger = false;
    initRightTrigger = false;
    calibrated = false;

    ROS_INFO(
        "You specified a controller with analog triggers. This requires calibration before any teleoperation can begin.  Please press and release both triggers before continuing.");
  }
  else
    calibrated = true;
}

void jaco_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // make sure triggers are calibrated before continuint if an analog controller was specified
  if (!calibrated)
  {
    if (!initLeftTrigger && joy->axes.at(2) == 1.0)
      initLeftTrigger = true;

    if (!initRightTrigger && joy->axes.at(5) == 1.0)
      initRightTrigger = true;

    if (initLeftTrigger && initRightTrigger)
    {
      calibrated = true;
      ROS_INFO("Controller calibration complete!");
    }

    return;
  }

  //software emergency stop
  if (controllerType == DIGITAL)
  {
    if (joy->buttons.at(8) == 1)
    {
      EStopEnabled = true;
      wpi_jaco_msgs::EStop srv;
      srv.request.enableEStop = true;
      if (!eStopClient.call(srv))
        ROS_INFO("Couldn't call software estop service.");
    }
    else if (joy->buttons.at(9) == 1)
    {
      EStopEnabled = false;
      wpi_jaco_msgs::EStop srv;
      srv.request.enableEStop = false;
      if (!eStopClient.call(srv))
        ROS_INFO("Couldn't call software estop service.");
    }
  }
  else
  {
    if (joy->buttons.at(6) == 1)
    {
      EStopEnabled = true;
      wpi_jaco_msgs::EStop srv;
      srv.request.enableEStop = true;
      if (!eStopClient.call(srv))
        ROS_INFO("Couldn't call software estop service.");
    }
    else if (joy->buttons.at(7) == 1)
    {
      EStopEnabled = false;
      wpi_jaco_msgs::EStop srv;
      srv.request.enableEStop = false;
      if (!eStopClient.call(srv))
        ROS_INFO("Couldn't call software estop service.");
    }
  }

  //help menu
  if ((controllerType == DIGITAL && joy->axes.at(5) == -1.0) || (controllerType == ANALOG && joy->axes.at(7) == -1.0))
  {
    if (!helpDisplayed)
    {
      helpDisplayed = true;
      puts(" ----------------------------------------");
      puts("| Jaco Joystick Teleop Help              |");
      puts("|----------------------------------------|*");
      if (mode == ARM_CONTROL)
        puts("| Current Mode: Arm Control              |*");
      else
        puts("| Current Mode: Finger Control           |*");
      puts("|----------------------------------------|*");
      puts("|                Controls                |*");
      puts("|   roll/down                 roll/up    |*");
      puts("|    ________                ________    |*");
      puts("|   /    _   \\______________/        \\   |*");
      puts("|  |   _| |_    < >    < >     (4)    |  |*");
      puts("|  |  |_   _|  Estop  start (1)   (3) |  |*");
      puts("|  |    |_|    ___      ___    (2)    |  |*");
      puts("|  |          /   \\    /   \\          |  |*");
      puts("|  |          \\___/    \\___/          |  |*");
      puts("|  |       x/y trans  pitch/yaw       |  |*");
      puts("|  |        _______/--\\_______        |  |*");
      puts("|  |       |                  |       |  |*");
      puts("|   \\     /                    \\     /   |*");
      puts("|    \\___/                      \\___/    |*");
      puts("|                                        |*");
      puts("| Buttons:                               |*");
      puts("|   (1) Switch to finger control mode    |*");
      puts("|   (2) Switch to arm control mode       |*");
      puts("|   (3) No function                      |*");
      puts("|   (4) No function                      |*");
      puts(" ----------------------------------------**");
      puts("  *****************************************");
    }
  }
  else if ((controllerType == DIGITAL && joy->axes.at(4) == 1.0)
      || (controllerType == ANALOG && joy->axes.at(6) == 1.0))
  {
    if (!helpDisplayed)
    {
      helpDisplayed = true;
      puts(" ----------------------------------------");
      puts("| Jaco Joystick Teleop Help              |");
      puts("|----------------------------------------|*");
      if (mode == ARM_CONTROL)
        puts("| Current Mode: Arm Control              |*");
      else
        puts("| Current Mode: Finger Control           |*");
      puts("|----------------------------------------|*");
      puts("|                Controls                |*");
      puts("| finger1 open/close  finger2 open/close |*");
      puts("|    ________                ________    |*");
      puts("|   /    _   \\______________/        \\   |*");
      puts("|  |   _| |_    < >    < >     (4)    |  |*");
      puts("|  |  |_   _|  Estop  start (1)   (3) |  |*");
      puts("|  |    |_|    ___      ___    (2)    |  |*");
      puts("|  |          /   \\    /   \\          |  |*");
      puts("|  |          \\___/    \\___/          |  |*");
      puts("|  | hand open/close  thumb open/close|  |*");
      puts("|  |        _______/--\\_______        |  |*");
      puts("|  |       |                  |       |  |*");
      puts("|   \\     /                    \\     /   |*");
      puts("|    \\___/                      \\___/    |*");
      puts("|                                        |*");
      puts("| Buttons:                               |*");
      puts("|   (1) Switch to finger control mode    |*");
      puts("|   (2) Switch to arm control mode       |*");
      puts("|   (3) No function                      |*");
      puts("|   (4) No function                      |*");
      puts(" ----------------------------------------**");
      puts("  *****************************************");
    }
  }
  else
    helpDisplayed = false;

  int buttonIndex;

  switch (mode)
  {
    case ARM_CONTROL:
      // left joystick controls the linear x and y movement
      cartesianCmd.arm.linear.x = joy->axes.at(0) * MAX_TRANS_VEL * linear_throttle_factor;
      cartesianCmd.arm.linear.y = -joy->axes.at(1) * MAX_TRANS_VEL * linear_throttle_factor;

      //triggers control the linear z movement
      if (controllerType == DIGITAL)
      {
        if (joy->buttons.at(7) == 1)
          cartesianCmd.arm.linear.z = MAX_TRANS_VEL * linear_throttle_factor;
        else if (joy->buttons.at(6) == 1)
          cartesianCmd.arm.linear.z = -MAX_TRANS_VEL * linear_throttle_factor;
        else
          cartesianCmd.arm.linear.z = 0.0;
      }
      else
      {
        if (joy->axes.at(5) < 1.0)
          cartesianCmd.arm.linear.z = (0.5 - joy->axes.at(5) / 2.0) * MAX_ANG_VEL * angular_throttle_factor;
        else
          cartesianCmd.arm.linear.z = -(0.5 - joy->axes.at(2) / 2.0) * MAX_ANG_VEL * angular_throttle_factor;
      }

      //bumpers control roll
      if (joy->buttons.at(5) == 1)
        cartesianCmd.arm.angular.z = MAX_ANG_VEL * angular_throttle_factor;
      else if (joy->buttons.at(4) == 1)
        cartesianCmd.arm.angular.z = -MAX_ANG_VEL * angular_throttle_factor;
      else
        cartesianCmd.arm.angular.z = 0.0;

      //right joystick controls pitch and yaw
      if (controllerType == DIGITAL)
      {
        cartesianCmd.arm.angular.x = -joy->axes.at(3) * MAX_ANG_VEL * angular_throttle_factor;
        cartesianCmd.arm.angular.y = joy->axes.at(2) * MAX_ANG_VEL * angular_throttle_factor;
      }
      else
      {
        cartesianCmd.arm.angular.x = -joy->axes.at(4) * MAX_ANG_VEL * angular_throttle_factor;
        cartesianCmd.arm.angular.y = joy->axes.at(3) * MAX_ANG_VEL * angular_throttle_factor;
      }

      //mode switching
      if (controllerType == DIGITAL)
        buttonIndex = 0;
      else
        buttonIndex = 2;

      if (joy->buttons.at(buttonIndex) == 1)
      {
        //cancel trajectory and switch to finger control mode
        cartesianCmd.arm.linear.x = 0.0;
        cartesianCmd.arm.linear.y = 0.0;
        cartesianCmd.arm.linear.z = 0.0;
        cartesianCmd.arm.angular.x = 0.0;
        cartesianCmd.arm.angular.y = 0.0;
        cartesianCmd.arm.angular.z = 0.0;
        cartesian_cmd.publish(cartesianCmd);
        mode = FINGER_CONTROL;

        ROS_INFO("Activated finger control mode");
      }
      break;
    case FINGER_CONTROL:
      if (joy->axes.at(1) == 0.0)
      {
        //individual finger control
        //thumb controlled by right thumbstick
        if (controllerType == DIGITAL)
          fingerCmd.fingers[0] = -joy->axes.at(3) * MAX_FINGER_VEL * finger_throttle_factor;
        else
          fingerCmd.fingers[0] = -joy->axes.at(4) * MAX_FINGER_VEL * finger_throttle_factor;

        //top finger controlled by left triggers
        if (controllerType == DIGITAL)
        {
          if (joy->buttons.at(4) == 1)
            fingerCmd.fingers[1] = -MAX_FINGER_VEL * finger_throttle_factor;
          else if (joy->buttons.at(6) == 1)
            fingerCmd.fingers[1] = MAX_FINGER_VEL * finger_throttle_factor;
          else
            fingerCmd.fingers[1] = 0.0;
        }
        else
        {
          if (joy->buttons.at(4) == 1)
            fingerCmd.fingers[1] = -MAX_FINGER_VEL * finger_throttle_factor;
          else
            fingerCmd.fingers[1] = (0.5 - joy->axes.at(2) / 2.0) * MAX_FINGER_VEL * finger_throttle_factor;
        }

        //bottom finger controlled by right bumpers
        if (controllerType == DIGITAL)
        {
          if (joy->buttons.at(5) == 1)
            fingerCmd.fingers[2] = -MAX_FINGER_VEL * finger_throttle_factor;
          else if (joy->buttons.at(7) == 1)
            fingerCmd.fingers[2] = MAX_FINGER_VEL * finger_throttle_factor;
          else
            fingerCmd.fingers[2] = 0.0;
        }
        else
        {
          if (joy->buttons.at(5) == 1)
            fingerCmd.fingers[2] = -MAX_FINGER_VEL * finger_throttle_factor;
          else
            fingerCmd.fingers[2] = (0.5 - joy->axes.at(5) / 2.0) * MAX_FINGER_VEL * finger_throttle_factor;
        }
      }
      else
      {
        //control full gripper (outprioritizes individual finger control)
        fingerCmd.fingers[0] = -joy->axes.at(1) * MAX_FINGER_VEL * finger_throttle_factor;
        fingerCmd.fingers[1] = fingerCmd.fingers[0];
        fingerCmd.fingers[2] = fingerCmd.fingers[0];
      }

      //mode switching
      if (controllerType == DIGITAL)
        buttonIndex = 1;
      else
        buttonIndex = 0;

      if (joy->buttons.at(buttonIndex) == 1)
      {
        //cancel trajectory and switch to arm control mode
        fingerCmd.fingers[0] = 0.0;
        fingerCmd.fingers[1] = 0.0;
        fingerCmd.fingers[2] = 0.0;
        angular_cmd.publish(fingerCmd);
        mode = ARM_CONTROL;

        ROS_INFO("Activated arm control mode");
      }
      break;
  }
}

void jaco_joy_teleop::publish_velocity()
{
  //publish stop commands if EStop is enabled
  if (EStopEnabled)
    return;

  switch (mode)
  {
    case ARM_CONTROL:
      //only publish stop message once; this allows other nodes to publish velocities
      //while the controller is not being used
      if (cartesianCmd.arm.linear.x == 0.0 && cartesianCmd.arm.linear.y == 0.0 && cartesianCmd.arm.linear.z == 0.0
          && cartesianCmd.arm.angular.x == 0.0 && cartesianCmd.arm.angular.y == 0.0
          && cartesianCmd.arm.angular.z == 0.0)
      {
        if (!stopMessageSentArm)
        {
          cartesian_cmd.publish(cartesianCmd);
          stopMessageSentArm = true;
        }
      }
      else
      {
        // send the twist command
        cartesian_cmd.publish(cartesianCmd);
        stopMessageSentArm = false;
      }
      break;
    case FINGER_CONTROL:
      //only publish stop message once; this allows other nodes to publish velocities
      //while the controller is not being used
      if (fingerCmd.fingers[0] == 0.0 && fingerCmd.fingers[1] == 0.0 && fingerCmd.fingers[2] == 0.0)
      {
        if (!stopMessageSentFinger)
        {
          angular_cmd.publish(fingerCmd);
          stopMessageSentFinger = true;
        }
      }
      else
      {
        //send the finger velocity command
        angular_cmd.publish(fingerCmd);
        stopMessageSentFinger = false;
      }
      break;
  }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "jaco_joy_teleop");

  // initialize the joystick controller
  jaco_joy_teleop controller;

  ros::Rate loop_rate(60);	//rate at which to publish velocity commands
  while (ros::ok())
  {
    controller.publish_velocity();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
