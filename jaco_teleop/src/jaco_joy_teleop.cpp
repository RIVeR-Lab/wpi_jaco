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
  // create the ROS topics
  cmd_vel = node.advertise<geometry_msgs::Twist>("jaco_arm/cmd_vel", 10);
  finger_vel = node.advertise<jaco_ros::JacoFingerVel>("jaco_arm/finger_cmd_vel", 10);
  joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 10, &jaco_joy_teleop::joy_cback, this);

  // read in throttle values
  double temp;
  if (node.getParam("/jaco_joy_teleop/linear_throttle_factor", temp))
    linear_throttle_factor = (float)temp;
  else
    linear_throttle_factor = 1.0;
	if (node.getParam("/jaco_joy_teleop/angular_throttle_factor", temp))
    angular_throttle_factor = (float)temp;
  else
    angular_throttle_factor = 1.0;
  if (node.getParam("/jaco_joy_teleop/finger_throttle_factor", temp))
    finger_throttle_factor = (float)temp;
  else
    finger_throttle_factor = 1.0;
  
  string str;
  if (node.getParam("/jaco_joy_teleop/controller_type", str))
  {
  	if (str.compare("digital") == 0)
  		controllerType = DIGITAL;
  	else
  		controllerType = ANALOG;
  }
  else
  {
  	ROS_INFO("Couldn't read controller_type paramaeter");
  	controllerType = ANALOG;
	}

	stopMessageSentArm = true;
	stopMessageSentFinger = true;
	EStopEnabled = false;
	helpDisplayed = false;
	
	mode = ARM_CONTROL;
	
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
		
		ROS_INFO("You specified a controller with analog triggers. This requires calibration before any teleoperation can begin.  Please press and release both triggers before continuing.");
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
			EStopEnabled = true;
		else if (joy->buttons.at(9) == 1)
			EStopEnabled = false;
	}
	else
	{
		if (joy->buttons.at(6) == 1)
			EStopEnabled = true;
		else if (joy->buttons.at(7) == 1)
			EStopEnabled = false;
	}

	//help menu
	if (controllerType == DIGITAL)
	{
		if (joy->axes.at(5) == -1.0)
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
		else if (joy->axes.at(4) == 1.0)
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
	}

	int buttonIndex;
	
	switch (mode)
	{
	case ARM_CONTROL:
		// left joystick controls the linear x and y movement
		twist.linear.x = joy->axes.at(0) * MAX_TRANS_VEL * linear_throttle_factor;
		twist.linear.y = -joy->axes.at(1) * MAX_TRANS_VEL * linear_throttle_factor;

		//triggers control the linear z movement
		if (controllerType == DIGITAL)
		{
			if (joy->buttons.at(7) == 1)
				twist.linear.z = MAX_TRANS_VEL * linear_throttle_factor;
			else if (joy->buttons.at(6) == 1)
				twist.linear.z = -MAX_TRANS_VEL * linear_throttle_factor;
			else
				twist.linear.z = 0.0;
		}
		else
		{
			if (joy->axes.at(5) < 1.0)
				twist.linear.z = (0.5 - joy->axes.at(5)/2.0) * MAX_ANG_VEL * angular_throttle_factor;
			else
				twist.linear.z = -(0.5 - joy->axes.at(2)/2.0) * MAX_ANG_VEL * angular_throttle_factor;
		}
		
		//bumpers control roll
		if (joy->buttons.at(5) == 1)
			twist.angular.z = MAX_ANG_VEL * angular_throttle_factor;
		else if (joy->buttons.at(4) == 1)
		  twist.angular.z = -MAX_ANG_VEL * angular_throttle_factor;
		else
			twist.angular.z = 0.0;
		
		//right joystick controls pitch and yaw
		if (controllerType == DIGITAL)
		{
			twist.angular.x = -joy->axes.at(3) * MAX_ANG_VEL * angular_throttle_factor;
			twist.angular.y = joy->axes.at(2) * MAX_ANG_VEL * angular_throttle_factor;
		}
		else
		{
			twist.angular.x = -joy->axes.at(4) * MAX_ANG_VEL * angular_throttle_factor;
			twist.angular.y = joy->axes.at(3) * MAX_ANG_VEL * angular_throttle_factor;
		}
		
		//mode switching
		if (controllerType == DIGITAL)
			buttonIndex = 0;
		else
			buttonIndex = 2;
			
		if (joy->buttons.at(buttonIndex) == 1)
		{
			//cancel trajectory and switch to finger control mode
			twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twist.linear.z = 0.0;
			twist.angular.x = 0.0;
			twist.angular.y = 0.0;
			twist.angular.z = 0.0;
			cmd_vel.publish(twist);
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
				fingerVel.finger1Vel = -joy->axes.at(3) * MAX_FINGER_VEL * finger_throttle_factor;
			else
				fingerVel.finger1Vel = -joy->axes.at(4) * MAX_FINGER_VEL * finger_throttle_factor;
		
			//top finger controlled by left triggers
			if (controllerType == DIGITAL)
			{
				if (joy->buttons.at(4) == 1)
					fingerVel.finger2Vel = -MAX_FINGER_VEL * finger_throttle_factor;
				else if (joy->buttons.at(6) == 1)
					fingerVel.finger2Vel = MAX_FINGER_VEL * finger_throttle_factor;
				else
					fingerVel.finger2Vel = 0.0;
			}
			else
			{
				if (joy->buttons.at(4) == 1)
					fingerVel.finger2Vel = -MAX_FINGER_VEL * finger_throttle_factor;
				else
					fingerVel.finger2Vel = (0.5 - joy->axes.at(2)/2.0) * MAX_FINGER_VEL * finger_throttle_factor;
			}
		
			//bottom finger controlled by right bumpers
			if (controllerType == DIGITAL)
			{
				if (joy->buttons.at(5) == 1)
					fingerVel.finger3Vel = -MAX_FINGER_VEL * finger_throttle_factor;
				else if (joy->buttons.at(7) == 1)
					fingerVel.finger3Vel = MAX_FINGER_VEL * finger_throttle_factor;
				else
					fingerVel.finger3Vel = 0.0;
			}
			else
			{
				if (joy->buttons.at(5) == 1)
					fingerVel.finger3Vel = -MAX_FINGER_VEL * finger_throttle_factor;
				else
					fingerVel.finger3Vel = (0.5 - joy->axes.at(5)/2.0) * MAX_FINGER_VEL * finger_throttle_factor;
			}
		}
		else
		{
			//control full gripper (outprioritizes individual finger control)
			fingerVel.finger1Vel = -joy->axes.at(1) * MAX_FINGER_VEL * finger_throttle_factor;
			fingerVel.finger2Vel = fingerVel.finger1Vel;
			fingerVel.finger3Vel = fingerVel.finger1Vel;
		}
	
		//mode switching
		if (controllerType == DIGITAL)
			buttonIndex = 1;
		else
			buttonIndex = 0;
			
		if (joy->buttons.at(buttonIndex) == 1)
		{
			//cancel trajectory and switch to arm control mode
			fingerVel.finger1Vel = 0.0;
			fingerVel.finger2Vel = 0.0;
			fingerVel.finger3Vel = 0.0;
			finger_vel.publish(fingerVel);
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
	{
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;
		fingerVel.finger1Vel = 0.0;
		fingerVel.finger2Vel = 0.0;
		fingerVel.finger3Vel = 0.0;
		
		cmd_vel.publish(twist);
		finger_vel.publish(fingerVel);
		
		return;
	}
	
	switch (mode)
	{
	case ARM_CONTROL:
		//only publish stop message once; this allows other nodes to publish velocities
		//while the controller is not being used
		if (twist.linear.x == 0.0 && twist.linear.y == 0.0 && twist.linear.z == 0.0
			&& twist.angular.x == 0.0	&& twist.angular.y == 0.0 && twist.angular.z == 0.0)
		{
			if (!stopMessageSentArm)
			{
				cmd_vel.publish(twist);
				stopMessageSentArm = true;
			}
		}
		else
		{
			// send the twist command
			cmd_vel.publish(twist);
			stopMessageSentArm = false;
		}
	break;
	case FINGER_CONTROL:
		//only publish stop message once; this allows other nodes to publish velocities
		//while the controller is not being used
		if (fingerVel.finger1Vel == 0.0 && fingerVel.finger2Vel == 0.0 && fingerVel.finger3Vel == 0.0)
		{
			if (!stopMessageSentFinger)
			{
				finger_vel.publish(fingerVel);
				stopMessageSentFinger = true;
			}
		}
		else
		{
			//send the finger velocity command
			finger_vel.publish(fingerVel);
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
