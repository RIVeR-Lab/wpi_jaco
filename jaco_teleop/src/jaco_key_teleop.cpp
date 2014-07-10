/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Willow Garage, Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \jaco_key_teleop.h
 * \brief Allows for control of the JACO arm with a keyboard.
 *
 * jaco_key_teleop creates a ROS node that allows the control of the JACO arm
 * with a keyboard.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date June 26, 2014
 */

#include <jaco_teleop/jaco_key_teleop.h>

// used for capturing keyboard input
int kfd = 0;
struct termios cooked, raw;

jaco_key_teleop::jaco_key_teleop()
{
	// create the ROS topics
  cmd_vel = nh_.advertise<geometry_msgs::Twist>("jaco_arm/cmd_vel", 10);
  finger_vel = nh_.advertise<jaco_ros::JacoFingerVel>("jaco_arm/finger_cmd_vel", 10);

  // read in throttle values
  double temp;
  if (nh_.getParam("/jaco_joy_teleop/linear_throttle_factor", temp))
    linear_throttle_factor = (float)temp;
  else
    linear_throttle_factor = 1.0;
	if (nh_.getParam("/jaco_joy_teleop/angular_throttle_factor", temp))
    angular_throttle_factor = (float)temp;
  else
    angular_throttle_factor = 1.0;
  if (nh_.getParam("/jaco_joy_teleop/finger_throttle_factor", temp))
    finger_throttle_factor = (float)temp;
  else
    finger_throttle_factor = 1.0;

	mode = ARM_CONTROL;

  ROS_INFO("JACO Joystick Teleop Started");
}

void jaco_key_teleop::watchdog()
{
	switch (mode)
	{
	case ARM_CONTROL:
	{
  	boost::mutex::scoped_lock lock(publish_mutex_);
  	if ((ros::Time::now() > last_publish_ + ros::Duration(0.15))
      && (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    {
    	geometry_msgs::Twist twist;
    	twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twist.linear.z = 0.0;
			twist.angular.x = 0.0;
			twist.angular.y = 0.0;
			twist.angular.z = 0.0;
			cmd_vel.publish(twist);
  	}
	}
	break;
	case FINGER_CONTROL:
	{
		boost::mutex::scoped_lock lock(publish_mutex_);
  	if ((ros::Time::now() > last_publish_ + ros::Duration(0.15))
      && (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    {
			jaco_ros::JacoFingerVel fingerVel;
			fingerVel.finger1Vel = 0.0;
			fingerVel.finger2Vel = 0.0;
			fingerVel.finger3Vel = 0.0;
			finger_vel.publish(fingerVel);
		}
	}
	break;
	}
}

void jaco_key_teleop::loop()
{
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("    Reading from Keyboard    ");
  puts("-----------------------------");
  puts("  Press the H key for help ");

  while (ros::ok())
  {
    // get the next event from the keyboard
    char c;
    if (read(kfd, &c, 1) < 0)
    {
      ROS_ERROR("Could not read input from keyboard.");
      exit(-1);
    }

		//Display help message
		if (c == KEYCODE_H)
		{
			displayHelp();
		}

		switch (mode)
		{
		case ARM_CONTROL:
		{
			//initialize twist command
			geometry_msgs::Twist twist;
			twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twist.linear.z = 0.0;
			twist.angular.x = 0.0;
			twist.angular.y = 0.0;
			twist.angular.z = 0.0;
		
		  // w/s control forward/backward translation
		  // a/d control left/right translation
		  // r/f control up/down translation
		  // q/e controls roll
		  // up/down controls pitch
		  // left/right controls yaw
		  switch(c)
		  {
		  case KEYCODE_W:	twist.linear.y = -MAX_TRANS_VEL * linear_throttle_factor;	break;
  		case KEYCODE_S:	twist.linear.y = MAX_TRANS_VEL * linear_throttle_factor;	break;
  		case KEYCODE_A:	twist.linear.x = MAX_TRANS_VEL * linear_throttle_factor;	break;
			case KEYCODE_D:	twist.linear.x = -MAX_TRANS_VEL * linear_throttle_factor;	break;
			case KEYCODE_R:	twist.linear.z = MAX_TRANS_VEL * linear_throttle_factor;	break;
			case KEYCODE_F: twist.linear.z = -MAX_TRANS_VEL * linear_throttle_factor;	break;
			case KEYCODE_Q:	twist.angular.z = -MAX_ANG_VEL * angular_throttle_factor;	break;
			case KEYCODE_E:	twist.angular.z = MAX_ANG_VEL * angular_throttle_factor;	break;
			case KEYCODE_UP:	twist.angular.x = -MAX_ANG_VEL * angular_throttle_factor;	break;
			case KEYCODE_DOWN:	twist.angular.x = MAX_ANG_VEL * angular_throttle_factor;	break;
			case KEYCODE_LEFT:	twist.angular.y = MAX_ANG_VEL * angular_throttle_factor;	break;
			case KEYCODE_RIGHT:	twist.angular.y = -MAX_ANG_VEL * angular_throttle_factor;	break;
			case
				KEYCODE_2: mode = FINGER_CONTROL;
				ROS_INFO("Activated finger control mode");
			break;
			}
  		
  		//publish twist to arm controller
  		boost::mutex::scoped_lock lock(publish_mutex_);
		  if (ros::Time::now() > last_publish_ + ros::Duration(1.0))
		  {
		    first_publish_ = ros::Time::now();
		  }
		  last_publish_ = ros::Time::now();
			cmd_vel.publish(twist);
		}
		break;
		case FINGER_CONTROL:
		{
			//initialize finger command
			jaco_ros::JacoFingerVel fingerVel;
			fingerVel.finger1Vel = 0.0;
			fingerVel.finger2Vel = 0.0;
			fingerVel.finger3Vel = 0.0;
			
			// q/a controls finger 1
			// w/s controls finger 2
			// e/d controls finger 3
			// r/f controls entire hand
			switch(c)
		  {
		  case KEYCODE_Q:	fingerVel.finger1Vel = -MAX_FINGER_VEL * finger_throttle_factor;	break;
  		case KEYCODE_A:	fingerVel.finger1Vel = MAX_FINGER_VEL * finger_throttle_factor;	break;
  		case KEYCODE_W:	fingerVel.finger2Vel = -MAX_FINGER_VEL * finger_throttle_factor;	break;
			case KEYCODE_S:	fingerVel.finger2Vel = MAX_FINGER_VEL * finger_throttle_factor;	break;
			case KEYCODE_E:	fingerVel.finger3Vel = -MAX_FINGER_VEL * finger_throttle_factor;	break;
			case KEYCODE_D: fingerVel.finger3Vel = MAX_FINGER_VEL * finger_throttle_factor;	break;
			case KEYCODE_R:
				fingerVel.finger1Vel = -MAX_FINGER_VEL * finger_throttle_factor;
				fingerVel.finger2Vel = fingerVel.finger1Vel;
				fingerVel.finger3Vel = fingerVel.finger1Vel;
			break;
			case KEYCODE_F:
				fingerVel.finger1Vel = MAX_FINGER_VEL * finger_throttle_factor;
				fingerVel.finger2Vel = fingerVel.finger1Vel;
				fingerVel.finger3Vel = fingerVel.finger1Vel;
			break;
			case
				KEYCODE_1: mode = ARM_CONTROL;
				ROS_INFO("Activated arm control mode");
			break;
			}
			
			//publish twist to finger controller
  		boost::mutex::scoped_lock lock(publish_mutex_);
		  if (ros::Time::now() > last_publish_ + ros::Duration(1.0))
		  {
		    first_publish_ = ros::Time::now();
		  }
		  last_publish_ = ros::Time::now();
			finger_vel.publish(fingerVel);
		}
		break;
		}
  }
}

void jaco_key_teleop::displayHelp()
{
	switch (mode)
	{
	case ARM_CONTROL:
		puts(" ------------------------------------");
		puts("| Jaco Keyboard Teleop Help          |");
		puts("|------------------------------------|*");
		puts("| Current Mode: Arm Control          |*");
		puts("|------------------------------------|*");
		puts("| w/s : forward/backward translation |*");
		puts("| a/d : left/right translation       |*");
		puts("| r/f : up/down translation          |*");
		puts("| q/e : roll                         |*");
		puts("| up/down : pitch                    |*");
		puts("| left/right : yaw                   |*");
		puts("| 2 : switch to Finger Control       |*");
		puts(" ------------------------------------**");
		puts("  *************************************");
	break;
	case FINGER_CONTROL:
		puts(" ------------------------------------");
		puts("| Jaco Keyboard Teleop Help          |");
		puts("|------------------------------------|*");
		puts("| Current Mode: Finger Control       |*");
		puts("|------------------------------------|*");
		puts("| q/a : open/close thumb             |*");
		puts("| w/s : open/close top finger        |*");
		puts("| e/d : open/close bottom finger     |*");
		puts("| r/f : open/close entire hand       |*");
		puts("| 1 : switch to Finger Control       |*");
		puts(" ------------------------------------**");
		puts("  *************************************");
	break;
	}
}

void shutdown(int sig)
{
  // shut everything down
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "jaco_key_teleop");

  // initialize the keyboard controller
  jaco_key_teleop key_controller;
  ros::NodeHandle n;

  // setup the SIGINT signal for exiting
  signal(SIGINT, shutdown);

  // setup the watchdog and key loop in a thread
  boost::thread my_thread(boost::bind(&jaco_key_teleop::loop, &key_controller));
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&jaco_key_teleop::watchdog, &key_controller));
  ros::spin();

  // wait for everything to end
  my_thread.interrupt();
  my_thread.join();

  return EXIT_SUCCESS;
}
