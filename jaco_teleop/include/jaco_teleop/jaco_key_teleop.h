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
 * \brief Allows for control of CARL with a keyboard.
 *
 * jaco_key_teleop creates a ROS node that allows the control of the JACO arm
 * with a keyboard.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date June 26, 2014
 */

#ifndef JACO_KEY_TELEOP_H_
#define JACO_KEY_TELEOP_H_

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <jaco_ros/JacoFingerVel.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>

//Keycodes
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_F 0x66
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_H 0x68

//Control modes
#define ARM_CONTROL 0 
#define FINGER_CONTROL 1

/*!
 * \def MAX_TRANS_VEL
 *
 * The maximum translational velocity.
 */
#define MAX_TRANS_VEL .175

/*!
 * \def MAX_FINGER_VEL
 * The maximum velocity for a finger.
 */
#define MAX_FINGER_VEL 30

/*!
 * \def MAX_ANG_VEL
 *
 * The maximum angular velocity.
 */
#define MAX_ANG_VEL 1.047

class jaco_key_teleop
{
public:
  /*!
   * Creates a jaco_key_teleop object that can be used control the JACO arm 
   * with a keyboard. ROS nodes, services, and publishers
   * are created and maintained within this object.
   */
  jaco_key_teleop();

  /*!
   * Monitors the keyboard and publishes to the arm controller whenever a key corresponding to a motion command is pressed.
   */
  void loop();

  /*!
   * Publishes cmd_vels to stop the robot if no key is pressed after a short time period.
   */
  void watchdog();

private:

  /*!
   * \brief Displays a help menu appropriate to the current mode
   *
   * Displays a help menu with keyboard controls for the current control mode
   */
  void displayHelp();

  ros::NodeHandle nh_;
  ros::Time first_publish_, last_publish_; /*! Publish times used by the watchdog. */

  boost::mutex publish_mutex_; /*! The mutex for the twist topic. */
  
  ros::Publisher cmd_vel; /*!< the cmd_vel topic */
  ros::Publisher finger_vel; /*!< the finger velocity topic */

  int mode; /*!< the controller mode */
  float linear_throttle_factor; /*!< factor for reducing the linear speed */
  float angular_throttle_factor; /*!< factor for reducing the angular speed */
  float finger_throttle_factor; /*!< factor for reducing the finger speed */
};

/*!
 * A function to close ROS and exit the program.
 *
 * \param sig The signal value.
 */
void shutdown(int sig);

/*!
 * Creates and runs the jaco_key_teleop node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
