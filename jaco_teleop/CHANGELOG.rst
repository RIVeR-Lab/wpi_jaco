^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jaco_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2014-07-30)
------------------
* cleanup of telop
* cleanup of jaco_ros
* Renamed joint_states topic to jaco_arm/joint_states so that it can work in a larger system
* Improved interaction between manipulation actions, adjusted topic names for organizational purposes
* Refactored various control methods so they are more robust to being run at the same time, created jaco_msgs package to clean up dependencies between jaco packages
* Added pickup action to arm control and interactive manipulation
* Added WPI to copyright for our changes and put LICENSE files
* Added velocity-control-based trajectory follower accessed by action server (joint_velocity_controller), the jaco_teleop package including joystick, keyboard, and interactive markers, and the jaco_model package so everything required is in the jaco_ros repository
* Contributors: Russell Toris, Velin Dimitrov, dekent
