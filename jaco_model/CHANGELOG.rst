^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jaco_model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2014-07-30)
------------------
* format of URDF files
* cleanup of model package
* Renamed joint_states topic to jaco_arm/joint_states so that it can work in a larger system
* Added WPI to copyright for our changes and put LICENSE files
* Added velocity-control-based trajectory follower accessed by action server (joint_velocity_controller), the jaco_teleop package including joystick, keyboard, and interactive markers, and the jaco_model package so everything required is in the jaco_ros repository
* Contributors: Russell Toris, Velin Dimitrov, dekent
