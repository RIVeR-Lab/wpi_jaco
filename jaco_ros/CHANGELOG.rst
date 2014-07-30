^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jaco_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2014-07-30)
------------------
* cleanup of telop
* missing dep added
* cleanup of jaco_ros
* autoformat of code
* Renamed joint_states topic to jaco_arm/joint_states so that it can work in a larger system
* Improved interaction between manipulation actions, adjusted topic names for organizational purposes
* Moved manipulation actions to their own node
* Refactored various control methods so they are more robust to being run at the same time, created jaco_msgs package to clean up dependencies between jaco packages
* Added pickup action to arm control and interactive manipulation
* Added WPI to copyright for our changes and put LICENSE files
* Added velocity-control-based trajectory follower accessed by action server (joint_velocity_controller), the jaco_teleop package including joystick, keyboard, and interactive markers, and the jaco_model package so everything required is in the jaco_ros repository
* File cleanup
* fixed revolute joint issue, added kineamtics, angle representation conversions, and some new ways to control the arm
* Added calculation of nearest equivelent joint angle
* Added debuggin information to trajectory node
* Fixed namespace issue
  Fixed segfault in driver
* Added controller that takes trajectory action and sends full trajectory once
* Initial commit of ros_control jaco driver
* Contributors: Mitchell Wills, Russell Toris, Velin Dimitrov, dekent
