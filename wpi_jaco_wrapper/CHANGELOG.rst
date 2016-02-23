^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wpi_jaco_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.25 (2016-02-23)
-------------------
* small bugfix on arm initialization status
* Fixed a trajectory following bug involving trajectories coming in from MoveIt
* Bugfix for kinova_gripper param
* Added kinova_gripper parameter, setting it to false will remove the action servers specifically using the two- or three-fingered kinova gripper designed for use with the jaco or mico
* Added initial support for the Jaco2 arm, see readme for details
* Added initial support for the Jaco2 arm, see readme for details
* Contributors: David Kent

0.0.24 (2015-08-18)
-------------------
* reverted changelog
* changelog updated
* Added parameter to make homing the arm on initialization optional
* Added a fingers_controller_radian action server to better integrate with MoveIt! using joint_state information for gripper close/open actions
* Contributors: David Kent, Russell Toris

0.0.23 (2015-05-04)
-------------------
* Adjusted startup delay on gripper action finished check
* Fixes to sometimes incorrect reporting of gripper action result in the jaco_manipulation action server
* Contributors: David Kent

0.0.22 (2015-04-22)
-------------------
* Removed an out of date debug statement that was filling up the logs
* Contributors: David Kent

0.0.21 (2015-04-17)
-------------------
* Check if InitApi is succesful.
* Contributors: Mathijs de Langen

0.0.20 (2015-04-14)
-------------------
* typo
* Server starts immediately and we check if the arm is initialized in the callback function
* Revert "file not used"
  This reverts commit 51053ff0f7325aef8ee1345cefbe790f48ba003b.
* file not used
* Forgotten { during merge
* Merge branch 'develop' of https://github.com/RobotRose/wpi_jaco into develop
  Conflicts:
  wpi_jaco_wrapper/include/wpi_jaco_wrapper/jaco_arm_trajectory_node.h
  wpi_jaco_wrapper/launch/arm.launch
  wpi_jaco_wrapper/src/jaco_arm_trajectory_node.cpp
* default to mico_arm for us :P
* moveitmodel
* Making the jaco_arm_trajectory node work for MoveIt! with the Mico
* Made some remapping for the arm.launch (so the topics are in order)
* Finger speed added as constant (3000 is ~half of the Mico speeds)
* debug ON
* Changed joint_state publisher to what MoveIt! understands (change back in future)
* movements somewhat slower
* Finger speed added as constant (3000 is ~half of the Mico speeds)
* Merge remote-tracking branch 'upstream/master' into develop
* debug ON
* Changed joint_state publisher to what MoveIt! understands (change back in future)
* movements somewhat slower
* Contributors: Mathijs de Langen

0.0.19 (2015-04-10)
-------------------

0.0.18 (2015-04-03)
-------------------
* Publish a message after the arm homes using the kinova api
* Contributors: David Kent

0.0.17 (2015-03-27)
-------------------
* CMake typo fix
* install files missing
* Contributors: Russell Toris

0.0.16 (2015-03-24)
-------------------
* Removed some debugging output
* Put back the remapping
* Removed the *0.8 for testing
* Added some comments for the parameters
* finger_threshold, made error counting for mico depend on two/three fingers
* added finger_error_threshold as configurable parameter
* added finger_error_threshold as configurable parameter
* Moved the } two lines to below (WRONG MERGE:/)
* removed space
* recommented stuff
* for loops for the fingers
* newline
* Newlines at end of file
* Load params for each node
* Deal with different number of fingers
* Removed test code
* private nodehandle not needed
* Made num finger joints configurable
* added parameters, fixed typo
* forgotten nodehandle
* actionlib typedefs, made actionlib constructs pointer so parameters can be loaded first
* Synced all topics names with arm_name_
* conversions topic uses arm name parameter
* made loadParameters a const function
* Renamed variables to follow ROS naming conventions
* Topic renaming
* Renamed topics
* Made jaco_arm_trajectory_node configurable
* Configurable parameters via #defines added
* Missing a lock in the gripper action server, should fix a potential crash with the jaco
* Changed gripper action success conditions to better reflect reality
* Removed redundant messages
* Updated jaco interaction to use the new rail_manipulation_msgs
* Switched jaco_manipulation to use rail_manipulation_msgs
* Contributors: David Kent, Mathijs de Langen

0.0.15 (2015-02-17)
-------------------
* Documentation
* adjustment to erase trajectories service
* Merge branch 'develop' of https://github.com/RIVeR-Lab/wpi_jaco into develop
* Added some minor service calls to support some other packages
* Contributors: David Kent

0.0.14 (2015-02-06)
-------------------
* Added software estop for the arm
* Contributors: David Kent

0.0.13 (2015-02-03)
-------------------
* Result on gripper control action server now reports correctly.
* Initial adjustment of gripper action server to fix result feedback
* Contributors: David Kent

0.0.12 (2015-01-20)
-------------------
* Tuned finger position controller, added detection and termination if the fingers are blocked from reaching their goal
* Finger position control test
* Removed unused debug statement
* Adjusted angular and cartesian command callbacks to correctly execute finger position commands when arm joint commands are not specified
* Contributors: David Kent

0.0.11 (2014-12-18)
-------------------

0.0.10 (2014-12-12)
-------------------

0.0.9 (2014-12-02)
------------------
* Added service call to get angular position of arm joints
* Contributors: David Kent

0.0.8 (2014-10-22)
------------------
* tuning
* Added check to see if the gripper is already open before the gripper opening loop
* Parameter tuning
* Added max_curvature parameter for trajectory planning, thresholded gripper opening to prevent a bug that causes gripper opening to terminate late
* Contributors: David Kent

0.0.7 (2014-09-19)
------------------
* bugfix on gripper closing
* Contributors: dekent

0.0.6 (2014-09-02)
------------------

0.0.5 (2014-08-25)
------------------
* release prep
* adjusted teleop due to a mode switching bug in the arm; moved teleop to jaco_teleop and included support for segmentation in jaco_interaction
* fix for issue with single angular position commands
* testing retract command
* adjusted retract position
* Home and retract actions added to interactive markers
* Contributors: Russell Toris, dekent

0.0.4 (2014-08-05)
------------------
* documentation
* renamed namespace in library
* fixed header names in cpp files
* renamed wrapper headers
* correctly links against JACO libraries via cmake
* updated package names in launch files
* Contributors: Russell Toris, dekent

0.0.3 (2014-08-01)
------------------

0.0.2 (2014-08-01)
------------------

0.0.1 (2014-07-31)
------------------
* renamed JACO to WPI packages
* Contributors: Russell Toris
