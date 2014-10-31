^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wpi_jaco_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
