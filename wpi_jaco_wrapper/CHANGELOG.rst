^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wpi_jaco_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
