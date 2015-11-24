wpi_jaco  [![Build Status](https://api.travis-ci.org/RIVeR-Lab/wpi_jaco.png)](https://travis-ci.org/RIVeR-Lab/wpi_jaco)
========

#### ROS Packages for the JACO, JACO2, and MICO Arms Developed at WPI
For full documentation, see [the ROS wiki](http://wiki.ros.org/wpi_jaco).

[Doxygen](http://docs.ros.org/indigo/api/wpi_jaco/html/) files can be found on the ROS wiki.

This package servers as an alternative to [jaco-arm](https://github.com/Kinovarobotics/jaco-ros).

### Notes about the JACO2
This package now supports the JACO2 as well as the JACO and the MICO.  Simply change the arm name in the wpi_jaco_wrapper arm.launch file from "jaco" to "jaco2", and the correct parameters will be loaded.  Presently there is no description package for the jaco2 (see [jaco_description](jaco_description) or [mico_description](jaco_description) as examples for Kinova's other arms), and no MoveIt! configuration yet, but the other functionality should work with the new model of the arm.  More details on the [wpi_jaco ROS wiki page](http://wiki.ros.org/wpi_jaco)...

### Contributing

[jaco_description](jaco_description) includes both minified versions of the 3D Collada models as well as pre-compiled URDF files. To properly contribute, do the following:

 1. Re-minify any modified Collada files
   * `cd /path/to/wpi_jaco/jaco_description/meshes`
   * `xmllint --noblanks my_modified_mesh.dae > my_modified_mesh.min.dae`
 1. Re-compile the modified URDF
   * `cd /path/to/wpi_jaco`
   * `rosrun xacro xacro jaco_description/robots/standalone_arm.urdf.xacro > jaco_description/robots/standalone_arm.urdf`

### License
For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
