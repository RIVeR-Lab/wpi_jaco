wpi_jaco  [![Build Status](https://api.travis-ci.org/RIVeR-Lab/wpi_jaco.png)](https://travis-ci.org/RIVeR-Lab/wpi_jaco)
========

#### ROS Packages for the JACO Arm Developed at WPI
For full documentation, see [the ROS wiki](http://wiki.ros.org/wpi_jaco).

[Doxygen](http://docs.ros.org/indigo/api/wpi_jaco/html/) files can be found on the ROS wiki.

This package servers as an alternative to [jaco-arm](https://github.com/Kinovarobotics/jaco-ros).

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
