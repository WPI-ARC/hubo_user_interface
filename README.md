hubo_user_interface (master 0.1)
===================

RVIZ Plugins and tools for interfacing with the Hubo robot.

**This README.md is written for tag 0.1 on the master branch.**


Repository Structure
--------------------
This repository does not contain a Catkin workspace and therefore must be cloned into the `/src` directory of your catkin workspace.

Please note that this software is written for ROS Groovy, and is incompatible with ROS Furete and earlier.

This repository currently contains 4 major packages:

1. `hubo_sensor_control` - An RVIZ panel plugin for controlling the sensor bandwidth of the robot from the RVIZ GUI. Version 0.1 contains sliders for sensors that pertain to the robot's state as well as the robot's vision. 

2. `hubo_dashboard` - An RVIZ panel plugin for indicating the last time that a message was received from the Hubo robot over the link software that has been created in `teleop_toolkit`

3. `localization_markers` - **CURRENTLY UNDER HEAVY DEVELOPMENT** A combination of RVIZ panel plugin and interactive_markers that are used for the localization of objects in the RVIZ environment. It is currently under heavy development and should not be used, but it will not cause build issues.

4. `localization_marker_msgs` - **CURRENTLY UNDER HEAVY DEVELOPMENT** Messages that are needed by the `localization_markers` package in order to function properly.


Stability
---------
All packages are currently stable and can be built if pulled from the master branch. The tag for the branch this README is written for is 0.1.


Dependencies
------------
1. Full ROS Groovy installation - on Ubuntu systems: `$ sudo apt-get install ros-groovy-desktop-full`

2. `teleop_toolkit` repository created by WPI ARC Lab. - `git clone https://github.com/WPI-ARC/teleop_toolkit.git`


Build and usage instructions
----------------------------
First, clone this repository:
```
$ cd /your/catkin/workspace/src
$ git clone https://github.com/WPI-ARC/hubo_user_interface.git
$ rospack profile
```
Second, clone the teleop_toolkit repository also into the catkin src directory:
```
$ cd /your/catkin/workspace/src
$ git clone https://github.com/WPI-ARC/teleop_toolkit.git
$ rospack profile
```
To build all packages in this repository:

```
(in the surrounding Catkin workspace directory)
$ catkin_make
```
To build a particular package in the repository:

```
(in the surrounding Catkin workspace directory)
$ catkin_make --pkg <package name>
```
To use, you must source the workspace:

```
(in the surrounding Catkin workspace directory)
$ source devel/setup.bash
```

More Information
----------------
For usage information and instructions on running components of these packages together, see the repository [Wiki](https://github.com/WPI-ARC/hubo_user_interface/wiki).
