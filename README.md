# Week_12_Turtlebot_Walker
*Working with turtlebot, Gazebo & Rviz*

## Table of Contents
- [Overview](#overview)
- [License](#lic)
- [Prerequisites](#pre)
- [Class Diagram](#class)
- [Activity Diagram](#activity)
- [Creating a package workspace](#workspace)
- [Standard install via command-line](#implementation)

## <a name="overview"></a> Overview
The repository includes a package for turtlebot to implement a simple walker algorithm much like Roomba robot vacuum cleaner. The robot should move forward until it reaches an obstacle but should not collide with it, then the robot will rotate in place until the way ahead is cleared. Once the way is cleared the robot moves forward, and repeat's the same process again when it comes across the obstacle.

## <a name="lic"></a> License
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## <a name="pre"></a> Prerequisites 

* [ROS Kinetic](https://wiki.ros.org/ROS/Installation) on Ubuntu 16.04. 
  >Note: Follow the installation steps given on ROS Kinetic installation page.

	After installing the full version of ROS kinetic distro we need to setup ROS by following the below steps. This steps are needed to be performed only once: <br/>
  **1.** Setting up rosdep systemwide: `$ sudo rosdep init` <br/>
	**2.** Setting up rosdep in a user account: `$ rosdep update` <br/>
	**3.** Setting up enivronment variables <br/>
  Updating the bashrc file: `$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc` <br/>
  Sourcing the bashrc file: `$ source ~/.bashrc` <br/>

* [Gazebo](http://gazebosim.org/) is a well-developed robust open source physic based simulator which can be used for simulating robotic systems. It can be used along with ROS as well as independently according to the user's need.

* [Rviz](http://wiki.ros.org/rviz) is specifically used for visualization of real & simulated robotic systems.

* [Turtlebot_Gazebo](http://wiki.ros.org/turtlebot_gazebo) 
  >Note: Gazebo & Rviz will be already installed as part of the ROS distro, however Turtlebot_Gazebo is need to be installed separately.

	To install turtlebot_Gazebo, open a terminal and run the following command: <br/>
  ```bash
  $ sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
  ```
  The developed package also depends on: *roscpp*, *geometry_msgs*, & *sensor_msgs*

## <a name="class"></a> Class Diagram
<p align="center">
  <a target="_blank"><img src="UML/ClassDiagram.png" alt="NMPC" width="360" height="360" border="10" />
  </a>
</p>

## <a name="activity"></a> Activity Diagram
<p align="center">
  <a target="_blank"><img src="UML/ActivityDiagram.png" alt="NMPC" width="480" height="480" border="10" />
  </a>
</p>

## <a name="workspace"></a> Creating a package workspace

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Before any update run the command `$ source devel/setup.bash` 

## <a name="implementation"></a> Standard install via command-line

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/Learner1729/turtlebot_walker.git
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH
```
>Note: The last command checks whether the environment variable includes the directory you are in or not. If it doesn't include please follow the above steps properly. This is the most important step to check that everything is installed and linked properly. 

