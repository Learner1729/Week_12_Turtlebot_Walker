# Week_12_Turtlebot_Walker
*Working with turtlebot, Gazebo & Rviz*

## Table of Contents
- [Overview](#overview)
- [License](#lic)
- [Prerequisites](#pre)
- [Creating a package workspace](#workspace)
- [Standard install via command-line](#implementation)
- [Run Walker using rosrun](#rosrun)
- [Run Walker using launch file](#roslaunch)
- [ROS bag](#bag)

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

## <a name="rosrun"></a> Run Walker using rosrun

After following the above standard install commands, follow the below commands to run turtlebot_gazebo and default world

```bash
$ roslaunch turtlebot_gazebo turtlebot_world.launch
```
You will see a gazebo window open along with default turtlebot_world with obstacles as well as the turtlebot robot. Now, we will run the walker node. Open a new shell and follow the below commands.

```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun turtlebot_walker turtlebot_walker_node
```
Now, the turtlebot should start moving forward. As the robot drives and when it encounters an obstacle, it will stop, turn in place to free itself and then continue driving.

>Note: We can add obstacles when the simulation is going on, which can be useful to check whether our code is working properly or not.

## <a name="roslaunch"></a> Run Walker using launch file

The ROS node and turtlebot_gazebo environment can be start using a single launch file as follows:

```bash
$ roslaunch turtlebot_walker walker.launch
```
>Note: No need to start roscore, roslaunch will automatically start if it detects that it is not already running.

To launch Walker, Gazebo and rviz altogether use the below command.
```bash
$ roslaunch turtlebot_walker walker_custom_world.launch
```
>Note: Output of the above launch file is stored in a results directory.

## <a name="bag"></a> ROS bag

Bags are created by rosbag, which subscribe to one or more ROS topics and stores the serialized message data in a file as it is received. These files can then be played back in ROS to the same topics they were recorded from, or even remapped to new topics. ROS bag can be run on simulation environment as well as on real robots. ROS provides executables named record and play that are members of the rosbag package. These executables make it easy to include bags as a part of our launch file.

```bash
$ roslaunch turtlebot_walker walker.launch record:=true
```

>Note: 35 sec is the maximum duration of data recorded in bagfile. The generated bag file is stored in the results directory.

Recorded bag file can be examine and played back using the command rosbag play and rosbag info.

**Playing the bag file:** The rosbag play command can be used to do it. First we will launch the Gazebo world and play the bag file, thus instead of running the walker node we will us the information recorded in results directory.

**1.** Launch the turtlebot_world in gazebo using the command below
  ```bash
  $ roslaunch turtlebot_gazebo turtlebot_world.launch
  ```

**2.** Open a new shell and play back the generated bag file
  ```bash
  $ cd ~/catkin_ws
  $ rosbag play src/turtlebot_walker/results/walker.bag
  ```

**Inspecting the recorded bag file:** The rosbag info command used below can provide a number of interesting snippets of information about a bag like path where rosbag file is stored, version, duration of the record time, start and end time of the recording, size of the recorded rosbag file, messages stored on a record rosbag file, compression method if applied, along with the types and topics from where messages recorded.

```bash
$ rosbag info src/turtlebot_walker/results/walker.bag
```
>Note: /camera/* topics are not being recorded as it quickly increases the bag file size. Screen shot of the output is stored in the result folder.