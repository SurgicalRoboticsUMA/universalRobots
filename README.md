#  ROS NODE FOR THE UNIVERSAL ROBOTS UR ARMS

## Prerequisites
* Ubuntu 16.04
* ROS - Tested with ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu)
* MoveIt - How to install:
```bash
sudo apt-get install ros-kinetic-moveit
```
* Package *libntcan*: dowload from (http://wiki.ros.org/libntcan).

## To build
Download the package (https://github.com/SurgicalRoboticsUMA/universalRobots.git) and copy it into folder /catkin_ws/src. Then:
```bash
cd ~/catkin_ws
catkin_make
```

## To Launch
```bash
roslaunch urx_driver init.launch
```
Before launching the node, make sure that your UR3 robot (real robot or simulator) is running, and that you have set the ip address of the robot properly in the launch file. The IP address should be the same of the one displayed in the robot console or simulator. 

## UR3 Simulator
* Install a virtual machine. For example VMWare (it can be downloadesd from www.sci.uma.es).

* Download URSim 3.5.1 (https://www.universal-robots.com/download/?option=33415#section16597).

* Follow the steps on the previous website.

To make sure that you have are able to communicate your computer with the robot simulator, open URSim UR3 in the virtual machine, then go to 'Setup Robot' and 'Network' and get the robot IP address.
Then open a terminal (ctrl+alt+T) in your computer and ping the robot to check the communication. 

![screenshots](https://github.com/SurgicalRoboticsUMA/universalRobots/blob/master/media/robot_ip.png)

![screenshots](https://github.com/SurgicalRoboticsUMA/universalRobots/blob/master/media/ping_robot.png)

