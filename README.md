#  ROS NODE FOR THE UNIVERSAL ROBOTS UR ARMS

## Prerequisites
* Ubuntu 16.04
* ROS - Tested with ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu)
* MoveIt - How to install:
```bash
sudo apt-get install ros-kinetic-moveit
```

## To build
Clone package
```bash
cd catkin_ws/
catkin_make
```

## To Launch
```bash
roslaunch urx_driver init.launch
```
Note: launch roscore first in another terminal. 

## UR3 Simulator
* Install a virtual machine. For example VMWare (it can be downloadesd from www.sci.uma.es).

* Download URSim 3.5.1 (https://www.universal-robots.com/download/?option=33415#section16597).

* Follow the steps on the previous website.

To make sure that you have are able to communicate your computer with the robot simulator, open URSim UR3 in the virtual machine, then go to 'Setup Robot' and 'Network' and get the robot IP address.
Then open a terminal (ctrl+alt+T) in your computer abd ping the robot to check the communication. 




```
