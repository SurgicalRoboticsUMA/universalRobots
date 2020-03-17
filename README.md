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
* Install a virtual machine. For example, VirtualBox (https://download.virtualbox.org/virtualbox/6.1.4/virtualbox-6.1_6.1.4-136177~Ubuntu~xenial_amd64.deb).

* Download URSim 3.5.1 (https://www.universal-robots.com/download/?option=33415#section16597).

* Follow the steps in the previous website depending on the virtual machine you have.




```
