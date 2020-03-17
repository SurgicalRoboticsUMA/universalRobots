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


