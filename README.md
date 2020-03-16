#  ROS NODE FOR THE UNIVERSAL ROBOTS UR ARMS

## Prerequisites
* Ubuntu 16.04
* ROS - Tested with ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu)
* MoveIt (https://moveit.ros.org/install/source/) - How to install:
```bash
sudo apt-get install python-wstool python-catkin-tools clang-format-3.9
source /opt/ros/kinetic/setup.bash
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## To build
Clone package```bash
```bash
cd catkin_ws/
catkin_make
```
If you get an error, try:
```bash
catkin_make_isolated
```

