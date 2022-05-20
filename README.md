# Power Wheelchair Digital Twin with ROS, MQTT and LEGO EV3

This repo includes a ROS workspace containing an implementation of a Digital Twin model under a power wheelchair plus an MQTT connection to Lego EV3 with ev3dev OS.

# Import Steps

```
git clone https://github.com/amgalves96/wheelchair.git
cd wheelchair
catkin_make
cd ~/catkin_ws
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
source devel/setup.bash
roslaunch wheelchair world.launch
```
