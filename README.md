# Software Development for Robotics (ENPM808x) ROS Exercise

[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)


## Overview of the project

1. [ROS Publisher/Subsciber example following the official ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

2. [ROS Services and Logging example following the official ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)



## Dependencies

The following dependencies are required to run this package:

1. ROS Noetic
2. [catkin](http://wiki.ros.org/catkin#Installing_catkin)
3. Ubuntu 20.04 For installing [ROS Noetic](http://wiki.ros.org/noetic/Installation)
4. xterm

# To install xterm
```
sudo apt install xterm
```

## Standard install via command-line

```
cd ~/catkin_ws/src
mkdir beginner_tutorials
cd beginner_tutorials
git clone --recursive https://github.com/karanamrahul/beginner_tutorials
cd ../..
catkin_make
```



To run the listener and talker please run the below commands


Open two terminals
#1
```
roscore
```

#2 You can input the logger_level from these 5 levels fatal,error,warn,debug,info. i.e default is set to warn.
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch param:=<logger_level>
```
#3 To view tf frames we need to use tf2_tools viewframes 
```
rosrun tf2_tools view_frames.py

```

