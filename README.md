# Software Development for Robotics (ENPM808x) ROS Exercise

[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)


## Overview of the project

1. [ROS Publisher/Subsciber example following the official ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

2. [ROS Services and Logging example following the official ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)

3. [ROS tf and bag files example following the official ROS Tutorials](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf)


## Dependencies

The following dependencies are required to run this package:

1. ROS Noetic
2. [catkin](http://wiki.ros.org/catkin#Installing_catkin)
3. Ubuntu 20.04 For installing [ROS Noetic](http://wiki.ros.org/noetic/Installation)

ROS Publisher and Subscriber tutorial from the official ROS Tutorials

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


Open three terminals
#1
```
roscore
```

#2
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```

#3  
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```
