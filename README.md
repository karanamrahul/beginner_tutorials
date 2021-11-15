# Software Development for Robotics (ENPM808x) ROS Exercise

[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)


## Overview of the project
1. [ROS Publisher/Subsciber example following the official ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
2. [ROS Services and Logging example following the official ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
3. [ROS Recording Bags,tf frames and rostest following the official Tutorilals](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data)

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

#2 You can input the logger_level from these 5 levels fatal,error,warn,debug,info. i.e default is set to info.
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch param:=<logger_level>
```

# Testing using rostest and gtest demo

#1 Terminal - To launch the test.launch using roslaunch

```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials test.launch
```
#2 Terminal To view the test results
```
cd catkin_ws
source devel/setup.bash
rostest beginner_tutorials test.launch
```
# Test Output
```
ubu@ubu-QEMU-Virtual-Machine:~/catkin_ws$ rostest beginner_tutorials test.launch 
... logging to /home/ubu/.ros/log/rostest-ubu-QEMU-Virtual-Machine-26247.log
[ROSUNIT] Outputting test results to /home/ubu/.ros/test_results/beginner_tutorials/rostest-test_test.xml
[FATAL] [1636935346.041828165]: FATAL Logger Level.
[Testcase: testtest_pub] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-test_pub/testpubName][passed]
[beginner_tutorials.rosunit-test_pub/testServiceOut][passed]
[beginner_tutorials.rosunit-test_pub/checkmaster][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 3
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/ubu/.ros/log/rostest-ubu-QEMU-Virtual-Machine-26247.log

```
# Bag recording and inspecting the TF frames demo 

#1 Terminal
```
roscore
```

#2 Terminal - We are recording our data with the warn logger level which will store the talker data in record_data.bag

``` 
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch param:=warn record:=true
```
#3 To view tf frames we need to use tf2_tools viewframes - frames.pdf is generated for the record_data.bag

```
cd catkin_ws
source devel/setup.bash
rosrun tf2_tools view_frames.py
|or|
rosrun rqt_tf_tree rqt_tf_tree
--> to open the frames.pdf in cmd
evince frames.pdf
```
#4 To view the bag recording info
```
cd catkin_ws
source devel/setup.bash
cd src/beginner_tutorials/results/record_data.bag
rosbag info record_data.bag
```
#Output
```
ros info record_data.bag 
path:        record_data.bag
version:     2.0
duration:    14.7s
start:       Nov 14 2021 14:35:40.61 (1636918540.61)
end:         Nov 14 2021 14:35:55.32 (1636918555.32)
size:        169.6 KB
messages:    821
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      139 msgs    : std_msgs/String   
             /rosout       271 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   272 msgs    : rosgraph_msgs/Log 
             /tf           139 msgs    : tf2_msgs/TFMessage
```
#Listener node demo from recorded bag

#1 Terminal 1
```
roscore
```

#2 Terminal 2
```
cd catkin_ws
source devel/setup.bash
rosbag play src/beginner_tutorials/results/record_data.bag
```

#3 Terminal 3
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```

