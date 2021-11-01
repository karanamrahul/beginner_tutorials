# Overview


ROS Publisher and Subscriber tutorial from the official ROS Tutorials

## Standard install via command-line

'''
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

roscore

#2

cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener


#3  

cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
