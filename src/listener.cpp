/*
 * MIT License
 * 
 * Copyright (c) 2021 Rahul karanam
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @copyright  MIT License (c) 2021 Rahul Karanam
 * @file       listener.cpp
 * @author     Rahul Karanam
 * @brief      This tutorial demonstrates simple listener of messages over the ROS system.
 *
 *
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddTwoInts.h"
/**
 * @brief This outputs the logger level messages.
 * @param[in] msg ; This is a variable which is given by the publisher.
 * @return None
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  if (msg->data == "FATAL") {
    ROS_FATAL_STREAM("I heard FATAL log message!!!");
  }  else if (msg->data == "ERROR") {
    ROS_ERROR_STREAM("I heard ERROR log message!!!");
  }  else if (msg->data == "WARN") {
    ROS_WARN_STREAM("I heard WARN log message!!!");
  }  else if (msg->data == "INFO") {
    ROS_INFO_STREAM("I heard INFO log message!!!");
  }  else if (msg->data == "DEBUG") {
    ROS_DEBUG_STREAM("I heard DEBUG log message!!!");
  }
}
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ROS_INFO_STREAM("Begin node listener");
  ros::NodeHandle listener;
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// Subscriber
  ros::Subscriber sub = listener.subscribe("chatter", 1000, chatterCallback);
// Creating a service client
  ros::ServiceClient client = listener.serviceClient
  <beginner_tutorials::AddTwoInts>("Adding_two_integers");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = 10;
  srv.request.b = 20;
  if (client.call(srv)) {
    ROS_INFO_STREAM("The sum of both the integers is : " << srv.response.sum);
  } else {
    ROS_ERROR_STREAM("Issue with the service !!!!");
  }
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  return 0;
}
