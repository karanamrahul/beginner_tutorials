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
 * @copyright  MIT License (c) 2021 Rahul Karanam
 * @file       listener.cpp
 * @author     Rahul Karanam
 * @brief      This tutorial demonstrates simple sending of messages over the ROS system.
 */



#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <sstream>


/**
 * @brief This function adds two integers.
 * @param[in] input - Requesting two inputs by the client.
 * @param[in] output - The output from adding the two integers.
 * @return bool
 */
bool add(beginner_tutorials::AddTwoInts::Request &input, beginner_tutorials::AddTwoInts::Response &output) {
  output.sum=input.a +input.b;
  return true;
} 
int main(int argc, char **argv)
{
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
// 
  ros::init(argc, argv, "talker");
// 

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// Getting param from the INFO logger level.
  ROS_INFO_STREAM("Begin Talker Node   !!!!");
  ros::NodeHandle node("~");
  std::string params;
  node.getParam("params",params);
  ros::NodeHandle n;
//

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// 
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// 

// 
  ros::Rate loop_rate(10);
// 
  // Displaying logger level message based upon the params
  if (params == "FATAL") {
    ROS_FATAL_STREAM("FATAL log message!!!");
   } else if (params == "ERROR") {
    ROS_ERROR_STREAM("ERROR log message!!!");
   } else if (params == "WARN") {
    ROS_WARN_STREAM("WARN log message!!!");
   } else if (params == "INFO") {
    ROS_INFO_STREAM("INFO log message!!!");
   } else if (params == "DEBUG") {
    ROS_DEBUG_STREAM("DEBUG log message!!!");
  }
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ROS_INFO_STREAM("Beginnning the log message in the loop");

    // Streaming the log level based upon the count
  if (count%11 == 0) {
            ss << "";
            ROS_FATAL_STREAM("FATAL Logger Level.");
        } else if (count%7 == 0) {
            ss << "ERROR";
            ROS_ERROR_STREAM("ERROR Logger Level.");
        } else if (count%6 == 0) {
            ss << "WARN";
            ROS_WARN_STREAM("WARN Logger Level.");
        } else if (count%8 == 0) {
            ss << "INFO";
            ROS_INFO_STREAM("INFO Logger Level.");
        } else {
            ss << "DEBUG";
            ROS_DEBUG_STREAM("DEBUG Logger Level.");
        }

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// Publishes the message
    msg.data = ss.str();
    ++count;
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    
  }


  return 0;
}
