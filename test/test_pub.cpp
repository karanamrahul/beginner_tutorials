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
 * @file       test_pub.cpp
 * @author     Rahul Karanam
 * @brief      This test file will test the publisher name and output
 *
 *
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <beginner_tutorials/AddTwoInts.h>
/**
 * @brief This test will check the name of the service.
 * @param      testTalkerNode         this is a gtest framework type
 * @param      testpubName            It is the name of our test method
 */
TEST(testTalkerNode, testpubName) {
  // Creating ROS nodehandle
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::AddTwoInts>(
  "Adding_two_integers");
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}
/**
 * @brief This test will check the output of our service
 * @param      testTalkerNode         this is a gtest framework type
 * @param      testServiceOut          It is the name of our test method
 */
TEST(testTalkerNode, testServiceOut) {
  // Create ros node handle.
  ros::NodeHandle node;
  // Create service client.
  ros::ServiceClient client = node.serviceClient
<beginner_tutorials::AddTwoInts>("Adding_two_integers");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = 3;
  srv.request.b = 2;
  //  To Call service.
  client.call(srv);
  // using EXPECT_EQ macro to check whether the output is equal or not
  EXPECT_EQ(5, srv.response.sum);
}
  TEST(testTalkerNode,checkmaster) {
    EXPECT_TRUE(ros::master::check())
  }
