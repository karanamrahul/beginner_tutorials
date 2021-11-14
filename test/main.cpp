#include "ros/ros.h"
#include <gtest/gtest.h>


int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "test_pub");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
