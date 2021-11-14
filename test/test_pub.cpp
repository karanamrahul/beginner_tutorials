#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <beginner_tutorials/AddTwoInts.h>




TEST(testTalkerNode, testpubName)
{
	ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::AddTwoInts>(
      "Adding_two_integers");
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));

}


TEST(testTalkerNode, testServiceOut) {
  // Create ros node handle.
  ros::NodeHandle node;

  // Create service client.
  ros::ServiceClient client = node.serviceClient
<beginner_tutorials::AddTwoInts>("Adding_two_integers");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = 1;
  srv.request.b = 2;

  // Call service.
  client.call(srv);

  // check whether output is same or not.
  EXPECT_EQ(3, srv.response.sum);
}
