#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <sstream>

void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("farthest:  %f",msg->data);
}
void chatterCallback2(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("nearest:  %f",msg->data);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pubs_test");
  ros::NodeHandle fr;
  ros::Subscriber sub_fr_scan = fr.subscribe("farthest", 1000, chatterCallback);
  ros::NodeHandle nr;

  // à modifier à while(ros::ok())
  ros::Subscriber sub_nr_scan = nr.subscribe("nearest", 1000, chatterCallback2);
  ros::spin();
  return 0;
}