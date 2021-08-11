#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>


void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO_STREAM(msg->header);
  ROS_INFO("Intensities: [%f]",msg->intensities[0]);
  ROS_INFO("time:  %f",msg->scan_time);
  std::vector<float> recu = msg->ranges;
  double max = 0;
  double min = 1;
  for ( double i : recu){
     if (!std::isnan(i)  && !std::isinf(i)){
       if (i>max) max=i;
       if (i<min) min=i;
     }
  }
  ROS_INFO("max_range detected %d", max);
  ROS_INFO("min_range detected %d", min);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listen_to_scan");
  ros::NodeHandle nw;
  ros::Subscriber sub_scan = nw.subscribe("scan", 1000, chatterCallback);
  ros::spin();
  return 0;
}